// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// IFC -> solid model + meshes (`rux import ifc`, reusex-3iy8.11). The only
// TU that includes web-ifc (its include dirs are generic and ship their own
// glm; reusexLibrary.cmake). The geometric readings are the Revit import's,
// shared through bim_mapping.hpp.

#include "io/ifc.hpp"

#include "bim_mapping.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef REUSEX_HAVE_WEB_IFC
#include <glm/glm.hpp>
#include <modelmanager/ModelManager.h>
#include <schema/ifc-schema.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <set>
#endif

namespace reusex::io::ifc {

using nlohmann::json;

std::string ifc_class_slug(const std::string &ifc_class) {
  std::string s = ifc_class;
  if (s.rfind("Ifc", 0) == 0 || s.rfind("IFC", 0) == 0)
    s = s.substr(3);
  for (std::string_view suffix : {"StandardCase", "ElementedCase"})
    if (s.size() > suffix.size() &&
        s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0)
      s.resize(s.size() - suffix.size());
  std::string out;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const auto c = static_cast<unsigned char>(s[i]);
    if (!std::isalnum(c) || c >= 128) {
      if (!out.empty() && out.back() != '_')
        out += '_';
      continue;
    }
    if (std::isupper(c) && i > 0 && !out.empty() && out.back() != '_') {
      const auto prev = static_cast<unsigned char>(s[i - 1]);
      const bool next_lower =
          i + 1 < s.size() &&
          std::islower(static_cast<unsigned char>(s[i + 1]));
      if (std::islower(prev) || std::isdigit(prev) ||
          (std::isupper(prev) && next_lower))
        out += '_';
    }
    out += static_cast<char>(std::tolower(c));
  }
  while (!out.empty() && out.back() == '_')
    out.pop_back();
  return out.empty() ? std::string("uncategorized") : out;
}

#ifndef REUSEX_HAVE_WEB_IFC

bool has_ifc_support() { return false; }

IfcImport import_ifc(const std::filesystem::path &, const IfcImportOptions &) {
  throw std::runtime_error(
      "IFC import is not available: ReUseX was built without web-ifc "
      "(REUSEX_HAVE_WEB_IFC off; see pkgs/web-ifc)");
}

#else

bool has_ifc_support() { return true; }

namespace {

using webifc::parsing::IfcTokenType;
namespace schema = webifc::schema;
using bim::P2;
using bim::Soup;

/// Namespace of the deterministic GUIDs when the caller gives none (fixed,
/// arbitrary; import_ifc_file always passes the project GUID).
constexpr const char *kDefaultGuidNamespace =
    "6f1e2d3c-4b5a-4987-8a6b-5c4d3e2f1a09";
/// An opening narrower / lower than this is not an opening [m].
constexpr double kMinOpeningM = 0.05;
/// Depth limit of the spatial tree walk.
constexpr int kMaxSpatialDepth = 32;

/// Typed attribute access over web-ifc's token tape. Every reader checks
/// the token first: web-ifc's own getters assume the type and misread an
/// empty ($) or derived (*) attribute.
struct Reader {
  webifc::parsing::IfcLoader &L;

  /// A line the file defines (a dangling #ref is not one).
  bool valid(uint32_t id) const { return id != 0 && L.IsValidExpressID(id); }

  /// The token type of an attribute; EMPTY for a line the file does not
  /// define (web-ifc's MoveToArgumentOffset would silently stay where the
  /// tape was and the next read would decode another line).
  IfcTokenType token(uint32_t id, uint32_t arg) const {
    if (!valid(id))
      return IfcTokenType::EMPTY;
    L.MoveToArgumentOffset(id, arg);
    return L.GetTokenType();
  }

  /// The line type; 0 for an invalid id.
  uint32_t type(uint32_t id) const { return valid(id) ? L.GetLineType(id) : 0; }

  /// A STRING or ENUM attribute, decoded; "" when empty / another type.
  std::string str(uint32_t id, uint32_t arg) const {
    const auto t = token(id, arg);
    if (t != IfcTokenType::STRING && t != IfcTokenType::ENUM)
      return {};
    L.StepBack();
    return L.GetDecodedStringArgument();
  }

  std::optional<double> real(uint32_t id, uint32_t arg) const {
    const auto t = token(id, arg);
    if (t != IfcTokenType::REAL && t != IfcTokenType::INTEGER)
      return std::nullopt;
    L.StepBack();
    return L.GetDoubleArgument();
  }

  /// A REF attribute; 0 when empty or dangling.
  uint32_t ref(uint32_t id, uint32_t arg) const {
    if (token(id, arg) != IfcTokenType::REF)
      return 0;
    L.StepBack();
    const uint32_t to = L.GetRefArgument();
    return valid(to) ? to : 0;
  }

  /// The REFs of a SET / LIST attribute.
  std::vector<uint32_t> refs(uint32_t id, uint32_t arg) const {
    std::vector<uint32_t> out;
    if (token(id, arg) != IfcTokenType::SET_BEGIN)
      return out;
    L.StepBack();
    const auto offsets = L.GetSetArgument();
    for (const auto off : offsets)
      if (L.GetTokenType(off) == IfcTokenType::REF)
        if (const uint32_t to = L.GetRefArgument(off); valid(to))
          out.push_back(to);
    return out;
  }

  /// The first ENUM of a SET (a typed value such as IFCBOOLEAN(.T.)).
  std::string enum_in_set(uint32_t id, uint32_t arg) const {
    if (token(id, arg) != IfcTokenType::SET_BEGIN)
      return {};
    L.StepBack();
    const auto offsets = L.GetSetArgument();
    for (const auto off : offsets)
      if (L.GetTokenType(off) == IfcTokenType::ENUM) {
        L.StepBack();
        return std::string(L.GetStringArgument());
      }
    return {};
  }

  /// The single REAL of a SET (a typed value's tape: LABEL then a set).
  std::optional<double> real_in_set(uint32_t id, uint32_t arg) const {
    if (token(id, arg) != IfcTokenType::SET_BEGIN)
      return std::nullopt;
    L.StepBack();
    const auto offsets = L.GetSetArgument();
    for (const auto off : offsets) {
      const auto t = L.GetTokenType(off);
      if (t == IfcTokenType::REAL || t == IfcTokenType::INTEGER)
        return L.GetDoubleArgument(off);
    }
    return std::nullopt;
  }
};

double si_prefix(std::string_view p) {
  if (p == "EXA")
    return 1e18;
  if (p == "PETA")
    return 1e15;
  if (p == "TERA")
    return 1e12;
  if (p == "GIGA")
    return 1e9;
  if (p == "MEGA")
    return 1e6;
  if (p == "KILO")
    return 1e3;
  if (p == "HECTO")
    return 1e2;
  if (p == "DECA")
    return 1e1;
  if (p == "DECI")
    return 1e-1;
  if (p == "CENTI")
    return 1e-2;
  if (p == "MILLI")
    return 1e-3;
  if (p == "MICRO")
    return 1e-6;
  if (p == "NANO")
    return 1e-9;
  return 1.0;
}

/// Metres per length unit of the project (IfcProject.UnitsInContext), as
/// web-ifc reads it for the geometry (IfcCache::ReadLinearScalingFactor):
/// an IfcSIUnit LENGTHUNIT (METRE with a prefix), or an
/// IfcConversionBasedUnit LENGTHUNIT (its factor times its SI unit's).
double length_unit(const Reader &r) {
  const auto projects = r.L.GetExpressIDsWithType(schema::IFCPROJECT);
  if (projects.empty())
    return 1.0;
  const uint32_t units = r.ref(projects[0], 8);
  if (units == 0)
    return 1.0;
  for (const uint32_t u : r.refs(units, 0)) {
    const auto type = r.type(u);
    if (type == schema::IFCSIUNIT) {
      if (r.str(u, 1) == "LENGTHUNIT" && r.str(u, 3) == "METRE")
        return si_prefix(r.str(u, 2));
    } else if (type == schema::IFCCONVERSIONBASEDUNIT) {
      if (r.str(u, 1) != "LENGTHUNIT")
        continue;
      const uint32_t factor = r.ref(u, 3); // IfcMeasureWithUnit
      if (factor == 0)
        continue;
      // ValueComponent is a typed value: LABEL, then a one-element set.
      const auto ratio = r.real_in_set(factor, 1);
      const uint32_t si = r.ref(factor, 2);
      double base = 1.0;
      if (si != 0 && r.type(si) == schema::IFCSIUNIT && r.str(si, 3) == "METRE")
        base = si_prefix(r.str(si, 2));
      if (ratio)
        return *ratio * base;
    }
  }
  return 1.0;
}

/// FILE_SCHEMA of the STEP header, read from the text (web-ifc's
/// GetSchema() reports IFC2X3 for IFC4 files).
std::string file_schema(const std::filesystem::path &file) {
  std::ifstream in(file, std::ios::binary);
  std::string head(1 << 16, '\0');
  in.read(head.data(), static_cast<std::streamsize>(head.size()));
  head.resize(static_cast<std::size_t>(in.gcount()));
  static const std::regex re(R"(FILE_SCHEMA\s*\(\s*\(\s*'([^']*)')",
                             std::regex::icase);
  std::smatch m;
  if (std::regex_search(head, m, re))
    return m[1].str();
  return {};
}

/// web-ifc emits meshes Y-up (GetFlatMesh pre-multiplies NormalizeIFC); its
/// inverse as the processor transformation gives the file's Z-up world.
std::array<double, 16> ifc_z_up() {
  const glm::dmat4 inv = glm::inverse(webifc::geometry::NormalizeIFC);
  std::array<double, 16> flat{};
  for (int c = 0; c < 4; ++c)
    for (int r = 0; r < 4; ++r)
      flat[static_cast<std::size_t>(c * 4 + r)] = inv[c][r];
  return flat;
}

struct Product {
  uint32_t id = 0;
  std::string cls; ///< "IfcWallStandardCase"
  std::string global_id, name, object_type, tag;
  Soup geom; ///< world metres (offset applied)
};

json identity(const Product &p) {
  json j = {{"GlobalId", p.global_id}, {"class", p.cls}, {"express_id", p.id}};
  if (!p.name.empty())
    j["Name"] = p.name;
  if (!p.object_type.empty())
    j["ObjectType"] = p.object_type;
  if (!p.tag.empty())
    j["Tag"] = p.tag;
  return j;
}

enum class Role { wall, opening, other };

Role role_of(uint32_t type) {
  switch (type) {
  case schema::IFCWALL:
  case schema::IFCWALLSTANDARDCASE:
  case schema::IFCWALLELEMENTEDCASE:
    return Role::wall;
  case schema::IFCDOOR:
  case schema::IFCDOORSTANDARDCASE:
  case schema::IFCWINDOW:
  case schema::IFCWINDOWSTANDARDCASE:
    return Role::opening;
  default:
    return Role::other;
  }
}

bool is_door(uint32_t type) {
  return type == schema::IFCDOOR || type == schema::IFCDOORSTANDARDCASE;
}

/// The class a StandardCase / ElementedCase specialises
/// ("IfcWallStandardCase" -> "IfcWall").
std::string base_class(std::string cls) {
  for (std::string_view suffix : {"StandardCase", "ElementedCase"})
    if (cls.size() > suffix.size() + 3 &&
        cls.compare(cls.size() - suffix.size(), suffix.size(), suffix) == 0)
      cls.resize(cls.size() - suffix.size());
  return cls;
}

/// Products that are no building part to mesh: the spatial structure
/// (spaces are mapped separately), voids, annotation.
bool skipped_class(uint32_t type) {
  switch (type) {
  case schema::IFCPROJECT:
  case schema::IFCSITE:
  case schema::IFCBUILDING:
  case schema::IFCBUILDINGSTOREY:
  case schema::IFCSPACE:
  case schema::IFCSPATIALZONE:
  case schema::IFCEXTERNALSPATIALELEMENT:
  case schema::IFCOPENINGELEMENT:
  case schema::IFCOPENINGSTANDARDCASE:
  case schema::IFCVIRTUALELEMENT:
  case schema::IFCANNOTATION:
  case schema::IFCGRID:
    return true;
  default:
    return false;
  }
}

/// web-ifc logs through spdlog's *default* logger: free spdlog::debug /
/// info / error calls throughout its parser and geometry kernel (about 70
/// sites; ModelManager::SetLogger reaches none of them). In rux the default
/// logger is rux's own, so `rux -vvv import ifc` printed web-ifc's debug
/// trace as [rux] lines. While a WebIfcLog::Scope is alive the default
/// logger is a logger named "web-ifc" over the same sinks, at warn or
/// above (never below the previous level), which also counts errors; the
/// previous default logger is restored when the scope ends. Scopes wrap
/// web-ifc calls only, never this file's own logging (which goes through
/// the same default logger in rux). Swaps a process global: not
/// thread-safe, like web-ifc's own logging.
class WebIfcLog {
    public:
  WebIfcLog() : counter_(std::make_shared<Counter>()) {
    const auto prev = spdlog::default_logger();
    std::vector<spdlog::sink_ptr> sinks;
    if (prev)
      sinks = prev->sinks();
    sinks.push_back(counter_);
    logger_ =
        std::make_shared<spdlog::logger>("web-ifc", sinks.begin(), sinks.end());
    const auto level = prev ? prev->level() : spdlog::level::info;
    logger_->set_level(std::max(level, spdlog::level::warn));
  }
  ~WebIfcLog() {
    if (spdlog::get("web-ifc") == logger_)
      spdlog::drop("web-ifc");
  }
  WebIfcLog(const WebIfcLog &) = delete;
  WebIfcLog &operator=(const WebIfcLog &) = delete;

  class Scope {
      public:
    explicit Scope(const WebIfcLog &log) : prev_(spdlog::default_logger()) {
      spdlog::set_default_logger(log.logger_);
    }
    ~Scope() { spdlog::set_default_logger(prev_); }
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;

      private:
    std::shared_ptr<spdlog::logger> prev_;
  };

  /// Errors web-ifc logged so far.
  [[nodiscard]] std::size_t errors() const { return counter_->errors; }

    private:
  struct Counter : spdlog::sinks::base_sink<std::mutex> {
    std::size_t errors = 0;
    void sink_it_(const spdlog::details::log_msg &msg) override {
      if (msg.level >= spdlog::level::err)
        ++errors;
    }
    void flush_() override {}
  };
  std::shared_ptr<Counter> counter_;
  std::shared_ptr<spdlog::logger> logger_;
};

/// Shape representations that are the element's body. web-ifc meshes every
/// representation of a product (it skips bounding boxes and curves, but
/// e.g. a 'Clearance' solid would be meshed).
bool is_body_representation(const std::string &identifier) {
  return identifier.empty() || identifier == "Body" ||
         identifier == "Facetation" || identifier == "Body-FallBack";
}

/// The ids web-ifc may file a product's meshed geometry under, split by the
/// kind of representation they come from. web-ifc files a geometry under
/// a representation item or a part of one (a mapped item's items, a
/// boolean operand, a surface model's face sets / shells, a geometric
/// set's elements), so items are expanded through those.
struct RepresentationItems {
  std::set<uint32_t> body, other;

  /// Whether a geometry web-ifc filed under @p id is to be left out: it
  /// belongs to a non-body representation only. Anything not traced to a
  /// representation is kept.
  [[nodiscard]] bool drop(uint32_t id) const {
    return other.count(id) > 0 && body.count(id) == 0;
  }
};

RepresentationItems representation_items(const Reader &r, uint32_t product) {
  RepresentationItems out;
  const uint32_t shape = r.ref(product, 6);
  if (shape == 0)
    return out;
  std::function<void(std::set<uint32_t> &, uint32_t, int)> add =
      [&](std::set<uint32_t> &set, uint32_t item, int depth) {
        if (depth > 8 || !set.insert(item).second)
          return;
        switch (r.type(item)) {
        case schema::IFCMAPPEDITEM: {
          const uint32_t map = r.ref(item, 0); // IfcRepresentationMap
          for (const uint32_t sub : r.refs(r.ref(map, 1), 3))
            add(set, sub, depth + 1);
          break;
        }
        case schema::IFCBOOLEANRESULT:
        case schema::IFCBOOLEANCLIPPINGRESULT:
          for (const uint32_t arg : {1u, 2u})
            if (const uint32_t operand = r.ref(item, arg); operand != 0)
              add(set, operand, depth + 1);
          break;
        case schema::IFCFACEBASEDSURFACEMODEL:
        case schema::IFCSHELLBASEDSURFACEMODEL:
        case schema::IFCGEOMETRICSET:
        case schema::IFCGEOMETRICCURVESET:
          for (const uint32_t sub : r.refs(item, 0))
            add(set, sub, depth + 1);
          break;
        default:
          break;
        }
      };
  for (const uint32_t rep : r.refs(shape, 2)) {
    auto &set = is_body_representation(r.str(rep, 1)) ? out.body : out.other;
    for (const uint32_t item : r.refs(rep, 3))
      add(set, item, 0);
  }
  return out;
}

/// A STEP file ends with END-ISO-10303-21; a truncated download or copy
/// does not, and web-ifc parses what it has without complaint.
void check_complete(const std::filesystem::path &file) {
  std::ifstream in(file, std::ios::binary | std::ios::ate);
  const auto size = static_cast<std::streamoff>(in.tellg());
  std::string head(static_cast<std::size_t>(std::min<std::streamoff>(size, 64)),
                   '\0');
  in.seekg(0);
  in.read(head.data(), static_cast<std::streamsize>(head.size()));
  const std::streamoff n = std::min<std::streamoff>(size, 4096);
  std::string tail(static_cast<std::size_t>(n), '\0');
  in.seekg(size - n);
  in.read(tail.data(), n);
  auto upper = [](std::string s) {
    for (auto &ch : s)
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    return s;
  };
  if (upper(head).find("ISO-10303-21") == std::string::npos)
    throw std::runtime_error(fmt::format(
        "import_ifc: '{}' is not an IFC STEP file (no ISO-10303-21 header)",
        file.string()));
  const std::string t = upper(tail);
  if (t.find("END-ISO-10303-21") == std::string::npos ||
      t.find("ENDSEC") == std::string::npos)
    throw std::runtime_error(fmt::format(
        "import_ifc: '{}' is truncated (no ENDSEC / END-ISO-10303-21 at the "
        "end)",
        file.string()));
}

} // namespace

IfcImport import_ifc(const std::filesystem::path &file,
                     const IfcImportOptions &opt) {
  std::ifstream in(file, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("import_ifc: cannot open '{}'", file.string()));

  check_complete(file);

  IfcImport out;
  auto &st = out.stats;
  st.schema = file_schema(file);

  WebIfcLog wlog;
  auto quiet = [&](auto &&f) -> decltype(auto) {
    WebIfcLog::Scope scope(wlog);
    return f();
  };
  webifc::manager::ModelManager manager(false);
  // Not SetLogLevel(): it re-levels and re-patterns the logger it is handed.
  // SetLogger only reaches ModelManager's own messages; the rest of web-ifc
  // logs through spdlog's default logger, see WebIfcLog.
  manager.SetLogger(std::make_shared<spdlog::logger>("web-ifc-model"));
  const uint32_t model_id = quiet(
      [&] { return manager.CreateModel(webifc::manager::LoaderSettings{}); });
  auto *loader = manager.GetIfcLoader(model_id);
  const bool loaded = quiet([&] { return loader->LoadFile(in); });
  if (!loaded || wlog.errors() > 0)
    throw std::runtime_error(fmt::format(
        "import_ifc: '{}' is not a readable IFC (STEP) file ({} parse "
        "errors)",
        file.string(), wlog.errors()));
  if (loader->GetExpressIDsWithType(schema::IFCPROJECT).empty())
    throw std::runtime_error(
        fmt::format("import_ifc: '{}' has no IfcProject", file.string()));
  const auto &schemas = manager.GetSchemaManager();
  auto *gp = manager.GetGeometryProcessor(model_id);
  gp->SetTransformation(ifc_z_up());
  Reader r{*loader};

  st.length_unit_m = length_unit(r);
  const double unit = st.length_unit_m;
  core::info(
      "import_ifc: '{}' ({}), length unit {} m", file.filename().string(),
      st.schema.empty() ? std::string("schema unknown") : st.schema, unit);

  // A product's body in world metres. web-ifc places it in file units
  // (applyLinearScalingFactor off): `unit` is the one length unit of this
  // import, for the meshes as for the attributes.
  auto soup_of = [&](uint32_t id, std::size_t *failed) {
    Soup s;
    const auto reps = representation_items(r, id);
    try {
      const auto fm = quiet([&] { return gp->GetFlatMesh(id, false); });
      for (const auto &pg : fm.geometries) {
        if (reps.drop(pg.geometryExpressID)) {
          ++st.non_body_geometries;
          continue;
        }
        const auto &g = quiet([&]() -> const auto & {
          return gp->GetGeometry(pg.geometryExpressID);
        });
        const auto base = static_cast<std::uint32_t>(s.v.size());
        const std::size_t nv = g.vertexData.size() / 6;
        for (std::size_t i = 0; i < nv; ++i) {
          const glm::dvec4 p =
              pg.transformation * glm::dvec4(g.vertexData[6 * i],
                                             g.vertexData[6 * i + 1],
                                             g.vertexData[6 * i + 2], 1.0);
          s.v.emplace_back(p.x * unit, p.y * unit, p.z * unit);
        }
        for (std::size_t k = 0; k + 2 < g.indexData.size(); k += 3) {
          const auto a = g.indexData[k], b = g.indexData[k + 1],
                     c = g.indexData[k + 2];
          if (a < nv && b < nv && c < nv)
            s.t.push_back({base + a, base + b, base + c});
        }
      }
    } catch (const std::exception &e) {
      core::debug("import_ifc: no geometry for #{}: {}", id, e.what());
      if (failed)
        ++*failed;
      s = Soup{};
    }
    return s;
  };

  auto read_product = [&](uint32_t id, uint32_t type) {
    Product p;
    p.id = id;
    p.cls = schemas.IfcTypeCodeToType(type);
    p.global_id = r.str(id, 0);
    p.name = r.str(id, 2);
    p.object_type = r.str(id, 4);
    return p;
  };

  // A space's plan outline and z range from its 'FootPrint' curve and
  // 'Box' bounding box, for spaces exported without a 'Body' (common from
  // ArchiCAD): world metres, the offset not yet applied.
  struct Footprint {
    std::vector<P2> ring;
    std::optional<std::pair<double, double>> z; ///< from the Box
  };
  auto space_footprint = [&](uint32_t id) -> std::optional<Footprint> {
    const uint32_t shape = r.ref(id, 6);
    if (shape == 0)
      return std::nullopt;
    const uint32_t placement = r.ref(id, 5);
    auto &geo = gp->GetLoader();
    const glm::dmat4 m =
        placement != 0 ? quiet([&] { return geo.GetLocalPlacement(placement); })
                       : glm::dmat4(1.0);
    auto world = [&](double x, double y, double z) {
      const glm::dvec4 p = m * glm::dvec4(x, y, z, 1.0);
      return glm::dvec3(p) * unit;
    };
    Footprint fp;
    for (const uint32_t rep : r.refs(shape, 2)) {
      const std::string ident = r.str(rep, 1);
      for (const uint32_t item : r.refs(rep, 3)) {
        const auto type = r.type(item);
        if (ident == "Box" && type == schema::IFCBOUNDINGBOX) {
          const uint32_t corner = r.ref(item, 0);
          const auto zdim = r.real(item, 3);
          if (corner == 0 || !zdim)
            continue;
          const glm::dvec3 c =
              quiet([&] { return geo.GetCartesianPoint3D(corner); });
          const double lo = world(c.x, c.y, c.z).z;
          const double hi = world(c.x, c.y, c.z + *zdim).z;
          fp.z = std::pair{std::min(lo, hi), std::max(lo, hi)};
        } else if (ident == "FootPrint") {
          std::vector<uint32_t> curves{item};
          if (type == schema::IFCGEOMETRICCURVESET ||
              type == schema::IFCGEOMETRICSET)
            curves = r.refs(item, 0);
          for (const uint32_t cid : curves) {
            std::vector<P2> ring;
            try {
              const auto curve = quiet([&] { return geo.GetCurve(cid, 2); });
              for (const auto &q : curve.points) {
                const glm::dvec3 w = world(q.x, q.y, 0.0);
                ring.emplace_back(w.x, w.y);
              }
            } catch (const std::exception &e) {
              core::debug("import_ifc: footprint curve #{}: {}", cid, e.what());
              continue;
            }
            if (ring.size() > 1 && (ring.front() - ring.back()).norm() < 1e-9)
              ring.pop_back();
            if (ring.size() >= 3 && std::abs(bim::ring_area(ring)) >
                                        std::abs(bim::ring_area(fp.ring)))
              fp.ring = std::move(ring);
          }
        }
      }
    }
    if (fp.ring.size() < 3)
      return std::nullopt;
    if (bim::ring_area(fp.ring) < 0)
      std::reverse(fp.ring.begin(), fp.ring.end());
    return fp;
  };

  // ---- spatial tree: element -> container, part -> whole
  std::unordered_map<uint32_t, uint32_t> parent_of;
  for (const uint32_t rel :
       loader->GetExpressIDsWithType(schema::IFCRELAGGREGATES)) {
    const uint32_t whole = r.ref(rel, 4);
    for (const uint32_t part : r.refs(rel, 5))
      parent_of.emplace(part, whole);
  }
  for (const uint32_t rel : loader->GetExpressIDsWithType(
           schema::IFCRELCONTAINEDINSPATIALSTRUCTURE)) {
    const uint32_t structure = r.ref(rel, 5);
    for (const uint32_t e : r.refs(rel, 4))
      parent_of.emplace(e, structure);
  }
  // door / window -> opening element -> voided element
  std::unordered_map<uint32_t, uint32_t> opening_of, voided_by;
  for (const uint32_t rel :
       loader->GetExpressIDsWithType(schema::IFCRELFILLSELEMENT))
    opening_of.emplace(r.ref(rel, 5), r.ref(rel, 4));
  for (const uint32_t rel :
       loader->GetExpressIDsWithType(schema::IFCRELVOIDSELEMENT))
    voided_by.emplace(r.ref(rel, 5), r.ref(rel, 4));

  // element -> its property sets (occurrence level; type psets not read)
  std::unordered_map<uint32_t, std::vector<uint32_t>> psets_of;
  for (const uint32_t rel :
       loader->GetExpressIDsWithType(schema::IFCRELDEFINESBYPROPERTIES)) {
    // RelatingPropertyDefinition: one property set, or (IFC4) an
    // IfcPropertySetDefinitionSet -- a typed value, so a LABEL attribute
    // followed by its set of property sets.
    std::vector<uint32_t> defs;
    if (r.token(rel, 5) == IfcTokenType::LABEL)
      defs = r.refs(rel, 6);
    else if (const uint32_t def = r.ref(rel, 5); def != 0)
      defs.push_back(def);
    for (const uint32_t def : defs) {
      if (r.type(def) != schema::IFCPROPERTYSET)
        continue;
      for (const uint32_t e : r.refs(rel, 4))
        psets_of[e].push_back(def);
    }
  }
  // A boolean single value (IFCBOOLEAN / IFCLOGICAL) of a property set.
  auto bool_property = [&](uint32_t id, std::string_view pset,
                           std::string_view prop) -> std::optional<bool> {
    auto it = psets_of.find(id);
    if (it == psets_of.end())
      return std::nullopt;
    for (const uint32_t ps : it->second) {
      if (r.str(ps, 2) != pset)
        continue;
      for (const uint32_t pv : r.refs(ps, 4)) {
        if (r.type(pv) != schema::IFCPROPERTYSINGLEVALUE ||
            r.str(pv, 0) != prop)
          continue;
        // NominalValue is a typed value: LABEL (arg 2), then its set (3).
        const std::string v = r.enum_in_set(pv, 3);
        if (v == "T")
          return true;
        if (v == "F")
          return false;
      }
    }
    return std::nullopt;
  };

  // ---- storeys (world z of their placement; Elevation as a fallback)
  struct Storey {
    Product p;
    std::string long_name;
    double z = 0.0; ///< world z of the floor [m]
    std::optional<double> elevation_m;
    std::string z_source;
  };
  std::vector<Storey> storeys;
  for (const uint32_t id :
       loader->GetExpressIDsWithType(schema::IFCBUILDINGSTOREY)) {
    Storey s;
    s.p = read_product(id, schema::IFCBUILDINGSTOREY);
    s.long_name = r.str(id, 7);
    if (auto e = r.real(id, 9))
      s.elevation_m = *e * unit;
    const uint32_t placement = r.ref(id, 5);
    if (placement != 0) {
      const glm::dmat4 m =
          quiet([&] { return gp->GetLoader().GetLocalPlacement(placement); });
      s.z = m[3][2] * unit;
      s.z_source = "placement";
    } else if (s.elevation_m) {
      s.z = *s.elevation_m;
      s.z_source = "Elevation";
    } else {
      core::warn("import_ifc: storey '{}' (#{}) has neither a placement nor "
                 "an Elevation; skipped",
                 s.p.name, id);
      continue;
    }
    storeys.push_back(std::move(s));
  }
  if (storeys.empty())
    throw std::runtime_error(fmt::format(
        "import_ifc: '{}' has no IfcBuildingStorey", file.string()));
  std::sort(storeys.begin(), storeys.end(),
            [](const Storey &a, const Storey &b) {
              if (a.z != b.z)
                return a.z < b.z;
              if (a.p.name != b.p.name)
                return a.p.name < b.p.name;
              return a.p.id < b.p.id;
            });
  st.storeys = storeys.size();

  // ---- products and their geometry (file world frame, metres)
  std::vector<Product> walls, openings, spaces;
  std::vector<Product> others; // meshes only
  std::unordered_map<uint32_t, uint32_t> type_of;
  std::vector<uint32_t> types(schemas.GetIfcElementList().begin(),
                              schemas.GetIfcElementList().end());
  std::sort(types.begin(), types.end()); // deterministic order
  std::size_t failed = 0;
  for (const uint32_t type : types) {
    if (skipped_class(type) && type != schema::IFCSPACE)
      continue;
    auto ids = loader->GetExpressIDsWithType(type);
    std::sort(ids.begin(), ids.end());
    for (const uint32_t id : ids) {
      Product p = read_product(id, type);
      p.geom = soup_of(id, &failed);
      type_of.emplace(id, type);
      if (type == schema::IFCSPACE) {
        spaces.push_back(std::move(p));
        continue;
      }
      ++st.products;
      ++st.classes[p.cls];
      if (p.geom.empty())
        ++st.products_no_geometry;
      switch (role_of(type)) {
      case Role::wall:
        walls.push_back(std::move(p));
        break;
      case Role::opening:
        p.tag = r.str(id, 7);
        openings.push_back(std::move(p));
        break;
      default:
        others.push_back(std::move(p));
      }
    }
  }
  for (auto &w : walls)
    w.tag = r.str(w.id, 7);
  if (failed > 0)
    core::warn("import_ifc: web-ifc could not mesh {} products; they have "
               "no geometry",
               failed);
  st.geometry_errors = wlog.errors();
  if (st.geometry_errors > 0)
    core::warn("import_ifc: web-ifc logged {} errors while meshing (shown "
               "above as [web-ifc]); the elements concerned may lack "
               "geometry",
               st.geometry_errors);
  if (st.non_body_geometries > 0)
    core::debug("import_ifc: left out {} meshed items of non-Body "
                "representations",
                st.non_body_geometries);

  // ---- precision: recentre a model far from the origin (STANDARDS §4),
  // as the Revit import does. Records are computed in double; the stored
  // meshes are float.
  {
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(
                        std::numeric_limits<double>::infinity()),
                    hi = -lo;
    for (const auto *group : {&walls, &openings, &spaces, &others})
      for (const auto &p : *group) {
        if (p.geom.empty())
          continue;
        const auto [plo, phi] = p.geom.bounds();
        lo = lo.cwiseMin(plo);
        hi = hi.cwiseMax(phi);
      }
    const Eigen::Vector3d off = bim::far_offset(
        lo, hi, opt.recenter, opt.recenter_threshold_m, "import_ifc");
    if (!off.isZero()) {
      for (auto *group : {&walls, &openings, &spaces, &others})
        for (auto &p : *group)
          for (auto &v : p.geom.v)
            v -= off;
      st.world_offset_m = {off.x(), off.y(), off.z()};
    }
  }
  const Eigen::Vector3d world_offset(st.world_offset_m[0], st.world_offset_m[1],
                                     st.world_offset_m[2]);

  // ---- storey records: one shared level frame, the lowest floor at z = 0
  const double z0 = storeys.front().z;
  std::unordered_map<uint32_t, int> storey_idx;
  bim::GuidAllocator guid_for(opt.guid_namespace.empty() ? kDefaultGuidNamespace
                                                         : opt.guid_namespace,
                              "imported/ifc");
  for (std::size_t i = 0; i < storeys.size(); ++i) {
    const auto &s = storeys[i];
    core::StoreyRecord rec;
    rec.idx = static_cast<int>(i);
    rec.name = !s.p.name.empty() ? s.p.name : s.long_name;
    rec.z_floor = s.z - z0;
    rec.z_ceiling = i + 1 < storeys.size()
                        ? storeys[i + 1].z - z0
                        : rec.z_floor + opt.default_storey_height_m;
    rec.level_transform = bim::level_frame(z0);
    rec.guid = guid_for("storeys", s.p.global_id, i);
    json md = {{"ifc", identity(s.p)},
               {"z_world_m", s.z},
               {"z_source", s.z_source},
               {"world_offset_m", st.world_offset_m}};
    if (!s.long_name.empty())
      md["ifc"]["LongName"] = s.long_name;
    if (s.elevation_m)
      md["ifc"]["Elevation_m"] = *s.elevation_m;
    rec.metadata = md.dump();
    storey_idx.emplace(s.p.id, rec.idx);
    out.model.storeys.push_back(std::move(rec));
  }
  out.model.provided.storeys = true;
  const auto &storey_recs = out.model.storeys;
  auto storey_for = [&](uint32_t id, double z_level) {
    uint32_t cur = id;
    for (int depth = 0; depth < kMaxSpatialDepth; ++depth) {
      auto it = parent_of.find(cur);
      if (it == parent_of.end())
        break;
      cur = it->second;
      if (auto s = storey_idx.find(cur); s != storey_idx.end())
        return s->second;
    }
    ++st.elements_no_storey;
    int idx = 0;
    for (const auto &s : storey_recs)
      if (s.z_floor <= z_level + 0.5)
        idx = s.idx;
    return idx;
  };
  auto storey_name = [&](int idx) {
    return storey_recs[static_cast<std::size_t>(idx)].name;
  };

  // ---- walls
  std::vector<bim::HostWall> hosts;
  std::unordered_map<uint32_t, std::size_t> wall_index; // express id -> record
  for (std::size_t wi = 0; wi < walls.size(); ++wi) {
    const auto &p = walls[wi];
    if (p.geom.empty()) {
      ++st.walls_no_geometry;
      continue;
    }
    const auto fit = bim::fit_wall(p.geom);
    const double thickness = fit.t_hi - fit.t_lo;
    if (fit.ok && thickness < opt.min_wall_thickness_m) {
      ++st.walls_thin;
      continue;
    }
    if (!fit.ok || fit.fill < opt.min_wall_fill) {
      ++st.walls_not_straight;
      continue;
    }
    core::WallRecord w;
    w.guid = guid_for("walls", p.global_id, wi);
    hosts.push_back(bim::wall_axis(fit, p.geom, thickness, z0, w));
    w.storey = storey_for(p.id, w.z_base);
    const auto external = bool_property(p.id, "Pset_WallCommon", "IsExternal");
    w.is_external = external.value_or(false);
    json md = {{"ifc", identity(p)},
               {"storey", storey_name(w.storey)},
               {"footprint_fill", fit.fill}};
    if (external)
      md["ifc"]["IsExternal"] = *external;
    w.metadata = md.dump();
    wall_index.emplace(p.id, out.model.walls.size());
    out.model.walls.push_back(std::move(w));
  }
  out.model.provided.walls = true;
  st.walls = out.model.walls.size();
  if (st.walls_thin > 0)
    core::warn("import_ifc: {} walls thinner than {:.3f} m (finish layers, "
               "linings) are not wall records; their geometry stays in the "
               "wall mesh",
               st.walls_thin, opt.min_wall_thickness_m);
  if (st.walls_not_straight > 0)
    core::warn("import_ifc: {} walls have no straight rectangular footprint "
               "(curved, profiled or irregular; fill < {:.2f}); they are "
               "mesh-only",
               st.walls_not_straight, opt.min_wall_fill);
  if (st.walls_no_geometry > 0)
    core::warn("import_ifc: {} walls have no geometry; skipped",
               st.walls_no_geometry);

  // ---- openings (doors, windows). Extent: the IfcOpeningElement the
  // element fills (the hole in the wall, as Revit's rough opening), else
  // the element's OverallWidth / OverallHeight (width centred on its extent
  // along the host, height from its bottom), else its geometry (which
  // includes frames and linings).
  std::unordered_map<uint32_t, Soup> void_geom;
  for (std::size_t oi = 0; oi < openings.size(); ++oi) {
    const auto &p = openings[oi];
    const Soup *span = &p.geom;
    std::optional<std::size_t> host;
    bool by_relation = false;
    if (auto o = opening_of.find(p.id); o != opening_of.end()) {
      auto [it, fresh] = void_geom.try_emplace(o->second);
      if (fresh) {
        it->second = soup_of(o->second, nullptr);
        for (auto &v : it->second.v)
          v -= world_offset;
      }
      if (!it->second.empty())
        span = &it->second;
      if (auto w = voided_by.find(o->second); w != voided_by.end())
        if (auto k = wall_index.find(w->second); k != wall_index.end()) {
          host = k->second;
          by_relation = true;
        }
    }
    if (span->empty()) {
      ++st.openings_unhosted;
      continue;
    }
    const auto [lo, hi] = span->bounds();
    if (!host)
      host = bim::find_host(
          hosts, P2(0.5 * (lo.x() + hi.x()), 0.5 * (lo.y() + hi.y())), lo.z(),
          hi.z(), opt.opening_host_tolerance_m);
    if (!host) {
      ++st.openings_unhosted;
      continue;
    }
    const auto &g = hosts[*host];
    auto [s_lo, s_hi] = bim::span_on(g, *span);
    double z_lo = lo.z(), z_hi = hi.z();
    std::string s_source = "geometry", z_source = "geometry";
    if (span != &p.geom) {
      s_source = z_source = "IfcOpeningElement";
    } else {
      // IfcDoor / IfcWindow: OverallHeight (8), OverallWidth (9).
      const auto height = r.real(p.id, 8);
      const auto width = r.real(p.id, 9);
      if (width && *width > 0) {
        const double sc = 0.5 * (s_lo + s_hi);
        s_lo = sc - 0.5 * *width * unit;
        s_hi = sc + 0.5 * *width * unit;
        s_source = "OverallWidth";
      }
      if (height && *height > 0) {
        z_hi = z_lo + *height * unit;
        z_source = "OverallHeight";
      }
    }
    s_lo = std::clamp(s_lo, 0.0, g.length);
    s_hi = std::clamp(s_hi, 0.0, g.length);
    if (s_hi - s_lo < kMinOpeningM || z_hi - z_lo < kMinOpeningM) {
      ++st.openings_unhosted;
      continue;
    }
    (s_source == "geometry" && z_source == "geometry"
         ? st.openings_extent_geometry
         : st.openings_extent_params)++;
    const auto &wall = out.model.walls[*host];
    core::OpeningRecord rec;
    rec.guid = guid_for("openings", p.global_id, oi);
    rec.wall = wall.guid;
    rec.kind = is_door(type_of[p.id]) ? "door" : "window";
    rec.s_start = s_lo;
    rec.s_end = s_hi;
    rec.z_sill = z_lo - z0;
    rec.z_head = z_hi - z0;
    rec.is_external = wall.is_external;
    json md = {{"ifc", identity(p)},
               {"host", by_relation ? "IfcRelFillsElement" : "geometry"},
               {"extent_source", {{"s", s_source}, {"z", z_source}}}};
    rec.metadata = md.dump();
    (by_relation ? st.openings_by_relation : st.openings_by_geometry)++;
    out.model.openings.push_back(std::move(rec));
  }
  out.model.provided.openings = true;
  st.openings = out.model.openings.size();
  if (st.openings_unhosted > 0)
    core::warn("import_ifc: {} of {} doors / windows have no host wall among "
               "the imported walls (or no geometry); they are mesh-only",
               st.openings_unhosted, openings.size());

  // ---- spaces: the outline of the body's bottom faces, else the
  // FootPrint curve (z from the Box, else the storey's floor to ceiling)
  for (std::size_t si = 0; si < spaces.size(); ++si) {
    const auto &p = spaces[si];
    const std::string long_name = r.str(p.id, 7);
    core::SpaceRecord s;
    std::string source;
    int storey = -1;
    if (!p.geom.empty()) {
      const auto [zlo, zhi] = p.geom.z_range();
      bim::OutlineDrops drops;
      auto poly = bim::room_outline(p.geom, zlo, drops);
      st.space_pieces_dropped += drops.pieces;
      st.space_islands_dropped += drops.islands;
      if (!poly) {
        ++st.spaces_bad_outline;
        continue;
      }
      s.polygon = std::move(*poly);
      s.z_floor = zlo - z0;
      s.z_ceiling = zhi - z0;
      source = "Body";
    } else if (auto fp = space_footprint(p.id)) {
      for (const auto &q : fp->ring)
        s.polygon.outer.push_back(
            {q.x() - world_offset.x(), q.y() - world_offset.y()});
      if (fp->z) {
        s.z_floor = fp->z->first - z0;
        s.z_ceiling = fp->z->second - z0;
        source = "FootPrint+Box";
      } else {
        storey = storey_for(p.id, 0.0);
        const auto &sr = storey_recs[static_cast<std::size_t>(storey)];
        s.z_floor = sr.z_floor;
        s.z_ceiling = sr.z_ceiling;
        source = "FootPrint";
      }
      ++st.spaces_from_footprint;
    } else {
      ++st.spaces_no_geometry;
      continue;
    }
    s.guid = guid_for("spaces", p.global_id, si);
    s.storey = storey >= 0 ? storey : storey_for(p.id, s.z_floor);
    s.name = !long_name.empty() ? long_name : p.name;
    s.kind = "room";
    bim::fill_space_metrics(s, p.geom);
    json md = {{"ifc", identity(p)},
               {"storey", storey_name(s.storey)},
               {"geometry", source}};
    if (!long_name.empty())
      md["ifc"]["LongName"] = long_name;
    s.metadata = md.dump();
    out.model.spaces.push_back(std::move(s));
  }
  out.model.provided.spaces = true;
  st.spaces = out.model.spaces.size();
  if (st.spaces_no_geometry > 0)
    core::warn("import_ifc: {} of {} IfcSpaces have neither a body nor a "
               "footprint; skipped",
               st.spaces_no_geometry, spaces.size());
  if (st.spaces_bad_outline > 0)
    core::warn("import_ifc: {} IfcSpaces have a body without a closed bottom "
               "outline; skipped",
               st.spaces_bad_outline);
  if (st.space_pieces_dropped + st.space_islands_dropped > 0)
    core::warn("import_ifc: space outlines left out {} disjoint pieces (only "
               "the largest piece of a space is kept) and {} islands inside "
               "holes",
               st.space_pieces_dropped, st.space_islands_dropped);
  if (st.elements_no_storey > 0)
    core::warn("import_ifc: {} elements are in no IfcBuildingStorey; placed "
               "on the storey their bottom stands in",
               st.elements_no_storey);

  // ---- meshes, one per IFC class (StandardCase folded into its base)
  if (opt.meshes) {
    bim::MeshMerger merger("imported/ifc/");
    for (const auto *group : {&walls, &openings, &others})
      for (const auto &p : *group)
        merger.add(base_class(p.cls), p.geom, ifc_class_slug(p.cls));
    out.meshes = merger.take();
  }

  core::info("import_ifc: {} products -> {} storeys, {} walls, {} openings, "
             "{} spaces, {} meshes",
             st.products, storey_recs.size(), st.walls, st.openings, st.spaces,
             out.meshes.size());
  return out;
}

#endif // REUSEX_HAVE_WEB_IFC

IfcFileImportResult import_ifc_file(ProjectDB &db,
                                    const std::filesystem::path &file,
                                    const IfcImportOptions &opt) {
  const json source = {{"source", "ifc"}, {"file", file.filename().string()}};
  const int log_id = db.log_pipeline_start("import_ifc", source.dump());
  IfcFileImportResult res;
  try {
    IfcImportOptions mapping = opt;
    mapping.guid_namespace = db.project_guid();
    IfcImport imported = import_ifc(file, mapping);
    res.stats = imported.stats;
    mark_imported(imported.model);

    ProjectDB::Transaction tx(db);
    res.stored = store_solid_model(db, imported.model);
    if (opt.meshes) {
      json params = source;
      params["schema"] = res.stats.schema;
      params["frame"] = "ifc_world";
      params["world_offset_m"] = res.stats.world_offset_m;
      const auto w =
          bim::write_meshes(db, imported.meshes, "import_ifc", params);
      res.meshes_saved = w.saved;
      res.meshes_removed = w.removed;
    }
    tx.commit();
    if (res.meshes_removed > 0)
      core::info("import_ifc: removed {} meshes of an earlier import that "
                 "this file no longer has",
                 res.meshes_removed);
    db.log_pipeline_end(log_id, true);
  } catch (const std::exception &e) {
    db.log_pipeline_end(log_id, false, e.what());
    throw;
  }
  return res;
}

} // namespace reusex::io::ifc
