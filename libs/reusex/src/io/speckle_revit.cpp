// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/speckle_revit.hpp"

#include "bim_mapping.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace reusex::io::speckle {

using nlohmann::json;

namespace {

/// Namespace of the deterministic GUIDs when the caller gives none (fixed,
/// arbitrary; import_speckle_revit always passes the project GUID).
constexpr const char *kDefaultGuidNamespace =
    "3d1c7a9e-5b0f-4e2a-9c4d-7f6e2b8a1c35";

constexpr int kMaxInstanceDepth = 8;
/// An opening narrower than this along its host is not an opening [m].
constexpr double kMinOpeningWidthM = 0.05;

// ---------------------------------------------------------------- units ---

/// Metres per unit, for Speckle unit strings ("mm", "m", "ft", ...) and the
/// Revit parameter unit names ("Millimeters", ...). nullopt when unknown.
std::optional<double> unit_scale(std::string u) {
  std::transform(u.begin(), u.end(), u.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (u == "mm" || u == "millimeters" || u == "millimetres" ||
      u == "millimeter")
    return 0.001;
  if (u == "cm" || u == "centimeters" || u == "centimetres")
    return 0.01;
  if (u == "m" || u == "meters" || u == "metres" || u == "meter")
    return 1.0;
  if (u == "km" || u == "kilometers")
    return 1000.0;
  if (u == "ft" || u == "feet" || u == "foot")
    return 0.3048;
  if (u == "in" || u == "inches" || u == "inch")
    return 0.0254;
  if (u == "yd" || u == "yards")
    return 0.9144;
  return std::nullopt;
}

double units_of(const json &obj, double fallback) {
  if (obj.is_object()) {
    auto it = obj.find("units");
    if (it != obj.end() && it->is_string())
      if (auto s = unit_scale(it->get<std::string>()))
        return *s;
  }
  return fallback;
}

std::string str_of(const json &obj, const char *key) {
  if (!obj.is_object())
    return {};
  auto it = obj.find(key);
  if (it == obj.end() || it->is_null())
    return {};
  if (it->is_string())
    return it->get<std::string>();
  return it->dump();
}

std::string speckle_type(const json &obj) {
  return str_of(obj, "speckle_type");
}

bool ends_with(std::string_view s, std::string_view suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool is_mesh(const json &obj) {
  return speckle_type(obj).find("Objects.Geometry.Mesh") != std::string::npos;
}

bool is_instance_proxy(const json &obj) {
  return ends_with(speckle_type(obj), "InstanceProxy");
}

/// A Revit element: a DataObject with a category.
bool is_element(const json &obj) {
  return obj.is_object() && obj.contains("category") &&
         obj["category"].is_string() &&
         speckle_type(obj).find("DataObject") != std::string::npos;
}

// ----------------------------------------------------------- parameters ---

/// The Revit parameter with internal name @p idn anywhere under
/// properties.Parameters ({value, name, units, internalDefinitionName}).
const json *find_param(const json &node, std::string_view idn) {
  if (!node.is_object())
    return nullptr;
  auto it = node.find("internalDefinitionName");
  if (it != node.end() && node.contains("value")) {
    return (it->is_string() && it->get<std::string>() == idn) ? &node : nullptr;
  }
  for (const auto &[k, v] : node.items())
    if (const json *p = find_param(v, idn))
      return p;
  return nullptr;
}

const json *param(const json &element, std::string_view idn) {
  auto p = element.find("properties");
  if (p == element.end() || !p->is_object())
    return nullptr;
  auto params = p->find("Parameters");
  if (params == p->end())
    return nullptr;
  return find_param(*params, idn);
}

std::string param_string(const json &element, std::string_view idn) {
  const json *p = param(element, idn);
  return p ? str_of(*p, "value") : std::string{};
}

/// A length parameter in metres (its own units, else @p fallback_scale).
std::optional<double> param_length(const json &element, std::string_view idn,
                                   double fallback_scale) {
  const json *p = param(element, idn);
  if (!p || !(*p)["value"].is_number())
    return std::nullopt;
  return (*p)["value"].get<double>() * units_of(*p, fallback_scale);
}

std::string property_string(const json &element, const char *key) {
  auto p = element.find("properties");
  if (p == element.end() || !p->is_object())
    return {};
  return str_of(*p, key);
}

// ------------------------------------------------------------- geometry ---

using bim::P2;
using bim::Soup;

struct Geometry {
  std::unordered_map<std::string, const json *> by_app_id;
  std::unordered_map<std::string, const json *> definitions;
  double root_scale = 1.0;
  std::size_t proxies = 0, unresolved = 0;

  void index(const json &node) {
    if (node.is_object()) {
      auto it = node.find("applicationId");
      if (it != node.end() && it->is_string())
        by_app_id.emplace(it->get<std::string>(), &node);
      for (const auto &[k, v] : node.items())
        if (v.is_structured())
          index(v);
    } else if (node.is_array()) {
      for (const auto &v : node)
        if (v.is_structured())
          index(v);
    }
  }

  /// Speckle mesh faces: [n, i0..in-1, ...]; n = 0 / 1 are the legacy
  /// triangle / quad markers. Polygons are fanned.
  void add_mesh(const json &mesh, const Eigen::Affine3d &to_world, double scale,
                Soup &out) const {
    const double s = units_of(mesh, scale);
    const auto vit = mesh.find("vertices");
    const auto fit = mesh.find("faces");
    if (vit == mesh.end() || fit == mesh.end() || !vit->is_array() ||
        !fit->is_array())
      return;
    const auto &verts = *vit;
    const std::size_t nv = verts.size() / 3;
    for (const auto &x : verts)
      if (!x.is_number()) {
        out.bad = true;
        return;
      }
    const auto base = static_cast<std::uint32_t>(out.v.size());
    out.v.reserve(out.v.size() + nv);
    for (std::size_t i = 0; i < nv; ++i)
      out.v.push_back(to_world *
                      (s * Eigen::Vector3d(verts[3 * i].get<double>(),
                                           verts[3 * i + 1].get<double>(),
                                           verts[3 * i + 2].get<double>())));
    const auto &faces = *fit;
    std::size_t i = 0;
    std::vector<std::uint32_t> poly;
    auto index_at = [&](std::size_t at) -> std::int64_t {
      const json &f = faces[at];
      return f.is_number_integer() ? f.get<std::int64_t>() : -1;
    };
    std::vector<std::array<std::uint32_t, 3>> tris;
    while (i < faces.size()) {
      std::int64_t n = index_at(i);
      if (n == 0)
        n = 3;
      else if (n == 1)
        n = 4;
      if (n < 3 || i + static_cast<std::size_t>(n) >= faces.size()) {
        out.bad = true; // malformed or truncated face list
        return;
      }
      poly.clear();
      for (std::int64_t k = 1; k <= n; ++k) {
        const std::int64_t idx = index_at(i + static_cast<std::size_t>(k));
        if (idx < 0 || static_cast<std::size_t>(idx) >= nv) {
          out.bad = true;
          return;
        }
        poly.push_back(base + static_cast<std::uint32_t>(idx));
      }
      for (std::size_t k = 1; k + 1 < poly.size(); ++k)
        tris.push_back({poly[0], poly[k], poly[k + 1]});
      i += static_cast<std::size_t>(n) + 1;
    }
    out.t.insert(out.t.end(), tris.begin(), tris.end());
  }

  /// Geometry of one displayValue item: a mesh, an instance proxy (its
  /// definition's objects, transformed), or an object with a displayValue.
  void gather(const json &item, const Eigen::Affine3d &to_world, double scale,
              int depth, Soup &out) {
    if (!item.is_object() || depth > kMaxInstanceDepth)
      return;
    if (is_mesh(item)) {
      add_mesh(item, to_world, scale, out);
      return;
    }
    if (is_instance_proxy(item)) {
      ++proxies;
      const std::string def_id = str_of(item, "definitionId");
      auto d = definitions.find(def_id);
      const auto tit = item.find("transform");
      if (d == definitions.end() || tit == item.end() || !tit->is_array() ||
          tit->size() != 16) {
        ++unresolved;
        return;
      }
      // Row-major 4x4 (translation at [3], [7], [11]); the translation is in
      // the proxy's units, the linear part unitless. Checked on a real Revit
      // export (rvt-oda-importer, 1305 proxies): every transform's bottom
      // row is (0, 0, 0, 1); read row-major, 163 of 343 furniture /
      // casework / plumbing items on the two levels that have placed rooms
      // land inside a room of their own level (the rest stand in corridors
      // and unplaced rooms) and the most common bottom height is exactly
      // the level floor; read column-major, none of them do.
      const double ps = units_of(item, scale);
      Eigen::Matrix4d m;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          m(r, c) = (*tit)[static_cast<std::size_t>(4 * r + c)].get<double>();
      Eigen::Affine3d local = Eigen::Affine3d::Identity();
      local.linear() = m.topLeftCorner<3, 3>();
      local.translation() = ps * m.topRightCorner<3, 1>();
      const Eigen::Affine3d next = to_world * local;
      const auto oit = d->second->find("objects");
      if (oit == d->second->end() || !oit->is_array())
        return;
      for (const auto &app : *oit) {
        if (!app.is_string())
          continue;
        auto o = by_app_id.find(app.get<std::string>());
        if (o == by_app_id.end()) {
          ++unresolved;
          continue;
        }
        gather(*o->second, next, units_of(*d->second, ps), depth + 1, out);
      }
      return;
    }
    const auto dv = item.find("displayValue");
    if (dv != item.end() && dv->is_array())
      for (const auto &x : *dv)
        gather(x, to_world, units_of(item, scale), depth + 1, out);
  }
};

// ---------------------------------------------------------------- misc ---

struct Level {
  std::string name;
  std::string app_id;
  std::string element_id;
  std::string ifc_guid;
  double elevation_m = 0.0; ///< as Revit reports it (its elevation base)
  bool building_storey = true;
};

struct Element {
  const json *obj = nullptr;
  std::string category;
  std::string app_id;
  std::string level;
  std::string parent_app; ///< applicationId of the element it is nested in
  Soup geom;
};

json revit_identity(const Element &e) {
  const json &o = *e.obj;
  json r = {{"applicationId", e.app_id},
            {"category", e.category},
            {"family", str_of(o, "family")},
            {"type", str_of(o, "type")},
            {"level", e.level},
            {"speckle_id", str_of(o, "id")}};
  if (auto id = property_string(o, "elementId"); !id.empty())
    r["elementId"] = id;
  if (auto g = param_string(o, "IFC_GUID"); !g.empty())
    r["IfcGUID"] = g;
  return r;
}

} // namespace

std::string category_slug(const std::string &category) {
  return mesh_slug(category);
}

RevitImport import_revit(const json &root, const RevitImportOptions &opt) {
  if (!root.is_object() || !root.contains("levelProxies") ||
      !root["levelProxies"].is_array() || root["levelProxies"].empty())
    throw std::runtime_error(
        "import_revit: the root has no levelProxies — not a Revit model from "
        "a Speckle v3 Revit connector");

  RevitImport out;
  auto &st = out.stats;
  const std::string ns =
      opt.guid_namespace.empty() ? kDefaultGuidNamespace : opt.guid_namespace;
  bim::GuidAllocator guid_for(ns, "imported/revit");

  Geometry geo;
  geo.root_scale = units_of(root, 0.0);
  if (geo.root_scale == 0.0) {
    core::warn("import_revit: root units '{}' unknown; assuming metres",
               str_of(root, "units"));
    geo.root_scale = 1.0;
  }
  geo.index(root);
  if (root.contains("instanceDefinitionProxies"))
    for (const auto &d : root["instanceDefinitionProxies"]) {
      std::string id = str_of(d, "applicationId");
      if (id.empty())
        id = str_of(d, "name");
      if (!id.empty())
        geo.definitions.emplace(id, &d);
    }

  // ---- levels
  std::vector<Level> levels;
  for (const auto &lp : root["levelProxies"]) {
    const json &v = lp.contains("value") ? lp["value"] : lp;
    Level l;
    l.name = str_of(v, "name");
    l.app_id = str_of(lp, "applicationId");
    l.element_id = property_string(v, "elementId");
    l.ifc_guid = param_string(v, "IFC_GUID");
    const std::string bs = param_string(v, "LEVEL_IS_BUILDING_STORY");
    l.building_storey = bs != "No";
    if (v.contains("elevation") && v["elevation"].is_number()) {
      l.elevation_m = v["elevation"].get<double>() *
                      units_of(v, units_of(lp, geo.root_scale));
    } else if (auto e = param_length(v, "LEVEL_ELEV", geo.root_scale)) {
      l.elevation_m = *e;
    } else {
      core::warn("import_revit: level '{}' has no elevation; skipped", l.name);
      continue;
    }
    levels.push_back(std::move(l));
  }
  if (levels.empty())
    throw std::runtime_error("import_revit: no level has an elevation");
  st.levels = levels.size();
  std::map<std::string, double> elevation_by_name;
  for (const auto &l : levels)
    elevation_by_name.emplace(l.name, l.elevation_m);

  // ---- elements and their geometry
  std::vector<Element> walls, openings, rooms;
  bim::MeshMerger merger("imported/revit/"); // per-category meshes
  std::function<void(const json &, const std::string &)> walk =
      [&](const json &node, const std::string &parent) {
        if (node.is_array()) {
          for (const auto &x : node)
            walk(x, parent);
          return;
        }
        if (!node.is_object())
          return;
        std::string self = parent;
        if (is_element(node)) {
          ++st.elements;
          Element e;
          e.obj = &node;
          e.category = node["category"].get<std::string>();
          e.app_id = str_of(node, "applicationId");
          e.level = str_of(node, "level");
          e.parent_app = parent;
          self = e.app_id;
          ++st.categories[e.category];
          if (auto dv = node.find("displayValue");
              dv != node.end() && dv->is_array())
            for (const auto &x : *dv)
              geo.gather(x, Eigen::Affine3d::Identity(),
                         units_of(node, geo.root_scale), 0, e.geom);
          if (e.geom.bad) {
            ++st.elements_bad_geometry;
            e.geom = Soup{};
          }
          if (e.geom.empty())
            ++st.elements_no_geometry;
          else if (opt.meshes && e.category != "Rooms")
            merger.add(e.category, e.geom);
          if (e.category == "Walls")
            walls.push_back(std::move(e));
          else if (e.category == "Doors" || e.category == "Windows")
            openings.push_back(std::move(e));
          else if (e.category == "Rooms")
            rooms.push_back(std::move(e));
        }
        if (auto it = node.find("elements"); it != node.end())
          walk(*it, self);
      };
  // Walk the root itself: a copy would leave Element::obj dangling.
  if (auto it = root.find("elements"); it != root.end())
    walk(*it, std::string{});
  if (st.elements_bad_geometry > 0)
    core::warn("import_revit: {} elements have a mesh with an invalid face "
               "list (negative / out-of-range index); skipped",
               st.elements_bad_geometry);

  // ---- precision: recentre a model far from the origin (STANDARDS §4).
  // Everything below (records and meshes) is computed in double; the stored
  // meshes are float, which keeps ~1 mm only within ~1e4 m of the origin.
  {
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(
                        std::numeric_limits<double>::infinity()),
                    hi = -lo;
    for (const auto *list : {&walls, &openings, &rooms})
      for (const auto &e : *list) {
        if (e.geom.empty())
          continue;
        const auto [elo, ehi] = e.geom.bounds();
        lo = lo.cwiseMin(elo);
        hi = hi.cwiseMax(ehi);
      }
    merger.extend(lo, hi);
    const Eigen::Vector3d off = bim::far_offset(
        lo, hi, opt.recenter, opt.recenter_threshold_m, "import_revit");
    if (!off.isZero()) {
      for (auto *list : {&walls, &openings, &rooms})
        for (auto &e : *list)
          for (auto &p : e.geom.v)
            p -= off;
      merger.shift(off);
      st.world_offset_m = {off.x(), off.y(), off.z()};
    }
  }
  st.instance_proxies = geo.proxies;
  st.instance_unresolved = geo.unresolved;
  if (geo.unresolved > 0)
    core::warn("import_revit: {} instance-proxy references could not be "
               "resolved (definition or object missing); their geometry is "
               "left out",
               geo.unresolved);

  // ---- Revit internal z vs level elevation, measured on the walls:
  // a wall's bottom sits at (its base level's internal z + its base offset).
  if (opt.elevation_offset_m) {
    st.elevation_offset_m = *opt.elevation_offset_m;
    st.elevation_offset_source = "option";
  } else {
    std::map<long long, std::size_t> votes;
    std::size_t total = 0;
    for (const auto &w : walls) {
      if (w.geom.empty())
        continue;
      const std::string base = param_string(*w.obj, "WALL_BASE_CONSTRAINT");
      auto lv = elevation_by_name.find(base);
      const auto off = param_length(*w.obj, "WALL_BASE_OFFSET",
                                    units_of(*w.obj, geo.root_scale));
      if (lv == elevation_by_name.end() || !off)
        continue;
      const double d = w.geom.z_range().first - *off - lv->second;
      ++votes[std::llround(d * 1000.0)];
      ++total;
    }
    if (votes.empty()) {
      st.elevation_offset_m = 0.0;
      st.elevation_offset_source = "assumed 0";
      core::warn("import_revit: no wall gives its base level and offset; "
                 "assuming the level elevations are Revit internal z");
    } else {
      const auto best = std::max_element(
          votes.begin(), votes.end(),
          [](const auto &a, const auto &b) { return a.second < b.second; });
      st.elevation_offset_m = static_cast<double>(best->first) / 1000.0;
      st.elevation_offset_source =
          fmt::format("walls ({} of {} agree)", best->second, total);
    }
  }
  core::info("import_revit: Revit internal z = level elevation {:+.3f} m "
             "({})",
             st.elevation_offset_m, st.elevation_offset_source);

  // ---- storeys: the building-storey levels (LEVEL_IS_BUILDING_STORY),
  // one shared level frame with the lowest storey's floor at z = 0.
  // Reference levels (not a building storey) become no storey; their
  // elements go to the storey at or below them.
  std::sort(levels.begin(), levels.end(), [](const Level &a, const Level &b) {
    return a.elevation_m < b.elevation_m ||
           (a.elevation_m == b.elevation_m && a.name < b.name);
  });
  std::vector<Level> storey_levels, reference_levels;
  for (const auto &l : levels)
    (l.building_storey ? storey_levels : reference_levels).push_back(l);
  if (storey_levels.empty()) {
    core::warn("import_revit: no level is flagged as a building storey; "
               "using all {} levels",
               levels.size());
    storey_levels = levels;
    reference_levels.clear();
  }
  st.reference_levels = reference_levels.size();
  for (const auto &l : reference_levels)
    st.reference_level_names.push_back(l.name);
  if (!reference_levels.empty())
    core::info("import_revit: {} reference levels (not building storeys) "
               "give no storey; their elements go to the storey at or below",
               reference_levels.size());
  const double z0 = storey_levels.front().elevation_m + st.elevation_offset_m;
  // The storey a level at elevation e belongs to: the highest storey at or
  // below it (1 mm tolerance), else the lowest.
  auto storey_at = [&](double elevation_m) {
    int idx = 0;
    for (std::size_t i = 0; i < storey_levels.size(); ++i)
      if (storey_levels[i].elevation_m <= elevation_m + 1e-3)
        idx = static_cast<int>(i);
    return idx;
  };
  std::map<std::string, int> storey_of_level;
  std::vector<json> refs_of_storey(storey_levels.size(), json::array());
  for (const auto &l : reference_levels) {
    const int idx = storey_at(l.elevation_m);
    storey_of_level.emplace(l.name, idx);
    refs_of_storey[static_cast<std::size_t>(idx)].push_back(
        {{"name", l.name},
         {"applicationId", l.app_id},
         {"elementId", l.element_id},
         {"elevation_m", l.elevation_m}});
  }
  for (std::size_t i = 0; i < storey_levels.size(); ++i) {
    const auto &l = storey_levels[i];
    core::StoreyRecord s;
    s.idx = static_cast<int>(i);
    s.name = l.name;
    s.z_floor = l.elevation_m + st.elevation_offset_m - z0;
    s.z_ceiling =
        i + 1 < storey_levels.size()
            ? storey_levels[i + 1].elevation_m + st.elevation_offset_m - z0
            : s.z_floor + opt.default_storey_height_m;
    s.level_transform = bim::level_frame(z0);
    s.guid =
        guid_for("storeys", l.app_id.empty() ? "name:" + l.name : l.app_id, i);
    json md = {{"revit",
                {{"applicationId", l.app_id},
                 {"elementId", l.element_id},
                 {"name", l.name},
                 {"elevation_m", l.elevation_m},
                 {"building_storey", l.building_storey}}},
               {"elevation_offset_m", st.elevation_offset_m},
               {"world_offset_m", st.world_offset_m}};
    if (!l.ifc_guid.empty())
      md["revit"]["IfcGUID"] = l.ifc_guid;
    if (!refs_of_storey[i].empty())
      md["revit"]["reference_levels"] = refs_of_storey[i];
    s.metadata = md.dump();
    storey_of_level[l.name] = s.idx;
    out.model.storeys.push_back(std::move(s));
  }
  out.model.provided.storeys = true;
  const auto &storeys = out.model.storeys;
  auto storey_for = [&](const Element &e, double z_level) {
    if (auto it = storey_of_level.find(e.level); it != storey_of_level.end())
      return it->second;
    ++st.elements_no_storey;
    int idx = 0;
    for (const auto &s : storeys)
      if (s.z_floor <= z_level + 0.5)
        idx = s.idx;
    return idx;
  };

  // ---- walls
  std::vector<bim::HostWall> wall_geom;
  std::unordered_map<std::string, std::size_t> wall_by_app;
  for (std::size_t wi = 0; wi < walls.size(); ++wi) {
    const auto &e = walls[wi];
    if (e.geom.empty()) {
      ++st.walls_no_geometry;
      continue;
    }
    const bim::WallFit fit = bim::fit_wall(e.geom);
    const auto width = param_length(*e.obj, "WALL_ATTR_WIDTH_PARAM",
                                    units_of(*e.obj, geo.root_scale));
    const double measured = fit.t_hi - fit.t_lo;
    const double thickness = width ? *width : measured;
    if (fit.ok && thickness < opt.min_wall_thickness_m) {
      ++st.walls_thin;
      continue;
    }
    if (!fit.ok || fit.fill < opt.min_wall_fill ||
        (width && measured > 1.5 * *width + 0.05)) {
      ++st.walls_not_straight;
      continue;
    }
    core::WallRecord w;
    w.guid = guid_for("walls", e.app_id, wi);
    wall_geom.push_back(bim::wall_axis(fit, e.geom, thickness, z0, w));
    w.storey = storey_for(e, w.z_base);
    const std::string function = param_string(*e.obj, "FUNCTION_PARAM");
    w.is_external = function == "Exterior";
    json md = {{"revit", revit_identity(e)},
               {"measured_thickness_m", measured},
               {"footprint_fill", fit.fill}};
    if (!function.empty())
      md["revit"]["function"] = function;
    w.metadata = md.dump();
    wall_by_app.emplace(e.app_id, out.model.walls.size());
    out.model.walls.push_back(std::move(w));
  }
  out.model.provided.walls = true;
  st.walls = out.model.walls.size();
  if (st.walls_thin > 0)
    core::warn("import_revit: {} walls thinner than {:.3f} m (finish layers, "
               "linings) are not wall records; their geometry stays in the "
               "walls mesh",
               st.walls_thin, opt.min_wall_thickness_m);
  if (st.walls_not_straight > 0)
    core::warn("import_revit: {} walls have no straight rectangular "
               "footprint (curved, profiled or irregular; fill < {:.2f}); "
               "they are mesh-only",
               st.walls_not_straight, opt.min_wall_fill);
  if (st.walls_no_geometry > 0)
    core::warn("import_revit: {} walls have no geometry; skipped",
               st.walls_no_geometry);

  // ---- openings (doors, windows)
  for (std::size_t oi = 0; oi < openings.size(); ++oi) {
    const auto &e = openings[oi];
    if (e.geom.empty()) {
      ++st.openings_unhosted;
      continue;
    }
    const auto [lo, hi] = e.geom.bounds();
    const P2 c(0.5 * (lo.x() + hi.x()), 0.5 * (lo.y() + hi.y()));
    std::optional<std::size_t> host;
    bool by_parent = false;
    std::string pid = property_string(*e.obj, "parentApplicationId");
    if (pid.empty())
      pid = e.parent_app; // hosted element nested under its host
    if (!pid.empty())
      if (auto it = wall_by_app.find(pid); it != wall_by_app.end()) {
        host = it->second;
        by_parent = true;
      }
    if (!host)
      host = bim::find_host(wall_geom, c, lo.z(), hi.z(),
                            opt.opening_host_tolerance_m);
    if (!host) {
      ++st.openings_unhosted;
      continue;
    }
    const auto &g = wall_geom[*host];
    auto [s_lo, s_hi] = bim::span_on(g, e.geom);
    // Extents: the Revit rough opening (the hole in the wall) when the
    // family gives one, else its nominal width / height, else the
    // geometry (which includes casings and linings). The width is centred on
    // the geometry's extent along the host; the height starts at the sill
    // height above the element's level. A parameter extent that does not
    // overlap the geometry at all (a wrong level, say) is not used.
    const double ps = units_of(*e.obj, geo.root_scale);
    std::string s_source = "geometry", z_source = "geometry";
    auto first_length = [&](std::initializer_list<const char *> idns,
                            std::string &which) -> std::optional<double> {
      for (const char *idn : idns)
        if (auto v = param_length(*e.obj, idn, ps); v && *v > 0) {
          which = idn;
          return v;
        }
      return std::nullopt;
    };
    std::string wsrc, hsrc;
    const auto width =
        first_length({"FAMILY_ROUGH_WIDTH_PARAM", "GENERIC_WIDTH",
                      "FAMILY_WIDTH_PARAM", "DOOR_WIDTH", "WINDOW_WIDTH"},
                     wsrc);
    const auto height =
        first_length({"FAMILY_ROUGH_HEIGHT_PARAM", "GENERIC_HEIGHT",
                      "FAMILY_HEIGHT_PARAM", "DOOR_HEIGHT", "WINDOW_HEIGHT"},
                     hsrc);
    if (width) {
      const double sc = 0.5 * (s_lo + s_hi);
      s_lo = sc - 0.5 * *width;
      s_hi = sc + 0.5 * *width;
      s_source = wsrc;
    }
    double z_lo = lo.z(), z_hi = hi.z();
    if (height) {
      auto lv = elevation_by_name.find(e.level);
      const auto sill = param_length(*e.obj, "INSTANCE_SILL_HEIGHT_PARAM", ps);
      if (lv != elevation_by_name.end() && sill) {
        const double zs = lv->second + st.elevation_offset_m + *sill;
        const double zh = zs + *height;
        if (zh > lo.z() && zs < hi.z()) {
          z_lo = zs;
          z_hi = zh;
          z_source = hsrc;
        }
      }
    }
    s_lo = std::clamp(s_lo, 0.0, g.length);
    s_hi = std::clamp(s_hi, 0.0, g.length);
    (s_source == "geometry" && z_source == "geometry"
         ? st.openings_extent_geometry
         : st.openings_extent_params)++;
    if (s_hi - s_lo < kMinOpeningWidthM || z_hi - z_lo < 0.05) {
      ++st.openings_unhosted;
      continue;
    }
    core::OpeningRecord o;
    o.guid = guid_for("openings", e.app_id, oi);
    o.wall = out.model.walls[*host].guid;
    o.kind = e.category == "Doors" ? "door" : "window";
    o.s_start = s_lo;
    o.s_end = s_hi;
    o.z_sill = z_lo - z0;
    o.z_head = z_hi - z0;
    o.is_external = out.model.walls[*host].is_external;
    json md = {{"revit", revit_identity(e)},
               {"host", by_parent ? "parentApplicationId" : "geometry"},
               {"extent_source", {{"s", s_source}, {"z", z_source}}}};
    if (auto m = param_string(*e.obj, "ALL_MODEL_MARK"); !m.empty())
      md["revit"]["mark"] = m;
    o.metadata = md.dump();
    (by_parent ? st.openings_by_parent : st.openings_by_geometry)++;
    out.model.openings.push_back(std::move(o));
  }
  out.model.provided.openings = true;
  st.openings = out.model.openings.size();
  if (st.openings_unhosted > 0)
    core::warn("import_revit: {} of {} doors / windows have no host wall "
               "among the imported walls (or no geometry); they are "
               "mesh-only",
               st.openings_unhosted, openings.size());

  // ---- spaces (placed rooms)
  for (std::size_t ri = 0; ri < rooms.size(); ++ri) {
    const auto &e = rooms[ri];
    const std::string name = str_of(*e.obj, "name");
    if (e.geom.empty()) {
      ++st.rooms_unplaced;
      st.unplaced_room_names.push_back(name);
      continue;
    }
    const auto [zlo, zhi] = e.geom.z_range();
    bim::OutlineDrops drops;
    auto poly = bim::room_outline(e.geom, zlo, drops);
    st.room_pieces_dropped += drops.pieces;
    st.room_islands_dropped += drops.islands;
    if (!poly) {
      ++st.rooms_bad_outline;
      continue;
    }
    core::SpaceRecord s;
    s.guid = guid_for("spaces", e.app_id, ri);
    s.storey = storey_for(e, zlo - z0);
    s.name = name;
    s.kind = "room";
    s.polygon = std::move(*poly);
    s.z_floor = zlo - z0;
    s.z_ceiling = zhi - z0;
    bim::fill_space_metrics(s, e.geom);
    json md = {{"revit", revit_identity(e)}};
    if (auto n = param_string(*e.obj, "ROOM_NUMBER"); !n.empty())
      md["revit"]["number"] = n;
    if (auto n = param_string(*e.obj, "ROOM_NAME"); !n.empty())
      md["revit"]["room_name"] = n;
    if (auto a = param(*e.obj, "ROOM_AREA"); a && (*a)["value"].is_number())
      md["revit"]["area_m2"] = (*a)["value"];
    s.metadata = md.dump();
    out.model.spaces.push_back(std::move(s));
  }
  out.model.provided.spaces = true;
  st.spaces = out.model.spaces.size();
  if (st.rooms_unplaced > 0) {
    std::string names;
    for (const auto &n : st.unplaced_room_names)
      names += (names.empty() ? "" : ", ") + n;
    core::warn("import_revit: {} rooms are unplaced or unbounded (no volume "
               "mesh); skipped: {}",
               st.rooms_unplaced, names);
  }
  if (st.rooms_bad_outline > 0)
    core::warn("import_revit: {} rooms have a volume mesh without a closed "
               "bottom outline; skipped",
               st.rooms_bad_outline);
  if (st.room_pieces_dropped + st.room_islands_dropped > 0)
    core::warn("import_revit: room outlines left out {} disjoint pieces "
               "(only the largest piece of a room is kept) and {} islands "
               "inside holes",
               st.room_pieces_dropped, st.room_islands_dropped);
  if (st.elements_no_storey > 0)
    core::warn("import_revit: {} elements name no known level; placed on the "
               "storey their bottom stands in",
               st.elements_no_storey);

  // ---- meshes
  out.meshes = merger.take();

  core::info("import_revit: {} elements -> {} storeys, {} walls, {} openings, "
             "{} spaces, {} meshes ({} instance proxies)",
             st.elements, storeys.size(), st.walls, st.openings, st.spaces,
             out.meshes.size(), st.instance_proxies);
  return out;
}

RevitStoreResult import_revit_into(ProjectDB &db,
                                   const std::function<json()> &resolved_root,
                                   const RevitImportOptions &options,
                                   const json &source, int caller_log_id) {
  RevitStoreResult res;
  // A caller-owned row (the job stage) is neither opened nor closed here.
  const bool own_row = caller_log_id < 0;
  const int log_id =
      own_row ? db.log_pipeline_start("import_speckle", source.dump()) : -1;
  try {
    const json root = resolved_root();
    RevitImportOptions mapping = options;
    mapping.guid_namespace = db.project_guid();
    RevitImport imported = import_revit(root, mapping);
    res.stats = imported.stats;
    mark_imported(imported.model);

    ProjectDB::Transaction tx(db);
    res.stored =
        store_solid_model(db, imported.model, own_row ? log_id : caller_log_id);

    // --no-meshes writes no mesh and deletes none.
    if (mapping.meshes) {
      json params = source;
      params["frame"] = "revit_internal";
      params["world_offset_m"] = res.stats.world_offset_m;
      const auto w =
          bim::write_meshes(db, imported.meshes, "import_speckle", params);
      res.meshes_saved = w.saved;
      res.meshes_removed = w.removed;
      res.mesh_names = w.saved_names;
    }
    tx.commit();
    if (res.meshes_removed > 0)
      core::info("import_speckle_revit: removed {} meshes of an earlier "
                 "import that this version no longer has",
                 res.meshes_removed);
    if (own_row)
      db.log_pipeline_end(log_id, true);
  } catch (const std::exception &e) {
    if (own_row)
      db.log_pipeline_end(log_id, false, e.what());
    throw;
  }
  return res;
}

SpeckleRevitImportResult
import_speckle_revit(ProjectDB &db, const SpeckleRevitImportOptions &opt,
                     int log_id) {
  if (opt.project.empty() || opt.model.empty())
    throw std::invalid_argument(
        "import_speckle_revit: a project and a model id are required");
  SpeckleReader reader(opt.server_url, opt.token);
  reader.set_cancel_token(opt.cancel_token);
  reader.set_timeout_seconds(opt.timeout_s);
  reader.set_connect_filter(opt.connect_filter);
  reader.set_curl_setup(opt.curl_setup);
  const auto throw_if_cancelled = [&](const char *where) {
    if (opt.cancel_token && opt.cancel_token->load(std::memory_order_acquire))
      throw std::runtime_error(
          fmt::format("import_speckle_revit: cancelled {}", where));
  };
  SpeckleRevitImportResult res;
  res.version = opt.version.empty()
                    ? reader.latest_version(opt.project, opt.model)
                    : reader.query_version(opt.project, opt.model, opt.version);
  core::info("import_speckle_revit: project {} model {} version {} ({}, {})",
             opt.project, opt.model, res.version.id,
             res.version.source_application, res.version.created_at);

  const json source = {{"source", "speckle"},
                       {"server", reader.server_url()},
                       {"project", opt.project},
                       {"model", opt.model},
                       {"version", res.version.id},
                       {"root_object", res.version.referenced_object},
                       {"source_application", res.version.source_application}};
  auto fetch = [&]() {
    throw_if_cancelled("before the download");
    const ObjectTree tree =
        reader.fetch_object_tree(opt.project, res.version.referenced_object);
    throw_if_cancelled("after the download");
    ResolveReport report;
    json root = tree.resolve_root(&report);
    core::info("import_speckle_revit: {} objects, {} references resolved",
               tree.size(), report.references_resolved);
    return root;
  };
  RevitStoreResult stored =
      import_revit_into(db, fetch, opt.mapping, source, log_id);
  res.stats = std::move(stored.stats);
  res.stored = stored.stored;
  res.meshes_saved = stored.meshes_saved;
  res.meshes_removed = stored.meshes_removed;
  res.mesh_names = std::move(stored.mesh_names);
  return res;
}

} // namespace reusex::io::speckle
