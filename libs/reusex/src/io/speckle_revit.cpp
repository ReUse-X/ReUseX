// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/speckle_revit.hpp"

#include "core/ProjectDB.hpp"
#include "core/guid.hpp"
#include "core/logging.hpp"
#include "core/space_polygon.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fmt/format.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
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
/// Tolerance for "on the bottom of a room volume" [m].
constexpr double kRoomBottomTolM = 0.02;
/// Plan-point weld grid for room outlines [m].
constexpr double kWeldM = 1e-4;
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

/// A triangle soup in world metres.
struct Soup {
  std::vector<Eigen::Vector3d> v;
  std::vector<std::array<std::uint32_t, 3>> t;

  [[nodiscard]] bool empty() const { return t.empty(); }

  void append(const Soup &o) {
    const auto base = static_cast<std::uint32_t>(v.size());
    v.insert(v.end(), o.v.begin(), o.v.end());
    for (const auto &tri : o.t)
      t.push_back({tri[0] + base, tri[1] + base, tri[2] + base});
  }

  [[nodiscard]] std::pair<double, double> z_range() const {
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const auto &p : v) {
      lo = std::min(lo, p.z());
      hi = std::max(hi, p.z());
    }
    return {lo, hi};
  }
};

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
    while (i < faces.size()) {
      std::size_t n = faces[i].get<std::size_t>();
      if (n == 0)
        n = 3;
      else if (n == 1)
        n = 4;
      if (i + n >= faces.size())
        break; // truncated face list
      poly.clear();
      bool ok = true;
      for (std::size_t k = 1; k <= n; ++k) {
        const auto idx = faces[i + k].get<std::size_t>();
        if (idx >= nv) {
          ok = false;
          break;
        }
        poly.push_back(base + static_cast<std::uint32_t>(idx));
      }
      if (ok)
        for (std::size_t k = 1; k + 1 < poly.size(); ++k)
          out.t.push_back({poly[0], poly[k], poly[k + 1]});
      i += n + 1;
    }
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
      // Row-major 4x4; its translation is in the proxy's units, its linear
      // part is unitless.
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

// ----------------------------------------------------------- 2D helpers ---

using P2 = Eigen::Vector2d;

double cross2(const P2 &o, const P2 &a, const P2 &b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

/// Convex hull (Andrew's monotone chain), counter-clockwise, no repeats.
std::vector<P2> convex_hull(std::vector<P2> pts) {
  std::sort(pts.begin(), pts.end(), [](const P2 &a, const P2 &b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  pts.erase(std::unique(pts.begin(), pts.end(),
                        [](const P2 &a, const P2 &b) {
                          return (a - b).squaredNorm() < 1e-14;
                        }),
            pts.end());
  if (pts.size() < 3)
    return pts;
  std::vector<P2> h(2 * pts.size());
  std::size_t k = 0;
  for (const auto &p : pts) {
    while (k >= 2 && cross2(h[k - 2], h[k - 1], p) <= 0)
      --k;
    h[k++] = p;
  }
  for (std::size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && cross2(h[k - 2], h[k - 1], pts[i]) <= 0)
      --k;
    h[k++] = pts[i];
  }
  h.resize(k - 1);
  return h;
}

double ring_area(const std::vector<P2> &r) { // signed, CCW > 0
  double a = 0.0;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const auto &p = r[i];
    const auto &q = r[(i + 1) % r.size()];
    a += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5 * a;
}

double ring_length(const std::vector<P2> &r) {
  double l = 0.0;
  for (std::size_t i = 0; i < r.size(); ++i)
    l += (r[(i + 1) % r.size()] - r[i]).norm();
  return l;
}

/// Drop vertices on a straight line between their neighbours.
std::vector<P2> simplify_ring(std::vector<P2> r) {
  bool changed = true;
  while (changed && r.size() > 3) {
    changed = false;
    for (std::size_t i = 0; i < r.size() && r.size() > 3; ++i) {
      const P2 &a = r[(i + r.size() - 1) % r.size()];
      const P2 &b = r[i];
      const P2 &c = r[(i + 1) % r.size()];
      const double len = (c - a).norm();
      const double dist =
          len > 0 ? std::abs(cross2(a, b, c)) / len : (b - a).norm();
      if (dist < 1e-4 || (b - a).norm() < 1e-6) {
        r.erase(r.begin() + static_cast<std::ptrdiff_t>(i));
        changed = true;
        --i;
      }
    }
  }
  return r;
}

/// The straight-wall reading of a footprint: the minimum-area rectangle of
/// its convex hull.
struct WallFit {
  bool ok = false;
  P2 u{1, 0}, v{0, 1};
  double s_lo = 0, s_hi = 0, t_lo = 0, t_hi = 0;
  double fill = 0.0; ///< hull area / rectangle area
};

WallFit fit_wall(const Soup &soup) {
  std::vector<P2> pts;
  pts.reserve(soup.v.size());
  for (const auto &p : soup.v)
    pts.emplace_back(p.x(), p.y());
  const auto hull = convex_hull(std::move(pts));
  WallFit best;
  if (hull.size() < 3)
    return best;
  double best_area = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < hull.size(); ++i) {
    P2 e = hull[(i + 1) % hull.size()] - hull[i];
    if (e.norm() < 1e-9)
      continue;
    e.normalize();
    const P2 n(-e.y(), e.x());
    double s0 = 1e300, s1 = -1e300, t0 = 1e300, t1 = -1e300;
    for (const auto &p : hull) {
      s0 = std::min(s0, p.dot(e));
      s1 = std::max(s1, p.dot(e));
      t0 = std::min(t0, p.dot(n));
      t1 = std::max(t1, p.dot(n));
    }
    const double area = (s1 - s0) * (t1 - t0);
    if (area < best_area - 1e-12) {
      best_area = area;
      best.ok = true;
      best.u = e;
      best.v = n;
      best.s_lo = s0;
      best.s_hi = s1;
      best.t_lo = t0;
      best.t_hi = t1;
    }
  }
  if (!best.ok)
    return best;
  // The long side is the axis.
  if (best.t_hi - best.t_lo > best.s_hi - best.s_lo) {
    const P2 u = best.v, v = -best.u;
    const double s_lo = best.t_lo, s_hi = best.t_hi;
    const double t_lo = -best.s_hi, t_hi = -best.s_lo;
    best.u = u;
    best.v = v;
    best.s_lo = s_lo;
    best.s_hi = s_hi;
    best.t_lo = t_lo;
    best.t_hi = t_hi;
  }
  // Canonical direction: +x, else +y.
  if (best.u.x() < -1e-9 || (std::abs(best.u.x()) <= 1e-9 && best.u.y() < 0)) {
    best.u = -best.u;
    best.v = -best.v;
    std::swap(best.s_lo, best.s_hi);
    best.s_lo = -best.s_lo;
    best.s_hi = -best.s_hi;
    std::swap(best.t_lo, best.t_hi);
    best.t_lo = -best.t_lo;
    best.t_hi = -best.t_hi;
  }
  best.fill = best_area > 0 ? std::abs(ring_area(hull)) / best_area : 0.0;
  return best;
}

/// Footprint of a room volume: the outline of its bottom faces.
std::optional<core::SpacePolygon> room_outline(const Soup &soup, double zmin) {
  // Welded plan vertices of the bottom faces.
  std::map<std::pair<long long, long long>, std::uint32_t> weld;
  std::vector<P2> pts;
  auto key_of = [&](const Eigen::Vector3d &p) -> std::uint32_t {
    const std::pair<long long, long long> k{std::llround(p.x() / kWeldM),
                                            std::llround(p.y() / kWeldM)};
    auto [it, fresh] = weld.emplace(k, static_cast<std::uint32_t>(pts.size()));
    if (fresh)
      pts.emplace_back(p.x(), p.y());
    return it->second;
  };
  std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
  for (const auto &t : soup.t) {
    const auto &a = soup.v[t[0]], &b = soup.v[t[1]], &c = soup.v[t[2]];
    if (a.z() > zmin + kRoomBottomTolM || b.z() > zmin + kRoomBottomTolM ||
        c.z() > zmin + kRoomBottomTolM)
      continue;
    const Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() < 1e-12 || std::abs(n.normalized().z()) < 0.9)
      continue;
    const std::uint32_t ia = key_of(a), ib = key_of(b), ic = key_of(c);
    if (ia == ib || ib == ic || ia == ic)
      continue;
    for (auto [p, q] :
         {std::pair{ia, ib}, std::pair{ib, ic}, std::pair{ic, ia}})
      ++edges[{std::min(p, q), std::max(p, q)}];
  }
  // Boundary edges (used by one bottom face) form the rings.
  std::multimap<std::uint32_t, std::uint32_t> adj;
  for (const auto &[e, n] : edges)
    if (n == 1) {
      adj.emplace(e.first, e.second);
      adj.emplace(e.second, e.first);
    }
  std::vector<std::vector<P2>> rings;
  while (!adj.empty()) {
    const std::uint32_t start = adj.begin()->first;
    std::vector<std::uint32_t> ring{start};
    std::uint32_t cur = start;
    bool closed = false;
    for (std::size_t guard = 0; guard <= edges.size(); ++guard) {
      auto it = adj.find(cur);
      if (it == adj.end())
        break;
      const std::uint32_t next = it->second;
      adj.erase(it);
      // Remove the reverse half-edge.
      for (auto r = adj.lower_bound(next); r != adj.end() && r->first == next;
           ++r)
        if (r->second == cur) {
          adj.erase(r);
          break;
        }
      if (next == start) {
        closed = true;
        break;
      }
      ring.push_back(next);
      cur = next;
    }
    if (!closed || ring.size() < 3)
      continue;
    std::vector<P2> r;
    r.reserve(ring.size());
    for (auto i : ring)
      r.push_back(pts[i]);
    r = simplify_ring(std::move(r));
    if (r.size() >= 3 && std::abs(ring_area(r)) > 1e-6)
      rings.push_back(std::move(r));
  }
  if (rings.empty())
    return std::nullopt;
  std::sort(rings.begin(), rings.end(), [](const auto &a, const auto &b) {
    return std::abs(ring_area(a)) > std::abs(ring_area(b));
  });
  auto inside = [](const std::vector<P2> &poly, const P2 &p) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
      const P2 &a = poly[i], &b = poly[j];
      if ((a.y() > p.y()) != (b.y() > p.y()) &&
          p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
        in = !in;
    }
    return in;
  };
  core::SpacePolygon poly;
  auto outer = rings.front();
  if (ring_area(outer) < 0)
    std::reverse(outer.begin(), outer.end());
  for (const auto &p : outer)
    poly.outer.push_back({p.x(), p.y()});
  for (std::size_t i = 1; i < rings.size(); ++i) {
    auto h = rings[i];
    if (!inside(outer, h.front()))
      continue; // a second, disjoint piece: dropped
    if (ring_area(h) > 0)
      std::reverse(h.begin(), h.end());
    std::vector<core::PlanPoint> hole;
    for (const auto &p : h)
      hole.push_back({p.x(), p.y()});
    poly.holes.push_back(std::move(hole));
  }
  return poly;
}

std::vector<P2> to_p2(const std::vector<core::PlanPoint> &r) {
  std::vector<P2> out;
  for (const auto &p : r)
    out.emplace_back(p[0], p[1]);
  return out;
}

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
  std::string out;
  bool sep = false;
  for (unsigned char c : category) {
    if (std::isalnum(c) && c < 128) {
      if (sep && !out.empty())
        out += '_';
      out += static_cast<char>(std::tolower(c));
      sep = false;
    } else {
      sep = true;
    }
  }
  return out.empty() ? std::string("uncategorized") : out;
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
  std::set<std::string> used_guids;
  auto guid_for = [&](const std::string &kind, const std::string &key,
                      std::size_t index) {
    std::string name = key.empty()
                           ? fmt::format("imported/revit/{}/#{}", kind, index)
                           : fmt::format("imported/revit/{}/{}", kind, key);
    std::string g = core::deterministic_guid(ns, name);
    for (int k = 2; !used_guids.insert(g).second; ++k)
      g = core::deterministic_guid(ns, fmt::format("{}#{}", name, k));
    return g;
  };

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
  std::map<std::string, std::pair<std::string, Soup>> by_slug; // meshes
  std::map<std::string, std::size_t> mesh_elements;
  std::function<void(const json &)> walk = [&](const json &node) {
    if (node.is_array()) {
      for (const auto &x : node)
        walk(x);
      return;
    }
    if (!node.is_object())
      return;
    if (is_element(node)) {
      ++st.elements;
      Element e;
      e.obj = &node;
      e.category = node["category"].get<std::string>();
      e.app_id = str_of(node, "applicationId");
      e.level = str_of(node, "level");
      ++st.categories[e.category];
      if (auto dv = node.find("displayValue");
          dv != node.end() && dv->is_array())
        for (const auto &x : *dv)
          geo.gather(x, Eigen::Affine3d::Identity(),
                     units_of(node, geo.root_scale), 0, e.geom);
      if (e.geom.empty())
        ++st.elements_no_geometry;
      else if (opt.meshes && e.category != "Rooms") {
        auto &[cat, soup] = by_slug[category_slug(e.category)];
        if (cat.empty())
          cat = e.category;
        soup.append(e.geom);
        ++mesh_elements[category_slug(e.category)];
      }
      if (e.category == "Walls")
        walls.push_back(std::move(e));
      else if (e.category == "Doors" || e.category == "Windows")
        openings.push_back(std::move(e));
      else if (e.category == "Rooms")
        rooms.push_back(std::move(e));
    }
    if (auto it = node.find("elements"); it != node.end())
      walk(*it);
  };
  // Walk the root itself: a copy would leave Element::obj dangling.
  if (auto it = root.find("elements"); it != root.end())
    walk(*it);
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

  // ---- storeys: one shared level frame, the lowest level's floor at z = 0
  std::sort(levels.begin(), levels.end(), [](const Level &a, const Level &b) {
    return a.elevation_m < b.elevation_m ||
           (a.elevation_m == b.elevation_m && a.name < b.name);
  });
  const double z0 = levels.front().elevation_m + st.elevation_offset_m;
  std::map<std::string, int> storey_of_level;
  for (std::size_t i = 0; i < levels.size(); ++i) {
    const auto &l = levels[i];
    core::StoreyRecord s;
    s.idx = static_cast<int>(i);
    s.name = l.name;
    s.z_floor = l.elevation_m + st.elevation_offset_m - z0;
    s.z_ceiling = i + 1 < levels.size()
                      ? levels[i + 1].elevation_m + st.elevation_offset_m - z0
                      : s.z_floor + opt.default_storey_height_m;
    s.level_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -z0, 0, 0, 0, 1};
    s.guid =
        guid_for("storeys", l.app_id.empty() ? "name:" + l.name : l.app_id, i);
    json md = {{"revit",
                {{"applicationId", l.app_id},
                 {"elementId", l.element_id},
                 {"name", l.name},
                 {"elevation_m", l.elevation_m},
                 {"building_storey", l.building_storey}}},
               {"elevation_offset_m", st.elevation_offset_m}};
    if (!l.ifc_guid.empty())
      md["revit"]["IfcGUID"] = l.ifc_guid;
    s.metadata = md.dump();
    storey_of_level.emplace(l.name, s.idx);
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
  struct WallGeom {
    P2 p0, u, v;
    double length = 0, half = 0;
  };
  std::vector<WallGeom> wall_geom;
  std::unordered_map<std::string, std::size_t> wall_by_app;
  for (std::size_t wi = 0; wi < walls.size(); ++wi) {
    const auto &e = walls[wi];
    if (e.geom.empty()) {
      ++st.walls_no_geometry;
      continue;
    }
    const WallFit fit = fit_wall(e.geom);
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
    const auto [zlo, zhi] = e.geom.z_range();
    const double tc = 0.5 * (fit.t_lo + fit.t_hi);
    const P2 a = fit.s_lo * fit.u + tc * fit.v;
    const P2 b = fit.s_hi * fit.u + tc * fit.v;
    core::WallRecord w;
    w.guid = guid_for("walls", e.app_id, wi);
    w.storey = storey_for(e, zlo - z0);
    w.x0 = a.x();
    w.y0 = a.y();
    w.x1 = b.x();
    w.y1 = b.y();
    w.z_base = zlo - z0;
    w.height = zhi - zlo;
    w.thickness = thickness;
    const std::string function = param_string(*e.obj, "FUNCTION_PARAM");
    w.is_external = function == "Exterior";
    json md = {{"revit", revit_identity(e)},
               {"measured_thickness_m", measured},
               {"footprint_fill", fit.fill}};
    if (!function.empty())
      md["revit"]["function"] = function;
    w.metadata = md.dump();
    wall_by_app.emplace(e.app_id, out.model.walls.size());
    wall_geom.push_back({a, fit.u, fit.v, (b - a).norm(), 0.5 * thickness});
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
    Eigen::Vector3d lo = e.geom.v.front(), hi = lo;
    for (const auto &p : e.geom.v) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
    const P2 c(0.5 * (lo.x() + hi.x()), 0.5 * (lo.y() + hi.y()));
    std::optional<std::size_t> host;
    bool by_parent = false;
    if (auto pid = property_string(*e.obj, "parentApplicationId"); !pid.empty())
      if (auto it = wall_by_app.find(pid); it != wall_by_app.end()) {
        host = it->second;
        by_parent = true;
      }
    if (!host) {
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t k = 0; k < wall_geom.size(); ++k) {
        const auto &g = wall_geom[k];
        const double s = (c - g.p0).dot(g.u);
        const double t = std::abs((c - g.p0).dot(g.v));
        const double zb = out.model.walls[k].z_base + z0;
        const double zt = zb + out.model.walls[k].height;
        if (t > g.half + opt.opening_host_tolerance_m ||
            s < -opt.opening_host_tolerance_m ||
            s > g.length + opt.opening_host_tolerance_m)
          continue;
        if (hi.z() < zb || lo.z() > zt)
          continue;
        if (t < best) {
          best = t;
          host = k;
        }
      }
    }
    if (!host) {
      ++st.openings_unhosted;
      continue;
    }
    const auto &g = wall_geom[*host];
    double s_lo = std::numeric_limits<double>::infinity(), s_hi = -s_lo;
    for (const auto &p : e.geom.v) {
      const double s = (P2(p.x(), p.y()) - g.p0).dot(g.u);
      s_lo = std::min(s_lo, s);
      s_hi = std::max(s_hi, s);
    }
    s_lo = std::clamp(s_lo, 0.0, g.length);
    s_hi = std::clamp(s_hi, 0.0, g.length);
    if (s_hi - s_lo < kMinOpeningWidthM || hi.z() - lo.z() < 0.05) {
      ++st.openings_unhosted;
      continue;
    }
    core::OpeningRecord o;
    o.guid = guid_for("openings", e.app_id, oi);
    o.wall = out.model.walls[*host].guid;
    o.kind = e.category == "Doors" ? "door" : "window";
    o.s_start = s_lo;
    o.s_end = s_hi;
    o.z_sill = lo.z() - z0;
    o.z_head = hi.z() - z0;
    o.is_external = out.model.walls[*host].is_external;
    json md = {{"revit", revit_identity(e)},
               {"host", by_parent ? "parentApplicationId" : "geometry"}};
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
    auto poly = room_outline(e.geom, zlo);
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
    s.area = core::space_polygon_area(s.polygon);
    double perim = ring_length(to_p2(s.polygon.outer));
    for (const auto &h : s.polygon.holes)
      perim += ring_length(to_p2(h));
    s.perimeter = perim;
    double vol = 0.0;
    for (const auto &t : e.geom.t)
      vol += e.geom.v[t[0]].dot(e.geom.v[t[1]].cross(e.geom.v[t[2]])) / 6.0;
    vol = std::abs(vol);
    const double prism = s.area * (zhi - zlo);
    s.volume = (vol > 1e-6 && vol < 1.5 * prism) ? vol : prism;
    s.compactness = perim > 0 ? 4.0 * M_PI * s.area / (perim * perim) : -1.0;
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
  if (st.elements_no_storey > 0)
    core::warn("import_revit: {} elements name no known level; placed on the "
               "storey their bottom stands in",
               st.elements_no_storey);

  // ---- meshes
  for (auto &[slug, cat_soup] : by_slug) {
    auto &[cat, soup] = cat_soup;
    RevitMesh m;
    m.name = "imported/" + slug;
    m.category = cat;
    m.elements = mesh_elements[slug];
    m.vertices.reserve(soup.v.size());
    for (const auto &p : soup.v)
      m.vertices.push_back({p.x(), p.y(), p.z()});
    m.triangles = std::move(soup.t);
    out.meshes.push_back(std::move(m));
  }

  core::info("import_revit: {} elements -> {} storeys, {} walls, {} openings, "
             "{} spaces, {} meshes ({} instance proxies)",
             st.elements, storeys.size(), st.walls, st.openings, st.spaces,
             out.meshes.size(), st.instance_proxies);
  return out;
}

SpeckleRevitImportResult
import_speckle_revit(ProjectDB &db, const SpeckleRevitImportOptions &opt) {
  if (opt.project.empty() || opt.model.empty())
    throw std::invalid_argument(
        "import_speckle_revit: a project and a model id are required");
  SpeckleReader reader(opt.server_url, opt.token);
  SpeckleRevitImportResult res;
  if (opt.version.empty()) {
    res.version = reader.latest_version(opt.project, opt.model);
  } else {
    const auto versions = reader.query_versions(opt.project, opt.model, 100);
    auto it = std::find_if(versions.begin(), versions.end(),
                           [&](const auto &v) { return v.id == opt.version; });
    if (it == versions.end())
      throw std::runtime_error(fmt::format(
          "import_speckle_revit: version '{}' is not among the newest 100 "
          "versions of model '{}'",
          opt.version, opt.model));
    res.version = *it;
  }
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
  const int log_id = db.log_pipeline_start("import_speckle", source.dump());
  try {
    const ObjectTree tree =
        reader.fetch_object_tree(opt.project, res.version.referenced_object);
    ResolveReport report;
    const json root = tree.resolve_root(&report);
    core::info("import_speckle_revit: {} objects, {} references resolved",
               tree.size(), report.references_resolved);

    RevitImportOptions mapping = opt.mapping;
    mapping.guid_namespace = db.project_guid();
    RevitImport imported = import_revit(root, mapping);
    res.stats = imported.stats;
    mark_imported(imported.model);

    ProjectDB::Transaction tx(db);
    res.stored = store_solid_model(db, imported.model);

    std::set<std::string> written;
    for (const auto &m : imported.meshes) {
      if (m.triangles.empty())
        continue;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.reserve(m.vertices.size());
      for (const auto &v : m.vertices)
        cloud.push_back(pcl::PointXYZ(static_cast<float>(v[0]),
                                      static_cast<float>(v[1]),
                                      static_cast<float>(v[2])));
      pcl::PolygonMesh mesh;
      pcl::toPCLPointCloud2(cloud, mesh.cloud);
      mesh.polygons.reserve(m.triangles.size());
      for (const auto &t : m.triangles) {
        pcl::Vertices f;
        f.vertices = {t[0], t[1], t[2]};
        mesh.polygons.push_back(std::move(f));
      }
      json params = source;
      params["category"] = m.category;
      params["elements"] = m.elements;
      params["units"] = "m";
      params["frame"] = "revit_internal";
      db.save_mesh(m.name, mesh, "import_speckle", params.dump());
      written.insert(m.name);
      ++res.meshes_saved;
    }
    for (const auto &name : db.list_meshes()) {
      if (name.rfind("imported/", 0) != 0 || written.count(name))
        continue;
      if (db.mesh_metadata(name).stage != "import_speckle")
        continue;
      db.delete_mesh(name);
      ++res.meshes_removed;
    }
    tx.commit();
    if (res.meshes_removed > 0)
      core::info("import_speckle_revit: removed {} meshes of an earlier "
                 "import that this version no longer has",
                 res.meshes_removed);
    db.log_pipeline_end(log_id, true);
  } catch (const std::exception &e) {
    db.log_pipeline_end(log_id, false, e.what());
    throw;
  }
  return res;
}

} // namespace reusex::io::speckle
