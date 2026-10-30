// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "bim_mapping.hpp"

#include "core/ProjectDB.hpp"
#include "core/guid.hpp"
#include "core/logging.hpp"
#include "core/space_polygon.hpp"

#include <fmt/format.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace reusex::io {

std::string mesh_slug(const std::string &category) {
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

} // namespace reusex::io

namespace reusex::io::bim {

namespace {
/// Tolerance for "on the bottom of a room volume" [m].
constexpr double kRoomBottomTolM = 0.02;
/// Plan-point weld grid for room outlines [m].
constexpr double kWeldM = 1e-4;

double cross2(const P2 &o, const P2 &a, const P2 &b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
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

bool inside(const std::vector<P2> &poly, const P2 &p) {
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const P2 &a = poly[i], &b = poly[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
        p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
      in = !in;
  }
  return in;
}
} // namespace

// ----------------------------------------------------------------- Soup ---

void Soup::append(const Soup &o) {
  const auto base = static_cast<std::uint32_t>(v.size());
  v.insert(v.end(), o.v.begin(), o.v.end());
  for (const auto &tri : o.t)
    t.push_back({tri[0] + base, tri[1] + base, tri[2] + base});
}

std::pair<double, double> Soup::z_range() const {
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (const auto &p : v) {
    lo = std::min(lo, p.z());
    hi = std::max(hi, p.z());
  }
  return {lo, hi};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Soup::bounds() const {
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::infinity()),
                  hi = -lo;
  for (const auto &p : v) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  return {lo, hi};
}

double Soup::volume() const {
  double vol = 0.0;
  for (const auto &tri : t)
    vol += v[tri[0]].dot(v[tri[1]].cross(v[tri[2]])) / 6.0;
  return std::abs(vol);
}

// ----------------------------------------------------------- 2D helpers ---

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

double ring_area(const std::vector<P2> &r) {
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

std::vector<P2> to_p2(const std::vector<core::PlanPoint> &r) {
  std::vector<P2> out;
  out.reserve(r.size());
  for (const auto &p : r)
    out.emplace_back(p[0], p[1]);
  return out;
}

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

std::optional<core::SpacePolygon> room_outline(const Soup &soup, double zmin,
                                               OutlineDrops &drops) {
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
  core::SpacePolygon poly;
  auto outer = rings.front();
  if (ring_area(outer) < 0)
    std::reverse(outer.begin(), outer.end());
  for (const auto &p : outer)
    poly.outer.push_back({p.x(), p.y()});
  std::vector<std::vector<P2>> holes;
  for (std::size_t i = 1; i < rings.size(); ++i) {
    auto h = rings[i];
    if (!inside(outer, h.front())) {
      ++drops.pieces; // a second, disjoint piece
      continue;
    }
    // Rings are sorted by area, so a ring inside an earlier hole is an
    // island (e.g. a room part enclosed by a column ring): not representable.
    if (std::any_of(holes.begin(), holes.end(),
                    [&](const auto &hh) { return inside(hh, h.front()); })) {
      ++drops.islands;
      continue;
    }
    holes.push_back(h);
    if (ring_area(h) > 0)
      std::reverse(h.begin(), h.end());
    std::vector<core::PlanPoint> hole;
    for (const auto &p : h)
      hole.push_back({p.x(), p.y()});
    poly.holes.push_back(std::move(hole));
  }
  return poly;
}

void fill_space_metrics(core::SpaceRecord &s, const Soup &soup) {
  s.area = core::space_polygon_area(s.polygon);
  double perim = ring_length(to_p2(s.polygon.outer));
  for (const auto &h : s.polygon.holes)
    perim += ring_length(to_p2(h));
  s.perimeter = perim;
  const double vol = soup.volume();
  const double prism = s.area * (s.z_ceiling - s.z_floor);
  s.volume = (vol > 1e-6 && vol < 1.5 * prism) ? vol : prism;
  s.compactness = perim > 0 ? 4.0 * M_PI * s.area / (perim * perim) : -1.0;
}

Eigen::Vector3d far_offset(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi,
                           bool recenter, double threshold_m,
                           const std::string &who) {
  Eigen::Vector3d off = Eigen::Vector3d::Zero();
  if (!(lo.x() <= hi.x()))
    return off; // nothing to measure
  const Eigen::Vector3d c = 0.5 * (lo + hi);
  if (recenter && std::hypot(c.x(), c.y()) > threshold_m) {
    off = Eigen::Vector3d(std::round(c.x() / 1000.0) * 1000.0,
                          std::round(c.y() / 1000.0) * 1000.0, 0.0);
    core::warn("{}: the model lies {:.0f} m from the origin; recentred by "
               "({:.0f}, {:.0f}) m (world = source - world_offset_m, kept in "
               "every storey's metadata and mesh parameters)",
               who, std::hypot(c.x(), c.y()), off.x(), off.y());
  }
  const double far = std::max((lo - off).cwiseAbs().maxCoeff(),
                              (hi - off).cwiseAbs().maxCoeff());
  if (far > threshold_m)
    core::warn("{}: coordinates reach {:.0f} m from the origin; stored "
               "(float) meshes lose precision beyond {:.0f} m",
               who, far, threshold_m);
  return off;
}

std::array<double, 16> level_frame(double z0) {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -z0, 0, 0, 0, 1};
}

// ---------------------------------------------------------------- walls ---

HostWall wall_axis(const WallFit &fit, const Soup &soup, double thickness,
                   double z0, core::WallRecord &w) {
  const auto [zlo, zhi] = soup.z_range();
  const double tc = 0.5 * (fit.t_lo + fit.t_hi);
  const P2 a = fit.s_lo * fit.u + tc * fit.v;
  const P2 b = fit.s_hi * fit.u + tc * fit.v;
  w.x0 = a.x();
  w.y0 = a.y();
  w.x1 = b.x();
  w.y1 = b.y();
  w.z_base = zlo - z0;
  w.height = zhi - zlo;
  w.thickness = thickness;
  return {a, fit.u, fit.v, (b - a).norm(), 0.5 * thickness, zlo, zhi};
}

std::optional<std::size_t> find_host(const std::vector<HostWall> &walls,
                                     const P2 &c, double z_lo, double z_hi,
                                     double tol) {
  std::optional<std::size_t> host;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < walls.size(); ++k) {
    const auto &g = walls[k];
    const double s = (c - g.p0).dot(g.u);
    const double t = std::abs((c - g.p0).dot(g.v));
    if (t > g.half + tol || s < -tol || s > g.length + tol)
      continue;
    if (z_hi < g.z_lo || z_lo > g.z_hi)
      continue;
    if (t < best) {
      best = t;
      host = k;
    }
  }
  return host;
}

std::pair<double, double> span_on(const HostWall &g, const Soup &soup) {
  double s_lo = std::numeric_limits<double>::infinity(), s_hi = -s_lo;
  for (const auto &p : soup.v) {
    const double s = (P2(p.x(), p.y()) - g.p0).dot(g.u);
    s_lo = std::min(s_lo, s);
    s_hi = std::max(s_hi, s);
  }
  return {s_lo, s_hi};
}

// ------------------------------------------------------------- identity ---

GuidAllocator::GuidAllocator(std::string ns, std::string prefix)
    : ns_(std::move(ns)), prefix_(std::move(prefix)) {}

std::string GuidAllocator::operator()(const std::string &kind,
                                      const std::string &key,
                                      std::size_t index) {
  const std::string name = key.empty()
                               ? fmt::format("{}/{}/#{}", prefix_, kind, index)
                               : fmt::format("{}/{}/{}", prefix_, kind, key);
  std::string g = core::deterministic_guid(ns_, name);
  for (int k = 2; !used_.insert(g).second; ++k)
    g = core::deterministic_guid(ns_, fmt::format("{}#{}", name, k));
  return g;
}

// --------------------------------------------------------------- meshes ---

void MeshMerger::add(const std::string &category, const Soup &soup,
                     const std::string &slug_in) {
  if (soup.empty())
    return;
  const std::string slug = slug_in.empty() ? mesh_slug(category) : slug_in;
  auto &[cat, merged] = by_slug_[slug];
  if (cat.empty())
    cat = category;
  merged.append(soup);
  ++elements_[slug];
}

void MeshMerger::extend(Eigen::Vector3d &lo, Eigen::Vector3d &hi) const {
  for (const auto &[slug, cs] : by_slug_)
    for (const auto &p : cs.second.v) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
}

void MeshMerger::shift(const Eigen::Vector3d &offset) {
  for (auto &[slug, cs] : by_slug_)
    for (auto &p : cs.second.v)
      p -= offset;
}

std::vector<ImportedMesh> MeshMerger::take() {
  std::vector<ImportedMesh> out;
  for (auto &[slug, cat_soup] : by_slug_) {
    auto &[cat, soup] = cat_soup;
    ImportedMesh m;
    m.name = prefix_ + slug;
    m.category = cat;
    m.elements = elements_[slug];
    m.vertices.reserve(soup.v.size());
    for (const auto &p : soup.v)
      m.vertices.push_back({p.x(), p.y(), p.z()});
    m.triangles = std::move(soup.t);
    out.push_back(std::move(m));
  }
  by_slug_.clear();
  elements_.clear();
  return out;
}

MeshWriteResult write_meshes(ProjectDB &db,
                             const std::vector<ImportedMesh> &meshes,
                             const std::string &stage,
                             const nlohmann::json &params) {
  MeshWriteResult res;
  std::set<std::string> written;
  for (const auto &m : meshes) {
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
    nlohmann::json p = params;
    p["category"] = m.category;
    p["elements"] = m.elements;
    p["units"] = "m";
    db.save_mesh(m.name, mesh, stage, p.dump());
    written.insert(m.name);
    res.saved_names.push_back(m.name);
    ++res.saved;
  }
  for (const auto &name : db.list_meshes()) {
    if (name.rfind("imported/", 0) != 0 || written.count(name))
      continue;
    if (db.mesh_metadata(name).stage != stage)
      continue;
    db.delete_mesh(name);
    ++res.removed;
  }
  return res;
}

} // namespace reusex::io::bim
