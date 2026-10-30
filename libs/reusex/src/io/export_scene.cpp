// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/export_scene.hpp"
#include "core/logging.hpp"

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/label_encoding.hpp>
#include <reusex/geometry/BuildingComponent.hpp>
#include <reusex/geometry/cgal_utils.hpp>
#include <reusex/geometry/component_persistence.hpp>
#include <reusex/geometry/unweld.hpp>

#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/reconstruction/solids.hpp>

#include <pcl/common/colors.h>
#include <pcl/common/io.h>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <regex>
#include <set>
#include <unordered_set>

namespace reusex::io {

namespace {

/// Parse instance label_definitions format: "SM{semantic_class}-{instance_id}
/// ({size}p)"
/// Returns the semantic class ID, or -1 on failure.
int parse_instance_semantic_class(const std::string &def) {
  // Match "SM<digits>-<digits> (<digits>p)"
  static const std::regex re(R"(SM(\d+)-\d+\s+\(\d+p\))");
  std::smatch match;
  if (std::regex_match(def, match, re)) {
    auto val = std::stoul(match[1].str());
    if (val > static_cast<unsigned long>(std::numeric_limits<int>::max()))
      return -1;
    return static_cast<int>(val);
  }
  return -1;
}

/// Extract translation from a row-major 4x4 SE(3) matrix.
void extract_position(const std::array<double, 16> &pose, double &x, double &y,
                      double &z) {
  x = pose[3];
  y = pose[7];
  z = pose[11];
}

/// Multiply two row-major 4x4 matrices: result = a * b.
std::array<double, 16> matmul4x4(const std::array<double, 16> &a,
                                 const std::array<double, 16> &b) {
  std::array<double, 16> r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  return r;
}

// ---- Export-only cloud reduction (reusex-f97m) ----------------------------

/// The points of @p cloud an export keeps: one per voxel (first in stored
/// order), then an even stride down to the cap. Empty = keep all.
std::optional<pcl::Indices>
export_cloud_selection(const Cloud &cloud, const ExportSceneOptions &o) {
  const std::size_t n = cloud.size();
  if (!(o.cloud_voxel_m > 0.0) &&
      (o.max_cloud_points == 0 || n <= o.max_cloud_points))
    return std::nullopt;
  pcl::Indices sel;
  if (o.cloud_voxel_m > 0.0) {
    const double inv = 1.0 / o.cloud_voxel_m;
    const auto cell = [&](float v) {
      return static_cast<std::uint64_t>(
                 static_cast<std::int64_t>(std::floor(v * inv))) &
             0x1FFFFFu;
    };
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(n / 4 + 1);
    for (std::size_t i = 0; i < n; ++i) {
      const auto &p = cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      const std::uint64_t key =
          cell(p.x) | (cell(p.y) << 21) | (cell(p.z) << 42);
      if (seen.insert(key).second)
        sel.push_back(static_cast<pcl::index_t>(i));
    }
  } else {
    sel.resize(n);
    for (std::size_t i = 0; i < n; ++i)
      sel[i] = static_cast<pcl::index_t>(i);
  }
  if (o.max_cloud_points > 0 && sel.size() > o.max_cloud_points) {
    pcl::Indices capped(o.max_cloud_points);
    for (std::size_t k = 0; k < o.max_cloud_points; ++k)
      capped[k] = sel[k * sel.size() / o.max_cloud_points];
    sel = std::move(capped);
  }
  return sel;
}

// ---- Typed elements (W6.2, reusex-z1v.8.2) --------------------------------

using json = nlohmann::json;
using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

constexpr std::array<std::string_view, 3> kSolidMeshes{
    "solid_walls", "solid_slabs", "solid_spaces"};

std::string num(double v) { return fmt::format("{:.3f}", v); }

double ring_area(const std::vector<core::PlanPoint> &ring) {
  double a = 0.0;
  for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % n];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return 0.5 * a;
}

double profile_area(const geometry::ExtrusionProfile &p) {
  double a = std::abs(ring_area(p.outer));
  for (const auto &h : p.holes)
    a -= std::abs(ring_area(h));
  return std::max(0.0, a);
}

Eigen::Matrix4d to_eigen(const std::array<double, 16> &m) {
  return Eigen::Map<const RowMajor4d>(m.data());
}

/// A closed prism: @p ring (any winding, open) in the local XY plane swept
/// from z = @p z0 to @p z1, mapped by @p placement (local -> world). Caps
/// are triangle fans, so the ring must be convex (a box, here).
pcl::PolygonMesh prism(std::vector<core::PlanPoint> ring, double z0, double z1,
                       const Eigen::Matrix4d &placement) {
  if (ring_area(ring) < 0.0)
    std::reverse(ring.begin(), ring.end());
  const auto n = static_cast<std::uint32_t>(ring.size());
  pcl::PointCloud<pcl::PointXYZ> pts;
  for (const double z : {z0, z1})
    for (const auto &p : ring) {
      const Eigen::Vector4d w = placement * Eigen::Vector4d(p[0], p[1], z, 1);
      pts.push_back(pcl::PointXYZ(static_cast<float>(w.x()),
                                  static_cast<float>(w.y()),
                                  static_cast<float>(w.z())));
    }
  pcl::PolygonMesh m;
  const auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    pcl::Vertices v;
    v.vertices = {a, b, c};
    m.polygons.push_back(std::move(v));
  };
  for (std::uint32_t i = 1; i + 1 < n; ++i) {
    tri(0, i + 1, i);         // bottom, facing down
    tri(n, n + i, n + i + 1); // top, facing up
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    const std::uint32_t j = (i + 1) % n;
    tri(i, j, j + n);
    tri(i, j + n, i + n);
  }
  pcl::toPCLPointCloud2(pts, m.cloud);
  return m;
}

geometry::ExtrusionProfile profile_from_json(const json &j) {
  geometry::ExtrusionProfile p;
  if (!j.is_object())
    return p;
  p.outer = j.value("outer", std::vector<core::PlanPoint>{});
  p.holes = j.value("holes", std::vector<std::vector<core::PlanPoint>>{});
  p.depth = j.value("depth", 0.0);
  if (j.contains("placement"))
    p.placement = j.at("placement").get<std::array<double, 16>>();
  return p;
}

/// The elements of the stored solids mesh @p name, split by the element
/// index in its parameters (pipeline run_solids / concat_solids).
/// @return false when the mesh is missing or carries no usable index.
bool stored_solids(const ProjectDB &db, std::string_view name,
                   geometry::SolidType type,
                   std::vector<geometry::ElementSolid> &out) {
  if (!db.has_mesh(name))
    return false;
  const auto mesh = db.mesh(name);
  json meta;
  try {
    meta = json::parse(db.mesh_metadata(name).parameters);
  } catch (const json::exception &e) {
    reusex::warn("ExportScene: mesh '{}' has no readable element index ({}); "
                 "it is exported as a plain mesh. Re-run `rux create solids`",
                 name, e.what());
    return false;
  }
  if (!mesh || !meta.is_object() || !meta.contains("elements") ||
      !meta.at("elements").is_array()) {
    reusex::warn("ExportScene: mesh '{}' carries no element index; it is "
                 "exported as a plain mesh. Re-run `rux create solids`",
                 name);
    return false;
  }
  pcl::PointCloud<pcl::PointXYZ> pts;
  pcl::fromPCLPointCloud2(mesh->cloud, pts);
  for (const auto &e : meta.at("elements")) {
    try {
      geometry::ElementSolid s;
      s.type = type;
      s.source_guid = e.value("guid", "");
      s.storey = e.value("storey", 0);
      s.predefined = e.value("predefined", "");
      s.part = e.value("part", 0);
      s.closed = e.value("closed", false);
      s.volume = e.value("volume", 0.0);
      if (e.contains("profile"))
        s.profile = profile_from_json(e.at("profile"));
      const auto f0 = e.at("faces").at(0).get<std::size_t>();
      const auto fn = e.at("faces").at(1).get<std::size_t>();
      const auto v0 = e.at("vertices").at(0).get<std::size_t>();
      const auto vn = e.at("vertices").at(1).get<std::size_t>();
      if (f0 + fn > mesh->polygons.size() || v0 + vn > pts.size())
        throw std::out_of_range("face or vertex range past the mesh");
      pcl::PointCloud<pcl::PointXYZ> sub;
      sub.points.assign(pts.points.begin() + static_cast<std::ptrdiff_t>(v0),
                        pts.points.begin() +
                            static_cast<std::ptrdiff_t>(v0 + vn));
      sub.width = static_cast<std::uint32_t>(vn);
      sub.height = 1;
      for (std::size_t f = f0; f < f0 + fn; ++f) {
        pcl::Vertices v;
        for (const auto i : mesh->polygons[f].vertices) {
          if (i < v0 || i >= v0 + vn)
            throw std::out_of_range("face names a vertex of another element");
          v.vertices.push_back(static_cast<std::uint32_t>(i - v0));
        }
        s.mesh.polygons.push_back(std::move(v));
      }
      pcl::toPCLPointCloud2(sub, s.mesh.cloud);
      out.push_back(std::move(s));
    } catch (const std::exception &ex) {
      reusex::warn("ExportScene: an element of mesh '{}' could not be read "
                   "({}); skipped",
                   name, ex.what());
    }
  }
  return true;
}

std::string label_of(std::string_view category) {
  if (category == "wall")
    return "Wall";
  if (category == "slab")
    return "Slab";
  if (category == "roof")
    return "Roof";
  if (category == "space")
    return "Space";
  if (category == "door")
    return "Door";
  if (category == "window")
    return "Window";
  return "Opening";
}

/// A slab piece's own GUID: uuid5 of its storey's GUID, else a path.
std::string slab_guid(const std::string &storey_guid,
                      const std::string &predefined, int part) {
  const std::string name = fmt::format(
      "slab/{}/{}", predefined.empty() ? "FLOOR" : predefined, part);
  try {
    return core::deterministic_guid(storey_guid, name);
  } catch (const std::invalid_argument &) {
    return storey_guid + "/" + name;
  }
}

void gather_typed_elements(const ProjectDB &db, ExportScene &scene,
                           std::set<std::string> &consumed_meshes) {
  if (!db.has_storeys())
    return;
  const core::SolidModel model = load_solid_model(db);
  if (model.storeys.empty())
    return;

  std::map<int, const core::StoreyRecord *> storey_of;
  for (const auto &s : model.storeys) {
    storey_of[s.idx] = &s;
    scene.storeys.push_back({s.idx, s.guid, s.name, s.z_floor, s.z_ceiling});
  }
  std::sort(scene.storeys.begin(), scene.storeys.end(),
            [](const auto &a, const auto &b) { return a.idx < b.idx; });

  // The solids: as the solids stage stored them, else built here.
  std::vector<geometry::ElementSolid> walls, slabs, spaces;
  const std::array<
      std::pair<geometry::SolidType, std::vector<geometry::ElementSolid> *>, 3>
      kinds{{{geometry::SolidType::wall, &walls},
             {geometry::SolidType::slab, &slabs},
             {geometry::SolidType::space, &spaces}}};
  for (std::size_t k = 0; k < kinds.size(); ++k)
    if (stored_solids(db, kSolidMeshes[k], kinds[k].first, *kinds[k].second))
      consumed_meshes.insert(std::string(kSolidMeshes[k]));
  if (!consumed_meshes.empty()) {
    // As stored.
  } else if (!model.walls.empty()) {
    reusex::info("ExportScene: no solid_* meshes (`rux create solids` has not "
                 "run); building the solids of {} wall(s) and {} space(s) "
                 "for the export only",
                 model.walls.size(), model.spaces.size());
    try {
      auto r = geometry::build_solids(model);
      walls = std::move(r.walls);
      slabs = std::move(r.slabs);
      spaces = std::move(r.spaces);
    } catch (const std::exception &e) {
      reusex::warn("ExportScene: building the solids failed ({}); no typed "
                   "walls, slabs or spaces are exported",
                   e.what());
    }
  }

  std::map<std::string, const core::WallRecord *> wall_of;
  for (const auto &w : model.walls)
    wall_of[w.guid] = &w;
  std::map<std::string, const core::SpaceRecord *> space_of;
  for (const auto &s : model.spaces)
    space_of[s.guid] = &s;
  std::map<std::string, std::vector<const core::OpeningRecord *>> hosted;
  for (const auto &o : model.openings)
    hosted[o.wall].push_back(&o);

  const auto add = [&](std::string category, std::string guid, std::string name,
                       int storey, const pcl::PolygonMesh &mesh,
                       std::map<std::string, std::string> props) {
    ExportScene::ElementEntry e;
    e.category = std::move(category);
    e.guid = std::move(guid);
    e.name = std::move(name);
    e.storey = storey;
    e.mesh = geometry::unweld_mesh(mesh, 0.7854f);
    props["guid"] = e.guid;
    props["type"] = e.category;
    if (!e.name.empty())
      props["name"] = e.name;
    props["storey"] = std::to_string(storey);
    if (auto it = storey_of.find(storey); it != storey_of.end()) {
      if (!it->second->name.empty())
        props["storey_name"] = it->second->name;
      props["storey_guid"] = it->second->guid;
    }
    e.properties = std::move(props);
    scene.elements.push_back(std::move(e));
  };

  // Walls (and remember each one's solid, for its openings).
  std::map<std::string, const geometry::ElementSolid *> wall_solid;
  for (const auto &s : walls) {
    if (s.mesh.polygons.empty())
      continue;
    wall_solid[s.source_guid] = &s;
    std::map<std::string, std::string> p;
    const core::WallRecord *w = nullptr;
    if (auto it = wall_of.find(s.source_guid); it != wall_of.end())
      w = it->second;
    double box_thickness = 0.0;
    if (s.profile.outer.size() == 4)
      box_thickness = std::abs(s.profile.outer[2][1] - s.profile.outer[0][1]);
    const double height = s.profile.depth;
    p["height"] = num(height);
    if (w) {
      const bool measured = w->thickness > 0.0 && !w->assumed;
      p["thickness"] = num(w->thickness > 0.0 ? w->thickness : box_thickness);
      p["thickness_assumed"] = measured ? "false" : "true";
      const double len = std::hypot(w->x1 - w->x0, w->y1 - w->y0);
      p["length"] = num(len);
      double holes = 0.0;
      for (const auto *o : hosted[w->guid]) {
        const double s0 = std::clamp(std::min(o->s_start, o->s_end), 0.0, len);
        const double s1 = std::clamp(std::max(o->s_start, o->s_end), 0.0, len);
        holes += (s1 - s0) * std::clamp(o->z_head - o->z_sill, 0.0, height);
      }
      p["area"] = num(std::max(0.0, len * height - holes));
      p["is_external"] = w->is_external ? "true" : "false";
      p["source"] = w->source;
    } else {
      p["thickness"] = num(box_thickness);
    }
    if (s.closed)
      p["volume"] = num(s.volume);
    add("wall", s.source_guid, "", s.storey, s.mesh, std::move(p));
  }

  // Slabs and roof pieces.
  for (const auto &s : slabs) {
    if (s.mesh.polygons.empty())
      continue;
    const bool roof = s.predefined == "ROOF";
    std::map<std::string, std::string> p;
    p["thickness"] = num(s.profile.depth);
    p["area"] = num(profile_area(s.profile));
    p["predefined"] = s.predefined;
    p["part"] = std::to_string(s.part);
    if (s.closed)
      p["volume"] = num(s.volume);
    add(roof ? "roof" : "slab", slab_guid(s.source_guid, s.predefined, s.part),
        "", s.storey, s.mesh, std::move(p));
  }

  // Space volumes, named by their rooms.
  for (const auto &s : spaces) {
    if (s.mesh.polygons.empty())
      continue;
    std::map<std::string, std::string> p;
    std::string name;
    if (auto it = space_of.find(s.source_guid); it != space_of.end()) {
      const auto &r = *it->second;
      name = r.name;
      if (!r.name.empty())
        p["room_name"] = r.name;
      p["kind"] = r.kind;
      p["area"] = num(r.area >= 0.0 ? r.area : profile_area(s.profile));
      p["height"] = num(r.z_ceiling - r.z_floor);
      p["z_floor"] = num(r.z_floor);
      p["z_ceiling"] = num(r.z_ceiling);
      if (r.perimeter >= 0.0)
        p["perimeter"] = num(r.perimeter);
      p["source"] = r.source;
    } else {
      p["area"] = num(profile_area(s.profile));
      p["height"] = num(s.profile.depth);
    }
    if (s.closed)
      p["volume"] = num(s.volume);
    add("space", s.source_guid, name, s.storey, s.mesh, std::move(p));
  }

  // Doors, windows and plain openings: a box filling the opening in its
  // host wall's solid (the wall's own frame and thickness), else -- the
  // wall has no solid -- a box about the wall's axis in its storey's frame.
  std::size_t no_host = 0;
  for (const auto &o : model.openings) {
    const auto wit = wall_of.find(o.wall);
    if (wit == wall_of.end()) {
      ++no_host;
      continue;
    }
    const core::WallRecord &w = *wit->second;
    const auto sit = storey_of.find(w.storey);
    const Eigen::Matrix4d T = sit != storey_of.end()
                                  ? to_eigen(sit->second->level_transform)
                                  : Eigen::Matrix4d::Identity();
    const double s0 = std::min(o.s_start, o.s_end);
    const double s1 = std::max(o.s_start, o.s_end);
    pcl::PolygonMesh mesh;
    if (auto ws = wall_solid.find(w.guid);
        ws != wall_solid.end() && ws->second->profile.outer.size() == 4) {
      const auto &pr = ws->second->profile;
      const Eigen::Matrix4d P = to_eigen(pr.placement);
      // The wall's base in the level frame: its local origin.
      const double z0 = (T * P)(2, 3);
      const double lo = std::min(pr.outer[0][1], pr.outer[2][1]);
      const double hi = std::max(pr.outer[0][1], pr.outer[2][1]);
      const double zs = std::clamp(o.z_sill - z0, 0.0, pr.depth);
      const double zh = std::clamp(o.z_head - z0, 0.0, pr.depth);
      if (s1 - s0 > 1e-3 && zh - zs > 1e-3)
        mesh = prism({{s0, lo}, {s1, lo}, {s1, hi}, {s0, hi}}, zs, zh, P);
    } else {
      const Eigen::Vector2d a(w.x0, w.y0), b(w.x1, w.y1);
      const double len = (b - a).norm();
      if (len > 1e-6 && s1 - s0 > 1e-3 && o.z_head - o.z_sill > 1e-3) {
        const Eigen::Vector2d d = (b - a) / len, nrm(-d.y(), d.x());
        const double t = 0.5 * (w.thickness > 0.0 ? w.thickness : 0.10);
        const auto at = [&](double s, double off) -> core::PlanPoint {
          const Eigen::Vector2d q = a + s * d + off * nrm;
          return {q.x(), q.y()};
        };
        mesh = prism({at(s0, -t), at(s1, -t), at(s1, t), at(s0, t)}, o.z_sill,
                     o.z_head, T.inverse());
      }
    }
    if (mesh.polygons.empty()) {
      ++no_host;
      continue;
    }
    const std::string category = o.kind == "door"     ? "door"
                                 : o.kind == "window" ? "window"
                                                      : "opening";
    std::map<std::string, std::string> p;
    p["opening_kind"] = o.kind;
    p["width"] = num(s1 - s0);
    p["height"] = num(o.z_head - o.z_sill);
    p["sill"] = num(o.z_sill);
    p["host_wall"] = o.wall;
    p["state"] = o.state;
    p["is_external"] = o.is_external ? "true" : "false";
    p["source"] = o.source;
    add(category, o.guid, "", w.storey, mesh, std::move(p));
  }
  if (no_host > 0)
    reusex::warn("ExportScene: {} of {} opening(s) have no host wall or no "
                 "extent; not exported",
                 no_host, model.openings.size());

  std::map<std::string, int> n;
  for (const auto &e : scene.elements)
    ++n[e.category];
  reusex::info("ExportScene: typed elements on {} storey(s): {} wall(s), {} "
               "slab(s), {} roof piece(s), {} space(s), {} door(s), {} "
               "window(s), {} other opening(s)",
               scene.storeys.size(), n["wall"], n["slab"], n["roof"],
               n["space"], n["door"], n["window"], n["opening"]);
}

} // namespace

std::string element_layer_name(std::string_view category) {
  if (category == "wall")
    return "Walls";
  if (category == "slab")
    return "Slabs";
  if (category == "roof")
    return "Roof";
  if (category == "space")
    return "Spaces";
  if (category == "door")
    return "Doors";
  if (category == "window")
    return "Windows";
  return "Other openings";
}

const std::vector<std::string> &element_categories() {
  static const std::vector<std::string> c{"wall", "slab",   "roof",   "space",
                                          "door", "window", "opening"};
  return c;
}

std::string element_object_name(const ExportScene::ElementEntry &element) {
  return fmt::format("{} ({})",
                     element.name.empty() ? label_of(element.category)
                                          : element.name,
                     element.guid);
}

std::map<int, std::string> storey_layer_names(const ExportScene &scene) {
  std::map<int, std::string> out;
  std::set<std::string> used;
  for (const auto &s : scene.storeys) {
    std::string name =
        s.name.empty() ? fmt::format("Storey {}", s.idx) : s.name;
    if (!used.insert(name).second) {
      name = fmt::format("{} ({})", name, s.idx);
      used.insert(name);
    }
    out[s.idx] = std::move(name);
  }
  return out;
}

ExportScene gather_export_scene(const ProjectDB &db,
                                const ExportSceneOptions &options) {
  ExportScene scene;

  // The points the export keeps (reusex-f97m): every one, or a reduced,
  // index-aligned selection shared by the cloud, normals and semantic layer.
  CloudConstPtr full_cloud;
  std::optional<pcl::Indices> selection;
  if (db.has_point_cloud("cloud")) {
    full_cloud = db.point_cloud_xyzrgb("cloud");
    selection = export_cloud_selection(*full_cloud, options);
    if (selection)
      reusex::info("ExportScene: exporting {} of {} cloud points (voxel {} m, "
                   "cap {}); the stored cloud is unchanged",
                   selection->size(), full_cloud->size(), options.cloud_voxel_m,
                   options.max_cloud_points);
  }
  // The stored-cloud indices the export walks, in order.
  const auto for_each_kept = [&](std::size_t limit, auto &&fn) {
    if (selection) {
      for (const auto i : *selection)
        if (static_cast<std::size_t>(i) < limit)
          fn(static_cast<std::size_t>(i));
    } else {
      for (std::size_t i = 0; i < limit; ++i)
        fn(i);
    }
  };

  // --- Cloud layer ---
  if (full_cloud) {
    reusex::debug("ExportScene: loading 'cloud'");
    ExportScene::CloudLayer layer;
    CloudNConstPtr normals;
    if (db.has_point_cloud("normals"))
      normals = db.point_cloud_normal("normals");
    if (selection) {
      CloudPtr kept(new Cloud);
      pcl::copyPointCloud(*full_cloud, *selection, *kept);
      layer.cloud = kept;
      if (normals && normals->size() == full_cloud->size()) {
        CloudNPtr kept_n(new CloudN);
        pcl::copyPointCloud(*normals, *selection, *kept_n);
        layer.normals = kept_n;
      } else if (normals) {
        reusex::warn("ExportScene: 'normals' has {} points, 'cloud' {}; the "
                     "reduced cloud is exported without normals",
                     normals->size(), full_cloud->size());
      }
    } else {
      layer.cloud = full_cloud;
      layer.normals = normals;
    }

    scene.cloud = std::move(layer);
    reusex::debug("ExportScene: cloud has {} points",
                  scene.cloud->cloud->size());
  }

  // --- Semantic layer ---
  if (db.has_point_cloud("instances") && full_cloud) {
    // Instance-based export
    reusex::debug("ExportScene: loading instance-based semantic data");

    auto instance_labels = db.point_cloud_label("instances");
    auto instance_defs = db.label_definitions("instances");

    // Get semantic name mapping from "labels" definitions
    std::map<int, std::string> semantic_names;
    if (db.has_point_cloud("labels")) {
      core::warn_if_unstamped_semantic_cloud(db, "labels");
      semantic_names = db.label_definitions("labels");
    }

    const auto &cloud = full_cloud;

    // Group instance IDs by semantic class
    // instance_id -> semantic_class_id
    std::map<uint32_t, int> instance_to_semantic;
    for (const auto &[inst_id, def] : instance_defs) {
      int sem_class = parse_instance_semantic_class(def);
      if (sem_class >= 0)
        instance_to_semantic[static_cast<uint32_t>(inst_id)] = sem_class;
    }

    // semantic_class_id -> list of instance_ids
    std::map<int, std::vector<uint32_t>> semantic_to_instances;
    for (const auto &[inst_id, sem_class] : instance_to_semantic)
      semantic_to_instances[sem_class].push_back(inst_id);

    // Build per-instance point clouds
    // instance_id -> CloudPtr
    std::map<uint32_t, CloudPtr> instance_clouds;
    for_each_kept(std::min(instance_labels->size(), cloud->size()),
                  [&](std::size_t i) {
                    uint32_t inst = instance_labels->points[i].label;
                    if (inst == 0)
                      return; // skip unlabeled
                    auto [it, inserted] = instance_clouds.try_emplace(inst);
                    if (inserted)
                      it->second = CloudPtr(new Cloud());
                    it->second->points.push_back(cloud->points[i]);
                  });

    // Build SemanticCategory entries
    for (const auto &[sem_class, inst_ids] : semantic_to_instances) {
      ExportScene::SemanticCategory cat;
      cat.label_id = sem_class;

      // Look up name from "labels" label_definitions
      auto it = semantic_names.find(sem_class);
      cat.name = (it != semantic_names.end())
                     ? it->second
                     : "class_" + std::to_string(sem_class);

      // Glasbey color
      auto c = pcl::GlasbeyLUT::at(sem_class % pcl::GlasbeyLUT::size());
      cat.color = {c.r, c.g, c.b};

      // Add instances
      for (uint32_t inst_id : inst_ids) {
        auto cloud_it = instance_clouds.find(inst_id);
        if (cloud_it == instance_clouds.end() || cloud_it->second->empty())
          continue;

        ExportScene::SemanticInstance si;
        si.instance_id = inst_id;
        si.points = cloud_it->second;
        cat.instances.push_back(std::move(si));
      }

      if (!cat.instances.empty())
        scene.semantic.push_back(std::move(cat));
    }

    reusex::debug("ExportScene: {} semantic categories (instance-based)",
                  scene.semantic.size());

  } else if (db.has_point_cloud("labels") && full_cloud) {
    // Label-based export (no instances)
    reusex::debug("ExportScene: loading label-based semantic data");

    auto labels = db.point_cloud_label("labels");
    core::warn_if_unstamped_semantic_cloud(db, "labels");
    auto label_defs = db.label_definitions("labels");
    const auto &cloud = full_cloud;

    // Group points by label
    std::map<uint32_t, CloudPtr> label_clouds;
    for_each_kept(std::min(labels->size(), cloud->size()), [&](std::size_t i) {
      uint32_t lbl = labels->points[i].label;
      if (lbl == 0)
        return; // skip unlabeled/background
      auto [it, inserted] = label_clouds.try_emplace(lbl);
      if (inserted)
        it->second = CloudPtr(new Cloud());
      it->second->points.push_back(cloud->points[i]);
    });

    for (const auto &[lbl, lbl_cloud] : label_clouds) {
      ExportScene::SemanticCategory cat;
      cat.label_id = static_cast<int>(lbl);

      auto it = label_defs.find(static_cast<int>(lbl));
      cat.name = (it != label_defs.end()) ? it->second
                                          : "class_" + std::to_string(lbl);

      auto c = pcl::GlasbeyLUT::at(lbl % pcl::GlasbeyLUT::size());
      cat.color = {c.r, c.g, c.b};

      ExportScene::SemanticInstance si;
      si.instance_id = 0;
      si.points = lbl_cloud;
      cat.instances.push_back(std::move(si));

      scene.semantic.push_back(std::move(cat));
    }

    reusex::debug("ExportScene: {} semantic categories (label-based)",
                  scene.semantic.size());
  }

  // --- Typed elements (before the meshes: a solid_* mesh read as typed
  // elements is not exported again as a plain mesh) ---
  std::set<std::string> typed_meshes;
  if (options.typed_elements)
    gather_typed_elements(db, scene, typed_meshes);

  // --- Mesh layer ---
  for (const auto &name : db.list_meshes()) {
    if (typed_meshes.count(name) > 0)
      continue;
    reusex::debug("ExportScene: loading mesh '{}'", name);
    auto mesh = db.mesh(name);
    if (!mesh || mesh->polygons.empty())
      continue;

    auto components = geometry::cgal::decompose_mesh(*mesh);
    if (components.size() == 1) {
      auto unwelded = geometry::unweld_mesh(*components[0], 0.7854f);
      scene.meshes.push_back({name, unwelded});
    } else {
      for (size_t i = 0; i < components.size(); ++i) {
        auto unwelded = geometry::unweld_mesh(*components[i], 0.7854f);
        scene.meshes.push_back({fmt::format("{}_{}", name, i), unwelded});
      }
    }
    reusex::debug("ExportScene: mesh '{}' -> {} components", name,
                  components.size());
  }

  // --- 360 Panoramas ---
  std::size_t panos_without_pose = 0;
  for (const auto &pano : db.list_panoramic_images()) {
    ExportScene::PanoEntry entry;
    entry.image_name = pano.filename;

    // Prefer the content-aligned pose (`rux align 360`) when present; fall
    // back to the timestamp-matched sensor frame's pose.
    //
    // The seed frame's pose has to be checked, not just its existence: an
    // unposed frame would fall through `sensor_frame_pose()`'s identity
    // fallback and drop the panorama marker at the world origin, which reads
    // in an exported scene as a real placement (#336). Leaving x/y/z at their
    // defaults is the same "unplaced" state a panorama with no linked frame
    // already gets.
    if (pano.has_pose) {
      extract_position(pano.pose, entry.x, entry.y, entry.z);
    } else if (pano.node_id >= 0 && db.has_sensor_frame(pano.node_id) &&
               db.has_sensor_frame_pose(pano.node_id)) {
      auto pose = db.sensor_frame_pose(pano.node_id);
      extract_position(pose, entry.x, entry.y, entry.z);
    } else {
      ++panos_without_pose;
    }

    scene.panoramas.push_back(std::move(entry));
  }

  if (!scene.panoramas.empty())
    reusex::debug("ExportScene: {} panoramic images", scene.panoramas.size());
  if (panos_without_pose > 0)
    reusex::warn("ExportScene: {} of {} panoramas have no usable pose (no "
                 "aligned pose and no posed seed frame); exported unplaced",
                 panos_without_pose, scene.panoramas.size());

  // --- Materials ---
  std::size_t materials_without_pose = 0;
  for (const auto &passport : db.all_material_passports()) {
    ExportScene::MaterialEntry entry;
    entry.name = passport.description.designation.empty()
                     ? passport.metadata.document_guid
                     : passport.description.designation;

    // Try to get stored properties
    try {
      entry.properties =
          db.passport_stored_properties(passport.metadata.document_guid);
    } catch (...) {
      // No stored properties - that's OK
    }

    // Pull the original filename out of properties (set by "rux import
    // photos") and onto a dedicated field so exporters don't have to peek
    // into the property bag.
    if (auto it = entry.properties.find("fileName");
        it != entry.properties.end()) {
      entry.image_filename = it->second;
      entry.properties.erase(it);
    }

    // Position + orientation: photo imports link the passport to the
    // nearest sensor frame via material_passports.id == node_id, so look
    // that up and pull the frame's world pose. The pose stored on the
    // frame is the sensor BODY pose (RTABMap convention); compose it with
    // the camera→base local_transform to get the optical frame's world
    // pose so the camera-frustum marker points along the actual viewing
    // direction.
    try {
      auto node_id =
          db.passport_linked_node_id(passport.metadata.document_guid);
      if (node_id && db.has_sensor_frame(*node_id)) {
        // A linked frame with no usable stored pose yields the identity
        // fallback, i.e. a camera frustum at the world origin pointing down
        // +Z — indistinguishable from a real photo position (#336). Keep the
        // entry's default identity transform instead, which is the same state
        // an unlinked passport gets, and count it.
        if (!db.has_sensor_frame_pose(*node_id)) {
          reusex::debug("ExportScene: material '{}' links node {}, which has "
                        "no usable stored pose; exported unplaced",
                        entry.name, *node_id);
          ++materials_without_pose;
        } else {
          auto pose = db.sensor_frame_pose(*node_id);
          try {
            auto intr = db.sensor_frame_intrinsics(*node_id);
            entry.transform = matmul4x4(pose, intr.local_transform);
          } catch (...) {
            // No intrinsics stored — fall back to the body pose.
            entry.transform = pose;
          }
          extract_position(entry.transform, entry.x, entry.y, entry.z);
        }
      }
    } catch (...) {
      // No linked frame -> keep identity transform / origin.
    }

    scene.materials.push_back(std::move(entry));
  }

  if (!scene.materials.empty())
    reusex::debug("ExportScene: {} material passports", scene.materials.size());
  if (materials_without_pose > 0)
    reusex::warn("ExportScene: {} of {} material passports link a sensor frame "
                 "with no usable stored pose; exported unplaced",
                 materials_without_pose, scene.materials.size());

  // --- Building Components ---
  for (const auto &name : db.list_building_components()) {
    auto comp = geometry::building_component(db, name);
    ExportScene::ComponentEntry entry;
    entry.name = comp.name;
    entry.type = comp.type;
    entry.boundary = comp.boundary;
    entry.confidence = comp.confidence;
    entry.notes = comp.notes;

    entry.properties["type"] = std::string(geometry::to_string(comp.type));
    if (comp.confidence >= 0)
      entry.properties["confidence"] = std::to_string(comp.confidence);
    if (!comp.notes.empty())
      entry.properties["notes"] = comp.notes;

    if (auto *w = std::get_if<geometry::WindowData>(&comp.data)) {
      if (!w->style.empty())
        entry.properties["style"] = w->style;
      if (w->pane_count > 0)
        entry.properties["pane_count"] = std::to_string(w->pane_count);
      entry.properties["operable"] = w->operable ? "true" : "false";
    } else if (auto *d = std::get_if<geometry::DoorData>(&comp.data)) {
      if (!d->style.empty())
        entry.properties["style"] = d->style;
      if (!d->swing.empty())
        entry.properties["swing"] = d->swing;
    }

    scene.components.push_back(std::move(entry));
  }

  if (!scene.components.empty())
    reusex::debug("ExportScene: {} building components",
                  scene.components.size());

  return scene;
}

} // namespace reusex::io
