// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Geometry and bookkeeping shared by the BIM importers (Revit via Speckle,
// speckle_revit.cpp; IFC, ifc.cpp): the readings of an element's triangle
// soup that turn it into a solid-model record (wall axis from the footprint's
// minimum-area rectangle, room footprint from the bottom faces, opening span
// along a host wall), the deterministic GUID scheme, the per-category mesh
// merge and the mesh write. PRIVATE to reusex_io (not installed): it names
// Eigen and nlohmann types freely, and neither importer's public header
// exposes it.

#include "core/solid_model.hpp"
#include "io/imported_mesh.hpp"

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::io::bim {

using P2 = Eigen::Vector2d;

/// A triangle soup in world metres.
struct Soup {
  std::vector<Eigen::Vector3d> v;
  std::vector<std::array<std::uint32_t, 3>> t;

  [[nodiscard]] bool empty() const { return t.empty(); }
  void append(const Soup &o);
  /// (min z, max z); (inf, -inf) when empty.
  [[nodiscard]] std::pair<double, double> z_range() const;
  /// Axis-aligned bounds (lo, hi); undefined when empty.
  [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> bounds() const;
  /// |signed volume| of the (closed) soup.
  [[nodiscard]] double volume() const;
};

// ------------------------------------------------------------ 2D helpers ---

/// Convex hull (Andrew's monotone chain), counter-clockwise, no repeats.
std::vector<P2> convex_hull(std::vector<P2> pts);
double ring_area(const std::vector<P2> &r); ///< signed, CCW > 0
double ring_length(const std::vector<P2> &r);
std::vector<P2> to_p2(const std::vector<core::PlanPoint> &r);

/// The straight-wall reading of a footprint: the minimum-area rectangle of
/// its convex hull. u = axis (long side, canonical +x else +y), v = normal;
/// s along u, t along v.
struct WallFit {
  bool ok = false;
  P2 u{1, 0}, v{0, 1};
  double s_lo = 0, s_hi = 0, t_lo = 0, t_hi = 0;
  double fill = 0.0; ///< hull area / rectangle area
};
WallFit fit_wall(const Soup &soup);

/// Footprint of a space / room volume: the outline of its bottom faces
/// (faces within 2 cm of @p zmin facing up or down). The largest ring is
/// the outer boundary; rings inside it are holes, rings outside it (a
/// second, disjoint piece) are dropped and counted in @p dropped_pieces.
std::optional<core::SpacePolygon>
room_outline(const Soup &soup, double zmin,
             std::size_t *dropped_pieces = nullptr);

/// Area, perimeter, volume (the soup's when it is plausible, else the
/// prism), compactness of a space whose polygon, z_floor and z_ceiling are
/// set; @p soup is its volume in world z (only its volume is read).
void fill_space_metrics(core::SpaceRecord &s, const Soup &soup);

/// The level frame every imported storey shares: world -> level is a
/// translation by -z0 (the lowest storey's floor at z = 0).
std::array<double, 16> level_frame(double z0);

// ---------------------------------------------------------------- walls ---

/// A wall record's axis, as opening hosting needs it (world xy).
struct HostWall {
  P2 p0, u, v;
  double length = 0, half = 0; ///< axis length, half thickness
  double z_lo = 0, z_hi = 0;   ///< world z range
};

/// The wall record of a straight footprint: the axis through the
/// rectangle's middle, z from the soup, in the level frame (z - z0).
/// Fills x0..y1, z_base, height, thickness; returns the hosting geometry.
HostWall wall_axis(const WallFit &fit, const Soup &soup, double thickness,
                   double z0, core::WallRecord &w);

/// The wall an opening with this centre and z range stands in: centre
/// within the wall's half thickness + @p tolerance of the axis, along its
/// run (± tolerance) and overlapping its z range; the nearest one by
/// distance to the axis.
std::optional<std::size_t> find_host(const std::vector<HostWall> &walls,
                                     const P2 &centre, double z_lo, double z_hi,
                                     double tolerance);

/// The opening's extent along its host's axis, clamped to the wall's run.
std::pair<double, double> span_on(const HostWall &host, const Soup &soup);

// ------------------------------------------------------------- identity ---

/// Deterministic GUIDs: uuid5(ns, "<prefix>/<kind>/<key>"), or
/// ".../#<index>" for an element without a key; a repeat gets "#2", "#3".
class GuidAllocator {
    public:
  GuidAllocator(std::string ns, std::string prefix);
  std::string operator()(const std::string &kind, const std::string &key,
                         std::size_t index);

    private:
  std::string ns_, prefix_;
  std::set<std::string> used_;
};

// --------------------------------------------------------------- meshes ---

/// Merges element soups per category slug into ImportedMesh-es.
class MeshMerger {
    public:
  /// @p slug empty = mesh_slug(category).
  void add(const std::string &category, const Soup &soup,
           const std::string &slug = {});
  /// The merged meshes, sorted by name ("imported/<slug>").
  std::vector<ImportedMesh> take();

    private:
  std::map<std::string, std::pair<std::string, Soup>> by_slug_;
  std::map<std::string, std::size_t> elements_;
};

struct MeshWriteResult {
  std::size_t saved = 0;
  std::size_t removed = 0; ///< stale meshes of the same stage deleted
};

/// Save @p meshes (stage @p stage, parameters = @p params plus category,
/// elements, units), then delete the earlier `imported/` meshes of the same
/// stage this import no longer has. Call inside the caller's transaction.
MeshWriteResult write_meshes(ProjectDB &db,
                             const std::vector<ImportedMesh> &meshes,
                             const std::string &stage,
                             const nlohmann::json &params);

} // namespace reusex::io::bim
