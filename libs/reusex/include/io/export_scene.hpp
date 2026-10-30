// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/geometry/BuildingComponent.hpp"
#include "reusex/types/point_types.hpp"

#include <pcl/PolygonMesh.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Intermediate representation for exporting all project data.
/// Used by both Rhino and Speckle exporters.
struct ExportScene {

  // --- Cloud ---
  struct CloudLayer {
    CloudConstPtr cloud;    ///< XYZRGB (always present if layer exists)
    CloudNConstPtr normals; ///< may be nullptr
  };
  std::optional<CloudLayer> cloud;

  // --- Semantic ---
  struct SemanticInstance {
    uint32_t instance_id = 0; ///< 0 if no instance segmentation
    CloudConstPtr points;
  };
  struct SemanticCategory {
    std::string name;                  ///< e.g., "wall", "ceiling"
    int label_id = 0;                  ///< semantic label ID
    std::array<uint8_t, 3> color = {}; ///< Glasbey RGB
    std::vector<SemanticInstance> instances;
  };
  std::vector<SemanticCategory> semantic;

  // --- Meshes ---
  struct MeshEntry {
    std::string name;
    pcl::PolygonMesh::Ptr mesh;
  };
  std::vector<MeshEntry> meshes;

  // --- 360 Panoramas ---
  struct PanoEntry {
    std::string image_name;     ///< filename
    std::string image_url;      ///< empty placeholder for future
    double x = 0, y = 0, z = 0; ///< position from sensor frame pose
  };
  std::vector<PanoEntry> panoramas;

  // --- Materials ---
  struct MaterialEntry {
    std::string name;           ///< designation or GUID
    std::string image_filename; ///< original photo filename, empty if unknown
    double x = 0, y = 0, z = 0; ///< translation from linked sensor frame
    /// Row-major 4x4 world pose of the linked sensor frame (identity if
    /// no frame is linked). Translation also mirrored to x/y/z for the
    /// translation-only Rhino exporter.
    std::array<double, 16> transform{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
    std::map<std::string, std::string> properties;
  };
  std::vector<MaterialEntry> materials;

  // --- Building Components ---
  struct ComponentEntry {
    std::string name;
    geometry::ComponentType type;
    geometry::CoplanarPolygon boundary;
    double confidence = -1.0;
    std::string notes;
    std::map<std::string, std::string> properties;
  };
  std::vector<ComponentEntry> components;

  // --- Typed building elements (W6.2, reusex-z1v.8.2) ---
  /// One storey of the typed model (core::StoreyRecord).
  struct StoreyEntry {
    int idx = 0;
    std::string guid;
    std::string name; ///< display name; may be empty
    double z_floor = 0.0, z_ceiling = 0.0;
  };
  std::vector<StoreyEntry> storeys; ///< by idx, lowest first

  /// One typed element: a wall, slab or roof piece, space volume, door,
  /// window or plain opening, with its own closed mesh.
  struct ElementEntry {
    std::string category;       ///< wall|slab|roof|space|door|window|opening
    std::string guid;           ///< the element's GUID (a slab's is derived)
    std::string name;           ///< its own name (a space's room name), or ""
    int storey = -1;            ///< StoreyEntry::idx; -1 = no storey
    pcl::PolygonMesh::Ptr mesh; ///< world coordinates, metres
    /// The key data as text: guid, type, storey, storey_name and per type
    /// thickness / height / length / area / volume / room_name /
    /// opening_kind / host_wall ... (numbers in metres, m2, m3).
    std::map<std::string, std::string> properties;
  };
  std::vector<ElementEntry> elements;
};

/// How gather_export_scene() reduces what it gathers.
struct ExportSceneOptions {
  /// > 0: keep one point per cube of this edge [m] in the exported cloud
  /// (the first in stored order, so the selection is deterministic). The
  /// stored cloud is not touched.
  double cloud_voxel_m = 0.0;
  /// > 0: keep at most this many points of the exported cloud (after the
  /// voxel filter), an even stride over the stored (Morton) order.
  std::size_t max_cloud_points = 0;
  /// Gather the typed elements (storeys / walls / slabs / spaces /
  /// openings) from the solid_* meshes, or, when the solids stage has not
  /// run, from solids built in memory from the typed tables.
  bool typed_elements = true;
};

/// Gather all exportable data from a ProjectDB.
/// Only populates layers that have data. The semantic layer is built over
/// the same (reduced) point selection as the cloud. When typed elements are
/// gathered from the solid_* meshes, those meshes are left out of
/// ExportScene::meshes so no element is exported twice.
ExportScene gather_export_scene(const ProjectDB &db,
                                const ExportSceneOptions &options = {});

/// The layer / collection name of an element category: "Walls", "Slabs",
/// "Roof", "Spaces", "Doors", "Windows", "Other openings".
std::string element_layer_name(std::string_view category);

/// The element categories in the order exporters list them.
const std::vector<std::string> &element_categories();

/// The object name of an element: its name (or its type, "Wall", "Door",
/// ...) followed by its GUID in parentheses.
std::string element_object_name(const ExportScene::ElementEntry &element);

/// The layer / collection name of each storey of @p scene, by idx: the
/// storey's name, else "Storey <idx>"; a repeated name gets " (<idx>)".
std::map<int, std::string> storey_layer_names(const ExportScene &scene);

/// The layer / collection name of the elements that sit on no storey.
inline constexpr std::string_view kNoStoreyLayerName = "No storey";

} // namespace reusex::io
