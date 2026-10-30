// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/core/solid_model.hpp"
#include "reusex/io/imported_mesh.hpp"
#include "reusex/io/solid_model_json.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

/// @file
/// IFC model (STEP, IFC2X3 / IFC4 / IFC4X3) -> ReUseX solid model + meshes,
/// through the native C++ core of web-ifc (That Open, MPL-2.0). The IFC
/// counterpart of `io/speckle_revit.hpp`, mapped by the same rules:
///  - IfcBuildingStorey -> storeys (one shared level frame, the lowest
///    storey's floor at z = 0);
///  - IfcWall / IfcWallStandardCase / IfcWallElementedCase with a straight
///    footprint -> walls (axis = centreline of the footprint's minimum-area
///    rectangle);
///  - IfcDoor / IfcWindow -> openings on their host wall (IfcRelFillsElement
///    -> IfcOpeningElement -> IfcRelVoidsElement -> wall, else by geometry);
///  - IfcSpace -> spaces (footprint polygon from the bottom faces of its
///    body, else its 'FootPrint' curve with z from its 'Box');
///  - every product's body geometry -> one mesh per IFC class
///    (`imported/ifc/<class slug>`).
///
/// web-ifc is an optional dependency: without it (has_ifc_support() false)
/// import_ifc() and import_ifc_file() throw.
///
/// Frame. World is the IFC file's own world coordinate system (z up), in
/// metres (the project's length unit is converted); nothing is aligned to a
/// scan. A model far from the origin (georeferenced) is recentred by a
/// whole-km x/y offset (IfcImportOptions::recenter), as the Revit import
/// does. All storeys share one level frame: a translation putting the
/// lowest storey's floor at z = 0.

namespace reusex {
class ProjectDB;
}

namespace reusex::io::ifc {

struct IfcImportOptions {
  /// UUID namespace for the records' deterministic GUIDs
  /// (`uuid5(ns, "imported/ifc/<kind>/<GlobalId>")`). Empty = a fixed
  /// ReUseX namespace; import_ifc_file() passes the project GUID.
  std::string guid_namespace;
  /// Height of the top storey (no storey above it) [m].
  double default_storey_height_m = 3.0;
  /// Walls thinner than this become no wall record (mesh only) [m].
  double min_wall_thickness_m = 0.05;
  /// A wall is straight when its footprint's convex hull fills at least
  /// this share of its minimum-area bounding rectangle.
  double min_wall_fill = 0.75;
  /// How far an opening's centre may lie outside its host wall's faces
  /// when the host is found geometrically [m].
  double opening_host_tolerance_m = 0.30;
  /// Recentre a model whose plan centre lies farther than
  /// recenter_threshold_m from the origin by a whole-km x/y offset
  /// (IfcImportStats::world_offset_m; world = file - offset), so the float
  /// meshes keep mm precision (STANDARDS §4) -- IFC files are often
  /// georeferenced (national grid, 1e5..1e7 m). Beyond the threshold a warn
  /// is logged either way. As the Revit import.
  bool recenter = true;
  double recenter_threshold_m = 1e4;
  /// Build the per-class meshes.
  bool meshes = true;
};

struct IfcImportStats {
  std::string schema;         ///< FILE_SCHEMA of the header, e.g. "IFC4"
  double length_unit_m = 1.0; ///< metres per file length unit
  /// World = file coordinates - world_offset_m (x/y only; zero unless the
  /// model was recentred).
  std::array<double, 3> world_offset_m{0.0, 0.0, 0.0};
  std::size_t products = 0; ///< products read (spatial ones excluded)
  std::size_t products_no_geometry = 0;
  /// Meshed items of non-Body representations left out (e.g. 'Clearance').
  std::size_t non_body_geometries = 0;
  /// Errors web-ifc logged while meshing (parse errors make import_ifc
  /// throw instead).
  std::size_t geometry_errors = 0;
  std::size_t storeys = 0;
  std::size_t walls = 0;              ///< wall records
  std::size_t walls_thin = 0;         ///< below min_wall_thickness_m
  std::size_t walls_not_straight = 0; ///< footprint not a rectangle
  std::size_t walls_no_geometry = 0;
  std::size_t openings = 0;
  std::size_t openings_by_relation = 0; ///< host via fills / voids
  std::size_t openings_by_geometry = 0; ///< host found geometrically
  std::size_t openings_unhosted = 0;
  /// s or z from the void it fills / OverallWidth / OverallHeight
  std::size_t openings_extent_params = 0;
  std::size_t openings_extent_geometry = 0; ///< both from its geometry
  std::size_t spaces = 0;
  std::size_t spaces_from_footprint = 0; ///< no body: FootPrint (+ Box)
  std::size_t spaces_no_geometry = 0;    ///< neither body nor footprint
  std::size_t spaces_bad_outline = 0;
  std::size_t space_pieces_dropped = 0;  ///< disjoint outline pieces left out
  std::size_t space_islands_dropped = 0; ///< rings inside a hole left out
  std::size_t elements_no_storey = 0;    ///< no storey via the spatial tree
  std::map<std::string, std::size_t> classes; ///< products per IFC class
};

struct IfcImport {
  core::SolidModel model; ///< source / locked as "detected"; see mark_imported
  std::vector<ImportedMesh> meshes; ///< sorted by name
  IfcImportStats stats;
};

/// True when reusex_io was built with web-ifc (REUSEX_HAVE_WEB_IFC).
bool has_ifc_support();

/// Read and map an IFC file. Pure apart from reading the file: nothing is
/// stored. Records carry their IFC identity in `metadata` (JSON: GlobalId,
/// class, Name, LongName / ObjectType, storey, ...). Every skipped element
/// kind is counted in the stats and logged at warn with its count.
/// @throws std::runtime_error when built without web-ifc, when the file
///         cannot be read, is not a STEP file, is truncated (no ENDSEC /
///         END-ISO-10303-21), web-ifc reports parse errors, or it has no
///         IfcProject / IfcBuildingStorey.
IfcImport import_ifc(const std::filesystem::path &file,
                     const IfcImportOptions &options = {});

/// The mesh slug of an IFC class: "Ifc" dropped, StandardCase /
/// ElementedCase folded into the base class, CamelCase -> snake_case
/// ("IfcWallStandardCase" -> "wall", "IfcFlowTerminal" -> "flow_terminal").
std::string ifc_class_slug(const std::string &ifc_class);

struct IfcFileImportResult {
  IfcImportStats stats;
  SolidModelStoreResult stored;
  std::size_t meshes_saved = 0;
  std::size_t meshes_removed = 0;      ///< stale imported meshes deleted
  std::vector<std::string> mesh_names; ///< the meshes saved, sorted
};

/// The `import_ifc` stage: map @p file with import_ifc() (guid_namespace =
/// the project GUID), store the records as imported (io::mark_imported +
/// io::store_solid_model) and the class meshes as `imported/ifc/<slug>` (stage
/// "import_ifc"; earlier meshes of that stage this import no longer has are
/// deleted; with options.meshes false no mesh is written or deleted). One
/// transaction, one pipeline_log row ("import_ifc").
///
/// @p log_id >= 0 is a pipeline_log row the CALLER has already opened and
/// will close (pipeline::run_logged_stage, the `import_ifc` job stage): no
/// row is opened or closed here then. -1 (the default) opens and closes an
/// own row.
/// @throws std::runtime_error as import_ifc(); no record or mesh is
///         written then, and the pipeline_log row ends failed with the
///         message.
IfcFileImportResult import_ifc_file(ProjectDB &db,
                                    const std::filesystem::path &file,
                                    const IfcImportOptions &options = {},
                                    int log_id = -1);

} // namespace reusex::io::ifc
