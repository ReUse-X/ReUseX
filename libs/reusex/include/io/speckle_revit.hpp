// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/core/solid_model.hpp"
#include "reusex/io/solid_model_json.hpp"
#include "reusex/io/speckle_reader.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

/// @file
/// Revit model (received from Speckle) -> ReUseX solid model + meshes.
///
/// Reads the object tree a Speckle v3 Revit connector publishes
/// (`Objects.Data.DataObject:Objects.Data.RevitObject` elements in nested
/// collections, `levelProxies`, `instanceDefinitionProxies`), already
/// resolved by `ObjectTree::resolve_root()`, and maps it to:
///  - storeys (one per Revit level), walls (Walls elements with a straight
///    footprint), openings (Doors / Windows, hosted on a wall) and spaces
///    (placed Rooms), as `core::SolidModel` records in the conventions the
///    detecting stages use (docs/design/solid-model-json.md);
///  - one triangle mesh per Revit category (Rooms excepted), for display.
///
/// Read-only: nothing here, nor in `import_speckle_revit`, writes to a
/// Speckle server (only `SpeckleReader` is used).
///
/// Frame. World is Revit's internal coordinate system, in metres, as the
/// connector writes mesh vertices; nothing is georeferenced or aligned to a
/// scan (that is out of scope). Level elevations arrive relative to the
/// level's elevation base (often the survey point), so their offset to the
/// internal z is measured from the walls (base constraint + base offset vs
/// mesh bottom; `RevitImportStats::elevation_offset_m`). All storeys share
/// one level frame, as detected storeys do: a translation putting the lowest
/// level's floor at z = 0, so storey k's `z_floor` is its height above the
/// lowest level, and walls / openings / spaces carry level-frame z.

namespace reusex {
class ProjectDB;
}

namespace reusex::io::speckle {

struct RevitImportOptions {
  /// UUID namespace for the records' deterministic GUIDs
  /// (`uuid5(ns, "imported/revit/<kind>/<Revit applicationId>")`). Empty =
  /// a fixed ReUseX namespace; `import_speckle_revit` passes the project
  /// GUID, as every other writer does.
  std::string guid_namespace;
  /// Height of the top storey (no level above it) [m].
  double default_storey_height_m = 3.0;
  /// Walls thinner than this (finish layers, linings modelled as walls)
  /// become no wall record; their geometry stays in the walls mesh [m].
  double min_wall_thickness_m = 0.05;
  /// A wall is straight when its footprint's convex hull fills at least
  /// this share of its minimum-area bounding rectangle.
  double min_wall_fill = 0.75;
  /// How far an opening's centre may lie outside its host wall's faces
  /// when the host is found geometrically [m].
  double opening_host_tolerance_m = 0.30;
  /// Revit internal z minus level elevation [m]. Unset = measured from the
  /// walls (falls back to 0 with a warn when no wall can tell).
  std::optional<double> elevation_offset_m;
  /// Build the per-category meshes.
  bool meshes = true;
};

/// One merged triangle mesh (world frame, metres).
struct RevitMesh {
  std::string name;     ///< "imported/<category slug>"
  std::string category; ///< Revit category, as received
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<std::uint32_t, 3>> triangles;
  std::size_t elements = 0; ///< elements that contributed geometry
};

struct RevitImportStats {
  std::size_t elements = 0; ///< RevitObjects seen
  std::size_t elements_no_geometry = 0;
  std::size_t instance_proxies = 0;    ///< InstanceProxies expanded
  std::size_t instance_unresolved = 0; ///< proxies whose definition is missing
  std::size_t levels = 0;
  /// Revit internal z minus level elevation [m], and how it was found:
  /// "option" | "walls (N of M agree)" | "assumed 0".
  double elevation_offset_m = 0.0;
  std::string elevation_offset_source;
  std::size_t walls = 0;              ///< wall records
  std::size_t walls_thin = 0;         ///< below min_wall_thickness_m
  std::size_t walls_not_straight = 0; ///< footprint not a rectangle
  std::size_t walls_no_geometry = 0;
  std::size_t openings = 0;
  std::size_t openings_by_parent = 0;   ///< host from parentApplicationId
  std::size_t openings_by_geometry = 0; ///< host found geometrically
  std::size_t openings_unhosted = 0;
  std::size_t spaces = 0;
  std::size_t rooms_unplaced = 0; ///< Rooms without a volume mesh
  std::vector<std::string> unplaced_room_names;
  std::size_t rooms_bad_outline = 0;  ///< volume mesh gave no footprint
  std::size_t elements_no_storey = 0; ///< level unknown, placed by height
  std::map<std::string, std::size_t> categories; ///< elements per category
};

struct RevitImport {
  core::SolidModel model; ///< source / locked as "detected"; see mark_imported
  std::vector<RevitMesh> meshes; ///< sorted by name
  RevitImportStats stats;
};

/// Map a resolved Revit object tree (`ObjectTree::resolve_root()`) to a
/// solid model and per-category meshes. Pure: no I/O. Records carry their
/// Revit identity in `metadata` (JSON: applicationId, elementId, category,
/// family, type, level, IfcGUID, ...). Every skipped element kind is counted
/// in the stats and logged at warn with its count.
/// @throws std::runtime_error when the root has no levelProxies (not a
///         Revit model, or a connector this importer does not know).
RevitImport import_revit(const nlohmann::json &resolved_root,
                         const RevitImportOptions &options = {});

/// The ASCII slug of a category name used in mesh names
/// ("Plumbing Fixtures" -> "plumbing_fixtures").
std::string category_slug(const std::string &category);

struct SpeckleRevitImportOptions {
  std::string server_url = "https://app.speckle.systems";
  std::string project;
  std::string model;
  std::string version;        ///< empty = the model's latest version
  std::string token;          ///< empty = SPECKLE_TOKEN
  RevitImportOptions mapping; ///< guid_namespace is set from the project
};

struct SpeckleRevitImportResult {
  VersionInfo version;
  RevitImportStats stats;
  SolidModelStoreResult stored;
  std::size_t meshes_saved = 0;
  std::size_t meshes_removed = 0; ///< stale imported meshes deleted
};

/// `rux import speckle`: fetch a Revit model version from Speckle (read
/// only), map it with import_revit(), store the records as imported
/// (io::mark_imported + io::store_solid_model) and the category meshes as
/// `imported/<slug>` (stage "import_speckle"; earlier meshes of that stage
/// that this import no longer has are deleted). Logs one pipeline_log row
/// ("import_speckle").
/// @throws std::runtime_error on a network / server error, a version that
///         does not exist, or a tree that is not a Revit model.
SpeckleRevitImportResult
import_speckle_revit(ProjectDB &db, const SpeckleRevitImportOptions &options);

} // namespace reusex::io::speckle
