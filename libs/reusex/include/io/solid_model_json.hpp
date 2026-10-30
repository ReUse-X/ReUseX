// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The solid-model JSON document (docs/design/solid-model-json.md, schema
// docs/design/solid-model-reference.schema.json).
//
// One format for both sides of the benchmark: a model detected by the
// pipeline (`load_solid_model`) and a reference plan authored by a person or
// an agent are the same document, so `rux analyze solid` compares like with
// like.

#include <reusex/core/solid_model.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
} // namespace reusex

namespace reusex::io {

/// The `model_version` this build reads and writes.
inline constexpr int kSolidModelJsonVersion = 1;

/// Serialise a model. Keys come out in a fixed order and every element array
/// the model provides (`SolidModel::has_*`) is written, so the output is
/// deterministic and a "provided but empty" kind survives a round trip.
///
/// @throws std::invalid_argument when a record's `metadata` is not JSON text
///         (the reader always stores JSON text there).
std::string solid_model_to_json(const core::SolidModel &model, int indent = 2);

/// Parse a document.
///
/// Strict: an unknown key, a wrong type, a non-finite number, an integer
/// outside `int`, a dangling reference (an opening whose `wall` names no
/// wall), a duplicate guid / storey idx, a zero-length wall axis, and a space
/// polygon that is not a valid polygon with holes (a ring with < 3 points,
/// self-intersecting or of zero area; a hole not strictly inside the outer
/// ring, or overlapping another hole) all throw. A ring whose last point
/// repeats its first is accepted and stored open (the point dropped).
///
/// @throws std::invalid_argument naming the offending element, e.g.
///         "walls[2] (guid 'w3'): axis has zero length".
core::SolidModel solid_model_from_json(std::string_view text);

/// Serialise a model as YAML (block style, the same fixed key order as
/// `solid_model_to_json`): a human-friendly front end over the same document,
/// meant for a hand-authored reference plan. Comments are not written back
/// (there is nothing in `core::SolidModel` to hold them).
std::string solid_model_to_yaml(const core::SolidModel &model);

/// Parse a YAML document: converted to a `nlohmann::json` value first (plain
/// scalars are typed as null / bool / integer / float / string the way YAML's
/// core schema would; anything explicitly quoted is always a string), then
/// read by the exact same strict validator as `solid_model_from_json` --
/// there is one validator, so an invalid document throws the same
/// element-naming `std::invalid_argument` either way. Comments are allowed
/// (yaml-cpp drops them while parsing).
///
/// @throws std::invalid_argument naming the offending element, or
///         ("document": ...) when the text is not valid YAML.
core::SolidModel solid_model_from_yaml(std::string_view text);

/// Read and parse a document file: `solid_model_from_yaml` for a `.yaml` /
/// `.yml` path (case-insensitive), `solid_model_from_json` for everything
/// else.
///
/// @throws std::runtime_error naming the path when the file cannot be read.
/// @throws std::invalid_argument when its content is not a valid document.
core::SolidModel solid_model_from_file(const std::filesystem::path &path);

/// Write @p model to @p path, creating parent directories:
/// `solid_model_to_yaml` for a `.yaml` / `.yml` path (case-insensitive),
/// `solid_model_to_json` (plus a final newline) for everything else.
///
/// @throws std::runtime_error naming the path when it cannot be written.
void solid_model_to_file(const core::SolidModel &model,
                         const std::filesystem::path &path, int indent = 2);

/// Mark every storey, wall, space and opening as `source` "imported",
/// `locked` — what an authored reference becomes when it is imported into a
/// project, so no re-run of a detecting stage overwrites it.
void mark_imported(core::SolidModel &model);

/// What store_solid_model() wrote (ProjectDB::SolidModelImportResult).
struct SolidModelStoreResult {
  std::size_t storeys = 0, walls = 0, spaces = 0, openings = 0; ///< rows
  /// Previously imported rows (source "imported") the store deleted.
  std::size_t replaced_imported = 0;
  /// Unlocked detected / manual rows a record replaced (same guid; a
  /// storey: same guid or idx).
  std::size_t taken_over = 0;
  /// Records not written: a locked detected / manual row holds their guid
  /// (a storey: guid or idx).
  std::size_t skipped_locked = 0;
  /// Wall joins / space boundaries that went with a taken-over row.
  std::size_t dropped_relations = 0;
  /// Imported wall joins not written: both walls held back by locked rows.
  std::size_t skipped_joins = 0;
  /// Parts of the model no table holds for an import, so they were not
  /// stored: "plane_classes" (they belong to a detected cloud's planes) and
  /// "scene_relations", in that order. Storeys, walls, spaces and openings
  /// are always stored.
  std::vector<std::string> not_stored;

  [[nodiscard]] std::size_t stored() const {
    return storeys + walls + spaces + openings;
  }
};

/// Write @p model's elements into the project's solid-model tables, the
/// inverse of load_solid_model() (`rux edit plan --import`; the shared
/// writer for model importers).
///
/// Records are written as the model gives them; call mark_imported() first
/// to store them as source "imported", locked. For each element kind the
/// model provides (SolidModel::has_*; wall joins go with walls, space
/// boundaries with spaces) the project's earlier imported rows of that kind
/// are replaced, so a re-import does not pile up; detected and manual rows
/// stay, except that an imported record takes over an UNLOCKED row holding
/// its guid (a storey: its guid or idx) and gives way to a LOCKED one. Kinds
/// the model does not provide are left alone.
///
/// A storey, wall, space or opening with no guid (the document allows that
/// where nothing refers to the element) gets a deterministic one,
/// `core::deterministic_guid(project_guid, <natural key>)`, so a re-import
/// of the same document writes the same rows and reordering, inserting or
/// deleting other elements does not change it (STANDARDS §3, §6):
///  - storey: `imported/storeys/<idx>`;
///  - wall: `imported/wall/<storey>/<ax>,<ay>;<bx>,<by>/t<thickness>`, the
///    axis endpoints on a 1 cm grid in either direction (sorted, so a
///    reversed axis is the same wall), the thickness in mm;
///  - space: `imported/space/<storey>/<cx>,<cy>/a<area>`, the footprint's
///    area centroid (holes subtracted) on a 1 cm grid and its area in
///    0.1 m^2 buckets;
///  - opening: `imported/opening/<host wall guid>/<s0>,<s1>/<kind>`, the
///    extent along the host axis on a 1 cm grid.
/// Moving, resizing or re-hosting an element changes its guid (it is a
/// different element by these keys); nothing else does. Elements that share
/// a key are told apart by every other field they carry (full precision),
/// in that sorted order: the first keeps the key, the next get `#2`, `#3`,
/// ... -- never by their position in the document. Give elements a guid
/// when they must keep their identity across edits that move them. All
/// in one SAVEPOINT; the rules, and what throws, are
/// ProjectDB::replace_imported_solid_model's.
///
/// For a model whose records are imported and locked, and a project with no
/// other rows on the same guids / idx,
/// `load_solid_model(db)` gives back the model's storeys, walls, joins,
/// openings, spaces and boundaries unchanged: `JSON -> tables -> JSON` is
/// lossless up to the order the tables list records in (and record ids,
/// which the document does not carry).
///
/// @throws std::invalid_argument for a record the tables refuse, or an
///         opening / space boundary naming a wall the project does not
///         hold; std::runtime_error on a read-only project.
///
/// @p run_id is the pipeline_log row the import belongs to, stored on every
/// written row (-1 = none); import_plan() and the model importers pass
/// theirs.
SolidModelStoreResult store_solid_model(ProjectDB &db,
                                        const core::SolidModel &model,
                                        int run_id = -1);

/// One line naming what store_solid_model() wrote, for a log line and the
/// import's pipeline_log row: "imported S storeys, W walls, P spaces, O
/// openings; replaced R earlier imported row(s)", plus the rows taken over,
/// held back by locked rows and the parts not stored, when there are any.
std::string store_summary(const SolidModelStoreResult &r);

/// `rux edit plan --import`: store_solid_model() under its own pipeline_log
/// row, stage "import_plan", parameters `{"source": "plan", "file":
/// <file name of @p source_file>}`. The row closes `success` with
/// store_summary() as its message, or `failed` with the exception's message
/// when the project refuses the model (nothing is stored then, see
/// store_solid_model()). Call mark_imported() on @p model first.
///
/// @p log_id >= 0 is a pipeline_log row the CALLER has already opened and
/// will close (as io::ifc::import_ifc_file): no row is opened or closed
/// here and the stored rows carry @p log_id.
///
/// @throws what store_solid_model() throws, after closing the row.
SolidModelStoreResult import_plan(ProjectDB &db, const core::SolidModel &model,
                                  const std::filesystem::path &source_file,
                                  int log_id = -1);

/// The typed building model stored in a project: storeys, plane classes
/// (of the "planes" cloud), walls and joins, openings, spaces and space
/// boundaries, from their ProjectDB tables -- detected, manual and imported
/// rows alike. A kind counts as provided when its table has rows; an empty
/// table reads as "not provided" (n/a in the benchmark), since it cannot
/// tell "the stage has not run" from "it found nothing". An opening or
/// space boundary naming a wall the walls table no longer holds is left out
/// (logged at warn).
core::SolidModel load_solid_model(const ProjectDB &db);

} // namespace reusex::io
