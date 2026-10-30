// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The BIM import stages' parameters (reusex-3iy8.13), one table per stage in
// the pattern of pipeline/space_parameters.hpp: the stage parser, the
// stage_parameters() descriptors, the `rux import speckle` / `rux import ifc`
// flags and the Qt client's "Kopiér som rux-kommando" spec all come from
// speckle_parameter_table() / ifc_parameter_table(): JSON key, CLI flag,
// label, description, bounds and an accessor into the io options struct the
// stage hands to the importer (io::speckle::SpeckleRevitImportOptions,
// io::ifc::IfcImportOptions), so every default is that struct's (STANDARDS
// §4).
//
// SECRETS: no key carries a credential. The Speckle access token is resolved
// per run by StageExecutorOptions::speckle_token (pipeline/stages.hpp) and is
// never a parameter: a job's parameters are persisted in the job history and
// in `pipeline_log.parameters`. A `token` key is refused (kSecretParameterKey).

#include <reusex/io/ifc.hpp>
#include <reusex/io/speckle_revit.hpp>
#include <reusex/pipeline/stage_parameters.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::pipeline {

/// One parameter of an import stage, over that stage's options struct @p O.
template <typename O> struct ImportParameter {
  using Field =
      std::variant<double &(*)(O &), bool &(*)(O &), std::string &(*)(O &),
                   std::optional<double> &(*)(O &)>;
  std::string_view key;  ///< JSON key
  std::string_view flag; ///< CLI flag; for a boolean, the flag that sets it
                         ///< to the opposite of its default
  std::string_view label;
  std::string_view description;
  Field field;
  std::optional<double> minimum, maximum; ///< inclusive bounds
  /// A job without this key (or with an empty string) is refused as invalid.
  bool required = false;
};

// --- import_speckle ---------------------------------------------------------

/// Everything the import_speckle stage is tuned by. `import.token` is NEVER
/// read from or written to parameters; the stage fills it from the
/// executor's token provider.
struct SpeckleStageOptions {
  io::speckle::SpeckleRevitImportOptions import;
};

using SpeckleParameter = ImportParameter<SpeckleStageOptions>;

/// Every parameter, in form-display order.
const std::vector<SpeckleParameter> &speckle_parameter_table();

/// stage_parameters(JobStage::import_speckle).
const std::vector<ParameterDescriptor> &speckle_stage_parameters();

/// Read the import_speckle stage's JSON parameters ("" = defaults). Unknown
/// keys are ignored.
/// @throws std::invalid_argument for a non-object JSON, a `token` key, a
///         value of the wrong type or outside its bounds, or a missing
///         required key (project, model).
SpeckleStageOptions
speckle_options_from_parameters(std::string_view parameters);

/// The JSON parameters for @p options: every number and boolean, the
/// strings that are set, elevation_offset only when set. Never the token.
std::string speckle_parameters_json(const SpeckleStageOptions &options);

// --- import_ifc -------------------------------------------------------------

/// Everything the import_ifc stage is tuned by.
struct IfcStageOptions {
  /// The IFC file. Absolute, or relative to the directory holding the
  /// project (`.rux`) — the case directory ruxd uploads into; a relative path
  /// may not leave that directory (resolve_ifc_path()).
  std::string path;
  io::ifc::IfcImportOptions import;
};

using IfcParameter = ImportParameter<IfcStageOptions>;

/// Every parameter, in form-display order.
const std::vector<IfcParameter> &ifc_parameter_table();

/// stage_parameters(JobStage::import_ifc).
const std::vector<ParameterDescriptor> &ifc_stage_parameters();

/// Read the import_ifc stage's JSON parameters ("" = defaults). Unknown keys
/// are ignored.
/// @throws std::invalid_argument as speckle_options_from_parameters()
///         (required: path).
IfcStageOptions ifc_options_from_parameters(std::string_view parameters);

/// The JSON parameters for @p options (path only when set).
std::string ifc_parameters_json(const IfcStageOptions &options);

/// The file @p path names for a project at @p project: an absolute path as
/// is, a relative one against the project's directory.
/// @throws std::invalid_argument for an empty path, a relative path that
///         leaves the project's directory (`..`), or one that is not an
///         existing regular file.
std::filesystem::path resolve_ifc_path(const std::filesystem::path &project,
                                       const std::string &path);

} // namespace reusex::pipeline
