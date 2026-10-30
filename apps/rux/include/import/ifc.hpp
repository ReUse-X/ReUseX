// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/pipeline/import_parameters.hpp>

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

struct SubcommandImportIfcOptions {
  /// Filled from the flags (pipeline::ifc_parameter_table()).
  reusex::pipeline::IfcStageOptions stage;
};

/// The import_ifc stage parameters the flags in @p opt amount to; a
/// relative path is made absolute against the working directory (the stage
/// would read it against the project's directory).
std::string ifc_stage_parameters(SubcommandImportIfcOptions const &opt);

void setup_subcommand_import_ifc(CLI::App &app,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_ifc(SubcommandImportIfcOptions const &opt,
                              const RuxOptions &global_opt);
