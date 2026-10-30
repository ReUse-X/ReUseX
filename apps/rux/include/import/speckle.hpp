// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/pipeline/import_parameters.hpp>

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

struct SubcommandImportSpeckleOptions {
  /// Filled from the flags (pipeline::speckle_parameter_table()); the token
  /// is never set here — the stage reads SPECKLE_TOKEN.
  reusex::pipeline::SpeckleStageOptions stage;
};

/// The import_speckle stage parameters the flags in @p opt amount to.
std::string speckle_stage_parameters(SubcommandImportSpeckleOptions const &opt);

void setup_subcommand_import_speckle(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_speckle(SubcommandImportSpeckleOptions const &opt,
                                  const RuxOptions &global_opt);
