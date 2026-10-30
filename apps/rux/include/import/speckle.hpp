// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/io/speckle_revit.hpp>

#include <CLI/CLI.hpp>
#include <memory>

struct SubcommandImportSpeckleOptions {
  reusex::io::speckle::SpeckleRevitImportOptions import;
  bool no_meshes = false;
  /// --elevation-offset given: overrides the measured offset.
  bool elevation_offset_set = false;
  double elevation_offset_m = 0.0;
};

void setup_subcommand_import_speckle(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_speckle(SubcommandImportSpeckleOptions const &opt,
                                  const RuxOptions &global_opt);
