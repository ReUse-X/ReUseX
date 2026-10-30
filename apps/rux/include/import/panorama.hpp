// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <vector>

namespace fs = std::filesystem;

struct SubcommandImportPanoramaOptions {
  std::vector<fs::path> input_paths; // directories or individual files
  bool no_stitch =
      false; // --no-stitch: skip .insp stitching, import equirects only
  /// --pose x,y,z[,yaw_deg]: register ONE panorama placed by hand
  /// (io::register_panorama, pose_source manual; no timestamp matching).
  std::vector<double> pose;
};

void setup_subcommand_import_panorama(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_panorama(SubcommandImportPanoramaOptions const &opt,
                                   const RuxOptions &global_opt);
