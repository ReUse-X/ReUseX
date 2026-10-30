// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/io/ifc.hpp>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

struct SubcommandImportIfcOptions {
  std::filesystem::path file;
  reusex::io::ifc::IfcImportOptions import;
  bool no_meshes = false;
};

void setup_subcommand_import_ifc(CLI::App &app,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_ifc(SubcommandImportIfcOptions const &opt,
                              const RuxOptions &global_opt);
