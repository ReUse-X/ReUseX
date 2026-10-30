// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/speckle.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <spdlog/spdlog.h>

#include <exception>

void setup_subcommand_import_speckle(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportSpeckleOptions>();
  auto *sub = app.add_subcommand(
      "speckle", "Import a Revit model from Speckle (read-only) as the "
                 "project's imported storeys, walls, openings, spaces and "
                 "category meshes");

  sub->footer(R"(
DESCRIPTION:
  Fetches one version of a Speckle model published by the Revit connector
  (Speckle v3) with GraphQL queries and HTTP GETs only -- nothing is ever
  written to the server -- and stores it in the project:
    - Revit levels  -> storeys (one shared level frame, lowest level at z = 0)
    - Walls         -> walls (axis + thickness from a straight footprint)
    - Doors/Windows -> openings on their host wall
    - placed Rooms  -> spaces (footprint polygon from the room volume)
    - every category's geometry -> meshes 'imported/<category>'
  Records are stored as source = imported, locked; a re-import replaces the
  earlier imported rows (docs/CONTRACTS.md, "Import write: solid-model
  plan"). Coordinates stay in Revit's internal frame, in metres; aligning
  the model to a scan is not done here.

  The access token is read from SPECKLE_TOKEN.

EXAMPLES:
  SPECKLE_TOKEN=... rux -p house.rux import speckle \
      --project 63142b1e80 --model c15702ceaf
  rux -p house.rux import speckle --project P --model M --version V
  rux -p house.rux import speckle --project P --model M --no-meshes
)");

  const reusex::io::speckle::SpeckleRevitImportOptions defaults;
  sub->add_option("--server", opt->import.server_url, "Speckle server URL")
      ->default_val(defaults.server_url);
  sub->add_option("--project", opt->import.project, "Speckle project id")
      ->required();
  sub->add_option("--model", opt->import.model, "Speckle model id")->required();
  sub->add_option("--version", opt->import.version,
                  "Model version id (default: the latest)");
  sub->add_flag("--no-meshes", opt->no_meshes,
                "Store only the solid-model records, no category meshes");
  sub->add_option("--storey-height",
                  opt->import.mapping.default_storey_height_m,
                  "Height of the top storey [m]")
      ->default_val(defaults.mapping.default_storey_height_m);
  sub->add_option("--min-wall-thickness",
                  opt->import.mapping.min_wall_thickness_m,
                  "Thinner walls (finish layers) get no wall record [m]")
      ->default_val(defaults.mapping.min_wall_thickness_m);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_speckle");
    rux::finish(run_subcommand_import_speckle(*opt, *global_opt));
  });
}

int run_subcommand_import_speckle(SubcommandImportSpeckleOptions const &opt,
                                  const RuxOptions &global_opt) {
  auto import = opt.import;
  import.mapping.meshes = !opt.no_meshes;
  try {
    reusex::ProjectDB db(global_opt.project_db);
    const auto r = reusex::io::speckle::import_speckle_revit(db, import);
    const auto &s = r.stats;
    spdlog::info("Imported version {} ({} elements): {} storeys, {} walls, "
                 "{} openings, {} spaces, {} meshes",
                 r.version.id, s.elements, r.stored.storeys, r.stored.walls,
                 r.stored.openings, r.stored.spaces, r.meshes_saved);
    spdlog::info("Skipped: {} thin walls, {} non-straight walls, {} unhosted "
                 "doors/windows, {} unplaced rooms; replaced {} earlier "
                 "imported rows",
                 s.walls_thin, s.walls_not_straight, s.openings_unhosted,
                 s.rooms_unplaced, r.stored.replaced_imported);
    if (r.stored.skipped_locked > 0)
      spdlog::warn("{} records were not written: a locked detected / manual "
                   "row holds their guid or storey idx",
                   r.stored.skipped_locked);
  } catch (const std::exception &e) {
    spdlog::error("import speckle: {}", e.what());
    return RuxError::GENERIC;
  }
  return RuxError::SUCCESS;
}
