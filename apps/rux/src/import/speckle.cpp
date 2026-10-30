// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/speckle.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "import/stage_flags.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
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
    - every category's geometry -> meshes 'imported/revit/<category>'
  Records are stored as source = imported, locked; a re-import replaces the
  earlier imported rows (docs/CONTRACTS.md, "Import write: solid-model
  plan"). Coordinates stay in Revit's internal frame, in metres; aligning
  the model to a scan is not done here.

  The access token is read from SPECKLE_TOKEN; it is never a parameter and
  never written to the project (pipeline_log records the flags below). The
  same import runs as the `import_speckle` job stage (ruxd), with the same
  parameters.

EXAMPLES:
  SPECKLE_TOKEN=... rux -p house.rux import speckle \
      --project 63142b1e80 --model c15702ceaf
  rux -p house.rux import speckle --project P --model M --version V
  rux -p house.rux import speckle --project P --model M --no-meshes
  rux -p house.rux import speckle --project P --model M \
      --elevation-offset -20.82

NOTES:
  - Only levels flagged as building storeys become storeys; elements on
    other (reference) levels go to the storey at or below them.
  - Door / window extents come from the Revit rough opening (else nominal
    width / height, else the geometry).
  - --no-meshes writes no mesh and deletes none.
  - --project and --model are required; without them the stage refuses.
)");

  sub->get_formatter()->column_width(40);
  rux::add_import_stage_flags(*sub, opt->stage,
                              reusex::pipeline::speckle_parameter_table());

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_speckle");
    if (rux::capture_stage_params("import_speckle",
                                  speckle_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_import_speckle(*opt, *global_opt));
  });
}

std::string
speckle_stage_parameters(SubcommandImportSpeckleOptions const &opt) {
  return reusex::pipeline::speckle_parameters_json(opt.stage);
}

int run_subcommand_import_speckle(SubcommandImportSpeckleOptions const &opt,
                                  const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);
    reusex::pipeline::StageContext ctx;
    ctx.project = global_opt.project_db;
    ctx.stage = reusex::pipeline::JobStage::import_speckle;
    ctx.parameters = speckle_stage_parameters(opt);
    // Default executor options: the token comes from SPECKLE_TOKEN.
    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Imported {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("import speckle: {}", e.what());
    return RuxError::GENERIC;
  }
}
