// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/ifc.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "import/stage_flags.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <spdlog/spdlog.h>

#include <exception>
#include <filesystem>

void setup_subcommand_import_ifc(CLI::App &app,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportIfcOptions>();
  auto *sub = app.add_subcommand(
      "ifc", "Import an IFC building model as the project's imported "
             "storeys, walls, openings, spaces and per-class meshes");

  sub->footer(R"(
DESCRIPTION:
  Reads an IFC file (IFC2X3 / IFC4 / IFC4X3, STEP) with web-ifc and stores
  it in the project:
    - IfcBuildingStorey -> storeys (one shared level frame, lowest at z = 0)
    - IfcWall*          -> walls (axis + thickness from a straight footprint)
    - IfcDoor/IfcWindow -> openings on their host wall (via the
                           IfcOpeningElement they fill, else geometrically)
    - IfcSpace          -> spaces (footprint polygon from the space body)
    - every product's geometry -> meshes 'imported/ifc/<class>' (e.g.
                           imported/ifc/wall, imported/ifc/slab, ...)
  Records are stored as source = imported, locked; a re-import replaces the
  earlier imported rows (docs/CONTRACTS.md, "Import write: IFC model").
  Coordinates stay in the file's world frame, in metres; a model whose plan
  centre lies beyond --recenter-threshold (georeferenced files) is moved by
  a whole-km x/y offset, kept as world_offset_m in every storey's metadata
  and mesh's parameters (world = file - offset).

NOTES:
  - Spaces without a body take their outline from the 'FootPrint'
    representation and their height from the 'Box'.
  - Door / window extents come from the IfcOpeningElement they fill (else
    OverallWidth / OverallHeight, else the geometry).
  - --no-meshes writes no mesh and deletes none.
  - The same import runs as the `import_ifc` job stage (ruxd), with the
    same parameters; there a relative path is read against the project's
    directory.

EXAMPLES:
  rux -p house.rux import ifc house.ifc
  rux -p house.rux import ifc site.ifc --recenter-threshold 1000
  rux -p house.rux import ifc house.ifc --no-meshes
)");

  // `path` is also the positional FILE, as before.
  sub->get_formatter()->column_width(40);
  rux::add_import_stage_flags(*sub, opt->stage,
                              reusex::pipeline::ifc_parameter_table(), "path",
                              "file");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_ifc");
    if (rux::capture_stage_params("import_ifc", ifc_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_import_ifc(*opt, *global_opt));
  });
}

std::string ifc_stage_parameters(SubcommandImportIfcOptions const &opt) {
  auto stage = opt.stage;
  // On the command line a relative FILE means the working directory; the
  // stage reads a relative path against the project's directory.
  if (!stage.path.empty())
    stage.path =
        std::filesystem::absolute(stage.path).lexically_normal().string();
  return reusex::pipeline::ifc_parameters_json(stage);
}

int run_subcommand_import_ifc(SubcommandImportIfcOptions const &opt,
                              const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);
    reusex::pipeline::StageContext ctx;
    ctx.project = global_opt.project_db;
    ctx.stage = reusex::pipeline::JobStage::import_ifc;
    ctx.parameters = ifc_stage_parameters(opt);
    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Imported {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("import ifc: {}", e.what());
    return RuxError::GENERIC;
  }
}
