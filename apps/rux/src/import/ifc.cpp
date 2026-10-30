// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/ifc.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <spdlog/spdlog.h>

#include <exception>

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
    - every product's geometry -> meshes 'imported/<class>' (e.g.
                           imported/wall, imported/slab, imported/door)
  Records are stored as source = imported, locked; a re-import replaces the
  earlier imported rows (docs/CONTRACTS.md, "Import write: IFC model").
  Coordinates stay in the file's world frame, in metres, unless --recenter
  moves the model's plan centre to the origin (georeferenced files).

EXAMPLES:
  rux -p house.rux import ifc house.ifc
  rux -p house.rux import ifc site.ifc --recenter
  rux -p house.rux import ifc house.ifc --no-meshes
)");

  const reusex::io::ifc::IfcImportOptions defaults;
  sub->add_option("file", opt->file, "IFC file (.ifc)")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_flag("--no-meshes", opt->no_meshes,
                "Store only the solid-model records; existing imported "
                "meshes are left as they are");
  sub->add_flag("--recenter", opt->import.recenter,
                "Translate the model's plan centre (rounded to 1 m) to the "
                "origin; the offset is kept in the metadata");
  sub->add_option("--storey-height", opt->import.default_storey_height_m,
                  "Height of the top storey [m]")
      ->default_val(defaults.default_storey_height_m);
  sub->add_option("--min-wall-thickness", opt->import.min_wall_thickness_m,
                  "Thinner walls (finish layers) get no wall record [m]")
      ->default_val(defaults.min_wall_thickness_m);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_ifc");
    rux::finish(run_subcommand_import_ifc(*opt, *global_opt));
  });
}

int run_subcommand_import_ifc(SubcommandImportIfcOptions const &opt,
                              const RuxOptions &global_opt) {
  if (!reusex::io::ifc::has_ifc_support()) {
    spdlog::error("import ifc: this rux was built without IFC support "
                  "(web-ifc not found at configure time)");
    return RuxError::GENERIC;
  }
  auto import = opt.import;
  import.meshes = !opt.no_meshes;
  try {
    reusex::ProjectDB db(global_opt.project_db);
    const auto r = reusex::io::ifc::import_ifc_file(db, opt.file, import);
    const auto &s = r.stats;
    spdlog::info("Imported {} ({} products): {} storeys, {} walls, "
                 "{} openings, {} spaces, {} meshes",
                 opt.file.filename().string(), s.products, r.stored.storeys,
                 r.stored.walls, r.stored.openings, r.stored.spaces,
                 r.meshes_saved);
    spdlog::info("Skipped: {} thin walls, {} non-straight walls, {} unhosted "
                 "doors/windows, {} spaces without geometry; replaced {} "
                 "earlier imported rows",
                 s.walls_thin, s.walls_not_straight, s.openings_unhosted,
                 s.spaces_no_geometry + s.spaces_bad_outline,
                 r.stored.replaced_imported);
    if (r.stored.skipped_locked > 0)
      spdlog::warn("{} records were not written: a locked detected / manual "
                   "row holds their guid or storey idx",
                   r.stored.skipped_locked);
  } catch (const std::exception &e) {
    spdlog::error("import ifc: {}", e.what());
    return RuxError::GENERIC;
  }
  return RuxError::SUCCESS;
}
