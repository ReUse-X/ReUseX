// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "edit/plan.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <stdexcept>

namespace io = reusex::io;

namespace {

std::string summary(const reusex::core::SolidModel &m) {
  return fmt::format("{} storeys, {} walls, {} spaces, {} openings",
                     m.storeys.size(), m.walls.size(), m.spaces.size(),
                     m.openings.size());
}

} // namespace

void setup_subcommand_edit_plan(CLI::App &app,
                                std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandEditPlanOptions>();
  auto *sub = app.add_subcommand(
      "plan", "Import or export the solid-model plan (walls, spaces, openings) "
              "as JSON or YAML");

  sub->footer(R"(
DESCRIPTION:
  Moves a solid-model plan between a document (docs/design/solid-model-json.md,
  JSON or YAML -- picked by extension: .yaml/.yml is YAML, anything else
  JSON) and the project. --import reads a plan strictly (an unknown key, a
  dangling reference or an invalid polygon is an error naming the element),
  marks every element source "imported" and locked, and stores it; --export
  writes the project's model, in the format --export's own extension picks.

  An import replaces the project's earlier imported storeys, walls, spaces
  and openings (each kind the plan provides) and leaves detected and manual
  rows alone, except that an imported element takes over an unlocked row
  with its guid (a storey: its guid or idx) and gives way to a locked one;
  both are logged. Each import is recorded in the pipeline log (stage
  import_plan, see rux log) with what it wrote, or why the project refused
  it; a plan that does not parse never reaches the project. Plane classes
  and scene relations in a plan are not stored. Elements without a guid
  get one from where they stand and how big they are (not from their
  position in the plan), so reordering a plan keeps every guid. Use --dry-run to validate a plan and write it back normalised
  without touching the project; the authoring loop that uses it is
  docs/guides/solid-model-reference-authoring.md.

EXAMPLES:
  # Validate an authored plan and write it back in canonical form
  rux edit plan --import ref.plan.json --dry-run --export ref.plan.json

  # Import a reference into the project
  rux -p scan.rux edit plan --import ref.plan.yaml

  # Export the project's model
  rux -p scan.rux edit plan --export model.json
)");

  sub->add_option("--import", opt->import_path,
                  "Solid-model plan (JSON or YAML) to import")
      ->check(CLI::ExistingFile);
  sub->add_option("--export", opt->export_path,
                  "Write the solid-model plan here (JSON or YAML by "
                  "extension; the project's model, or the imported one with "
                  "--dry-run)");
  sub->add_flag("--dry-run", opt->dry_run,
                "Validate --import without opening the project");

  sub->callback([opt, global_opt]() {
    spdlog::trace("Running edit plan subcommand");
    rux::finish(run_subcommand_edit_plan(*opt, *global_opt));
  });
}

int run_subcommand_edit_plan(SubcommandEditPlanOptions const &opt,
                             const RuxOptions &global_opt) {
  if (opt.import_path.empty() && opt.export_path.empty()) {
    spdlog::error("edit plan: pass --import <plan.json>, --export "
                  "<plan.json>, or both");
    return RuxError::INVALID_ARGUMENT;
  }
  if (opt.dry_run && opt.import_path.empty()) {
    spdlog::error("edit plan: --dry-run validates an --import; none given");
    return RuxError::INVALID_ARGUMENT;
  }

  try {
    if (!opt.import_path.empty()) {
      reusex::core::SolidModel model;
      try {
        model = io::solid_model_from_file(opt.import_path);
      } catch (const std::invalid_argument &e) {
        spdlog::error("edit plan: invalid plan '{}': {}",
                      opt.import_path.string(), e.what());
        return RuxError::INVALID_ARGUMENT;
      }
      io::mark_imported(model);
      spdlog::info("edit plan: '{}' is valid: {}", opt.import_path.string(),
                   summary(model));

      if (opt.dry_run) {
        if (!opt.export_path.empty()) {
          io::solid_model_to_file(model, opt.export_path);
          spdlog::info("edit plan: wrote the imported plan to '{}'",
                       opt.export_path.string());
        }
        return RuxError::SUCCESS;
      }

      reusex::ProjectDB db(global_opt.project_db);
      io::SolidModelStoreResult stored;
      try {
        // One pipeline_log row, stage "import_plan": success with the
        // counts, or failed with the reason the project refused the plan.
        stored = io::import_plan(db, model, opt.import_path);
      } catch (const std::invalid_argument &e) {
        spdlog::error("edit plan: the project refused plan '{}': {}",
                      opt.import_path.string(), e.what());
        return RuxError::INVALID_ARGUMENT;
      }
      if (!stored.not_stored.empty())
        spdlog::warn("edit plan: not stored (no project table holds them for "
                     "an import): {}",
                     fmt::join(stored.not_stored, ", "));
      if (stored.stored() == 0 && stored.replaced_imported == 0)
        spdlog::warn("edit plan: nothing was imported into '{}': the plan has "
                     "no storey, wall, space or opening the project could "
                     "take ({} held back by locked rows)",
                     global_opt.project_db.string(), stored.skipped_locked);
      spdlog::info("edit plan: {}", io::store_summary(stored));
    }

    if (!opt.export_path.empty()) {
      const reusex::ProjectDB db(global_opt.project_db, /* readOnly */ true);
      const auto model = io::load_solid_model(db);
      if (!model.has_storeys() && !model.has_walls() && !model.has_spaces() &&
          !model.has_openings())
        spdlog::warn("edit plan: '{}' has no solid-model element yet (no "
                     "storeys, walls, spaces or openings table); the export "
                     "is an empty document",
                     global_opt.project_db.string());
      io::solid_model_to_file(model, opt.export_path);
      spdlog::info("edit plan: wrote {} to '{}'", summary(model),
                   opt.export_path.string());
    }
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("edit plan failed: {}", e.what());
    return RuxError::IO;
  }
}
