// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Database-level pipeline stage execution (#265, Phase 1).
//
// The algorithms in segmentation/ and reconstruction/ are pure functions over
// PCL clouds; the "load the named clouds out of a ProjectDB, run the algorithm,
// write the results back, record it in pipeline_log" wrapper used to live only
// inside apps/rux/src/create/*.cpp. This header lifts that wrapper DOWN into
// the library so both `rux create <stage>` and the GUI/daemon job runners drive
// the exact same code path (issue #265: "shared logic goes DOWN into the
// library, not sideways between apps").
//
// Layering: this is the `pipeline` module, which sits ABOVE the Layer-3 peers
// (segmentation / reconstruction / slam / io) — it is the one place allowed to
// link several of them at once. See docs/STANDARDS.md §1.

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::pipeline {

/// A pipeline stage that can be executed as a job.
///
/// Deliberately narrower than core::PipelineStage (which also covers stages
/// with no runner yet) and than core::Stage (which is a progress-reporting
/// phase, not a unit of work). Extending this enum is how a stage becomes
/// GUI-runnable; see `ruxd`'s `docs/gui/openapi.yaml` (in its own repo),
/// which lists the same names.
enum class JobStage {
  clouds,    ///< back-project sensor frames into "cloud" + "normals"
  planes,    ///< detect planar surfaces -> "planes"/"plane_*"
  rooms,     ///< partition planes into rooms -> "rooms"
  instances, ///< split semantic labels into spatial instances
  mesh,      ///< MIP-solve the cell complex into a watertight mesh
  optimize,  ///< refine per-frame sensor poses via plane-landmark pose graph
  annotate,  ///< ML semantic labelling of the stored sensor frames
  graph_discover, ///< find loop edges and store them in the pose graph
  storeys,        ///< detect storeys and the level frame -> "storeys" table
  classify,       ///< classify each plane -> "plane_classes" table
  walls,          ///< walls + their joins -> "walls" / "wall_joins" tables
  openings,       ///< doors, windows, openings in the walls -> "openings"
  spaces,         ///< band rooms as polygons -> "spaces" / "space_boundaries"
  solids,         ///< typed solids -> "solid_walls" / "solid_slabs" /
                  ///< "solid_spaces" meshes
  /// Revit model from Speckle -> imported storeys / walls / openings /
  /// spaces + `imported/revit/<category>` meshes. Needs a Speckle token
  /// from StageExecutorOptions::speckle_token (never a parameter).
  import_speckle,
  /// IFC file -> imported storeys / walls / openings / spaces +
  /// `imported/ifc/<class>` meshes (needs web-ifc, REUSEX_HAVE_WEB_IFC).
  import_ifc,
};

/// Canonical lower-case stage name (the token used on the wire and the CLI).
std::string_view to_string(JobStage stage);

/// The name this stage writes into `pipeline_log.stage`.
///
/// Deliberately NOT the same as to_string(): the CLI has been writing
/// `segment_planes` / `cloud_reconstruction` into that column since long
/// before the GUI existed, and existing project databases are full of those
/// values. The wire token stays short (`planes`); the log name stays
/// compatible, so one table does not end up with two names for one operation.
std::string_view pipeline_log_name(JobStage stage);

/// Parse a canonical stage name. Returns nullopt for an unknown name.
std::optional<JobStage> parse_job_stage(std::string_view name);

/// Every name accepted by parse_job_stage(), in pipeline order.
std::vector<std::string> job_stage_names();

/// True when the stage threads the cancel token into its inner loop, i.e.
/// cancellation takes effect mid-run rather than only between jobs.
bool stage_supports_cancellation(JobStage stage);

/// One artifact a stage wrote, so a client can refresh exactly what changed
/// instead of re-fetching everything.
struct StageArtifact {
  /// "cloud" | "mesh" | "table".
  std::string kind;
  /// Stored name, e.g. "planes". For a `table` artifact this is the key the
  /// table is scoped by — the instances table of cloud "instances" is
  /// `{kind: "table", name: "instances"}`, reachable at
  /// `GET /api/v1/instances/instances`.
  std::string name;
  /// Points/rows written; -1 when not meaningful, or when the stage does not
  /// have the number in hand and counting it would mean re-reading the
  /// artifact purely to report it. Never a guess.
  int64_t count = -1;
};

/// Outcome of one stage execution.
struct StageResult {
  bool ok = false;        ///< The stage completed and wrote its outputs.
  bool cancelled = false; ///< The stage stopped early on a cancel request.
  /// The request was refused before/without doing the work because an input or
  /// a parameter was invalid, as opposed to the stage failing part-way through.
  /// Front ends need the distinction: `rux` returns INVALID_ARGUMENT rather
  /// than a generic error, and an HTTP front end would answer 400, not 500.
  /// Always accompanied by `ok == false`.
  bool invalid_input = false;
  std::string message; ///< Human-readable summary or failure reason.
  /// What this run actually wrote. Only populated on `ok == true`, and only
  /// with artifacts the stage can attest to having written on *this* run — a
  /// name that merely exists in the project afterwards is not reported.
  std::vector<StageArtifact> outputs;

  static StageResult success(std::string message = {});
  static StageResult success(std::string message,
                             std::vector<StageArtifact> outputs);
  static StageResult failure(std::string message);
  static StageResult invalid(std::string message);
  static StageResult cancel(std::string message = "cancelled");
};

/// Everything a stage execution needs, independent of any HTTP/CLI front end.
struct StageContext {
  /// Path to the `.rux` project the stage reads and writes.
  std::filesystem::path project;
  /// Which stage to run.
  JobStage stage = JobStage::clouds;
  /// Stage parameters as a JSON object (empty string = all defaults). Keys
  /// mirror the option-struct fields of the underlying algorithm; unknown keys
  /// are ignored. The same string is persisted into `pipeline_log.parameters`.
  std::string parameters;
  /// Cooperative cancellation flag, owned by the caller. May be null.
  const std::atomic_bool *cancel_token = nullptr;
  /// Identifier of the job driving this run, or empty for a direct call.
  /// Recorded into the stage's `pipeline_log.parameters` under `"job_id"` so
  /// the durable history can be joined back to the job that caused it (#265).
  std::string job_id;
  /// Who submitted the job (JobRecord::submitted_by; ruxd: the user id), or
  /// empty. Not recorded anywhere by the stage; it is there so a front end's
  /// StageExecutorOptions::speckle_token can pick that user's credential.
  std::string submitted_by;

  /// True when a cancel token is present and set.
  bool is_cancelled() const noexcept;
};

/// The parameter key a credential would travel under. Refused for every
/// stage: by JobQueue::submit() (so it never reaches the job store) and by
/// run_logged_stage() before the pipeline_log row is opened.
inline constexpr std::string_view kSecretParameterKey = "token";

/// What a stage needs from its front end beyond the project and its
/// parameters: credentials, and the network seam of the import stages.
///
/// SECRETS never travel in StageContext::parameters — those are persisted in
/// the job history and in `pipeline_log.parameters`. A credential is resolved
/// per run through a provider set here, and no stage writes it to the log, to
/// pipeline_log, to the StageResult or to any stored row. A `token` key in
/// any stage's parameters is refused (JobQueue::submit, run_logged_stage).
struct StageExecutorOptions {
  /// The Speckle access token for one `import_speckle` run against
  /// @p server_url (already validated: https://, or http:// to localhost).
  /// nullopt / empty = none configured: the stage is then refused with "no
  /// Speckle token configured" and nothing is fetched. A multi-user front
  /// end (ruxd) sets this and picks the submitting user's token
  /// (StageContext::submitted_by) for that server — it must NOT fall back to
  /// a process-wide token, or one user could send it to a server of their
  /// choosing. Unset (the default) reads `SPECKLE_TOKEN` from the
  /// environment, as `rux import speckle` always has.
  std::function<std::optional<std::string>(const StageContext &ctx,
                                           const std::string &server_url)>
      speckle_token;

  /// What the import_speckle stage hands speckle_download.
  struct SpeckleDownloadRequest {
    std::string server_url;
    std::string project;
    std::string model;
    std::string version; ///< empty = the latest
    std::string token;   ///< resolved by speckle_token
  };
  /// Replaces the stage's network read (tests; a front end with its own
  /// cache): returns the version's object tree, already resolved
  /// (io::speckle::ObjectTree::resolve_root()), as JSON text. Unset = read
  /// it from the server with io::speckle::SpeckleReader.
  std::function<std::string(const SpeckleDownloadRequest &)> speckle_download;
};

/// Run a stage against an already-open, writable ProjectDB.
///
/// Never throws: algorithm/IO failures are converted into a failed StageResult
/// and, when a pipeline_log row was opened, closed out with the error message
/// (STANDARDS §5 — no silent failure).
StageResult run_stage(ProjectDB &db, const StageContext &ctx,
                      const StageExecutorOptions &options);

/// run_stage() with default StageExecutorOptions (SPECKLE_TOKEN from the
/// environment).
StageResult run_stage(ProjectDB &db, const StageContext &ctx);

/// Run `body` wrapped in the same `pipeline_log` open/close protocol
/// run_stage() uses for the stages it dispatches directly: opens a row under
/// `log_name` (`ctx.job_id` folded into the recorded parameters exactly as
/// run_stage() does), evaluates `body`, and closes the row according to the
/// StageResult it returns — or, if `body` throws, the same way run_stage()'s
/// catch clause does (distinguishing a cancel from a failure via
/// `ctx.is_cancelled()`). `name` is the stage name used in log/error messages
/// (`to_string(stage)`); `log_name` is the value written to
/// `pipeline_log.stage` (`pipeline_log_name(stage)`).
///
/// Exists for a stage whose algorithm needs a module reusex_pipeline must not
/// link (optimize/graph_discover need reusex_slam, #464): their header-only
/// executor (pipeline/optimize_stage.hpp) calls this instead of
/// re-implementing the open/close bookkeeping, so the CLI and in-process
/// paths cannot drift on what a pipeline_log row looks like.
StageResult run_logged_stage(ProjectDB &db, const StageContext &ctx,
                             std::string_view name, std::string_view log_name,
                             const std::function<StageResult()> &body);

/// As above, handing @p body the id of the pipeline_log row it runs under,
/// so a stage can attribute the rows it writes to its run (storeys.run_id).
StageResult
run_logged_stage(ProjectDB &db, const StageContext &ctx, std::string_view name,
                 std::string_view log_name,
                 const std::function<StageResult(int log_id)> &body);

/// Open `ctx.project` read-write and run the stage against it.
StageResult run_stage(const StageContext &ctx,
                      const StageExecutorOptions &options = {});

/// Stage execution strategy. Exists so JobRunner's state machine can be tested
/// against a fake executor without touching real scan data.
using StageExecutor = std::function<StageResult(const StageContext &)>;

/// The executor that actually runs the pipeline (a thin wrap of run_stage),
/// with @p options for every run (StageExecutorOptions).
StageExecutor default_stage_executor(StageExecutorOptions options = {});

} // namespace reusex::pipeline
