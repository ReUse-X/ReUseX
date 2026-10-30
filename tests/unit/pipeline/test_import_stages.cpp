// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The BIM import job stages, import_speckle and import_ifc (reusex-3iy8.13),
// run offline through the standard executor: the Speckle download is the
// executor's injected `speckle_download` returning the synthetic Revit model
// of test_speckle_revit.cpp, and the IFC file is the small fixture.
//
// The point of most of these tests is the SECRET: the Speckle token comes
// from StageExecutorOptions::speckle_token and must never land in the job
// record, pipeline_log, the StageResult, the log, the project file or a
// stored row.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/io/ifc.hpp>
#include <reusex/pipeline/JobScheduler.hpp>
#include <reusex/pipeline/import_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>

#include "../../support/revit_fixture.hpp"
#include "../../support/source_dir.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::pipeline;
using nlohmann::json;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;

namespace {

constexpr const char *kToken = "s3cr3t-speckle-token-4f7a";
constexpr const char *kServer = "https://speckle.example.org";

const char *kSpeckleParams = R"({"server":"https://speckle.example.org",
  "project":"proj01","model":"model01","storey_height":3.2})";

/// Every library log line while alive (all levels).
struct LogCapture {
  std::mutex mutex;
  std::vector<std::string> lines;
  core::LogLevel saved = core::get_log_level();

  LogCapture() {
    core::set_log_level(core::LogLevel::trace);
    core::set_log_handler([this](core::LogLevel, std::string_view m) {
      std::lock_guard lock(mutex);
      lines.emplace_back(m);
    });
  }
  ~LogCapture() {
    core::reset_log_handler();
    core::set_log_level(saved);
  }
  bool mentions(std::string_view needle) {
    std::lock_guard lock(mutex);
    return std::any_of(lines.begin(), lines.end(), [&](const auto &l) {
      return l.find(needle) != std::string::npos;
    });
  }
};

/// The raw bytes of @p path (and its -wal), to search for a leaked secret.
std::string file_bytes(const std::filesystem::path &path) {
  std::string out;
  for (const auto &p : {path, std::filesystem::path(path.string() + "-wal")}) {
    std::ifstream in(p, std::ios::binary);
    if (in)
      out.append(std::istreambuf_iterator<char>(in), {});
  }
  return out;
}

/// Executor options with a token provider (recording what it was asked) and
/// an offline download returning the synthetic Revit model.
struct FakeSpeckle {
  std::optional<std::string> token = std::string(kToken);
  int token_calls = 0;
  int downloads = 0;
  std::string asked_server;
  std::string asked_user;
  StageExecutorOptions::SpeckleDownloadRequest last;

  StageExecutorOptions options() {
    StageExecutorOptions o;
    o.speckle_token = [this](const StageContext &ctx,
                             const std::string &server) {
      ++token_calls;
      asked_server = server;
      asked_user = ctx.submitted_by;
      return token;
    };
    o.speckle_download =
        [this](const StageExecutorOptions::SpeckleDownloadRequest &r) {
          ++downloads;
          last = r;
          return test_support::RevitFixture{}.model().dump();
        };
    return o;
  }
};

bool has_output(const StageResult &r, const std::string &kind,
                const std::string &name) {
  return std::any_of(r.outputs.begin(), r.outputs.end(), [&](const auto &a) {
    return a.kind == kind && a.name == name;
  });
}

std::filesystem::path ifc_fixture() {
  return test_support::source_dir() / "tests" / "fixtures" / "ifc" /
         "small_building.ifc";
}

} // namespace

// ===========================================================================
// import_speckle
// ===========================================================================

TEST_CASE("ImportSpeckleStage_InjectedToken_ImportsAndNeverPersistsToken",
          "[pipeline][stages][import][speckle]") {
  TempPath project("test_import_stages");
  FakeSpeckle fake;
  LogCapture log;

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;
  ctx.parameters = kSpeckleParams;
  ctx.job_id = "job-speckle-1";
  ctx.submitted_by = "user-7";

  const StageResult r = default_stage_executor(fake.options())(ctx);
  INFO(r.message);
  REQUIRE(r.ok);

  // The provider was asked once, for the validated server and the
  // submitting user; the download got its token.
  CHECK(fake.token_calls == 1);
  CHECK(fake.asked_server == kServer);
  CHECK(fake.asked_user == "user-7");
  CHECK(fake.downloads == 1);
  CHECK(fake.last.token == kToken);
  CHECK(fake.last.project == "proj01");
  CHECK(fake.last.model == "model01");
  CHECK(fake.last.version.empty());

  // Outputs: the four solid-model tables and every mesh written.
  CHECK(has_output(r, "table", "storeys"));
  CHECK(has_output(r, "table", "walls"));
  CHECK(has_output(r, "table", "openings"));
  CHECK(has_output(r, "table", "spaces"));
  CHECK(has_output(r, "mesh", "imported/revit/walls"));
  for (const auto &a : r.outputs)
    if (a.kind == "table" && a.name == "walls")
      CHECK(a.count == 1);

  {
    ProjectDB db(project.path, /*readOnly=*/true);
    CHECK(db.walls().size() == 1);
    CHECK(db.has_mesh("imported/revit/walls"));
    // Mesh parameters carry the source, never the token.
    const auto md = db.mesh_metadata("imported/revit/walls");
    CHECK(md.stage == "import_speckle");
    CHECK(md.parameters.find(kToken) == std::string::npos);

    // ONE pipeline_log row: the stage's, with the job's parameters + job id.
    const auto runs = db.pipeline_log();
    REQUIRE(runs.size() == 1);
    CHECK(runs.front().stage == "import_speckle");
    CHECK(runs.front().status == "success");
    const auto params = json::parse(runs.front().parameters);
    CHECK(params.at("job_id") == "job-speckle-1");
    CHECK(params.at("project") == "proj01");
    CHECK_FALSE(params.contains("token"));
    CHECK(runs.front().parameters.find(kToken) == std::string::npos);
  }

  // Not in the result, not in any log line, not anywhere in the file.
  CHECK(r.message.find(kToken) == std::string::npos);
  for (const auto &a : r.outputs)
    CHECK(a.name.find(kToken) == std::string::npos);
  CHECK_FALSE(log.mentions(kToken));
  CHECK(file_bytes(project.path).find(kToken) == std::string::npos);
}

TEST_CASE("ImportSpeckleStage_ProviderHasNoToken_RefusedWithoutFetching",
          "[pipeline][stages][import][speckle]") {
  TempPath project("test_import_stages");
  FakeSpeckle fake;

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;
  ctx.parameters = kSpeckleParams;

  SECTION("nullopt") { fake.token.reset(); }
  SECTION("empty") { fake.token = std::string(); }

  const StageResult r = default_stage_executor(fake.options())(ctx);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("no Speckle token configured") != std::string::npos);
  CHECK(fake.downloads == 0);

  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().empty());
  // The refusal is recorded, as every refused stage is.
  const auto runs = db.pipeline_log();
  REQUIRE(runs.size() == 1);
  CHECK(runs.front().status == "failed");
}

TEST_CASE("ImportSpeckleStage_NoProvider_RefusedEvenWithSpeckleTokenEnv",
          "[pipeline][stages][import][speckle]") {
  // No provider = no token: the environment is only read when the front end
  // says so (StageExecutorOptions::env_speckle_token, local_user()).
  const char *had = std::getenv("SPECKLE_TOKEN");
  const std::optional<std::string> saved =
      had ? std::optional<std::string>(had) : std::nullopt;
  ::setenv("SPECKLE_TOKEN", "env-token-91", 1);

  TempPath project("test_import_stages");
  FakeSpeckle fake;
  StageExecutorOptions options = fake.options();
  options.speckle_token = nullptr; // the default

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;
  ctx.parameters = kSpeckleParams;

  SECTION("no provider: refused") {
    const StageResult r = run_stage(ctx, options);
    CHECK(r.invalid_input);
    CHECK(r.message.find("no Speckle token configured") != std::string::npos);
    CHECK(fake.downloads == 0);
    // ... also through the default executor, with no options at all
    const StageResult d = default_stage_executor()(ctx);
    CHECK(d.invalid_input);
  }
  SECTION("env_speckle_token: used") {
    options.speckle_token = &StageExecutorOptions::env_speckle_token;
    const StageResult r = run_stage(ctx, options);
    INFO(r.message);
    CHECK(r.ok);
    CHECK(fake.last.token == "env-token-91");
  }
  SECTION("local_user(): env token, localhost and outside files allowed") {
    const auto local = StageExecutorOptions::local_user();
    REQUIRE(local.speckle_token);
    CHECK(local.speckle_token(ctx, kServer) == std::string("env-token-91"));
    CHECK(local.allow_insecure_localhost);
    CHECK(local.allow_paths_outside_project);
  }

  if (saved)
    ::setenv("SPECKLE_TOKEN", saved->c_str(), 1);
  else
    ::unsetenv("SPECKLE_TOKEN");
}

TEST_CASE("ImportSpeckleStage_ServerPolicy_CheckedBeforeTheToken",
          "[pipeline][stages][import][speckle]") {
  TempPath project("test_import_stages");
  FakeSpeckle fake;
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;
  const auto with_server = [](const std::string &server) {
    return json{{"server", server}, {"project", "p1"}, {"model", "m1"}}.dump();
  };

  SECTION("default policy refuses localhost and private addresses") {
    for (const char *server :
         {"http://localhost:3000", "https://localhost", "https://127.0.0.1",
          "https://10.1.2.3", "https://192.168.0.5", "https://169.254.169.254",
          "https://[::1]", "https://[fd00::1]", "https://user@speckle.xyz"}) {
      INFO(server);
      ctx.parameters = with_server(server);
      const StageResult r = run_stage(ctx, fake.options());
      CHECK(r.invalid_input);
    }
    CHECK(fake.token_calls == 0);
    CHECK(fake.downloads == 0);
  }
  SECTION("allow_insecure_localhost lets localhost through") {
    auto options = fake.options();
    options.allow_insecure_localhost = true;
    ctx.parameters = with_server("http://localhost:3000");
    CHECK(run_stage(ctx, options).ok);
  }
  SECTION("a hook is the whole policy") {
    auto options = fake.options();
    std::string asked;
    options.allowed_speckle_server = [&](const std::string &url) {
      asked = url;
      return false;
    };
    ctx.parameters = kSpeckleParams;
    const StageResult r = run_stage(ctx, options);
    CHECK(r.invalid_input);
    CHECK(asked == kServer);
    CHECK(fake.token_calls == 0);
  }
  CHECK(default_speckle_server_allowed("https://app.speckle.systems", false));
  CHECK(default_speckle_server_allowed("https://speckle.example.org:8443/x",
                                       false));
  CHECK_FALSE(
      default_speckle_server_allowed("http://speckle.example.org", true));
  CHECK_FALSE(
      default_speckle_server_allowed("ftp://speckle.example.org", true));
}

TEST_CASE("ImportSpeckleStage_CancelDuringDownload_StoresNothing",
          "[pipeline][stages][import][speckle]") {
  TempPath project("test_import_stages");
  FakeSpeckle fake;
  std::atomic_bool cancel{false};
  auto options = fake.options();
  const auto inner = options.speckle_download;
  options.speckle_download = [&](const auto &r) {
    cancel = true; // the user cancels while the download runs
    return inner(r);
  };
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;
  ctx.parameters = kSpeckleParams;
  ctx.cancel_token = &cancel;

  CHECK(stage_supports_cancellation(JobStage::import_speckle));
  const StageResult r = run_stage(ctx, options);
  CHECK(r.cancelled);
  CHECK_FALSE(r.ok);
  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().empty());
  CHECK_FALSE(db.has_mesh("imported/revit/walls"));
  const auto runs = db.pipeline_log();
  REQUIRE(runs.size() == 1);
  CHECK(runs.front().status == "cancelled");
}

TEST_CASE("ImportSpeckleStage_TokenParameter_RefusedBeforeAnythingIsStored",
          "[pipeline][stages][import][speckle]") {
  const std::string with_token =
      R"({"project":"proj01","model":"model01","token":"leaked-77"})";

  SECTION("run_stage: no pipeline_log row is written at all") {
    TempPath project("test_import_stages");
    FakeSpeckle fake;
    LogCapture log;
    StageContext ctx;
    ctx.project = project.path;
    ctx.stage = JobStage::import_speckle;
    ctx.parameters = with_token;
    const StageResult r = run_stage(ctx, fake.options());
    CHECK(r.invalid_input);
    CHECK(r.message.find("credential") != std::string::npos);
    CHECK(r.message.find("leaked-77") == std::string::npos);
    CHECK(fake.downloads == 0);
    CHECK_FALSE(log.mentions("leaked-77"));
    ProjectDB db(project.path, /*readOnly=*/true);
    CHECK(db.pipeline_log().empty());
  }

  SECTION("any stage refuses it") {
    TempPath project("test_import_stages");
    StageContext ctx;
    ctx.project = project.path;
    ctx.stage = JobStage::planes;
    ctx.parameters = R"({"token":"leaked-77"})";
    const StageResult r = run_stage(ctx);
    CHECK(r.invalid_input);
    ProjectDB db(project.path, /*readOnly=*/true);
    CHECK(db.pipeline_log().empty());
  }

  SECTION("JobQueue::submit: never reaches the job store") {
    TempPath project("test_import_stages");
    JobScheduler scheduler;
    auto queue = scheduler.make_queue("case", project.path);
    CHECK_THROWS_AS(queue->submit(JobStage::import_speckle, with_token),
                    std::runtime_error);
    CHECK(queue->jobs().empty());
  }

  SECTION("the parameter reader refuses it too") {
    CHECK_THROWS_AS(speckle_options_from_parameters(with_token),
                    std::invalid_argument);
  }
}

TEST_CASE("ImportStages_UnknownKeys_NeverPersisted",
          "[pipeline][stages][import]") {
  // A misspelt secret is an unknown key: refused, never stored.
  for (const char *key : {"speckle_token", "Token", "auth"}) {
    INFO(key);
    const std::string params =
        json{{"project", "proj01"}, {"model", "model01"}, {key, "leaked-88"}}
            .dump();
    TempPath project("test_import_stages");
    FakeSpeckle fake;
    LogCapture log;

    StageContext ctx;
    ctx.project = project.path;
    ctx.stage = JobStage::import_speckle;
    ctx.parameters = params;
    const StageResult r = run_stage(ctx, fake.options());
    CHECK(r.invalid_input);
    CHECK(r.message.find("unknown key") != std::string::npos);
    CHECK(r.message.find("leaked-88") == std::string::npos);
    CHECK_FALSE(log.mentions("leaked-88"));
    {
      ProjectDB db(project.path, /*readOnly=*/true);
      CHECK(db.pipeline_log().empty());
    }

    JobScheduler scheduler;
    auto queue = scheduler.make_queue("case", project.path);
    CHECK_THROWS_AS(queue->submit(JobStage::import_speckle, params),
                    std::runtime_error);
    CHECK_THROWS_AS(
        queue->submit(JobStage::import_ifc,
                      json{{"path", "a.ifc"}, {key, "leaked-88"}}.dump()),
        std::runtime_error);
    CHECK(queue->jobs().empty());
  }
}

TEST_CASE("ImportStages_Submit_StoresTheReSerialisedTableParameters",
          "[pipeline][stages][import][job]") {
  TempPath project("test_import_stages");
  JobScheduler scheduler;
  // An executor that does nothing: only the stored record matters here.
  auto queue =
      scheduler.make_queue("case", project.path, [](const StageContext &) {
        return StageResult::success("noop");
      });
  const std::string id = queue->submit(JobStage::import_speckle,
                                       R"({"project":"p1","model":"m1"})");
  queue->wait_idle();
  const auto job = queue->job(id);
  REQUIRE(job);
  const auto stored = json::parse(job->parameters);
  for (const auto &d : stage_parameters(JobStage::import_speckle)) {
    INFO(d.key);
    // every table key with a value, nothing else
    if (d.key == "version" || d.key == "elevation_offset")
      CHECK_FALSE(stored.contains(d.key));
    else
      CHECK(stored.contains(d.key));
  }
  CHECK(stored.size() == stage_parameters(JobStage::import_speckle).size() - 2);
  // Missing required keys are refused at submit, not in the worker.
  CHECK_THROWS_AS(queue->submit(JobStage::import_speckle, R"({"model":"m"})"),
                  std::runtime_error);
}

TEST_CASE("ImportSpeckleStage_JobQueue_RunsAsAnOrdinaryJob",
          "[pipeline][stages][import][speckle][job]") {
  TempPath project("test_import_stages");
  FakeSpeckle fake;
  JobScheduler scheduler;
  auto queue = scheduler.make_queue("case-a", project.path,
                                    default_stage_executor(fake.options()));
  const std::string id =
      queue->submit(JobStage::import_speckle, kSpeckleParams, "user-42");
  queue->wait_idle();

  const auto job = queue->job(id);
  REQUIRE(job);
  INFO(job->error);
  CHECK(job->status == JobStatus::succeeded);
  CHECK(fake.asked_user == "user-42");
  CHECK(fake.last.token == kToken);
  CHECK(job->parameters.find(kToken) == std::string::npos);
  CHECK(job->result_summary.find(kToken) == std::string::npos);
  CHECK(std::any_of(
      job->result_outputs.begin(), job->result_outputs.end(),
      [](const auto &a) { return a.kind == "table" && a.name == "storeys"; }));
}

TEST_CASE("ImportSpeckleStage_InvalidParameters_Refused",
          "[pipeline][stages][import][speckle]") {
  TempPath project("test_import_stages");
  FakeSpeckle fake;
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;

  const auto refused = [&](const std::string &params) {
    ctx.parameters = params;
    const StageResult r = run_stage(ctx, fake.options());
    INFO(params << " -> " << r.message);
    CHECK(r.invalid_input);
    return r.message;
  };
  CHECK(refused(R"({"model":"m1"})").find("'project'") != std::string::npos);
  CHECK(refused(R"({"project":"p1"})").find("'model'") != std::string::npos);
  CHECK(refused(R"({"project":"","model":"m1"})").find("required") !=
        std::string::npos);
  CHECK(refused(R"({"project":"p1","model":"m1","storey_height":-1})")
            .find("outside") != std::string::npos);
  CHECK(refused(R"({"project":"p1","model":"m1","meshes":"yes"})")
            .find("true or false") != std::string::npos);
  CHECK(
      refused(R"({"project":"p1","model":"m1","server":"http://evil.example"})")
          .find("server") != std::string::npos);
  CHECK(refused(R"({"project":"p/../1","model":"m1"})").find("Speckle id") !=
        std::string::npos);
  CHECK(fake.downloads == 0);
  CHECK(fake.token_calls == 0); // refused before a token is asked for
}

TEST_CASE("ImportSpeckleParameters_RoundTripTheOptionsStruct",
          "[pipeline][params][import]") {
  SpeckleStageOptions o;
  o.import.project = "p1";
  o.import.model = "m1";
  o.import.version = "v9";
  o.import.mapping.meshes = false;
  o.import.mapping.elevation_offset_m = -20.82;
  o.import.mapping.default_storey_height_m = 2.8;
  o.import.token = "never-written";
  const std::string text = speckle_parameters_json(o);
  CHECK(text.find("never-written") == std::string::npos);

  const auto back = speckle_options_from_parameters(text);
  CHECK(back.import.project == "p1");
  CHECK(back.import.model == "m1");
  CHECK(back.import.version == "v9");
  CHECK_FALSE(back.import.mapping.meshes);
  REQUIRE(back.import.mapping.elevation_offset_m);
  CHECK(*back.import.mapping.elevation_offset_m == -20.82);
  CHECK(back.import.mapping.default_storey_height_m == 2.8);
  CHECK(back.import.token.empty());

  // Defaults are the options struct's (STANDARDS §4).
  const io::speckle::SpeckleRevitImportOptions d;
  const auto def = speckle_options_from_parameters(R"({"project":"p",
    "model":"m"})");
  CHECK(def.import.server_url == d.server_url);
  CHECK(def.import.mapping.default_storey_height_m ==
        d.mapping.default_storey_height_m);
  CHECK(def.import.mapping.recenter_threshold_m ==
        d.mapping.recenter_threshold_m);
  CHECK_FALSE(def.import.mapping.elevation_offset_m);

  for (const auto &desc : stage_parameters(JobStage::import_speckle))
    CHECK(desc.key != "token");
}

// ===========================================================================
// import_ifc
// ===========================================================================

TEST_CASE("ImportIfcStage_Fixture_ImportsThroughTheExecutor",
          "[pipeline][stages][import][ifc]") {
  if (!io::ifc::has_ifc_support())
    SKIP("built without web-ifc");
  TempPath project("test_import_stages");

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_ifc;
  ctx.parameters = json{{"path", ifc_fixture().string()}}.dump();
  ctx.job_id = "job-ifc-1";

  // The fixture lies outside the project's directory.
  StageExecutorOptions options;
  options.allow_paths_outside_project = true;
  const StageResult r = default_stage_executor(options)(ctx);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(has_output(r, "table", "storeys"));
  CHECK(has_output(r, "table", "walls"));
  CHECK(has_output(r, "table", "openings"));
  CHECK(has_output(r, "table", "spaces"));
  CHECK(has_output(r, "mesh", "imported/ifc/wall"));

  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.storeys().size() == 2);
  const auto runs = db.pipeline_log();
  REQUIRE(runs.size() == 1); // the stage's row only
  CHECK(runs.front().stage == "import_ifc");
  CHECK(runs.front().status == "success");
  CHECK(json::parse(runs.front().parameters).at("job_id") == "job-ifc-1");
}

TEST_CASE("ImportIfcStage_Path_ConfinedToTheProjectDirectory",
          "[pipeline][stages][import][ifc]") {
  if (!io::ifc::has_ifc_support())
    SKIP("built without web-ifc");
  TempDir dir("test_import_stages_case");
  TempDir elsewhere("test_import_stages_other");
  std::filesystem::create_directories(dir.path / "uploads");
  std::filesystem::copy_file(ifc_fixture(), dir.path / "uploads" / "model.ifc");
  std::filesystem::copy_file(ifc_fixture(), elsewhere.path / "other.ifc");
  std::filesystem::create_symlink(elsewhere.path / "other.ifc",
                                  dir.path / "uploads" / "link.ifc");

  StageContext ctx;
  ctx.project = dir.path / "case.rux";
  ctx.stage = JobStage::import_ifc;
  const auto run = [&](const std::string &path,
                       const StageExecutorOptions &options = {}) {
    ctx.parameters = json{{"path", path}, {"meshes", false}}.dump();
    return run_stage(ctx, options);
  };
  const std::string refusal = "is not a file in the project's directory";

  SECTION("relative inside: imported") {
    const StageResult r = run("uploads/model.ifc");
    INFO(r.message);
    CHECK(r.ok);
    CHECK_FALSE(std::any_of(r.outputs.begin(), r.outputs.end(),
                            [](const auto &a) { return a.kind == "mesh"; }));
  }
  SECTION("absolute inside: imported") {
    CHECK(run((dir.path / "uploads" / "model.ifc").string()).ok);
  }
  SECTION("relative escaping with ..: refused") {
    const StageResult r =
        run("../" + elsewhere.path.filename().string() + "/other.ifc");
    CHECK(r.invalid_input);
    CHECK(r.message.find(refusal) != std::string::npos);
  }
  SECTION("absolute outside: refused") {
    const StageResult r = run((elsewhere.path / "other.ifc").string());
    CHECK(r.invalid_input);
    CHECK(r.message.find(refusal) != std::string::npos);
  }
  SECTION("a symlink out of the directory: refused") {
    const StageResult r = run("uploads/link.ifc");
    CHECK(r.invalid_input);
    CHECK(r.message.find(refusal) != std::string::npos);
  }
  SECTION("missing and outside read the same") {
    const StageResult missing = run("uploads/none.ifc");
    const StageResult outside = run("/nonexistent/model.ifc");
    CHECK(missing.invalid_input);
    CHECK(outside.invalid_input);
    CHECK(missing.message.find(refusal) != std::string::npos);
    CHECK(outside.message.find(refusal) != std::string::npos);
  }
  SECTION("allow_paths_outside_project: outside and symlinks allowed") {
    StageExecutorOptions options;
    options.allow_paths_outside_project = true;
    CHECK(run((elsewhere.path / "other.ifc").string(), options).ok);
    CHECK(run("uploads/link.ifc", options).ok);
    const StageResult missing = run("/nonexistent/model.ifc", options);
    CHECK(missing.invalid_input);
    CHECK(missing.message.find("not an existing file") != std::string::npos);
  }
}

TEST_CASE("ImportIfcStage_InvalidParameters_Refused",
          "[pipeline][stages][import][ifc]") {
  if (!io::ifc::has_ifc_support())
    SKIP("built without web-ifc");
  TempPath project("test_import_stages");
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_ifc;

  const auto refused = [&](const std::string &params) {
    ctx.parameters = params;
    const StageResult r = run_stage(ctx);
    INFO(params << " -> " << r.message);
    CHECK(r.invalid_input);
    return r.message;
  };
  CHECK(refused("{}").find("'path'") != std::string::npos);
  CHECK(
      refused(
          json{{"path", ifc_fixture().string()}, {"min_wall_fill", 2.0}}.dump())
          .find("outside") != std::string::npos);
}

TEST_CASE("ImportStages_Names_AndPipelineLogNames",
          "[pipeline][stages][import]") {
  CHECK(parse_job_stage("import_speckle") == JobStage::import_speckle);
  CHECK(parse_job_stage("import_ifc") == JobStage::import_ifc);
  CHECK(pipeline_log_name(JobStage::import_speckle) == "import_speckle");
  CHECK(pipeline_log_name(JobStage::import_ifc) == "import_ifc");
  CHECK(stage_supports_cancellation(JobStage::import_speckle));
  CHECK_FALSE(stage_supports_cancellation(JobStage::import_ifc));
}
