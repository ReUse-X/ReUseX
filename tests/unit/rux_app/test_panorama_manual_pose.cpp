// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The CLI side of manual panorama poses (reusex-3iy8.5): `rux set
// panoramas.<name>.pose`, `rux import 360 --pose`, and `rux align 360`
// keeping a manual pose.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <align/panorama.hpp>
#include <database/path_parser.hpp>
#include <database/resource_router.hpp>
#include <global-params.hpp>
#include <import/panorama.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/panorama_pose.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace rux::database;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

std::vector<uint8_t> equirect_jpeg() {
  cv::Mat img(32, 64, CV_8UC3, cv::Scalar(10, 200, 90));
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  return buf;
}

void set(RouterRegistry &registry, const std::string &path, const json &value) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  registry.get_router(components[0].value).set(rel, value);
}

json get(RouterRegistry &registry, const std::string &path) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  return std::get<json>(registry.get_router(components[0].value).get(rel));
}

} // namespace

TEST_CASE("PanoramaRouter_SetPose_FullUprightAndNudge",
          "[rux_app][router][panorama]") {
  reusex::test_support::TempPath tmp("test_pano_router_pose");
  auto db = std::make_shared<reusex::ProjectDB>(tmp.path);
  db->save_panoramic_image("hall.jpg", equirect_jpeg());
  RouterRegistry registry(db);

  // [x, y, z, yaw]: upright there. The dotted file name survives the path
  // parser's '.' split.
  set(registry, "panoramas.hall.jpg.pose", json::array({1.0, 2.0, 1.5, 90.0}));
  auto row = db->list_panoramic_images().at(0);
  CHECK(row.pose_source == "manual");
  CHECK(row.pose[3] == Catch::Approx(1.0));
  CHECK(reusex::core::panorama_yaw_deg(row.pose) == Catch::Approx(90.0));

  // A nudge object keeps what it does not name.
  set(registry, "panoramas[0].pose", json{{"x", 3.0}, {"yaw_deg", -45.0}});
  row = db->list_panoramic_images().at(0);
  CHECK(row.pose[3] == Catch::Approx(3.0));
  CHECK(row.pose[7] == Catch::Approx(2.0));
  CHECK(row.pose[11] == Catch::Approx(1.5));
  CHECK(reusex::core::panorama_yaw_deg(row.pose) == Catch::Approx(-45.0));

  // A full 4x4.
  const auto full = reusex::core::upright_panorama_pose({-1, -2, 0.5}, 10.0);
  set(registry, "panoramas.hall.jpg.pose", json(full));
  CHECK(db->list_panoramic_images().at(0).pose[11] == Catch::Approx(0.5));

  const auto got = get(registry, "panoramas[0].pose");
  CHECK(got["pose_source"] == "manual");
  CHECK(got["pose"].size() == 16);
  // Dotted names resolve the same way for get as for set.
  const auto by_name = get(registry, "panoramas.hall.jpg.pose");
  CHECK(by_name["pose"] == got["pose"]);
  CHECK(get(registry, "panoramas.hall.jpg.metadata")["filename"] == "hall.jpg");

  CHECK_THROWS(set(registry, "panoramas.hall.jpg.pose", json{{"roll", 3}}));
  CHECK_THROWS(set(registry, "panoramas.hall.jpg.pose",
                   json::array({1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                1}))); // scaled: not rigid
  CHECK_THROWS(
      set(registry, "panoramas.nope.jpg.pose", json::array({0, 0, 0})));
  CHECK_THROWS(set(registry, "panoramas.hall.jpg.metadata", json::object()));

  // ... and for del.
  db->save_panoramic_image("b.v2.jpg", equirect_jpeg());
  {
    auto components = parse_path("panoramas.b.v2.jpg");
    std::vector<PathComponent> rel(components.begin() + 1, components.end());
    registry.get_router("panoramas").del(rel);
  }
  CHECK_FALSE(db->has_panoramic_image("b.v2.jpg"));
  CHECK(db->has_panoramic_image("hall.jpg"));
}

TEST_CASE("ImportPanorama_PoseFlag_RegistersOneManualPanorama",
          "[rux_app][import][panorama]") {
  reusex::test_support::TempDir tmp("test_import_360_pose");
  const fs::path file = tmp.path / "room.jpg";
  {
    const auto bytes = equirect_jpeg();
    std::ofstream f(file, std::ios::binary);
    f.write(reinterpret_cast<const char *>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  }
  RuxOptions global;
  global.project_db = tmp.path / "p.rux";

  SubcommandImportPanoramaOptions opt;
  opt.input_paths = {file};
  opt.pose = {2.0, 3.0, 1.6, 180.0};
  REQUIRE(run_subcommand_import_panorama(opt, global) == RuxError::SUCCESS);

  reusex::ProjectDB db(global.project_db);
  const auto panos = db.list_panoramic_images();
  REQUIRE(panos.size() == 1);
  CHECK(panos[0].pose_source == "manual");
  CHECK(panos[0].node_id == -1);
  CHECK(panos[0].pose[3] == Catch::Approx(2.0));
  CHECK(std::abs(reusex::core::panorama_yaw_deg(panos[0].pose)) ==
        Catch::Approx(180.0));

  // --pose with more than one file is refused.
  SubcommandImportPanoramaOptions two = opt;
  two.input_paths = {file, file};
  CHECK(run_subcommand_import_panorama(two, global) ==
        RuxError::INVALID_ARGUMENT);
}

TEST_CASE("AlignPanorama_ManualPose_KeptEvenWithOverwrite",
          "[rux_app][align][panorama]") {
  reusex::test_support::TempPath tmp("test_align_360_manual");
  const auto manual =
      reusex::core::upright_panorama_pose({5.0, 6.0, 1.5}, 33.0);
  {
    reusex::ProjectDB db(tmp.path);
    db.save_sensor_frame(
        1, reusex::test_support::make_color(16, 12), cv::Mat(), cv::Mat(),
        reusex::test_support::translation_pose(0.0, 0.0, 1.4),
        reusex::test_support::make_intrinsics(8, 8, 8, 6, 16, 12));
    const int id = db.save_panoramic_image("p.jpg", equirect_jpeg(), -1.0, 1);
    reusex::core::set_panorama_pose_manual(db, id, manual);
  }
  RuxOptions global;
  global.project_db = tmp.path;
  SubcommandAlignPanoramaOptions opt;
  opt.overwrite = true;
  REQUIRE(run_subcommand_align_panorama(opt, global) == RuxError::SUCCESS);

  reusex::ProjectDB db(tmp.path);
  const auto row = db.list_panoramic_images().at(0);
  CHECK(row.pose_source == "manual");
  for (size_t i = 0; i < 16; ++i)
    CHECK(row.pose[i] == Catch::Approx(manual[i]));
}
