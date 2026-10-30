// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// io::register_panorama (reusex-3iy8.5): a new 360 image stored with a
// manual initial pose, no timestamp matching, no alignment.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/panorama_pose.hpp>
#include <reusex/io/panoramas.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;
using reusex::ProjectDB;
using reusex::test_support::TempDir;

namespace {

std::vector<uint8_t> jpeg(int rows, int cols) {
  cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(30, 120, 210));
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  return buf;
}

void write(const fs::path &p, const std::vector<uint8_t> &bytes) {
  std::ofstream f(p, std::ios::binary);
  f.write(reinterpret_cast<const char *>(bytes.data()),
          static_cast<std::streamsize>(bytes.size()));
}

} // namespace

TEST_CASE("RegisterPanorama_File_StoredUnlinkedWithManualPose",
          "[io][panoramas][manual]") {
  TempDir tmp("test_register_pano_file");
  const fs::path file = tmp.path / "hall.jpg";
  write(file, jpeg(32, 64));
  ProjectDB db(tmp.path / "p.rux");

  const auto pose = reusex::core::upright_panorama_pose({4.2, 1.5, 1.6}, 90.0);
  const int id = reusex::io::register_panorama(db, file, pose);

  const auto row = reusex::core::panorama_record(db, id);
  CHECK(row.filename == "hall.jpg");
  CHECK(row.node_id == -1);
  CHECK(row.has_pose);
  CHECK(row.pose_source == "manual");
  for (size_t i = 0; i < 16; ++i)
    CHECK(row.pose[i] == Catch::Approx(pose[i]));
  CHECK(db.panoramic_image(id).cols == 64);

  // The same name again is refused, and nothing more is stored.
  CHECK_THROWS_AS(reusex::io::register_panorama(db, file, pose),
                  std::invalid_argument);
  CHECK(db.panoramic_image_count() == 1);

  // Stored under another name it is a second panorama.
  reusex::io::RegisterPanoramaOptions o;
  o.filename = "hall-2.jpg";
  CHECK(reusex::io::register_panorama(db, file, pose, o) != id);
  CHECK(db.panoramic_image_count() == 2);
}

TEST_CASE("RegisterPanorama_Bytes_StoredWithTimestamp",
          "[io][panoramas][manual]") {
  TempDir tmp("test_register_pano_bytes");
  ProjectDB db(tmp.path / "p.rux");
  const auto pose = reusex::core::upright_panorama_pose({0, 0, 1.5}, 0.0);
  const int id =
      reusex::io::register_panorama(db, "upload.jpg", jpeg(40, 80), pose, 12.5);
  const auto row = reusex::core::panorama_record(db, id);
  CHECK(row.pose_source == "manual");
  CHECK(row.timestamp == Catch::Approx(12.5));
  CHECK(row.pose[11] == Catch::Approx(1.5));
}

TEST_CASE("RegisterPanorama_BadInput_WritesNothing",
          "[io][panoramas][manual]") {
  TempDir tmp("test_register_pano_bad");
  ProjectDB db(tmp.path / "p.rux");
  const auto pose = reusex::core::upright_panorama_pose({0, 0, 0}, 0.0);

  // Not an equirect (4:3), as a file and as bytes.
  const fs::path photo = tmp.path / "photo.jpg";
  write(photo, jpeg(48, 64));
  CHECK_THROWS_AS(reusex::io::register_panorama(db, photo, pose),
                  std::invalid_argument);
  CHECK_THROWS_AS(
      reusex::io::register_panorama(db, "x.jpg", jpeg(48, 64), pose),
      std::invalid_argument);
  CHECK_THROWS_AS(reusex::io::register_panorama(
                      db, "y.jpg", std::vector<uint8_t>{1, 2, 3}, pose),
                  std::invalid_argument);

  // A non-rigid pose.
  auto bad = pose;
  bad[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(reusex::io::register_panorama(db, "z.jpg", jpeg(32, 64), bad),
                  std::invalid_argument);

  // A missing file.
  CHECK_THROWS_AS(
      reusex::io::register_panorama(db, tmp.path / "missing.jpg", pose),
      std::runtime_error);

  CHECK(db.panoramic_image_count() == 0);
}
