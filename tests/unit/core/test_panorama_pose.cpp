// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The manual panorama pose path (reusex-3iy8.5): validate, write as
// pose_source "manual", nudge position / yaw, and the align-360 decision
// that keeps a manual pose.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/panorama_pose.hpp>
#include <reusex/core/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace reusex;
using core::PanoramaAlignDecision;
using core::PanoramaPose;

namespace {

std::vector<uint8_t> fake_jpeg() { return {0xFF, 0xD8, 0xFF, 0xD9}; }

Eigen::Matrix4d m4(const PanoramaPose &p) {
  return core::pose_graph::matrix_from_row_major(p);
}

/// A tilted panorama: yaw 30 deg, then pitched 10 deg and rolled 5 deg in
/// its own frame (camera on a leaning stick).
PanoramaPose tilted_pose() {
  const PanoramaPose upright = core::upright_panorama_pose({1.0, 2.0, 1.5}, 30);
  Eigen::Matrix4d m = m4(upright);
  const Eigen::Matrix3d tilt =
      (Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();
  m.topLeftCorner<3, 3>() = m.topLeftCorner<3, 3>() * tilt;
  return core::pose_graph::row_major_from_matrix(m);
}

ProjectDB::PanoramicImage pano_row(int node_id, bool has_pose,
                                   const std::string &source) {
  ProjectDB::PanoramicImage p{};
  p.id = 1;
  p.node_id = node_id;
  p.has_pose = has_pose;
  p.pose_source = source;
  return p;
}

} // namespace

TEST_CASE("UprightPanoramaPose_YawZero_LooksAlongWorldX_HorizonLevel",
          "[core][panorama]") {
  const auto p = core::upright_panorama_pose({1.0, 2.0, 3.0}, 0.0);
  REQUIRE_NOTHROW(core::validate_panorama_pose(p));
  const Eigen::Matrix4d m = m4(p);
  CHECK(m.block<3, 1>(0, 2).isApprox(Eigen::Vector3d::UnitX()));  // forward
  CHECK(m.block<3, 1>(0, 1).isApprox(-Eigen::Vector3d::UnitZ())); // down
  CHECK(m.block<3, 1>(0, 3).isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  CHECK(core::panorama_yaw_deg(p) == Catch::Approx(0.0).margin(1e-9));
  CHECK(core::panorama_yaw_deg(core::upright_panorama_pose({0, 0, 0}, 90)) ==
        Catch::Approx(90.0));
  CHECK(core::panorama_yaw_deg(core::upright_panorama_pose({0, 0, 0}, -135)) ==
        Catch::Approx(-135.0));
}

TEST_CASE("ValidatePanoramaPose_RejectsNonFiniteAndNonRigid",
          "[core][panorama]") {
  auto p = core::upright_panorama_pose({0, 0, 0}, 0);
  auto bad = p;
  bad[3] = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(core::validate_panorama_pose(bad), std::invalid_argument);
  bad = p;
  for (int r = 0; r < 3; ++r) // scaled column
    bad[static_cast<size_t>(r * 4)] *= 2.0;
  CHECK_THROWS_AS(core::validate_panorama_pose(bad), std::invalid_argument);
  bad = p;
  for (int r = 0; r < 3; ++r) // reflection: det -1
    bad[static_cast<size_t>(r * 4)] = -bad[static_cast<size_t>(r * 4)];
  CHECK_THROWS_AS(core::validate_panorama_pose(bad), std::invalid_argument);
  bad = p;
  bad[12] = 0.5; // bottom row
  CHECK_THROWS_AS(core::validate_panorama_pose(bad), std::invalid_argument);
}

TEST_CASE("SetPanoramaPoseManual_WritesPoseAndSource_RoundTrips",
          "[core][panorama]") {
  test_support::TempPath tmp("test_panorama_pose_manual");
  ProjectDB db(tmp.path);
  const int id = db.save_panoramic_image("a.jpg", fake_jpeg());
  const auto pose = tilted_pose();
  core::set_panorama_pose_manual(db, id, pose);

  const auto row = core::panorama_record(db, id);
  CHECK(row.has_pose);
  CHECK(row.pose_source == "manual");
  CHECK(row.align_inliers == -1);
  CHECK(row.align_rms == -1.0);
  for (size_t i = 0; i < 16; ++i)
    CHECK(row.pose[i] == Catch::Approx(pose[i]));

  // An invalid pose writes nothing; an unknown id throws out_of_range.
  auto bad = pose;
  bad[5] = std::numeric_limits<double>::infinity();
  CHECK_THROWS_AS(core::set_panorama_pose_manual(db, id, bad),
                  std::invalid_argument);
  CHECK(core::panorama_record(db, id).pose[5] == Catch::Approx(pose[5]));
  CHECK_THROWS_AS(core::set_panorama_pose_manual(db, 999, pose),
                  std::out_of_range);
}

TEST_CASE("NudgePanoramaPose_YawAndPosition_KeepPitchAndRoll",
          "[core][panorama]") {
  test_support::TempPath tmp("test_panorama_pose_nudge");
  ProjectDB db(tmp.path);
  const int id = db.save_panoramic_image("a.jpg", fake_jpeg());
  const auto before = tilted_pose();
  db.save_panorama_pose(id, before, 40, 0.3); // an aligned pose

  core::PanoramaPoseNudge n;
  n.x = 5.0;
  n.yaw_deg = -60.0;
  const auto after = core::nudge_panorama_pose(db, id, n);

  const auto row = core::panorama_record(db, id);
  CHECK(row.pose_source == "manual");
  CHECK(row.align_inliers == -1);
  for (size_t i = 0; i < 16; ++i)
    CHECK(row.pose[i] == Catch::Approx(after[i]));

  const Eigen::Matrix4d a = m4(before), b = m4(after);
  // Position: x set, y and z kept.
  CHECK(b(0, 3) == Catch::Approx(5.0));
  CHECK(b(1, 3) == Catch::Approx(a(1, 3)));
  CHECK(b(2, 3) == Catch::Approx(a(2, 3)));
  // Yaw set.
  CHECK(core::panorama_yaw_deg(after) == Catch::Approx(-60.0));
  // Pitch and roll against world up kept: every optical axis keeps its
  // elevation, and the rotation between them is a pure rotation about z.
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  for (int c = 0; c < 3; ++c)
    CHECK(b.block<3, 1>(0, c).dot(up) ==
          Catch::Approx(a.block<3, 1>(0, c).dot(up)).margin(1e-12));
  const Eigen::Matrix3d d =
      b.topLeftCorner<3, 3>() * a.topLeftCorner<3, 3>().transpose();
  CHECK((d * up).isApprox(up));
  REQUIRE_NOTHROW(core::validate_panorama_pose(after));

  // A position-only nudge leaves the rotation untouched.
  core::PanoramaPoseNudge only_z;
  only_z.z = 0.25;
  const auto moved = core::nudge_panorama_pose(db, id, only_z);
  CHECK(m4(moved).topLeftCorner<3, 3>().isApprox(b.topLeftCorner<3, 3>()));
  CHECK(moved[11] == Catch::Approx(0.25));

  CHECK_THROWS_AS(core::nudge_panorama_pose(db, id, {}), std::invalid_argument);
}

TEST_CASE("NudgePanoramaPose_NoOwnPose_StartsFromLinkedFrame",
          "[core][panorama]") {
  test_support::TempPath tmp("test_panorama_pose_seed");
  ProjectDB db(tmp.path);
  db.save_sensor_frame(3, test_support::make_color(16, 12), cv::Mat(),
                       cv::Mat(), test_support::translation_pose(7.0, 1.0, 1.4),
                       test_support::make_intrinsics(
                           8, 8, 8, 6, 16, 12, test_support::identity16()));
  const int linked = db.save_panoramic_image("l.jpg", fake_jpeg(), -1.0, 3);
  const int loose = db.save_panoramic_image("u.jpg", fake_jpeg());

  core::PanoramaPoseNudge n;
  n.y = -2.0;
  const auto a = core::nudge_panorama_pose(db, linked, n);
  CHECK(a[3] == Catch::Approx(7.0));
  CHECK(a[7] == Catch::Approx(-2.0));
  CHECK(a[11] == Catch::Approx(1.4));

  // No pose anywhere: an upright panorama at the origin is the start.
  const auto b = core::nudge_panorama_pose(db, loose, n);
  CHECK(b[3] == Catch::Approx(0.0));
  CHECK(b[7] == Catch::Approx(-2.0));
  CHECK(m4(b).block<3, 1>(0, 1).isApprox(-Eigen::Vector3d::UnitZ()));
}

TEST_CASE("NudgePanoramaPose_CustomUp_RotatesAboutIt", "[core][panorama]") {
  // A y-up world (ARKit-style): yaw about +y keeps elevation against +y.
  const std::array<double, 3> up{0.0, 1.0, 0.0};
  const auto start = core::upright_panorama_pose({0, 0, 0}, 20.0, up);
  CHECK(core::panorama_yaw_deg(start, up) == Catch::Approx(20.0));
  core::PanoramaPoseNudge n;
  n.yaw_deg = 110.0;
  n.up = up;
  const auto out = core::apply_panorama_nudge(start, n);
  CHECK(core::panorama_yaw_deg(out, up) == Catch::Approx(110.0));
  CHECK(m4(out).block<3, 1>(0, 1).isApprox(-Eigen::Vector3d::UnitY()));
}

TEST_CASE("PanoramaAlignDecision_ManualPoseNeedsForce", "[core][panorama]") {
  using D = PanoramaAlignDecision;
  CHECK(core::panorama_align_decision(pano_row(-1, false, "timestamp"), true,
                                      true) == D::skip_unlinked);
  CHECK(core::panorama_align_decision(pano_row(3, false, "timestamp"), false,
                                      false) == D::align);
  CHECK(core::panorama_align_decision(pano_row(3, true, "aligned"), false,
                                      false) == D::skip_aligned);
  CHECK(core::panorama_align_decision(pano_row(3, true, "aligned"), true,
                                      false) == D::align);
  CHECK(core::panorama_align_decision(pano_row(3, true, "manual"), false,
                                      false) == D::skip_manual);
  CHECK(core::panorama_align_decision(pano_row(3, true, "manual"), true,
                                      false) == D::skip_manual);
  CHECK(core::panorama_align_decision(pano_row(3, true, "manual"), false,
                                      true) == D::align);
}

TEST_CASE("SavePanoramaPose_UnknownIdOrEmptySource_Throws",
          "[core][panorama][projectdb]") {
  test_support::TempPath tmp("test_panorama_pose_save");
  ProjectDB db(tmp.path);
  const auto pose = core::upright_panorama_pose({0, 0, 0}, 0);
  CHECK_THROWS_AS(db.save_panorama_pose(42, pose, 1, 0.1), std::out_of_range);
  const int id = db.save_panoramic_image("a.jpg", fake_jpeg());
  CHECK_THROWS_AS(db.save_panorama_pose(id, pose, 1, 0.1, ""),
                  std::invalid_argument);
  CHECK(id > 0);
}
