// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core::which_storey (reusex-3iy8.4): a world point -> the storey idx whose
// [z_floor, z_ceiling) band, in its own level frame, holds it.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/storey_lookup.hpp>
#include <reusex/geometry/level_frame.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

using reusex::core::StoreyRecord;
using reusex::core::which_storey;
using reusex::core::WhichStoreyOptions;

namespace {

StoreyRecord storey(int idx, double floor, double ceiling,
                    const std::array<double, 16> &frame = {
                        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}) {
  StoreyRecord s;
  s.idx = idx;
  s.z_floor = floor;
  s.z_ceiling = ceiling;
  s.level_transform = frame;
  return s;
}

/// Two storeys in the identity frame: [0, 2.6) and [3.0, 5.6); 2.6..3.0 is
/// the slab between them.
std::vector<StoreyRecord> two_storeys() {
  return {storey(0, 0.0, 2.6), storey(1, 3.0, 5.6)};
}

WhichStoreyOptions clamped() {
  WhichStoreyOptions o;
  o.clamp_to_nearest = true;
  return o;
}

} // namespace

TEST_CASE("WhichStorey_InsideBands_ReturnsTheirIdx", "[core][storeys]") {
  const auto s = two_storeys();
  CHECK(which_storey(s, {1.0, 2.0, 1.5}) == 0);
  CHECK(which_storey(s, {1.0, 2.0, 4.0}) == 1);
  // Half-open: the floor belongs, the ceiling does not.
  CHECK(which_storey(s, {0.0, 0.0, 0.0}) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 3.0}) == 1);
  CHECK_FALSE(which_storey(s, {0.0, 0.0, 5.6}).has_value());
}

TEST_CASE("WhichStorey_OrderOfRecords_DoesNotMatter", "[core][storeys]") {
  const std::vector<StoreyRecord> s = {storey(1, 3.0, 5.6),
                                       storey(0, 0.0, 2.6)};
  CHECK(which_storey(s, {0.0, 0.0, 1.0}) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 4.0}) == 1);
}

TEST_CASE("WhichStorey_OutsideEveryBand_NulloptUnlessClamped",
          "[core][storeys]") {
  const auto s = two_storeys();
  // Below the lowest floor, above the highest ceiling, inside the slab gap.
  CHECK_FALSE(which_storey(s, {0.0, 0.0, -0.5}).has_value());
  CHECK_FALSE(which_storey(s, {0.0, 0.0, 9.0}).has_value());
  CHECK_FALSE(which_storey(s, {0.0, 0.0, 2.8}).has_value());

  CHECK(which_storey(s, {0.0, 0.0, -0.5}, clamped()) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 9.0}, clamped()) == 1);
  CHECK(which_storey(s, {0.0, 0.0, 2.7}, clamped()) == 0); // nearer storey 0
  CHECK(which_storey(s, {0.0, 0.0, 2.9}, clamped()) == 1); // nearer storey 1
  // Exactly mid-gap: the upper storey (its floor is the one stood on).
  CHECK(which_storey(s, {0.0, 0.0, 2.8}, clamped()) == 1);
}

TEST_CASE("WhichStorey_NoStoreysOrNonFinitePoint_Nullopt", "[core][storeys]") {
  CHECK_FALSE(which_storey({}, {0.0, 0.0, 1.0}, clamped()).has_value());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(
      which_storey(two_storeys(), {0.0, nan, 1.0}, clamped()).has_value());
}

TEST_CASE("WhichStorey_UnknownCeiling_IsOpenUpward", "[core][storeys]") {
  // Levels with elevations only (z_ceiling == z_floor, as a plan import
  // without heights gives): each band runs up to the next floor.
  const std::vector<StoreyRecord> s = {storey(0, 0.0, 0.0),
                                       storey(1, 3.2, 3.2)};
  CHECK(which_storey(s, {0.0, 0.0, 1.0}) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 3.2}) == 1);
  CHECK(which_storey(s, {0.0, 0.0, 50.0}) == 1);
  CHECK_FALSE(which_storey(s, {0.0, 0.0, -0.1}).has_value());
}

TEST_CASE("WhichStorey_SharedNonIdentityFrame_TestsLevelZ", "[core][storeys]") {
  // A tilted vertical and a floor point away from the origin: the shared
  // frame detect_storeys() writes (geometry::make_level_frame).
  const Eigen::Vector3d vertical =
      Eigen::Vector3d(0.15, -0.1, 1.0).normalized();
  const Eigen::Vector3d floor_pt(10.0, -4.0, 37.0);
  const auto frame =
      reusex::geometry::make_level_frame(vertical, floor_pt).to_array();
  const std::vector<StoreyRecord> s = {storey(0, 0.0, 2.6, frame),
                                       storey(1, 3.0, 5.6, frame)};

  auto at = [&](double h) {
    const Eigen::Vector3d w = floor_pt + h * vertical;
    return std::array<double, 3>{w.x(), w.y(), w.z()};
  };
  CHECK(which_storey(s, at(1.0)) == 0);
  CHECK(which_storey(s, at(4.0)) == 1);
  CHECK_FALSE(which_storey(s, at(2.8)).has_value());
  CHECK_FALSE(which_storey(s, at(-1.0)).has_value());
  // 1 m above the floor point in world z, but 30 m along +x on a floor
  // tilted up toward +x: level z = (0.15 * 30 + 1) / |v| ~ 5.41, storey 1.
  const Eigen::Vector3d side = floor_pt + Eigen::Vector3d(30.0, 0.0, 1.0);
  const std::array<double, 3> side_w{side.x(), side.y(), side.z()};
  CHECK(reusex::core::level_z(s[0], side_w) ==
        Catch::Approx(5.5 / Eigen::Vector3d(0.15, -0.1, 1.0).norm()));
  CHECK(which_storey(s, side_w) == 1);

  const auto up = reusex::core::storey_up(s[0]);
  CHECK(up[0] == Catch::Approx(vertical.x()));
  CHECK(up[1] == Catch::Approx(vertical.y()));
  CHECK(up[2] == Catch::Approx(vertical.z()));
}

TEST_CASE("WhichStorey_PerStoreyFrames_EachBandInItsOwnFrame",
          "[core][storeys]") {
  // Storey 1 has its own frame (per_storey_frame): its level z is offset so
  // that its floor, at world z = 3, reads as level z = 0.
  const std::array<double, 16> lifted = {1, 0, 0, 0,  0, 1, 0, 0,
                                         0, 0, 1, -3, 0, 0, 0, 1};
  const std::vector<StoreyRecord> s = {storey(0, 0.0, 2.6),
                                       storey(1, 0.0, 2.6, lifted)};
  CHECK(which_storey(s, {0.0, 0.0, 1.0}) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 4.0}) == 1);
  CHECK_FALSE(which_storey(s, {0.0, 0.0, 2.8}).has_value());
}

TEST_CASE("WhichStorey_OverlappingBands_NearestFloorBelowWins",
          "[core][storeys]") {
  // Storey 0's measured ceiling reaches past storey 1's floor.
  const std::vector<StoreyRecord> s = {storey(0, 0.0, 3.4),
                                       storey(1, 3.0, 5.6)};
  CHECK(which_storey(s, {0.0, 0.0, 2.9}) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 3.2}) == 1);
}

TEST_CASE("WhichStorey_OpenTopUnderClosedStorey_CappedAtNextFloor",
          "[core][storeys]") {
  // Ground floor with an unknown ceiling under a measured upper storey: a
  // point on the roof above the upper storey is not on the ground floor.
  const std::vector<StoreyRecord> s = {storey(0, 0.0, 0.0),
                                       storey(1, 3.0, 5.6)};
  CHECK(which_storey(s, {0.0, 0.0, 2.9}) == 0);
  CHECK(which_storey(s, {0.0, 0.0, 3.1}) == 1);
  CHECK_FALSE(which_storey(s, {0.0, 0.0, 7.0}).has_value());
  CHECK(which_storey(s, {0.0, 0.0, 7.0}, clamped()) == 1);

  // The cap is the next floor's world point measured in the open storey's
  // own frame: upper storey in a frame lifted by 3 m, floor at level 0.
  const std::array<double, 16> lifted = {1, 0, 0, 0,  0, 1, 0, 0,
                                         0, 0, 1, -3, 0, 0, 0, 1};
  const std::vector<StoreyRecord> t = {storey(0, 0.0, 0.0),
                                       storey(1, 0.0, 2.6, lifted)};
  CHECK(which_storey(t, {0.0, 0.0, 2.9}) == 0);
  CHECK(which_storey(t, {0.0, 0.0, 3.5}) == 1);
  CHECK_FALSE(which_storey(t, {0.0, 0.0, 6.0}).has_value());
}

TEST_CASE("WhichStorey_NonFiniteFrame_RecordIgnored", "[core][storeys]") {
  auto broken = storey(1, 0.0, 10.0);
  broken.level_transform[11] = std::numeric_limits<double>::quiet_NaN();
  const std::vector<StoreyRecord> s = {storey(0, 0.0, 2.6), broken};
  CHECK(which_storey(s, {0.0, 0.0, 1.0}) == 0);
  CHECK_FALSE(which_storey(s, {0.0, 0.0, 5.0}, clamped()) == 1);
  CHECK(which_storey(s, {0.0, 0.0, 5.0}, clamped()) == 0);
  CHECK_FALSE(which_storey({broken}, {0.0, 0.0, 5.0}, clamped()).has_value());
}
