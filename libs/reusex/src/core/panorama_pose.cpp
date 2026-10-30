// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/panorama_pose.hpp"
#include "core/logging.hpp"
#include "reusex/core/pose_graph.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>

namespace reusex::core {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

Eigen::Vector3d to_vec(const std::array<double, 3> &a) {
  return {a[0], a[1], a[2]};
}

Eigen::Vector3d unit_up(const std::array<double, 3> &up) {
  const Eigen::Vector3d u = to_vec(up);
  if (!u.allFinite() || !(u.norm() > 1e-12))
    throw std::invalid_argument("panorama pose: 'up' must be a finite, "
                                "non-zero direction");
  return u.normalized();
}

/// Reference heading (yaw 0) in the plane normal to @p up: world x
/// projected, or world y when x is (nearly) parallel to up.
Eigen::Vector3d reference_heading(const Eigen::Vector3d &up) {
  Eigen::Vector3d r = Eigen::Vector3d::UnitX() - up.x() * up;
  if (r.norm() < 1e-6)
    r = Eigen::Vector3d::UnitY() - up.y() * up;
  return r.normalized();
}

/// Heading of @p dir about @p up in radians, or NaN when @p dir is
/// (nearly) parallel to @p up.
double heading(const Eigen::Vector3d &dir, const Eigen::Vector3d &up) {
  const Eigen::Vector3d h = dir - dir.dot(up) * up;
  if (h.norm() < 1e-9)
    return std::nan("");
  const Eigen::Vector3d ref = reference_heading(up);
  return std::atan2(up.cross(ref).dot(h), ref.dot(h));
}

double require_finite(double v, const char *what) {
  if (!std::isfinite(v))
    throw std::invalid_argument(std::string("panorama pose: ") + what +
                                " is not finite");
  return v;
}

} // namespace

void validate_panorama_pose(const PanoramaPose &pose) {
  for (double v : pose)
    if (!std::isfinite(v))
      throw std::invalid_argument("panorama pose: non-finite element");
  if (!pose_graph::is_rigid_transform(pose))
    throw std::invalid_argument(
        "panorama pose: not a rigid transform (bottom row must be [0 0 0 1] "
        "and the 3x3 block a proper rotation, orthonormal with det +1)");
}

PanoramaPose upright_panorama_pose(const std::array<double, 3> &position,
                                   double yaw_deg,
                                   const std::array<double, 3> &up) {
  for (double v : position)
    require_finite(v, "position");
  require_finite(yaw_deg, "yaw");
  const Eigen::Vector3d u = unit_up(up);
  const Eigen::Vector3d ref = reference_heading(u);
  const Eigen::Vector3d fwd =
      Eigen::AngleAxisd(yaw_deg * kDeg, u) * ref; // optical z
  const Eigen::Vector3d down = -u;                // optical y
  const Eigen::Vector3d right = down.cross(fwd);  // optical x = y x z
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 1>(0, 0) = right;
  m.block<3, 1>(0, 1) = down;
  m.block<3, 1>(0, 2) = fwd;
  m.block<3, 1>(0, 3) = to_vec(position);
  return pose_graph::row_major_from_matrix(m);
}

double panorama_yaw_deg(const PanoramaPose &pose,
                        const std::array<double, 3> &up) {
  const Eigen::Vector3d u = unit_up(up);
  const Eigen::Matrix4d m = pose_graph::matrix_from_row_major(pose);
  double a = heading(m.block<3, 1>(0, 2), u);
  if (std::isnan(a)) {
    // Looking straight up/down: the optical up axis (-y) carries the heading.
    a = heading(-m.block<3, 1>(0, 1), u);
    if (std::isnan(a))
      a = 0.0;
  }
  double deg = a / kDeg;
  if (deg <= -180.0)
    deg += 360.0;
  return deg;
}

PanoramaPose apply_panorama_nudge(const PanoramaPose &pose,
                                  const PanoramaPoseNudge &nudge) {
  Eigen::Matrix4d m = pose_graph::matrix_from_row_major(pose);
  if (nudge.yaw_deg) {
    const Eigen::Vector3d u = unit_up(nudge.up);
    const double target = require_finite(*nudge.yaw_deg, "yaw");
    const double delta = (target - panorama_yaw_deg(pose, nudge.up)) * kDeg;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(delta, u).toRotationMatrix();
    m.topLeftCorner<3, 3>() = R * m.topLeftCorner<3, 3>();
  }
  if (nudge.x)
    m(0, 3) = require_finite(*nudge.x, "x");
  if (nudge.y)
    m(1, 3) = require_finite(*nudge.y, "y");
  if (nudge.z)
    m(2, 3) = require_finite(*nudge.z, "z");
  return pose_graph::row_major_from_matrix(m);
}

ProjectDB::PanoramicImage panorama_record(const ProjectDB &db, int id) {
  for (auto &p : db.list_panoramic_images())
    if (p.id == id)
      return p;
  throw std::out_of_range("no panorama with id " + std::to_string(id));
}

std::optional<PanoramaPose>
effective_panorama_pose(const ProjectDB &db,
                        const ProjectDB::PanoramicImage &pano) {
  if (pano.has_pose)
    return pano.pose;
  if (pano.node_id >= 0 && db.has_sensor_frame_pose(pano.node_id)) {
    const auto f = db.sensor_frame_pose(pano.node_id);
    return upright_panorama_pose({f[3], f[7], f[11]}, 0.0);
  }
  return std::nullopt;
}

void set_panorama_pose_manual(ProjectDB &db, int id, const PanoramaPose &pose) {
  validate_panorama_pose(pose);
  db.save_panorama_pose(id, pose, /*inliers=*/-1, /*rms=*/-1.0,
                        kPanoramaPoseManual);
  reusex::info("panorama {}: manual pose set, centre ({:.3f}, {:.3f}, {:.3f})",
               id, pose[3], pose[7], pose[11]);
}

PanoramaPose nudge_panorama_pose(ProjectDB &db, int id,
                                 const PanoramaPoseNudge &nudge) {
  if (nudge.empty())
    throw std::invalid_argument(
        "panorama nudge: set at least one of x, y, z, yaw");
  const auto pano = panorama_record(db, id);
  PanoramaPose start;
  if (pano.has_pose) {
    start = pano.pose;
  } else {
    // The levelled placement it is shown at, upright about nudge.up.
    std::array<double, 3> centre{0.0, 0.0, 0.0};
    if (pano.node_id >= 0 && db.has_sensor_frame_pose(pano.node_id)) {
      const auto f = db.sensor_frame_pose(pano.node_id);
      centre = {f[3], f[7], f[11]};
    } else {
      reusex::info("panorama {} has no pose yet; the nudge starts from an "
                   "upright panorama at the origin facing yaw 0",
                   id);
    }
    start = upright_panorama_pose(centre, 0.0, nudge.up);
  }
  const PanoramaPose out = apply_panorama_nudge(start, nudge);
  set_panorama_pose_manual(db, id, out);
  return out;
}

PanoramaAlignDecision
panorama_align_decision(const ProjectDB::PanoramicImage &pano, bool overwrite,
                        bool force) {
  if (pano.node_id < 0)
    return PanoramaAlignDecision::skip_unlinked;
  if (pano.has_pose && pano.pose_source == kPanoramaPoseManual && !force)
    return PanoramaAlignDecision::skip_manual;
  if (pano.has_pose && !(overwrite || force))
    return PanoramaAlignDecision::skip_aligned;
  return PanoramaAlignDecision::align;
}

} // namespace reusex::core
