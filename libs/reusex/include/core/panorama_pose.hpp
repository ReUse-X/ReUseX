// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Placing a 360 panorama by hand (reusex-3iy8.5): the manual pose write path
/// that bypasses content alignment (`slam/PanoramaAlignment.hpp`).
///
/// A panorama pose is row-major 4x4, panorama-optical -> world, in the
/// convention of `geometry/EquirectProjection.hpp`: optical x right, y down,
/// z forward (the centre column of the equirect). A pose written here is
/// stored with `pose_source = "manual"`, no inliers and no RMS; `rux align
/// 360` then leaves it alone unless forced (panorama_align_decision()).
///
/// "Yaw" is the heading of the optical forward axis about an `up` direction
/// (world +z unless given; core::storey_up() gives a storey's measured
/// vertical): 0 = forward along the world x axis projected onto the plane
/// normal to `up` (world y when x is parallel to `up`), positive counter-
/// clockwise seen from above. A camera looking straight up or down takes the
/// heading of its optical up axis (-y) instead.

#include "reusex/core/ProjectDB.hpp"

#include <array>
#include <optional>
#include <string_view>

namespace reusex::core {

inline constexpr std::string_view kPanoramaPoseManual = "manual";
inline constexpr std::string_view kPanoramaPoseAligned = "aligned";

using PanoramaPose = std::array<double, 16>;

/// Throws std::invalid_argument naming the problem unless @p pose is a
/// finite rigid transform: bottom row [0 0 0 1], orthonormal rotation with
/// det +1 (core::pose_graph::is_rigid_transform's tolerances).
void validate_panorama_pose(const PanoramaPose &pose);

/// An upright panorama (horizon level: optical y = -up) at @p position,
/// facing @p yaw_deg about @p up.
PanoramaPose
upright_panorama_pose(const std::array<double, 3> &position, double yaw_deg,
                      const std::array<double, 3> &up = {0.0, 0.0, 1.0});

/// The heading of @p pose's optical forward axis about @p up, in degrees
/// (-180, 180]. See the file comment for the reference direction.
double panorama_yaw_deg(const PanoramaPose &pose,
                        const std::array<double, 3> &up = {0.0, 0.0, 1.0});

/// A partial pose edit: set any of the world position components and/or the
/// yaw. Unset fields keep their value; a yaw edit rotates about @p up through
/// the panorama centre, so pitch and roll against @p up are kept.
struct PanoramaPoseNudge {
  std::optional<double> x, y, z; ///< world position [m]
  std::optional<double> yaw_deg; ///< absolute heading, see panorama_yaw_deg()
  std::array<double, 3> up{0.0, 0.0, 1.0};

  [[nodiscard]] bool empty() const { return !x && !y && !z && !yaw_deg; }
};

/// @p pose with @p nudge applied (pure). @throws std::invalid_argument for a
/// non-finite field or a zero @p nudge.up.
PanoramaPose apply_panorama_nudge(const PanoramaPose &pose,
                                  const PanoramaPoseNudge &nudge);

/// The panorama row with @p id. @throws std::out_of_range when none.
ProjectDB::PanoramicImage panorama_record(const ProjectDB &db, int id);

/// The pose a panorama is shown at: its own (aligned or manual) pose, else,
/// for a timestamp placement, an upright panorama at its linked sensor
/// frame's centre facing yaw 0 (world +x, horizon level, about world +z) —
/// the levelled basis the viewers draw it with (frontend `LEVELLED_BASIS`,
/// core/instance_evidence.cpp), never the phone camera's rotation — else
/// none.
std::optional<PanoramaPose>
effective_panorama_pose(const ProjectDB &db,
                        const ProjectDB::PanoramicImage &pano);

/// Store @p pose as panorama @p id's pose, `pose_source = "manual"`.
/// @throws std::invalid_argument for an invalid pose (nothing is written),
///         std::out_of_range for an unknown id.
void set_panorama_pose_manual(ProjectDB &db, int id, const PanoramaPose &pose);

/// Nudge panorama @p id: start from its own pose; with none, from an upright
/// panorama (about nudge.up, yaw 0) at its linked frame's centre, or at the
/// origin when it has no posed frame either; apply @p nudge and store the
/// result as a manual pose. @return the stored pose.
/// @throws std::invalid_argument for an empty nudge or a bad field,
///         std::out_of_range for an unknown id.
PanoramaPose nudge_panorama_pose(ProjectDB &db, int id,
                                 const PanoramaPoseNudge &nudge);

/// What content alignment (`rux align 360`) does with one panorama.
enum class PanoramaAlignDecision {
  align,         ///< align it
  skip_unlinked, ///< no linked sensor frame to seed from
  skip_aligned,  ///< already aligned; re-align only with overwrite
  skip_manual,   ///< placed by hand; re-align only with force
};

/// The decision for @p pano. A manual pose is a user decision, so it is kept
/// even with @p overwrite; only @p force replaces it (force implies
/// overwrite).
PanoramaAlignDecision
panorama_align_decision(const ProjectDB::PanoramicImage &pano, bool overwrite,
                        bool force);

} // namespace reusex::core
