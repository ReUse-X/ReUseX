// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Which storey a world point is on (reusex-3iy8.4).
///
/// A storey is the band [z_floor, z_ceiling) of its own level frame
/// (`StoreyRecord::level_transform`, world -> level, row-major 4x4; the
/// convention of `geometry/level_frame.hpp` and `segmentation/level.hpp`'s
/// `detect_storeys()`). Storeys may share one frame (the usual case) or each
/// carry its own (`per_storey_frame`); every band is tested in its own frame.
///
/// Plain arrays, no Eigen, so anything from core up (and ruxd) can call it
/// with a panorama position, a sensor-frame position or a clicked point.

#include "reusex/core/solid_model.hpp"

#include <array>
#include <optional>
#include <vector>

namespace reusex::core {

struct WhichStoreyOptions {
  /// A point inside no band (below the lowest floor, above the highest
  /// ceiling, or in a gap between two storeys, e.g. inside a slab) goes to
  /// the storey whose band is nearest, measured along each storey's level z.
  /// A tie (within 1e-9 m) goes to the upper storey (the floor one would
  /// stand on).
  /// Off (default): such a point has no storey.
  bool clamp_to_nearest = false;
};

/// The point's height in @p s's level frame: row 3 of the level transform
/// applied to @p world.
double level_z(const StoreyRecord &s, const std::array<double, 3> &world);

/// World direction of @p s's level-frame +z (its measured vertical), unit.
std::array<double, 3> storey_up(const StoreyRecord &s);

/// The `idx` of the storey whose band holds @p world.
///
/// - Bands are half-open, [z_floor, z_ceiling): a point exactly on storey
///   k+1's floor, which is also storey k's ceiling, is on storey k+1.
/// - A storey with z_ceiling <= z_floor (ceiling unknown, e.g. a level
///   imported from a plan that only has elevations) is open upward.
/// - When several bands hold the point (overlapping bands, an open-topped
///   storey under another one) the storey whose floor is nearest below the
///   point wins; equal floors go to the lower idx.
/// - Outside every band: std::nullopt, or with `clamp_to_nearest` the
///   nearest band's storey (see WhichStoreyOptions).
/// - No storeys, or a non-finite point: std::nullopt.
std::optional<int> which_storey(const std::vector<StoreyRecord> &storeys,
                                const std::array<double, 3> &world,
                                const WhichStoreyOptions &opt = {});

} // namespace reusex::core
