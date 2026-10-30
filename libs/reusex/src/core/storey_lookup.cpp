// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/storey_lookup.hpp"

#include <cmath>
#include <limits>

namespace reusex::core {

double level_z(const StoreyRecord &s, const std::array<double, 3> &world) {
  const auto &m = s.level_transform;
  return m[8] * world[0] + m[9] * world[1] + m[10] * world[2] + m[11];
}

std::array<double, 3> storey_up(const StoreyRecord &s) {
  // world -> level is rigid, so level +z in world is the rotation's third
  // row (R^T e_z).
  const auto &m = s.level_transform;
  const double n = std::sqrt(m[8] * m[8] + m[9] * m[9] + m[10] * m[10]);
  if (!(n > 0.0))
    return {0.0, 0.0, 1.0};
  return {m[8] / n, m[9] / n, m[10] / n};
}

std::optional<int> which_storey(const std::vector<StoreyRecord> &storeys,
                                const std::array<double, 3> &world,
                                const WhichStoreyOptions &opt) {
  if (storeys.empty())
    return std::nullopt;
  for (double c : world)
    if (!std::isfinite(c))
      return std::nullopt;

  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kTie = 1e-9; // [m]
  const StoreyRecord *inside = nullptr;
  double inside_above_floor = kInf; // z - z_floor of the current pick
  const StoreyRecord *nearest = nullptr;
  double nearest_d = kInf;
  double nearest_floor = -kInf;

  for (const auto &s : storeys) {
    const double z = level_z(s, world);
    const bool open_top = !(s.z_ceiling > s.z_floor);
    const double hi = open_top ? kInf : s.z_ceiling;
    if (z >= s.z_floor && z < hi) {
      const double above = z - s.z_floor;
      if (!inside || above < inside_above_floor ||
          (above == inside_above_floor && s.idx < inside->idx)) {
        inside = &s;
        inside_above_floor = above;
      }
      continue;
    }
    // Distance to the band, along this storey's vertical.
    const double d = z < s.z_floor ? s.z_floor - z : z - hi;
    // Ties go to the upper storey: the one whose floor is higher in world
    // terms is the one whose floor the point is below (z < z_floor).
    const bool upper = z < s.z_floor;
    const double floor_rank = upper ? 1.0 : 0.0;
    // Distances within kTie are a tie (a mid-gap point computed in floating
    // point lands a few ulp off either side).
    if (!nearest || d < nearest_d - kTie ||
        (d <= nearest_d + kTie && floor_rank > nearest_floor)) {
      nearest = &s;
      nearest_d = d;
      nearest_floor = floor_rank;
    }
  }
  if (inside)
    return inside->idx;
  if (opt.clamp_to_nearest && nearest)
    return nearest->idx;
  return std::nullopt;
}

} // namespace reusex::core
