// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/storey_lookup.hpp"

#include <cmath>
#include <limits>

namespace reusex::core {

namespace {

bool finite_frame(const StoreyRecord &s) {
  for (double v : s.level_transform)
    if (!std::isfinite(v))
      return false;
  const auto &m = s.level_transform;
  return m[8] * m[8] + m[9] * m[9] + m[10] * m[10] > 0.0;
}

/// World point of @p s's floor on its level z axis: level (0, 0, z_floor)
/// through the inverse of the rigid world -> level transform.
std::array<double, 3> floor_point(const StoreyRecord &s) {
  const auto &m = s.level_transform;
  const double l[3] = {-m[3], -m[7], s.z_floor - m[11]};
  // R^T * (l - t)
  return {m[0] * l[0] + m[4] * l[1] + m[8] * l[2],
          m[1] * l[0] + m[5] * l[1] + m[9] * l[2],
          m[2] * l[0] + m[6] * l[1] + m[10] * l[2]};
}

} // namespace

double level_z(const StoreyRecord &s, const std::array<double, 3> &world) {
  const auto &m = s.level_transform;
  return m[8] * world[0] + m[9] * world[1] + m[10] * world[2] + m[11];
}

std::array<double, 3> storey_up(const StoreyRecord &s) {
  // world -> level is rigid, so level +z in world is the rotation's third
  // row (R^T e_z).
  const auto &m = s.level_transform;
  const double n = std::sqrt(m[8] * m[8] + m[9] * m[9] + m[10] * m[10]);
  if (!(n > 0.0) || !std::isfinite(n))
    return {0.0, 0.0, 1.0};
  return {m[8] / n, m[9] / n, m[10] / n};
}

std::optional<int> which_storey(const std::vector<StoreyRecord> &storeys,
                                const std::array<double, 3> &world,
                                const WhichStoreyOptions &opt) {
  for (double c : world)
    if (!std::isfinite(c))
      return std::nullopt;

  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kTie = 1e-9; // [m]

  // Records with an unusable frame take no part.
  std::vector<const StoreyRecord *> rows;
  rows.reserve(storeys.size());
  for (const auto &s : storeys)
    if (finite_frame(s) && std::isfinite(s.z_floor) &&
        std::isfinite(s.z_ceiling))
      rows.push_back(&s);
  if (rows.empty())
    return std::nullopt;

  const StoreyRecord *inside = nullptr;
  double inside_above_floor = kInf; // z - z_floor of the current pick
  const StoreyRecord *nearest = nullptr;
  double nearest_d = kInf;
  double nearest_floor = -kInf;

  for (const StoreyRecord *sp : rows) {
    const StoreyRecord &s = *sp;
    const double z = level_z(s, world);
    double hi = s.z_ceiling;
    if (!(s.z_ceiling > s.z_floor)) {
      // Unknown ceiling: open upward to the next storey's floor (measured
      // in this storey's frame), unbounded only for the top storey.
      hi = kInf;
      for (const StoreyRecord *tp : rows) {
        if (tp == sp)
          continue;
        const double f = level_z(s, floor_point(*tp));
        if (f > s.z_floor + kTie && f < hi)
          hi = f;
      }
    }
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
    // Ties go to the upper storey: the one whose floor the point is below.
    const double floor_rank = z < s.z_floor ? 1.0 : 0.0;
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
