// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for panoramic image resources
 *
 * Handles paths like:
 * - panoramas → list all (JSON array of {id, filename, timestamp, node_id})
 * - panoramas.myfile → get JPEG image data (binary)
 * - panoramas.myfile.metadata → get metadata (JSON)
 * - panoramas.myfile.image → get JPEG image data (binary)
 * - panoramas[0] → get first panorama by filename order (binary)
 * - panoramas[0].metadata → get first panorama metadata (JSON)
 * - panoramas.myfile.pose → get / set its pose (JSON). Set takes a
 *   16-number row-major array, [x,y,z(,yaw_deg)] (upright there) or a
 *   nudge object with any of x, y, z, yaw_deg; stored as pose_source
 *   "manual" (core/panorama_pose.hpp)
 */
class PanoramaRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

    private:
  nlohmann::json get_metadata(std::string_view filename) const;
  std::vector<uint8_t> get_image_data(std::string_view filename) const;
};

} // namespace rux::database
