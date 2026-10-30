// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/panorama_router.hpp"

#include <reusex/core/panorama_pose.hpp>

#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <optional>
#include <string>

namespace rux::database {

namespace {

/// The pose fields `rux get` shows next to id / filename / timestamp.
void add_pose_fields(nlohmann::json &j,
                     const reusex::ProjectDB::PanoramicImage &img) {
  j["pose_source"] = img.pose_source;
  j["pose"] = img.has_pose ? nlohmann::json(img.pose) : nlohmann::json(nullptr);
}

nlohmann::json payload_json(const DataPayload &data) {
  if (std::holds_alternative<nlohmann::json>(data)) {
    const auto &j = std::get<nlohmann::json>(data);
    return j.is_string() ? nlohmann::json::parse(j.get<std::string>()) : j;
  }
  if (std::holds_alternative<std::string>(data))
    return nlohmann::json::parse(std::get<std::string>(data));
  throw std::runtime_error("panorama pose: expected a JSON value");
}

std::optional<double> number_field(const nlohmann::json &o, const char *key) {
  if (!o.contains(key))
    return std::nullopt;
  if (!o.at(key).is_number())
    throw std::runtime_error(std::string("panorama pose: '") + key +
                             "' must be a number");
  return o.at(key).get<double>();
}

} // namespace

std::vector<std::string> PanoramaRouter::list() const {
  auto images = db_->list_panoramic_images();
  std::vector<std::string> names;
  names.reserve(images.size());
  for (const auto &img : images) {
    names.push_back(img.filename);
  }
  return names; // Already sorted by filename from DB query
}

PanoramaRouter::Resolved PanoramaRouter::resolve_item(
    const std::vector<PathComponent> &components) const {
  if (components.empty())
    throw std::runtime_error("Expected a panorama name or index");
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved)
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    return {*resolved, 1};
  }
  if (!components[0].is_item())
    throw std::runtime_error("Expected item or index after collection");
  // The path parser splits on '.', so "hall.jpg.pose" arrives as
  // hall / jpg / pose: the longest prefix that names a stored panorama is
  // the name, the rest is the property.
  for (size_t k = components.size(); k >= 1; --k) {
    std::string name;
    bool items = true;
    for (size_t i = 0; i < k; ++i) {
      items = items && components[i].is_item();
      name += (i ? "." : "") + components[i].value;
    }
    if (items && db_->has_panoramic_image(name))
      return {name, k};
  }
  // Unknown: name it by every component but a trailing property, so the
  // caller's "not found" names the whole file name.
  size_t k = components.size();
  const auto &last = components.back().value;
  if (k > 1 && (last == "pose" || last == "metadata" || last == "image"))
    --k;
  std::string name;
  for (size_t i = 0; i < k; ++i)
    name += (i ? "." : "") + components[i].value;
  return {name, k};
}

nlohmann::json PanoramaRouter::get_metadata(std::string_view filename) const {
  auto images = db_->list_panoramic_images();
  for (const auto &img : images) {
    if (img.filename == filename) {
      nlohmann::json meta;
      meta["id"] = img.id;
      meta["filename"] = img.filename;
      meta["timestamp"] = img.timestamp >= 0.0 ? nlohmann::json(img.timestamp)
                                               : nlohmann::json(nullptr);
      meta["node_id"] = img.node_id >= 0 ? nlohmann::json(img.node_id)
                                         : nlohmann::json(nullptr);
      add_pose_fields(meta, img);
      return meta;
    }
  }
  throw std::runtime_error("Panoramic image not found: " +
                           std::string(filename));
}

std::vector<uint8_t>
PanoramaRouter::get_image_data(std::string_view filename) const {
  cv::Mat image = db_->panoramic_image(filename);
  if (image.empty()) {
    throw std::runtime_error("Panoramic image not found: " +
                             std::string(filename));
  }

  std::vector<uint8_t> buf;
  cv::imencode(".jpg", image, buf, {cv::IMWRITE_JPEG_QUALITY, 95});
  return buf;
}

DataPayload PanoramaRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list with metadata
    auto images = db_->list_panoramic_images();
    nlohmann::json arr = nlohmann::json::array();
    for (const auto &img : images) {
      nlohmann::json j;
      j["id"] = img.id;
      j["filename"] = img.filename;
      j["timestamp"] = img.timestamp >= 0.0 ? nlohmann::json(img.timestamp)
                                            : nlohmann::json(nullptr);
      j["node_id"] = img.node_id >= 0 ? nlohmann::json(img.node_id)
                                      : nlohmann::json(nullptr);
      j["pose_source"] = img.pose_source;
      arr.push_back(std::move(j));
    }
    return arr;
  }

  const auto [item_name, used] = resolve_item(components);
  if (item_name.find('*') != std::string::npos) {
    throw std::runtime_error("Wildcard not expanded: " + item_name);
  }

  if (components.size() == used) {
    // panoramas.myfile → return JPEG image data
    return get_image_data(item_name);
  }
  if (components.size() > used + 1)
    throw std::runtime_error("Unexpected path after panoramas." + item_name +
                             "." + components[used].value);

  // Property access
  const auto &prop = components[used].value;

  if (prop == "metadata") {
    return get_metadata(item_name);
  } else if (prop == "image") {
    return get_image_data(item_name);
  } else if (prop == "pose") {
    const auto meta = get_metadata(item_name);
    return nlohmann::json{{"pose", meta["pose"]},
                          {"pose_source", meta["pose_source"]}};
  } else {
    throw std::runtime_error("Unknown property: " + prop +
                             "\nAvailable properties: metadata, image, pose");
  }
}

void PanoramaRouter::set(const std::vector<PathComponent> &components,
                         const DataPayload &data) {
  const auto [item_name, used] =
      components.empty() ? Resolved{} : resolve_item(components);
  if (!item_name.empty() && !db_->has_panoramic_image(item_name))
    throw std::runtime_error("Panoramic image not found: " + item_name);
  if (components.size() != used + 1 || components[used].value != "pose")
    throw std::runtime_error(
        "Only a panorama's pose can be set: rux set panoramas.<name>.pose "
        "<value>. Use 'rux import 360 <path>' to import panoramic images.");
  int id = -1;
  for (const auto &img : db_->list_panoramic_images())
    if (img.filename == item_name)
      id = img.id;
  if (id < 0)
    throw std::runtime_error("Panoramic image not found: " + item_name);

  const nlohmann::json v = payload_json(data);
  if (v.is_array() && v.size() == 16) {
    // A full row-major 4x4 panorama-optical -> world pose.
    std::array<double, 16> pose{};
    for (size_t i = 0; i < 16; ++i)
      pose[i] = v.at(i).get<double>();
    reusex::core::set_panorama_pose_manual(*db_, id, pose);
  } else if (v.is_array() && (v.size() == 3 || v.size() == 4)) {
    // [x, y, z(, yaw_deg)]: an upright panorama there.
    const auto pose = reusex::core::upright_panorama_pose(
        {v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>()},
        v.size() == 4 ? v.at(3).get<double>() : 0.0);
    reusex::core::set_panorama_pose_manual(*db_, id, pose);
  } else if (v.is_object()) {
    // A nudge: any of x, y, z, yaw_deg; the rest is kept.
    for (const auto &[key, _] : v.items())
      if (key != "x" && key != "y" && key != "z" && key != "yaw_deg")
        throw std::runtime_error("panorama pose: unknown key '" + key +
                                 "' (x, y, z, yaw_deg)");
    reusex::core::PanoramaPoseNudge n;
    n.x = number_field(v, "x");
    n.y = number_field(v, "y");
    n.z = number_field(v, "z");
    n.yaw_deg = number_field(v, "yaw_deg");
    reusex::core::nudge_panorama_pose(*db_, id, n);
  } else {
    throw std::runtime_error(
        "panorama pose: expected a 16-number row-major array, [x,y,z(,yaw)] "
        "or an object with any of x, y, z, yaw_deg");
  }
  spdlog::info("Set manual pose of panoramic image '{}'", item_name);
}

void PanoramaRouter::del(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    throw std::runtime_error("Cannot delete entire collection. "
                             "Specify a filename: panoramas.<filename>");
  }

  const auto [item_name, used] = resolve_item(components);
  if (used != components.size())
    throw std::runtime_error("A panorama is deleted whole: rux del "
                             "panoramas.<name>");

  if (!db_->has_panoramic_image(item_name)) {
    throw std::runtime_error("Panoramic image not found: " + item_name);
  }

  db_->delete_panoramic_image(item_name);
  spdlog::info("Deleted panoramic image '{}'", item_name);
}

} // namespace rux::database
