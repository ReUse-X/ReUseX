// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A small hand-written synthetic Revit model in the shape the Speckle v3
// Revit connector publishes (already resolved: references inlined); no
// customer data. Shared by the io mapping tests (test_speckle_revit.cpp) and
// the import_speckle job-stage tests (test_import_stages.cpp).

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace reusex::test_support {

/// Builds the synthetic model in one unit: every length is given in mm and
/// divided by `per_mm` (1 for "mm", 10 for "cm", 1000 for "m").
struct RevitFixture {
  using json = nlohmann::json;

  std::string units = "mm";
  double per_mm = 1.0;
  double shift_x_mm = 0.0; ///< added to every x (a model far from origin)

  double L(double mm) const { return mm / per_mm; }

  /// Definition geometry lives in the instance's own frame: never shifted.
  RevitFixture local() const {
    RevitFixture f = *this;
    f.shift_x_mm = 0.0;
    return f;
  }

  json param(const std::string &idn, json value, bool length = false) const {
    json p = {{"value", value}, {"name", idn}, {"internalDefinitionName", idn}};
    if (length) {
      p["value"] = L(value.get<double>());
      p["units"] = units == "mm"   ? "Millimeters"
                   : units == "cm" ? "Centimeters"
                                   : "Meters";
    }
    return p;
  }

  /// Axis-aligned box mesh; quads = true writes 6 quads (n = 4), else 12
  /// triangles, the first in the legacy n = 0 encoding.
  json box(double x0, double y0, double z0, double x1, double y1, double z1,
           bool quads = false) const {
    std::vector<double> v;
    for (int i = 0; i < 8; ++i) {
      v.push_back(L((i & 1 ? x1 : x0) + shift_x_mm));
      v.push_back(L(i & 2 ? y1 : y0));
      v.push_back(L(i & 4 ? z1 : z0));
    }
    const int q[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                         {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    std::vector<int> f;
    bool first = true;
    for (const auto &face : q) {
      if (quads) {
        f.insert(f.end(), {4, face[0], face[1], face[2], face[3]});
      } else {
        f.insert(f.end(), {first ? 0 : 3, face[0], face[1], face[2]});
        f.insert(f.end(), {3, face[0], face[2], face[3]});
        first = false;
      }
    }
    return {{"speckle_type", "Objects.Geometry.Mesh"},
            {"units", units},
            {"vertices", v},
            {"faces", f}};
  }

  json element(const std::string &category, const std::string &app_id,
               const std::string &level, json display,
               json params = json::object(),
               json extra_props = json::object()) const {
    json props = {{"elementId", app_id + "-eid"},
                  {"Parameters", {{"Instance Parameters", {{"G", params}}}}}};
    props.update(extra_props);
    return {
        {"speckle_type", "Objects.Data.DataObject:Objects.Data.RevitObject"},
        {"category", category},
        {"family", category + " family"},
        {"type", category + " type"},
        {"name", category + " " + app_id},
        {"level", level},
        {"units", units},
        {"applicationId", app_id},
        {"location", nullptr},
        {"properties", props},
        {"displayValue", display}};
  }

  json proxy(const std::string &def, std::vector<double> m) const {
    m[3] = L(m[3] + shift_x_mm);
    m[7] = L(m[7]);
    m[11] = L(m[11]);
    return {{"speckle_type", "Speckle.Core.Models.Instances.InstanceProxy"},
            {"definitionId", def},
            {"transform", m},
            {"units", units},
            {"maxDepth", 1}};
  }

  json wall_params(double width_mm, const std::string &function) const {
    return {{"base", param("WALL_BASE_CONSTRAINT", "L0")},
            {"off", param("WALL_BASE_OFFSET", 0.0, true)},
            {"w", param("WALL_ATTR_WIDTH_PARAM", width_mm, true)},
            {"fn", param("FUNCTION_PARAM", function)}};
  }

  json level(const std::string &name, double elevation_mm,
             const std::string &app, bool storey = true) const {
    json props = {{"elementId", app + "-eid"}};
    if (!storey)
      props["Parameters"] = {
          {"Instance Parameters",
           {{"Identity Data",
             {{"bs", param("LEVEL_IS_BUILDING_STORY", "No")}}}}}};
    return {{"speckle_type", "Objects.Other.LevelProxy"},
            {"applicationId", app},
            {"objects", json::array()},
            {"value",
             {{"speckle_type", "Objects.Data.DataObject"},
              {"name", name},
              {"elevation", L(elevation_mm)},
              {"units", units},
              {"properties", props}}}};
  }

  /// Levels L0 (elevation 10 000 mm) and L1 (13 000 mm), with Revit internal
  /// z = elevation - 9 500 mm (L0's floor at internal z 500 mm).
  json model() const {
    const double z = 500; // L0 floor, internal
    json wall_a = element("Walls", "wall-a", "L0",
                          json::array({box(0, -100, z, 5000, 100, z + 3000)}),
                          wall_params(200, "Exterior"));
    // An L-shaped footprint: no straight wall.
    json wall_b = element("Walls", "wall-b", "L0",
                          json::array({box(0, 1000, z, 3000, 1200, z + 3000)}),
                          wall_params(200, "Interior"));
    wall_b["displayValue"].push_back(box(0, 1000, z, 200, 4000, z + 3000));
    // A 7 mm finish layer.
    json wall_c = element("Walls", "wall-c", "L0",
                          json::array({box(0, 100, z, 5000, 107, z + 3000)}),
                          wall_params(7, "Interior"));
    // Door: hosted by parentApplicationId, via an instance proxy.
    json door =
        element("Doors", "door-1", "L0",
                json::array({proxy("DEF-DOOR", {1, 0, 0, 2000, 0, 1, 0, 0, 0, 0,
                                                1, z, 0, 0, 0, 1})}),
                {{"rw", param("FAMILY_ROUGH_WIDTH_PARAM", 1000.0, true)},
                 {"rh", param("FAMILY_ROUGH_HEIGHT_PARAM", 2200.0, true)},
                 {"sill", param("INSTANCE_SILL_HEIGHT_PARAM", 0.0, true)}},
                {{"parentApplicationId", "wall-a"}});
    // Window: no parent, found on wall A geometrically.
    json window =
        element("Windows", "win-1", "L0",
                json::array({box(3500, -50, z + 900, 4300, 50, z + 2100)}));
    // Furniture: a proxy rotated 90 degrees about z, at (1000, 2000).
    json chair =
        element("Furniture", "chair-1", "L0",
                json::array({proxy("DEF-CHAIR", {0, -1, 0, 1000, 1, 0, 0, 2000,
                                                 0, 0, 1, z, 0, 0, 0, 1})}));
    json room =
        element("Rooms", "room-1", "L0",
                json::array({box(200, 100, z, 4800, 4000, z + 2800, true)}),
                {{"num", param("ROOM_NUMBER", "101")},
                 {"nm", param("ROOM_NAME", "Kitchen")}});
    room["name"] = "Kitchen 101";
    json unplaced = element("Rooms", "room-2", "L1", json::array());
    unplaced["name"] = "Storage 202";

    json door_mesh = local().box(-450, -100, 0, 450, 100, 2100);
    door_mesh["applicationId"] = "door-geom";
    json chair_mesh = local().box(0, 0, 0, 1000, 500, 800);
    chair_mesh["applicationId"] = "chair-geom";

    json level_coll = {
        {"speckle_type", "Speckle.Core.Models.Collections.Collection"},
        {"name", "L0"},
        {"elements",
         json::array(
             {{{"speckle_type", "Speckle.Core.Models.Collections.Collection"},
               {"name", "Walls"},
               {"elements", json::array({wall_a, wall_b, wall_c})}},
              door,
              window,
              chair,
              room,
              unplaced})}};
    return {
        {"speckle_type", "Speckle.Core.Models.Collections.Collection"},
        {"units", units},
        {"version", "3"},
        {"elements",
         json::array(
             {level_coll,
              {{"speckle_type", "Speckle.Core.Models.Collections.Collection"},
               {"name", "definitionGeometry"},
               {"elements", json::array({door_mesh, chair_mesh})}}})},
        {"levelProxies", json::array({level("L1", 13000, "lvl-1"),
                                      level("L0", 10000, "lvl-0")})},
        {"instanceDefinitionProxies",
         json::array(
             {{{"speckle_type",
                "Speckle.Core.Models.Instances.InstanceDefinitionProxy"},
               {"applicationId", "DEF-DOOR"},
               {"objects", json::array({"door-geom"})}},
              {{"speckle_type",
                "Speckle.Core.Models.Instances.InstanceDefinitionProxy"},
               {"applicationId", "DEF-CHAIR"},
               {"objects", json::array({"chair-geom"})}}})}};
  }
};

} // namespace reusex::test_support
