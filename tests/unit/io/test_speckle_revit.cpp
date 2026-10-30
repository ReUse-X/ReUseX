// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Offline tests for the Revit (via Speckle) -> solid-model mapping. The
// object tree is a small hand-written synthetic model in the shape the
// Speckle v3 Revit connector publishes (already resolved: references
// inlined); no customer data.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/io/speckle_revit.hpp>

#include "../../support/temp_path.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace reusex::io::speckle;
using Catch::Approx;
using nlohmann::json;

namespace {

/// Builds the synthetic model in one unit: every length is given in mm and
/// divided by `per_mm` (1 for "mm", 10 for "cm", 1000 for "m").
struct Fixture {
  std::string units = "mm";
  double per_mm = 1.0;

  double L(double mm) const { return mm / per_mm; }

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
      v.push_back(L(i & 1 ? x1 : x0));
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
    m[3] = L(m[3]);
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
             const std::string &app) const {
    return {{"speckle_type", "Objects.Other.LevelProxy"},
            {"applicationId", app},
            {"objects", json::array()},
            {"value",
             {{"speckle_type", "Objects.Data.DataObject"},
              {"name", name},
              {"elevation", L(elevation_mm)},
              {"units", units},
              {"properties", {{"elementId", app + "-eid"}}}}}};
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
                json::object(), {{"parentApplicationId", "wall-a"}});
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

    json door_mesh = box(-450, -100, 0, 450, 100, 2100);
    door_mesh["applicationId"] = "door-geom";
    json chair_mesh = box(0, 0, 0, 1000, 500, 800);
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

const RevitMesh *mesh_named(const RevitImport &r, const std::string &name) {
  for (const auto &m : r.meshes)
    if (m.name == name)
      return &m;
  return nullptr;
}

void check_model(const RevitImport &r) {
  const auto &m = r.model;
  const auto &st = r.stats;

  // Storeys: sorted by elevation, one shared frame with L0's floor at 0.
  REQUIRE(m.storeys.size() == 2);
  CHECK(st.elevation_offset_m == Approx(-9.5));
  CHECK(m.storeys[0].name == "L0");
  CHECK(m.storeys[0].idx == 0);
  CHECK(m.storeys[0].z_floor == Approx(0.0).margin(1e-9));
  CHECK(m.storeys[0].z_ceiling == Approx(3.0));
  CHECK(m.storeys[1].name == "L1");
  CHECK(m.storeys[1].z_floor == Approx(3.0));
  CHECK(m.storeys[1].z_ceiling == Approx(6.0)); // + default storey height
  for (const auto &s : m.storeys) {
    CHECK(s.level_transform[11] == Approx(-0.5));
    CHECK(s.level_transform[3] == 0.0);
    CHECK(!s.guid.empty());
  }

  // Walls: A is straight; B (L-shaped) and C (7 mm) are not wall records.
  REQUIRE(m.walls.size() == 1);
  const auto &w = m.walls[0];
  CHECK(w.storey == 0);
  CHECK(w.x0 == Approx(0.0).margin(1e-9));
  CHECK(w.y0 == Approx(0.0).margin(1e-9));
  CHECK(w.x1 == Approx(5.0));
  CHECK(w.y1 == Approx(0.0).margin(1e-9));
  CHECK(w.thickness == Approx(0.2));
  CHECK(w.z_base == Approx(0.0).margin(1e-9));
  CHECK(w.height == Approx(3.0));
  CHECK(w.is_external);
  CHECK(json::parse(w.metadata)["revit"]["applicationId"] == "wall-a");
  CHECK(st.walls_not_straight == 1);
  CHECK(st.walls_thin == 1);

  // Openings: the door by parentApplicationId, the window by geometry.
  REQUIRE(m.openings.size() == 2);
  const auto door =
      std::find_if(m.openings.begin(), m.openings.end(),
                   [](const auto &o) { return o.kind == "door"; });
  REQUIRE(door != m.openings.end());
  CHECK(door->wall == w.guid);
  CHECK(door->s_start == Approx(1.55));
  CHECK(door->s_end == Approx(2.45));
  CHECK(door->z_sill == Approx(0.0).margin(1e-9));
  CHECK(door->z_head == Approx(2.1));
  CHECK(door->is_external);
  const auto win =
      std::find_if(m.openings.begin(), m.openings.end(),
                   [](const auto &o) { return o.kind == "window"; });
  REQUIRE(win != m.openings.end());
  CHECK(win->wall == w.guid);
  CHECK(win->s_start == Approx(3.5));
  CHECK(win->s_end == Approx(4.3));
  CHECK(win->z_sill == Approx(0.9));
  CHECK(win->z_head == Approx(2.1));
  CHECK(st.openings_by_parent == 1);
  CHECK(st.openings_by_geometry == 1);
  CHECK(st.openings_unhosted == 0);

  // Spaces: the placed room; the unplaced one is skipped by name.
  REQUIRE(m.spaces.size() == 1);
  const auto &s = m.spaces[0];
  CHECK(s.name == "Kitchen 101");
  CHECK(s.kind == "room");
  CHECK(s.storey == 0);
  CHECK(s.polygon.outer.size() == 4);
  CHECK(s.polygon.holes.empty());
  CHECK(s.area == Approx(4.6 * 3.9));
  CHECK(s.perimeter == Approx(2 * (4.6 + 3.9)));
  CHECK(s.volume == Approx(4.6 * 3.9 * 2.8));
  CHECK(s.z_floor == Approx(0.0).margin(1e-9));
  CHECK(s.z_ceiling == Approx(2.8));
  CHECK(json::parse(s.metadata)["revit"]["number"] == "101");
  CHECK(st.rooms_unplaced == 1);
  REQUIRE(st.unplaced_room_names.size() == 1);
  CHECK(st.unplaced_room_names[0] == "Storage 202");

  // Meshes: one per category, Rooms excepted; world frame, metres.
  std::vector<std::string> names;
  for (const auto &mm : r.meshes)
    names.push_back(mm.name);
  CHECK(names ==
        std::vector<std::string>{"imported/doors", "imported/furniture",
                                 "imported/walls", "imported/windows"});
  const auto *walls = mesh_named(r, "imported/walls");
  REQUIRE(walls);
  CHECK(walls->elements == 3);
  CHECK(walls->triangles.size() == 4 * 12);
  const auto *chair = mesh_named(r, "imported/furniture");
  REQUIRE(chair);
  REQUIRE(chair->vertices.size() == 8);
  // Rotated 90 degrees: x in [1000 - 500, 1000], y in [2000, 3000] mm.
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9;
  for (const auto &v : chair->vertices) {
    xmin = std::min(xmin, v[0]);
    xmax = std::max(xmax, v[0]);
    ymin = std::min(ymin, v[1]);
    ymax = std::max(ymax, v[1]);
    zmin = std::min(zmin, v[2]);
  }
  CHECK(xmin == Approx(0.5));
  CHECK(xmax == Approx(1.0));
  CHECK(ymin == Approx(2.0));
  CHECK(ymax == Approx(3.0));
  CHECK(zmin == Approx(0.5)); // world (Revit internal) z, not level frame
  CHECK(st.instance_proxies == 2);
  CHECK(st.instance_unresolved == 0);
  CHECK(st.categories.at("Walls") == 3);
}

} // namespace

TEST_CASE("import_revit maps a synthetic Revit model", "[io][speckle][revit]") {
  const Fixture f;
  const auto r = import_revit(f.model());
  check_model(r);
}

TEST_CASE("import_revit converts units", "[io][speckle][revit]") {
  for (const auto &[units, per_mm] :
       {std::pair<std::string, double>{"cm", 10.0}, {"m", 1000.0}}) {
    DYNAMIC_SECTION(units) {
      Fixture f;
      f.units = units;
      f.per_mm = per_mm;
      check_model(import_revit(f.model()));
    }
  }
}

TEST_CASE("import_revit GUIDs are deterministic", "[io][speckle][revit]") {
  const Fixture f;
  RevitImportOptions opt;
  opt.guid_namespace = "0f9a3c1e-1d2b-4c3d-8e4f-5a6b7c8d9e0f";
  const auto a = import_revit(f.model(), opt);
  const auto b = import_revit(f.model(), opt);
  REQUIRE(a.model.walls.size() == 1);
  CHECK(a.model.walls[0].guid == b.model.walls[0].guid);
  CHECK(a.model.storeys[1].guid == b.model.storeys[1].guid);
  RevitImportOptions other = opt;
  other.guid_namespace = "1f9a3c1e-1d2b-4c3d-8e4f-5a6b7c8d9e0f";
  CHECK(import_revit(f.model(), other).model.walls[0].guid !=
        a.model.walls[0].guid);
}

TEST_CASE("import_revit honours options", "[io][speckle][revit]") {
  const Fixture f;
  RevitImportOptions opt;
  opt.meshes = false;
  opt.elevation_offset_m = -10.0; // L0 at internal z 0 instead
  opt.min_wall_thickness_m = 0.005;
  const auto r = import_revit(f.model(), opt);
  CHECK(r.meshes.empty());
  CHECK(r.stats.elevation_offset_source == "option");
  CHECK(r.model.storeys[0].level_transform[11] == Approx(0.0).margin(1e-9));
  CHECK(r.model.walls.size() == 2); // the 7 mm layer is now a wall
  CHECK(r.model.walls[0].z_base == Approx(0.5));
}

TEST_CASE("import_revit refuses a tree without levels",
          "[io][speckle][revit]") {
  json root = {{"speckle_type", "Speckle.Core.Models.Collections.Collection"},
               {"elements", json::array()}};
  CHECK_THROWS_AS(import_revit(root), std::runtime_error);
}

TEST_CASE("category_slug is ASCII snake case", "[io][speckle][revit]") {
  CHECK(category_slug("Plumbing Fixtures") == "plumbing_fixtures");
  CHECK(category_slug("Walls") == "walls");
  CHECK(category_slug("Vægge / Æbler") == "v_gge_bler");
  CHECK(category_slug("") == "uncategorized");
}

TEST_CASE("an imported Revit model round-trips through the project tables",
          "[io][speckle][revit][projectdb]") {
  reusex::test_support::TempPath tmp("test_speckle_revit_store");
  const Fixture f;
  RevitImport r;
  {
    reusex::ProjectDB db(tmp.path);
    RevitImportOptions opt;
    opt.guid_namespace = db.project_guid();
    r = import_revit(f.model(), opt);
    reusex::io::mark_imported(r.model);
    const auto stored = reusex::io::store_solid_model(db, r.model);
    CHECK(stored.storeys == 2);
    CHECK(stored.walls == 1);
    CHECK(stored.openings == 2);
    CHECK(stored.spaces == 1);
    // A re-import replaces the earlier imported rows, it does not pile up.
    const auto again = reusex::io::store_solid_model(db, r.model);
    CHECK(again.replaced_imported == 6);
  }
  const reusex::ProjectDB db(tmp.path, /*readOnly*/ true);
  const auto back = reusex::io::load_solid_model(db);
  REQUIRE(back.storeys.size() == 2);
  REQUIRE(back.walls.size() == 1);
  REQUIRE(back.openings.size() == 2);
  REQUIRE(back.spaces.size() == 1);
  CHECK(back.walls[0].guid == r.model.walls[0].guid);
  CHECK(back.walls[0].source == "imported");
  CHECK(back.walls[0].locked);
  CHECK(back.walls[0].x1 == Approx(5.0));
  CHECK(back.spaces[0].area == Approx(4.6 * 3.9));
  CHECK(back.spaces[0].polygon.outer.size() == 4);
  for (const auto &o : back.openings)
    CHECK(o.wall == r.model.walls[0].guid);
  CHECK(back.storeys[1].level_transform[11] == Approx(-0.5));
}
