// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux import ifc` (reusex-3iy8.11): IFC -> solid model + per-class meshes,
// on a hand-written IFC4 building in millimetres
// (tests/fixtures/ifc/small_building.ifc): two storeys (0 and 3000 mm), an
// external IfcWall along x with a door (IfcOpeningElement, voids + fills), an
// IfcWallStandardCase along y on the upper storey, a 4 x 3 m IfcSpace
// aggregated into the ground floor and a floor slab.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/ifc.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/source_dir.hpp"
#include "../../support/temp_path.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <string>

using Catch::Approx;
using namespace reusex::io::ifc;

namespace {

std::filesystem::path fixture() {
  return reusex::test_support::source_dir() / "tests" / "fixtures" / "ifc" /
         "small_building.ifc";
}

const reusex::io::ImportedMesh *mesh_named(const IfcImport &r,
                                           const std::string &name) {
  for (const auto &m : r.meshes)
    if (m.name == name)
      return &m;
  return nullptr;
}

/// A one-triangle mesh saved under @p name by @p stage.
void save_dummy_mesh(reusex::ProjectDB &db, const std::string &name,
                     const std::string &stage) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({0, 0, 0});
  cloud.push_back({1, 0, 0});
  cloud.push_back({0, 1, 0});
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  pcl::Vertices f;
  f.vertices = {0, 1, 2};
  mesh.polygons.push_back(f);
  db.save_mesh(name, mesh, stage, "{}");
}

bool has_mesh(const reusex::ProjectDB &db, const std::string &name) {
  const auto names = db.list_meshes();
  return std::find(names.begin(), names.end(), name) != names.end();
}

} // namespace

TEST_CASE("ifc_class_slug folds and snake-cases IFC classes", "[io][ifc]") {
  CHECK(ifc_class_slug("IfcWall") == "wall");
  CHECK(ifc_class_slug("IfcWallStandardCase") == "wall");
  CHECK(ifc_class_slug("IfcWallElementedCase") == "wall");
  CHECK(ifc_class_slug("IfcFlowTerminal") == "flow_terminal");
  CHECK(ifc_class_slug("IfcBuildingElementProxy") == "building_element_proxy");
  CHECK(ifc_class_slug("IfcSlab") == "slab");
  CHECK(ifc_class_slug("") == "uncategorized");
}

#ifdef REUSEX_HAVE_WEB_IFC

TEST_CASE("import_ifc maps a small IFC4 building", "[io][ifc]") {
  REQUIRE(has_ifc_support());
  const IfcImport r = import_ifc(fixture());
  const auto &st = r.stats;
  CHECK(st.schema == "IFC4");
  CHECK(st.length_unit_m == Approx(0.001));
  CHECK(st.products == 4); // 2 walls, 1 door, 1 slab
  CHECK(st.classes.at("IfcWall") == 1);
  CHECK(st.classes.at("IfcWallStandardCase") == 1);
  CHECK(st.elements_no_storey == 0);

  // Storeys: placement z in metres, one shared level frame.
  REQUIRE(r.model.storeys.size() == 2);
  const auto &s0 = r.model.storeys[0];
  const auto &s1 = r.model.storeys[1];
  CHECK(s0.name == "Ground Floor");
  CHECK(s1.name == "First Floor");
  CHECK(s0.idx == 0);
  CHECK(s1.idx == 1);
  CHECK(s0.z_floor == Approx(0.0).margin(1e-9));
  CHECK(s0.z_ceiling == Approx(3.0));
  CHECK(s1.z_floor == Approx(3.0));
  CHECK(s1.z_ceiling == Approx(6.0)); // + default storey height
  CHECK(s1.level_transform[11] == Approx(0.0).margin(1e-9));
  const auto md0 = nlohmann::json::parse(s0.metadata);
  CHECK(md0["ifc"]["GlobalId"] == "1RXStorey0000000000000");
  CHECK(md0["ifc"]["LongName"] == "Stueetage");

  // Walls: centreline axis of the footprint rectangle.
  REQUIRE(r.model.walls.size() == 2);
  const auto &wa = r.model.walls[0];
  CHECK(wa.x0 == Approx(0.0).margin(1e-6));
  CHECK(wa.y0 == Approx(0.0).margin(1e-6));
  CHECK(wa.x1 == Approx(6.0));
  CHECK(wa.y1 == Approx(0.0).margin(1e-6));
  CHECK(wa.thickness == Approx(0.2));
  CHECK(wa.height == Approx(3.0));
  CHECK(wa.z_base == Approx(0.0).margin(1e-6));
  CHECK(wa.storey == 0);
  CHECK(wa.is_external); // Pset_WallCommon.IsExternal = .T.
  const auto &wb = r.model.walls[1];
  CHECK(wb.x0 == Approx(0.0).margin(1e-6));
  CHECK(wb.y0 == Approx(0.0).margin(1e-6));
  CHECK(wb.x1 == Approx(0.0).margin(1e-6));
  CHECK(wb.y1 == Approx(4.0));
  CHECK(wb.z_base == Approx(3.0)); // level frame
  CHECK(wb.storey == 1);
  CHECK_FALSE(wb.is_external);

  // The door: hosted through fills -> opening -> voids, extent of the void.
  REQUIRE(r.model.openings.size() == 1);
  const auto &o = r.model.openings[0];
  CHECK(o.wall == wa.guid);
  CHECK(o.kind == "door");
  CHECK(o.s_start == Approx(1.0));
  CHECK(o.s_end == Approx(1.9));
  CHECK(o.z_sill == Approx(0.0).margin(1e-6));
  CHECK(o.z_head == Approx(2.1));
  CHECK(o.is_external);
  CHECK(st.openings_by_relation == 1);
  CHECK(nlohmann::json::parse(o.metadata)["host"] == "IfcRelFillsElement");

  // The space: footprint of its body, LongName as the name.
  REQUIRE(r.model.spaces.size() == 1);
  const auto &sp = r.model.spaces[0];
  CHECK(sp.name == "Kitchen");
  CHECK(sp.storey == 0);
  CHECK(sp.kind == "room");
  CHECK(sp.polygon.outer.size() == 4);
  CHECK(sp.polygon.holes.empty());
  CHECK(sp.area == Approx(12.0));
  CHECK(sp.perimeter == Approx(14.0));
  CHECK(sp.volume == Approx(12.0 * 2.8));
  CHECK(sp.z_floor == Approx(0.0).margin(1e-6));
  CHECK(sp.z_ceiling == Approx(2.8));

  // Meshes per class, StandardCase folded; no space / opening mesh.
  REQUIRE(r.meshes.size() == 3);
  const auto *walls = mesh_named(r, "imported/wall");
  REQUIRE(walls != nullptr);
  CHECK(walls->category == "IfcWall");
  CHECK(walls->elements == 2);
  REQUIRE(mesh_named(r, "imported/door") != nullptr);
  const auto *slab = mesh_named(r, "imported/slab");
  REQUIRE(slab != nullptr);
  double zmin = 1e9;
  for (const auto &v : slab->vertices)
    zmin = std::min(zmin, v[2]);
  CHECK(zmin == Approx(-0.2));
}

TEST_CASE("import_ifc GUIDs are deterministic per namespace", "[io][ifc]") {
  IfcImportOptions opt;
  opt.guid_namespace = "0a0b0c0d-1111-5222-8333-444455556666";
  const auto a = import_ifc(fixture(), opt);
  const auto b = import_ifc(fixture(), opt);
  REQUIRE(a.model.walls.size() == b.model.walls.size());
  CHECK(a.model.walls[0].guid == b.model.walls[0].guid);
  CHECK(a.model.storeys[1].guid == b.model.storeys[1].guid);
  opt.guid_namespace = "0a0b0c0d-1111-5222-8333-444455556667";
  const auto c = import_ifc(fixture(), opt);
  CHECK(c.model.walls[0].guid != a.model.walls[0].guid);
}

TEST_CASE("import_ifc recentres in xy on request", "[io][ifc]") {
  IfcImportOptions opt;
  opt.recenter = true;
  const auto r = import_ifc(fixture(), opt);
  // Plan bounds x -0.1..6.0, y -0.1..4.0 -> centre (3, 2) after rounding.
  CHECK(r.stats.offset_m[0] == Approx(-3.0));
  CHECK(r.stats.offset_m[1] == Approx(-2.0));
  CHECK(r.stats.offset_m[2] == Approx(0.0).margin(1e-12));
  REQUIRE(r.model.walls.size() == 2);
  CHECK(r.model.walls[0].x0 == Approx(-3.0));
  CHECK(r.model.walls[0].y0 == Approx(-2.0));
  CHECK(r.model.storeys[1].z_floor == Approx(3.0)); // z is kept
  REQUIRE(r.model.openings.size() == 1);
  CHECK(r.model.openings[0].s_start == Approx(1.0));
}

TEST_CASE("import_ifc refuses a missing file", "[io][ifc]") {
  CHECK_THROWS_AS(import_ifc("/nonexistent/building.ifc"), std::runtime_error);
}

TEST_CASE("import_ifc_file stores the model and replaces it on re-import",
          "[io][ifc][projectdb]") {
  reusex::test_support::TempPath tmp("test_ifc_import_store");
  {
    reusex::ProjectDB db(tmp.path);
    const auto first = import_ifc_file(db, fixture());
    CHECK(first.stored.storeys == 2);
    CHECK(first.stored.walls == 2);
    CHECK(first.stored.openings == 1);
    CHECK(first.stored.spaces == 1);
    CHECK(first.meshes_saved == 3);
    CHECK(first.meshes_removed == 0);
    CHECK(has_mesh(db, "imported/wall"));
    CHECK(db.mesh_metadata("imported/wall").stage == "import_ifc");

    // Meshes another import wrote, and a stale one of an earlier IFC import.
    save_dummy_mesh(db, "imported/stale", "import_ifc");
    save_dummy_mesh(db, "imported/walls", "import_speckle");

    // --no-meshes neither writes nor deletes meshes.
    IfcImportOptions no_meshes;
    no_meshes.meshes = false;
    const auto again = import_ifc_file(db, fixture(), no_meshes);
    CHECK(again.stored.replaced_imported == 6);
    CHECK(again.meshes_saved == 0);
    CHECK(again.meshes_removed == 0);
    CHECK(has_mesh(db, "imported/stale"));

    const auto third = import_ifc_file(db, fixture());
    CHECK(third.stored.replaced_imported == 6);
    CHECK(third.meshes_saved == 3);
    CHECK(third.meshes_removed == 1);
    CHECK_FALSE(has_mesh(db, "imported/stale"));
    CHECK(has_mesh(db, "imported/walls")); // another stage's mesh stays
  }
  const reusex::ProjectDB db(tmp.path, /*readOnly*/ true);
  const auto back = reusex::io::load_solid_model(db);
  REQUIRE(back.storeys.size() == 2);
  REQUIRE(back.walls.size() == 2);
  REQUIRE(back.openings.size() == 1);
  REQUIRE(back.spaces.size() == 1);
  for (const auto &w : back.walls) {
    CHECK(w.source == "imported");
    CHECK(w.locked);
  }
  CHECK(back.spaces[0].area == Approx(12.0));
  CHECK(back.openings[0].kind == "door");
  const auto runs = db.pipeline_log();
  const auto n = std::count_if(runs.begin(), runs.end(), [](const auto &e) {
    return e.stage == "import_ifc";
  });
  CHECK(n == 3);
}

#else

TEST_CASE("import_ifc without web-ifc", "[io][ifc]") {
  CHECK_FALSE(has_ifc_support());
  CHECK_THROWS_AS(import_ifc(fixture()), std::runtime_error);
  SKIP("built without web-ifc (REUSEX_HAVE_WEB_IFC off)");
}

#endif
