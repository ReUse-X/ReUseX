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
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

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

std::string fixture_text() {
  std::ifstream in(fixture());
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// The fixture with the lines starting `#<id>=` replaced by @p with ("" =
/// removed); each edited line must exist.
std::string
variant(const std::vector<std::pair<std::string, std::string>> &edits) {
  std::string text = fixture_text();
  for (const auto &[id, with] : edits) {
    const std::string key = "\n#" + id + "=";
    const auto at = text.find(key);
    REQUIRE(at != std::string::npos);
    const auto end = text.find('\n', at + 1);
    text.replace(at + 1, end - at - 1, with);
  }
  return text;
}

/// Write @p text to a fresh temp .ifc and import it.
struct VariantFile {
  reusex::test_support::TempPath tmp{"test_ifc_variant", ".ifc"};
  explicit VariantFile(const std::string &text) {
    std::ofstream(tmp.path) << text;
  }
};

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
  const auto omd = nlohmann::json::parse(o.metadata);
  CHECK(omd["host"] == "IfcRelFillsElement");
  CHECK(omd["extent_source"]["s"] == "IfcOpeningElement");

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
  const auto *walls = mesh_named(r, "imported/ifc/wall");
  REQUIRE(walls != nullptr);
  CHECK(walls->category == "IfcWall");
  CHECK(walls->elements == 2);
  REQUIRE(mesh_named(r, "imported/ifc/door") != nullptr);
  const auto *slab = mesh_named(r, "imported/ifc/slab");
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

TEST_CASE("import_ifc recentres a georeferenced model", "[io][ifc]") {
  // The fixture with its site placed 600 km east, 6200 km north (mm).
  std::ifstream in(fixture());
  std::string text((std::istreambuf_iterator<char>(in)),
                   std::istreambuf_iterator<char>());
  const std::string site = "#31=IFCLOCALPLACEMENT($,#21);";
  const auto at = text.find(site);
  REQUIRE(at != std::string::npos);
  text.replace(at, site.size(),
               "#31=IFCLOCALPLACEMENT($,#900);\n"
               "#900=IFCAXIS2PLACEMENT3D(#901,$,$);\n"
               "#901=IFCCARTESIANPOINT((600000000.,6200000000.,0.));");
  reusex::test_support::TempPath tmp("test_ifc_import_far", ".ifc");
  std::ofstream(tmp.path) << text;

  const auto r = import_ifc(tmp.path);
  // Plan centre (600002.95, 6200001.95) -> whole-km offset.
  CHECK(r.stats.world_offset_m[0] == Approx(600000.0));
  CHECK(r.stats.world_offset_m[1] == Approx(6200000.0));
  CHECK(r.stats.world_offset_m[2] == Approx(0.0).margin(1e-12));
  REQUIRE(r.model.walls.size() == 2);
  CHECK(r.model.walls[0].x0 == Approx(0.0).margin(1e-6));
  CHECK(r.model.walls[0].x1 == Approx(6.0));
  CHECK(r.model.walls[0].y0 == Approx(0.0).margin(1e-6));
  CHECK(r.model.storeys[1].z_floor == Approx(3.0)); // z is kept
  REQUIRE(r.model.openings.size() == 1);
  CHECK(r.model.openings[0].s_start == Approx(1.0));
  REQUIRE(r.model.spaces.size() == 1);
  CHECK(r.model.spaces[0].polygon.outer[0][0] < 10.0);
  const auto md = nlohmann::json::parse(r.model.storeys[0].metadata);
  CHECK(md["world_offset_m"][0] == Approx(600000.0));

  IfcImportOptions keep;
  keep.recenter = false;
  const auto k = import_ifc(tmp.path, keep);
  CHECK(k.stats.world_offset_m[0] == 0.0);
  CHECK(k.model.walls[0].x0 == Approx(600000.0));
}

TEST_CASE("import_ifc reads a space without a body from FootPrint and Box",
          "[io][ifc]") {
  const std::string footprint =
      "#203=IFCPRODUCTDEFINITIONSHAPE($,$,(#230,#235));\n"
      "#230=IFCSHAPEREPRESENTATION(#23,'FootPrint','GeometricCurveSet',"
      "(#231));\n"
      "#231=IFCGEOMETRICCURVESET((#232));\n"
      "#232=IFCPOLYLINE((#240,#241,#242,#243,#240));\n"
      "#240=IFCCARTESIANPOINT((500.,500.));\n"
      "#241=IFCCARTESIANPOINT((4500.,500.));\n"
      "#242=IFCCARTESIANPOINT((4500.,3500.));\n"
      "#243=IFCCARTESIANPOINT((500.,3500.));";
  const std::string box =
      "\n#235=IFCSHAPEREPRESENTATION(#23,'Box','BoundingBox',(#236));\n"
      "#236=IFCBOUNDINGBOX(#237,4000.,3000.,2500.);\n"
      "#237=IFCCARTESIANPOINT((500.,500.,0.));";

  SECTION("FootPrint + Box") {
    const VariantFile f(variant({{"203", footprint + box}}));
    const auto r = import_ifc(f.tmp.path);
    REQUIRE(r.model.spaces.size() == 1);
    const auto &s = r.model.spaces[0];
    CHECK(s.area == Approx(12.0));
    CHECK(s.polygon.outer.size() == 4);
    CHECK(s.z_floor == Approx(0.0).margin(1e-9));
    CHECK(s.z_ceiling == Approx(2.5));
    CHECK(s.volume == Approx(30.0));
    CHECK(s.storey == 0);
    CHECK(nlohmann::json::parse(s.metadata)["geometry"] == "FootPrint+Box");
    CHECK(r.stats.spaces_from_footprint == 1);
  }
  SECTION("FootPrint alone: the storey's floor to ceiling") {
    const VariantFile f(
        variant({{"203", "#203=IFCPRODUCTDEFINITIONSHAPE($,$,(#230));" +
                             footprint.substr(footprint.find('\n'))}}));
    const auto r = import_ifc(f.tmp.path);
    REQUIRE(r.model.spaces.size() == 1);
    const auto &s = r.model.spaces[0];
    CHECK(s.area == Approx(12.0));
    CHECK(s.z_floor == Approx(0.0).margin(1e-9));
    CHECK(s.z_ceiling == Approx(3.0));
    CHECK(nlohmann::json::parse(s.metadata)["geometry"] == "FootPrint");
  }
}

TEST_CASE("import_ifc sizes an unrelated door from OverallWidth / Height",
          "[io][ifc]") {
  // No voids / fills: the door is hosted geometrically, and its body (a
  // 1.1 m x 2.3 m lining) is wider and taller than its nominal size.
  const VariantFile f(variant(
      {{"110", ""},
       {"130", ""},
       {"125", "#125=IFCEXTRUDEDAREASOLID(#126,#21,#24,2300.);"},
       {"126", "#126=IFCRECTANGLEPROFILEDEF(.AREA.,$,#127,1100.,50.);"}}));
  const auto r = import_ifc(f.tmp.path);
  REQUIRE(r.model.openings.size() == 1);
  const auto &o = r.model.openings[0];
  CHECK(o.wall == r.model.walls[0].guid);
  CHECK(o.s_start == Approx(1.0));
  CHECK(o.s_end == Approx(1.9));
  CHECK(o.z_sill == Approx(0.0).margin(1e-6));
  CHECK(o.z_head == Approx(2.1));
  const auto md = nlohmann::json::parse(o.metadata);
  CHECK(md["host"] == "geometry");
  CHECK(md["extent_source"]["s"] == "OverallWidth");
  CHECK(md["extent_source"]["z"] == "OverallHeight");
  CHECK(r.stats.openings_by_geometry == 1);
  CHECK(r.stats.openings_extent_params == 1);
}

TEST_CASE("import_ifc converts a conversion-based unit (feet)", "[io][ifc]") {
  const VariantFile f(variant(
      {{"11", "#11=IFCCONVERSIONBASEDUNIT(#12,.LENGTHUNIT.,'FOOT',#13);\n"
              "#12=IFCDIMENSIONALEXPONENTS(1,0,0,0,0,0,0);\n"
              "#13=IFCMEASUREWITHUNIT(IFCLENGTHMEASURE(0.3048),#14);\n"
              "#14=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);"}}));
  IfcImportOptions opt;
  opt.recenter = false;
  const auto r = import_ifc(f.tmp.path, opt);
  constexpr double ft = 0.3048;
  CHECK(r.stats.length_unit_m == Approx(ft));
  REQUIRE(r.model.walls.size() == 2);
  CHECK(r.model.walls[0].x1 == Approx(6000.0 * ft));
  CHECK(r.model.walls[0].thickness == Approx(200.0 * ft));
  CHECK(r.model.storeys[1].z_floor == Approx(3000.0 * ft));
  REQUIRE(r.model.openings.size() == 1);
  CHECK(r.model.openings[0].z_head == Approx(2100.0 * ft));
  REQUIRE(r.model.spaces.size() == 1);
  CHECK(r.model.spaces[0].area == Approx(12e6 * ft * ft));
}

TEST_CASE("import_ifc places a storey without placement at its Elevation",
          "[io][ifc]") {
  const VariantFile f(
      variant({{"52", "#52=IFCBUILDINGSTOREY('1RXStorey1000000000000',$,'First "
                      "Floor',$,$,$,$,$,.ELEMENT.,3100.);"}}));
  const auto r = import_ifc(f.tmp.path);
  REQUIRE(r.model.storeys.size() == 2);
  CHECK(r.model.storeys[1].z_floor == Approx(3.1));
  const auto md = nlohmann::json::parse(r.model.storeys[1].metadata);
  CHECK(md["z_source"] == "Elevation");
  CHECK(r.model.walls[1].storey == 1); // still contained in it
}

TEST_CASE("import_ifc places an uncontained element by height", "[io][ifc]") {
  const VariantFile f(variant({{"161", ""}}));
  const auto r = import_ifc(f.tmp.path);
  CHECK(r.stats.elements_no_storey == 1);
  REQUIRE(r.model.walls.size() == 2);
  CHECK(r.model.walls[1].storey == 1); // its bottom stands at 3.0 m
}

TEST_CASE("import_ifc reads IsExternal through a property set definition set",
          "[io][ifc]") {
  const VariantFile f(variant(
      {{"92", "#92=IFCRELDEFINESBYPROPERTIES('1RXDefWallA00000000000',$,$,$,"
              "(#70),IFCPROPERTYSETDEFINITIONSET((#90)));"}}));
  const auto r = import_ifc(f.tmp.path);
  REQUIRE(r.model.walls.size() == 2);
  CHECK(r.model.walls[0].is_external);
}

TEST_CASE("import_ifc ignores dangling references", "[io][ifc]") {
  const VariantFile f(variant(
      {{"160", "#160=IFCRELCONTAINEDINSPATIALSTRUCTURE('1RXContained0000000000'"
               ",$,$,$,(#70,#120,#140,#999),#998);"},
       {"92", "#92=IFCRELDEFINESBYPROPERTIES('1RXDefWallA00000000000',$,$,$,"
              "(#70,#997),#996);"}}));
  const auto r = import_ifc(f.tmp.path);
  CHECK(r.stats.products == 4);
  REQUIRE(r.model.walls.size() == 2);
  CHECK_FALSE(r.model.walls[0].is_external); // its pset is gone
  CHECK(r.model.walls[0].storey == 0);       // placed by height
  CHECK(r.stats.elements_no_storey >= 1);
}

TEST_CASE("import_ifc meshes only the body representation", "[io][ifc]") {
  // Wall A also carries a 'Clearance' solid 100 m away.
  const VariantFile f(variant(
      {{"73", "#73=IFCPRODUCTDEFINITIONSHAPE($,$,(#74,#250));\n"
              "#250=IFCSHAPEREPRESENTATION(#23,'Clearance','SweptSolid',"
              "(#251));\n"
              "#251=IFCEXTRUDEDAREASOLID(#252,#21,#24,3000.);\n"
              "#252=IFCRECTANGLEPROFILEDEF(.AREA.,$,#253,1000.,1000.);\n"
              "#253=IFCAXIS2PLACEMENT2D(#254,$);\n"
              "#254=IFCCARTESIANPOINT((100000.,0.));"}}));
  const auto r = import_ifc(f.tmp.path);
  CHECK(r.stats.non_body_geometries == 1);
  REQUIRE(r.model.walls.size() == 2);
  CHECK(r.model.walls[0].x1 == Approx(6.0));
  const auto *walls = mesh_named(r, "imported/ifc/wall");
  REQUIRE(walls != nullptr);
  double xmax = -1e9;
  for (const auto &v : walls->vertices)
    xmax = std::max(xmax, v[0]);
  CHECK(xmax == Approx(6.0));
}

TEST_CASE("import_ifc refuses truncated and non-IFC files", "[io][ifc]") {
  const std::string text = fixture_text();
  SECTION("truncated") {
    const VariantFile f(text.substr(0, text.size() / 2));
    CHECK_THROWS_WITH(import_ifc(f.tmp.path),
                      Catch::Matchers::ContainsSubstring("truncated"));
  }
  SECTION("garbage") {
    const VariantFile f("hello, this is not a building\n");
    CHECK_THROWS_WITH(import_ifc(f.tmp.path),
                      Catch::Matchers::ContainsSubstring("not an IFC"));
  }
  SECTION("import_ifc_file logs the failure and writes nothing") {
    const VariantFile f(text.substr(0, text.size() / 2));
    reusex::test_support::TempPath db_path("test_ifc_import_trunc");
    reusex::ProjectDB db(db_path.path);
    CHECK_THROWS_AS(import_ifc_file(db, f.tmp.path), std::runtime_error);
    const auto runs = db.pipeline_log();
    REQUIRE_FALSE(runs.empty());
    CHECK(runs.front().stage == "import_ifc");
    CHECK(runs.front().status == "failed");
    CHECK_THAT(runs.front().error_msg,
               Catch::Matchers::ContainsSubstring("truncated"));
    CHECK(db.storeys().empty());
    CHECK(db.list_meshes().empty());
  }
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
    CHECK(has_mesh(db, "imported/ifc/wall"));
    CHECK(db.mesh_metadata("imported/ifc/wall").stage == "import_ifc");

    // Meshes another import wrote, and a stale one of an earlier IFC import.
    save_dummy_mesh(db, "imported/ifc/stale", "import_ifc");
    save_dummy_mesh(db, "imported/revit/walls", "import_speckle");

    // --no-meshes neither writes nor deletes meshes.
    IfcImportOptions no_meshes;
    no_meshes.meshes = false;
    const auto again = import_ifc_file(db, fixture(), no_meshes);
    CHECK(again.stored.replaced_imported == 6);
    CHECK(again.meshes_saved == 0);
    CHECK(again.meshes_removed == 0);
    CHECK(has_mesh(db, "imported/ifc/stale"));

    const auto third = import_ifc_file(db, fixture());
    CHECK(third.stored.replaced_imported == 6);
    CHECK(third.meshes_saved == 3);
    CHECK(third.meshes_removed == 1);
    CHECK_FALSE(has_mesh(db, "imported/ifc/stale"));
    CHECK(has_mesh(db, "imported/revit/walls")); // another stage's mesh stays
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
