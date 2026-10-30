// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Typed elements in the export scene (W6.2, reusex-z1v.8.2): the storeys,
// walls, slabs, roof, spaces and openings of the typed model, as the Rhino
// layer tree "<root>/<storey>/<Walls|Slabs|Roof|Spaces|Doors|Windows|Other
// openings>" with named objects and user text, and as Speckle DataObjects;
// plus the export-only point-cloud reduction (reusex-f97m).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/export_scene.hpp>
#include <reusex/io/rhino.hpp>
#include <reusex/io/speckle.hpp>
#include <reusex/pipeline/stages.hpp>

#include <nlohmann/json.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../../support/temp_path.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using reusex::test_support::TempPath;
using json = nlohmann::json;

namespace {

/// One storey ("Stueetage"), two rooms either side of a partition with a
/// door in it, a window in the south wall and a plain opening in the north
/// wall. GUIDs are real UUIDs, as the stages write them.
core::SolidModel two_rooms() {
  core::SolidModel m;
  core::StoreyRecord st;
  st.guid = "6f1c1f5e-0000-4000-8000-000000000001";
  st.name = "Stueetage";
  st.z_floor = 0.0;
  st.z_ceiling = 2.5;
  m.storeys = {st};
  const auto wall = [](const char *g, double x0, double y0, double x1,
                       double y1) {
    core::WallRecord w;
    w.guid = g;
    w.x0 = x0;
    w.y0 = y0;
    w.x1 = x1;
    w.y1 = y1;
    w.thickness = 0.2;
    w.height = 2.5;
    return w;
  };
  m.walls = {wall("a0000000-0000-4000-8000-00000000000s", 0, 0, 8, 0),
             wall("a0000000-0000-4000-8000-00000000000e", 8, 0, 8, 4),
             wall("a0000000-0000-4000-8000-00000000000n", 8, 4, 0, 4),
             wall("a0000000-0000-4000-8000-00000000000w", 0, 4, 0, 0),
             wall("a0000000-0000-4000-8000-00000000000p", 4, 0, 4, 4)};
  m.walls[0].is_external = true;
  core::OpeningRecord door;
  door.guid = "d0000000-0000-4000-8000-000000000001";
  door.wall = "a0000000-0000-4000-8000-00000000000p";
  door.kind = "door";
  door.s_start = 1.0;
  door.s_end = 1.9;
  door.z_sill = 0.0;
  door.z_head = 2.1;
  core::OpeningRecord window = door;
  window.guid = "d0000000-0000-4000-8000-000000000002";
  window.wall = "a0000000-0000-4000-8000-00000000000s";
  window.kind = "window";
  window.s_start = 5.0;
  window.s_end = 6.5;
  window.z_sill = 0.9;
  window.z_head = 2.0;
  core::OpeningRecord hole = window;
  hole.guid = "d0000000-0000-4000-8000-000000000003";
  hole.wall = "a0000000-0000-4000-8000-00000000000n";
  hole.kind = "opening";
  hole.s_start = 2.0;
  hole.s_end = 3.0;
  m.openings = {door, window, hole};
  const auto space = [](const char *g, const char *name, double x0, double x1) {
    core::SpaceRecord s;
    s.guid = g;
    s.name = name;
    s.kind = "room";
    s.polygon.outer = {{x0, 0.1}, {x1, 0.1}, {x1, 3.9}, {x0, 3.9}};
    s.z_floor = 0.0;
    s.z_ceiling = 2.5;
    return s;
  };
  m.spaces = {space("b0000000-0000-4000-8000-000000000001", "Køkken", 0.1, 3.9),
              space("b0000000-0000-4000-8000-000000000002", "Stue", 4.1, 7.9)};
  return m;
}

void store(ProjectDB &db, const core::SolidModel &m) {
  db.save_storeys(m.storeys);
  db.save_walls(m.walls, m.wall_joins);
  db.save_openings(m.openings);
  db.save_spaces(m.spaces, {});
}

void run_solids(ProjectDB &db, const TempPath &project) {
  pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = pipeline::JobStage::solids;
  ctx.parameters = "{}";
  const auto r = pipeline::run_stage(db, ctx);
  REQUIRE(r.ok);
}

std::map<std::string, int> count_by_category(const io::ExportScene &scene) {
  std::map<std::string, int> n;
  for (const auto &e : scene.elements)
    ++n[e.category];
  return n;
}

const io::ExportScene::ElementEntry &by_guid(const io::ExportScene &scene,
                                             const std::string &guid) {
  const auto it = std::find_if(scene.elements.begin(), scene.elements.end(),
                               [&](const auto &e) { return e.guid == guid; });
  REQUIRE(it != scene.elements.end());
  return *it;
}

void check_elements(const io::ExportScene &scene) {
  REQUIRE(scene.storeys.size() == 1);
  CHECK(scene.storeys[0].name == "Stueetage");
  const auto n = count_by_category(scene);
  CHECK(n.at("wall") == 5);
  CHECK(n.at("slab") >= 1);
  CHECK(n.at("roof") >= 1);
  CHECK(n.at("space") == 2);
  CHECK(n.at("door") == 1);
  CHECK(n.at("window") == 1);
  CHECK(n.at("opening") == 1);
  std::set<std::string> guids;
  for (const auto &e : scene.elements) {
    CHECK(e.storey == 0);
    REQUIRE(e.mesh);
    CHECK_FALSE(e.mesh->polygons.empty());
    CHECK_FALSE(e.guid.empty());
    CHECK(guids.insert(e.guid).second); // every element its own GUID
    CHECK(e.properties.at("guid") == e.guid);
    CHECK(e.properties.at("type") == e.category);
    CHECK(e.properties.at("storey") == "0");
    CHECK(e.properties.at("storey_name") == "Stueetage");
  }

  const auto &wall = by_guid(scene, "a0000000-0000-4000-8000-00000000000s");
  CHECK(wall.category == "wall");
  CHECK(wall.properties.at("thickness") == "0.200");
  CHECK(wall.properties.at("height") == "2.500");
  CHECK(wall.properties.at("length") == "8.000");
  CHECK(wall.properties.at("is_external") == "true");
  // 8 x 2.5 minus the 1.5 x 1.1 window.
  CHECK_THAT(std::stod(wall.properties.at("area")), WithinAbs(18.35, 1e-3));

  const auto &kitchen = by_guid(scene, "b0000000-0000-4000-8000-000000000001");
  CHECK(kitchen.name == "Køkken");
  CHECK(kitchen.properties.at("room_name") == "Køkken");
  CHECK(kitchen.properties.at("height") == "2.500");
  CHECK_THAT(std::stod(kitchen.properties.at("area")), WithinAbs(14.44, 1e-2));

  const auto &door = by_guid(scene, "d0000000-0000-4000-8000-000000000001");
  CHECK(door.category == "door");
  CHECK(door.properties.at("opening_kind") == "door");
  CHECK(door.properties.at("width") == "0.900");
  CHECK(door.properties.at("height") == "2.100");
  CHECK(door.properties.at("host_wall") ==
        "a0000000-0000-4000-8000-00000000000p");
  // The door box sits within the partition (x = 4 +- 0.1), not past it.
  pcl::PointCloud<pcl::PointXYZ> v;
  pcl::fromPCLPointCloud2(door.mesh->cloud, v);
  for (const auto &p : v.points) {
    CHECK(p.x > 3.89);
    CHECK(p.x < 4.11);
  }

  for (const auto &e : scene.elements)
    if (e.category == "slab" || e.category == "roof") {
      CHECK_FALSE(e.properties.at("thickness").empty());
      CHECK(std::stod(e.properties.at("area")) > 30.0);
    }
}

std::string layer_path(const ONX_Model &model, const ON_Layer &layer) {
  std::string path = ON_String(layer.Name()).Array();
  ON_UUID parent = layer.ParentLayerId();
  while (ON_UuidIsNotNil(parent)) {
    const ON_Layer *p = ON_Layer::Cast(
        model.ComponentFromId(ON_ModelComponent::Type::Layer, parent)
            .ModelComponent());
    if (!p)
      break;
    path = std::string(ON_String(p->Name()).Array()) + "/" + path;
    parent = p->ParentLayerId();
  }
  // Drop the timestamped root.
  return path.substr(path.find('/') + 1);
}

/// Layer path (without the root) -> object count, of a .3dm read back.
std::map<std::string, int> layer_counts(const ONX_Model &model) {
  std::map<int, std::string> path_of_index;
  ONX_ModelComponentIterator layers(model, ON_ModelComponent::Type::Layer);
  for (const ON_ModelComponent *c = layers.FirstComponent(); c;
       c = layers.NextComponent()) {
    const ON_Layer *l = ON_Layer::Cast(c);
    if (l)
      path_of_index[l->Index()] = layer_path(model, *l);
  }
  std::map<std::string, int> n;
  for (const auto &[idx, path] : path_of_index)
    n[path] += 0;
  ONX_ModelComponentIterator objs(model,
                                  ON_ModelComponent::Type::ModelGeometry);
  for (const ON_ModelComponent *c = objs.FirstComponent(); c;
       c = objs.NextComponent()) {
    const auto *g = ON_ModelGeometryComponent::Cast(c);
    if (g && g->Attributes(nullptr))
      ++n[path_of_index[g->Attributes(nullptr)->m_layer_index]];
  }
  return n;
}

} // namespace

TEST_CASE("TypedExport_StoredSolids_GatherTypedElements",
          "[io][export][typed]") {
  TempPath project("test_typed_export_stored");
  ProjectDB db(project.path);
  store(db, two_rooms());
  run_solids(db, project);

  const auto scene = io::gather_export_scene(db);
  check_elements(scene);
  // The solid_* meshes are exported as typed elements, not again as meshes.
  for (const auto &m : scene.meshes)
    CHECK(m.name.rfind("solid_", 0) != 0);

  io::ExportSceneOptions off;
  off.typed_elements = false;
  const auto plain = io::gather_export_scene(db, off);
  CHECK(plain.elements.empty());
  CHECK_FALSE(plain.meshes.empty()); // the old mesh path, unchanged
}

TEST_CASE("TypedExport_NoSolidsStage_BuildsSolidsInMemory",
          "[io][export][typed]") {
  TempPath project("test_typed_export_memory");
  ProjectDB db(project.path);
  store(db, two_rooms());
  REQUIRE_FALSE(db.has_mesh("solid_walls"));

  const auto scene = io::gather_export_scene(db);
  check_elements(scene);
  CHECK_FALSE(db.has_mesh("solid_walls")); // export never writes
}

TEST_CASE("TypedExport_NoTypedModel_NoElements", "[io][export][typed]") {
  TempPath project("test_typed_export_empty");
  ProjectDB db(project.path);
  const auto scene = io::gather_export_scene(db);
  CHECK(scene.elements.empty());
  CHECK(scene.storeys.empty());
}

TEST_CASE("TypedExport_Names", "[io][export][typed]") {
  CHECK(io::element_layer_name("wall") == "Walls");
  CHECK(io::element_layer_name("slab") == "Slabs");
  CHECK(io::element_layer_name("roof") == "Roof");
  CHECK(io::element_layer_name("space") == "Spaces");
  CHECK(io::element_layer_name("door") == "Doors");
  CHECK(io::element_layer_name("window") == "Windows");
  CHECK(io::element_layer_name("opening") == "Other openings");

  io::ExportScene::ElementEntry e;
  e.category = "space";
  e.guid = "g1";
  e.name = "Køkken";
  CHECK(io::element_object_name(e) == "Køkken (g1)");
  e.name.clear();
  CHECK(io::element_object_name(e) == "Space (g1)");
  e.category = "opening";
  CHECK(io::element_object_name(e) == "Opening (g1)");

  io::ExportScene scene;
  scene.storeys = {{0, "s0", "", 0, 3},
                   {1, "s1", "1. sal", 3, 6},
                   {2, "s2", "1. sal", 6, 9}};
  const auto names = io::storey_layer_names(scene);
  CHECK(names.at(0) == "Storey 0");
  CHECK(names.at(1) == "1. sal");
  CHECK(names.at(2) == "1. sal (2)");
}

TEST_CASE("TypedExport_Rhino_LayerTreeNamesAndUserText_RoundTrip",
          "[io][rhino][typed]") {
  TempPath project("test_typed_export_rhino");
  ProjectDB db(project.path);
  store(db, two_rooms());
  run_solids(db, project);
  const auto scene = io::gather_export_scene(db);

  TempPath file("test_typed_export_rhino_3dm", ".3dm");
  {
    auto model = io::export_to_rhino(scene);
    REQUIRE(model->Write(file.path.c_str(), 0));
  }

  ONX_Model back;
  REQUIRE(back.Read(file.path.c_str()));
  const auto n = layer_counts(back);
  CHECK(n.at("Stueetage") == 0);
  CHECK(n.at("Stueetage/Walls") == 5);
  CHECK(n.at("Stueetage/Slabs") >= 1);
  CHECK(n.at("Stueetage/Roof") >= 1);
  CHECK(n.at("Stueetage/Spaces") == 2);
  CHECK(n.at("Stueetage/Doors") == 1);
  CHECK(n.at("Stueetage/Windows") == 1);
  CHECK(n.at("Stueetage/Other openings") == 1);

  // Named objects with user text, UTF-8 intact.
  bool kitchen = false, door = false;
  ONX_ModelComponentIterator objs(back, ON_ModelComponent::Type::ModelGeometry);
  for (const ON_ModelComponent *c = objs.FirstComponent(); c;
       c = objs.NextComponent()) {
    const auto *g = ON_ModelGeometryComponent::Cast(c);
    const ON_3dmObjectAttributes *a = g ? g->Attributes(nullptr) : nullptr;
    if (!a)
      continue;
    const std::string name = ON_String(a->m_name).Array();
    ON_wString value;
    if (name == "Køkken (b0000000-0000-4000-8000-000000000001)") {
      kitchen = true;
      REQUIRE(a->GetUserString(L"room_name", value));
      CHECK(std::string(ON_String(value).Array()) == "Køkken");
      REQUIRE(a->GetUserString(L"area", value));
      CHECK_FALSE(value.IsEmpty());
      CHECK(ON_Mesh::Cast(g->Geometry(nullptr)) != nullptr);
    }
    if (name == "Door (d0000000-0000-4000-8000-000000000001)") {
      door = true;
      REQUIRE(a->GetUserString(L"opening_kind", value));
      CHECK(std::string(ON_String(value).Array()) == "door");
      REQUIRE(a->GetUserString(L"guid", value));
      CHECK(std::string(ON_String(value).Array()) ==
            "d0000000-0000-4000-8000-000000000001");
      REQUIRE(a->GetUserString(L"storey_name", value));
      CHECK(std::string(ON_String(value).Array()) == "Stueetage");
    }
  }
  CHECK(kitchen);
  CHECK(door);
}

TEST_CASE("TypedExport_Speckle_ElementsModel", "[io][speckle][typed]") {
  TempPath project("test_typed_export_speckle");
  ProjectDB db(project.path);
  store(db, two_rooms());
  run_solids(db, project);
  const auto scene = io::gather_export_scene(db);

  const auto models = io::speckle::export_to_speckle(scene, {});
  const auto it = std::find_if(models.begin(), models.end(), [](const auto &m) {
    return m.model_name == "elements";
  });
  REQUIRE(it != models.end());
  CHECK(std::none_of(models.begin(), models.end(), [](const auto &m) {
    return m.model_name == "mesh";
  })); // the solid_* meshes went to "elements" only

  const auto [root_id, objects] = io::speckle::flatten(*it->root);
  const json &root = objects.front();
  CHECK(root.at("speckle_type") ==
        "Speckle.Core.Models.Collections.Collection");
  CHECK(root.at("version") == "3");

  std::map<std::string, const json *> by_id;
  for (const auto &o : objects)
    by_id[o.at("id").get<std::string>()] = &o;
  const auto deref = [&](const json &ref) -> const json & {
    if (ref.contains("referencedId"))
      return *by_id.at(ref.at("referencedId").get<std::string>());
    return ref;
  };
  const auto children = [&](const json &o) {
    std::vector<const json *> out;
    const char *key = o.contains("elements") ? "elements" : "@elements";
    if (o.contains(key))
      for (const auto &c : o.at(key))
        out.push_back(&deref(c));
    return out;
  };

  const auto storeys = children(root);
  REQUIRE(storeys.size() == 1);
  CHECK(storeys[0]->at("name") == "Stueetage");
  std::map<std::string, const json *> type_col;
  for (const json *c : children(*storeys[0]))
    type_col[c->at("name").get<std::string>()] = c;
  REQUIRE(type_col.count("Walls"));
  CHECK(children(*type_col["Walls"]).size() == 5);
  CHECK(children(*type_col["Spaces"]).size() == 2);
  CHECK(children(*type_col["Doors"]).size() == 1);

  // One wall on the wire: a DataObject with its GUID, name, properties and
  // a detached mesh display value.
  const json *wall = nullptr;
  for (const json *w : children(*type_col["Walls"]))
    if (w->at("applicationId") == "a0000000-0000-4000-8000-00000000000s")
      wall = w;
  REQUIRE(wall);
  CHECK(wall->at("speckle_type") == "Objects.Data.DataObject");
  CHECK(wall->at("name") == "Wall (a0000000-0000-4000-8000-00000000000s)");
  const json &props = wall->at("properties");
  CHECK(props.at("guid") == "a0000000-0000-4000-8000-00000000000s");
  CHECK(props.at("type") == "wall");
  CHECK_THAT(props.at("thickness").get<double>(), WithinAbs(0.2, 1e-9));
  CHECK(props.at("is_external") == true);
  REQUIRE(wall->at("displayValue").size() == 1);
  const json &mesh = deref(wall->at("displayValue")[0]);
  CHECK(mesh.at("speckle_type") == "Objects.Geometry.Mesh");
  CHECK_FALSE(mesh.at("vertices").empty());
  // The detached mesh is in the wall's closure.
  CHECK(wall->at("__closure").contains(mesh.at("id").get<std::string>()));
}

TEST_CASE("TypedExport_CloudReduction_KeepsNormalsAndLabelsAligned",
          "[io][export][f97m]") {
  TempPath project("test_typed_export_cloud");
  ProjectDB db(project.path);
  // 1000 points on a 10 x 10 x 10 lattice with 0.1 m spacing; colour and
  // normal both encode the point's index, so alignment can be checked.
  CloudPtr cloud(new Cloud);
  CloudNPtr normals(new CloudN);
  CloudLPtr labels(new CloudL);
  for (int i = 0; i < 1000; ++i) {
    PointT p;
    p.x = 0.1f * static_cast<float>(i % 10);
    p.y = 0.1f * static_cast<float>((i / 10) % 10);
    p.z = 0.1f * static_cast<float>(i / 100);
    p.r = static_cast<uint8_t>(i % 256);
    p.g = static_cast<uint8_t>(i / 256);
    p.b = 0;
    cloud->push_back(p);
    NormalT n;
    n.normal_x = static_cast<float>(i);
    n.normal_y = 0;
    n.normal_z = 1;
    normals->push_back(n);
    LabelT l;
    l.label = (i % 2) + 1;
    labels->push_back(l);
  }
  db.save_point_cloud("cloud", *cloud);
  db.save_point_cloud("normals", *normals);
  db.save_point_cloud("labels", *labels);
  db.save_label_definitions("labels", {{1, "wall"}, {2, "floor"}});

  SECTION("default keeps every point") {
    const auto scene = io::gather_export_scene(db);
    REQUIRE(scene.cloud);
    CHECK(scene.cloud->cloud->size() == 1000);
  }
  SECTION("max points: an even stride, normals aligned") {
    io::ExportSceneOptions o;
    o.max_cloud_points = 100;
    const auto scene = io::gather_export_scene(db, o);
    REQUIRE(scene.cloud);
    REQUIRE(scene.cloud->cloud->size() == 100);
    REQUIRE(scene.cloud->normals->size() == 100);
    for (std::size_t k = 0; k < 100; ++k) {
      const auto &p = scene.cloud->cloud->points[k];
      const int idx = p.r + 256 * p.g;
      CHECK(idx == static_cast<int>(k * 10));
      CHECK(scene.cloud->normals->points[k].normal_x ==
            static_cast<float>(idx));
    }
    std::size_t semantic = 0;
    for (const auto &cat : scene.semantic)
      for (const auto &inst : cat.instances)
        semantic += inst.points->size();
    CHECK(semantic == 100);
  }
  SECTION("voxel: one point per cube") {
    io::ExportSceneOptions o;
    o.cloud_voxel_m = 0.2; // 5 x 5 x 5 cubes of 2 x 2 x 2 points
    const auto scene = io::gather_export_scene(db, o);
    REQUIRE(scene.cloud);
    CHECK(scene.cloud->cloud->size() == 125);
    for (std::size_t k = 0; k < scene.cloud->cloud->size(); ++k) {
      const auto &p = scene.cloud->cloud->points[k];
      CHECK(scene.cloud->normals->points[k].normal_x ==
            static_cast<float>(p.r + 256 * p.g));
    }
  }
  // The stored cloud is never touched.
  CHECK(db.point_cloud_xyzrgb("cloud")->size() == 1000);
}

TEST_CASE("TypedExport_Rhino_SameTypeLayerOnTwoStoreys_KeepsItsName",
          "[io][rhino][typed]") {
  // A layer name is unique only among its siblings: "Walls" under two
  // storeys must not come back as "Walls" and "Walls 01".
  io::ExportScene scene;
  scene.storeys = {{0, "s0", "Etage 0", 0, 3}, {1, "s1", "Etage 1", 3, 6}};
  for (int k = 0; k < 2; ++k) {
    io::ExportScene::ElementEntry e;
    e.category = "wall";
    e.guid = "w" + std::to_string(k);
    e.storey = k;
    e.mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ> v;
    v.push_back({0, 0, 0});
    v.push_back({1, 0, 0});
    v.push_back({0, 1, 0});
    pcl::toPCLPointCloud2(v, e.mesh->cloud);
    pcl::Vertices t;
    t.vertices = {0, 1, 2};
    e.mesh->polygons.push_back(t);
    scene.elements.push_back(std::move(e));
  }
  TempPath file("test_typed_export_two_storeys", ".3dm");
  REQUIRE(io::export_to_rhino(scene)->Write(file.path.c_str(), 0));
  ONX_Model back;
  REQUIRE(back.Read(file.path.c_str()));
  const auto n = layer_counts(back);
  CHECK(n.at("Etage 0/Walls") == 1);
  CHECK(n.at("Etage 1/Walls") == 1);
  CHECK(n.count("Etage 1/Walls 01") == 0);
}

TEST_CASE("Rhino_MaterialImagesBlob_NotWrittenAsUserText",
          "[io][rhino][f97m]") {
  io::ExportScene scene;
  io::ExportScene::MaterialEntry m;
  m.name = "M1";
  m.properties = {{"images", std::string(1000, 'A')}, {"Material", "Wood"}};
  scene.materials.push_back(m);
  const auto model = io::export_to_rhino(scene);
  bool found = false;
  ONX_ModelComponentIterator objs(*model,
                                  ON_ModelComponent::Type::ModelGeometry);
  for (const ON_ModelComponent *c = objs.FirstComponent(); c;
       c = objs.NextComponent()) {
    const auto *g = ON_ModelGeometryComponent::Cast(c);
    const ON_3dmObjectAttributes *a = g ? g->Attributes(nullptr) : nullptr;
    if (!a)
      continue;
    found = true;
    ON_wString v;
    CHECK(a->GetUserString(L"Material", v));
    CHECK_FALSE(a->GetUserString(L"images", v));
  }
  CHECK(found);
}
