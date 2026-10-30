// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/rhino.hpp"
#include "core/logging.hpp"
#include "io/export_scene.hpp"

#include <fmt/format.h>
#include <pcl/common/colors.h>
#include <pcl/common/io.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <map>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::io {

auto configure_rhino_model() -> std::unique_ptr<ONX_Model> {

  auto model = std::make_unique<ONX_Model>();
  // Properties
  reusex::trace("setting up model properties");
  model->m_properties.m_RevisionHistory.NewRevision();
  model->m_properties.m_Application.m_application_name = "ReUseX";
  model->m_properties.m_Application.m_application_URL =
      L"https://github.com/pfmephisto/ReUseX";
  model->m_properties.m_Application.m_application_details =
      "ReUseX Export to Rhino 3dm file";
  model->m_properties.m_Notes.m_notes =
      "This file was made with the ReUseX Export tool";
  model->m_properties.m_Notes.m_bVisible = true;

  // Settings
  reusex::trace("setting up model settings");
  model->m_settings.m_ModelUnitsAndTolerances.m_unit_system =
      ON::LengthUnitSystem::Meters;
  model->m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance = 0.01;
  model->m_settings.m_ModelUnitsAndTolerances.m_angle_tolerance = ON_PI / 180.0;
  model->m_settings.m_ModelUnitsAndTolerances.m_relative_tolerance = 0.01;

  return model;
}

auto make_rhino_pointcloud(CloudConstPtr cloud)
    -> std::unique_ptr<ON_PointCloud> {

  const size_t num_points = cloud->points.size();
  auto rhino_cloud = std::make_unique<ON_PointCloud>();

  rhino_cloud->m_P.Reserve(num_points);
  rhino_cloud->m_C.Reserve(num_points);
  rhino_cloud->m_N.Reserve(num_points);

  rhino_cloud->m_P.SetCount(num_points);
  rhino_cloud->m_C.SetCount(num_points);
  rhino_cloud->m_N.SetCount(num_points);

#pragma omp parallel for shared(rhino_cloud, cloud)
  for (int j = 0; j < (int)num_points; j++) {
    ON_3dPoint pt(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
    ON_Color color(cloud->points[j].r, cloud->points[j].g, cloud->points[j].b);
    ON_3dVector dir(0.0, 0.0, 1.0);

    rhino_cloud->m_P[j] = pt;
    rhino_cloud->m_C[j] = color;
    rhino_cloud->m_N[j] = dir;
  }

  return rhino_cloud;
}

auto make_rhino_pointcloud(CloudConstPtr cloud, CloudNConstPtr normals)
    -> std::unique_ptr<ON_PointCloud> {

  if (!normals || normals->size() != cloud->size())
    return make_rhino_pointcloud(cloud);

  const size_t num_points = cloud->points.size();
  auto rhino_cloud = std::make_unique<ON_PointCloud>();

  rhino_cloud->m_P.Reserve(num_points);
  rhino_cloud->m_C.Reserve(num_points);
  rhino_cloud->m_N.Reserve(num_points);

  rhino_cloud->m_P.SetCount(num_points);
  rhino_cloud->m_C.SetCount(num_points);
  rhino_cloud->m_N.SetCount(num_points);

#pragma omp parallel for shared(rhino_cloud, cloud, normals)
  for (int j = 0; j < (int)num_points; j++) {
    rhino_cloud->m_P[j] =
        ON_3dPoint(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
    rhino_cloud->m_C[j] =
        ON_Color(cloud->points[j].r, cloud->points[j].g, cloud->points[j].b);
    rhino_cloud->m_N[j] =
        ON_3dVector(normals->points[j].normal_x, normals->points[j].normal_y,
                    normals->points[j].normal_z);
  }

  return rhino_cloud;
}

auto make_rhino_mesh(const pcl::PolygonMesh &mesh) -> std::unique_ptr<ON_Mesh> {
  // Check for color data
  bool has_color = false;
  for (const auto &field : mesh.cloud.fields) {
    if (field.name == "rgb" || field.name == "rgba") {
      has_color = true;
      break;
    }
  }

  int vertex_count = mesh.cloud.width * mesh.cloud.height;
  int face_count = static_cast<int>(mesh.polygons.size());

  auto rhino_mesh =
      std::make_unique<ON_Mesh>(face_count, vertex_count, true, false);

  if (has_color) {
    Cloud cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    for (size_t i = 0; i < cloud.size(); ++i) {
      rhino_mesh->SetVertex(
          static_cast<int>(i),
          ON_3dPoint(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));
    }

    rhino_mesh->m_C.Reserve(cloud.size());
    rhino_mesh->m_C.SetCount(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
      rhino_mesh->m_C[static_cast<int>(i)] =
          ON_Color(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    for (size_t i = 0; i < cloud.size(); ++i) {
      rhino_mesh->SetVertex(
          static_cast<int>(i),
          ON_3dPoint(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));
    }
  }

  // Faces
  for (int fi = 0; fi < face_count; ++fi) {
    const auto &poly = mesh.polygons[fi];
    if (poly.vertices.size() == 3) {
      rhino_mesh->SetTriangle(fi, poly.vertices[0], poly.vertices[1],
                              poly.vertices[2]);
    } else if (poly.vertices.size() == 4) {
      rhino_mesh->SetQuad(fi, poly.vertices[0], poly.vertices[1],
                          poly.vertices[2], poly.vertices[3]);
    }
  }

  rhino_mesh->ComputeVertexNormals();
  return rhino_mesh;
}

auto make_sphere_mesh(double cx, double cy, double cz, double radius,
                      int resolution) -> std::unique_ptr<ON_Mesh> {
  int lat_segments = resolution;
  int lon_segments = resolution * 2;

  int vertex_count = (lat_segments - 1) * lon_segments + 2; // poles
  int face_count = lon_segments * 2 + (lat_segments - 2) * lon_segments *
                                          2; // cap tris + band quads as tris

  auto mesh = std::make_unique<ON_Mesh>(face_count, vertex_count, false, false);

  // Top pole (index 0)
  mesh->SetVertex(0, ON_3dPoint(cx, cy, cz + radius));

  // Latitude rings
  int idx = 1;
  for (int lat = 1; lat < lat_segments; ++lat) {
    double phi = M_PI * lat / lat_segments;
    double sp = std::sin(phi);
    double cp = std::cos(phi);
    for (int lon = 0; lon < lon_segments; ++lon) {
      double theta = 2.0 * M_PI * lon / lon_segments;
      double x = cx + radius * sp * std::cos(theta);
      double y = cy + radius * sp * std::sin(theta);
      double z = cz + radius * cp;
      mesh->SetVertex(idx++, ON_3dPoint(x, y, z));
    }
  }

  // Bottom pole
  int bottom_pole = idx;
  mesh->SetVertex(bottom_pole, ON_3dPoint(cx, cy, cz - radius));

  // Faces
  int fi = 0;

  // Top cap (triangles from pole to first ring)
  for (int lon = 0; lon < lon_segments; ++lon) {
    int next = (lon + 1) % lon_segments;
    mesh->SetTriangle(fi++, 0, 1 + lon, 1 + next);
  }

  // Middle bands (two triangles per quad)
  for (int lat = 0; lat < lat_segments - 2; ++lat) {
    int ring_start = 1 + lat * lon_segments;
    int next_ring_start = ring_start + lon_segments;
    for (int lon = 0; lon < lon_segments; ++lon) {
      int next = (lon + 1) % lon_segments;
      int a = ring_start + lon;
      int b = ring_start + next;
      int c = next_ring_start + next;
      int d = next_ring_start + lon;
      mesh->SetTriangle(fi++, a, d, c);
      mesh->SetTriangle(fi++, a, c, b);
    }
  }

  // Bottom cap
  int last_ring = 1 + (lat_segments - 2) * lon_segments;
  for (int lon = 0; lon < lon_segments; ++lon) {
    int next = (lon + 1) % lon_segments;
    mesh->SetTriangle(fi++, bottom_pole, last_ring + next, last_ring + lon);
  }

  mesh->ComputeVertexNormals();

  // Set a neutral semi-transparent color
  mesh->m_C.Reserve(vertex_count);
  mesh->m_C.SetCount(vertex_count);
  ON_Color sphere_color(100, 150, 200); // Semi-neutral blue
  for (int i = 0; i < vertex_count; ++i)
    mesh->m_C[i] = sphere_color;

  return mesh;
}

namespace {

/// Create a sublayer under a parent layer, returning its index.
///
/// The parent is set BEFORE the layer joins the model: a layer name is
/// unique only among its siblings, so a layer added by name alone and
/// re-parented afterwards is renamed ("Walls 01") when another parent
/// already holds one of that name.
int add_sublayer(ONX_Model &model, const std::string &name,
                 const ON_Layer *parent, ON_Color color = ON_Color::Black) {
  ON_Layer layer;
  layer.SetName(ON_wString(name.c_str()));
  layer.SetColor(color);
  if (parent)
    layer.SetParentLayerId(parent->Id());
  const ON_ModelComponentReference ref = model.AddModelComponent(layer, true);
  const ON_Layer *added = ON_Layer::Cast(ref.ModelComponent());
  return added ? added->Index() : ON_UNSET_INT_INDEX;
}

/// The layer colour of an element category.
ON_Color element_color(std::string_view category) {
  if (category == "wall")
    return ON_Color(170, 170, 170);
  if (category == "slab")
    return ON_Color(110, 110, 110);
  if (category == "roof")
    return ON_Color(160, 82, 45);
  if (category == "space")
    return ON_Color(120, 170, 220);
  if (category == "door")
    return ON_Color(139, 90, 43);
  if (category == "window")
    return ON_Color(80, 200, 220);
  return ON_Color(230, 140, 40);
}

/// Get the current timestamp as "YYYYMMDD-HHmmss".
std::string timestamp_string() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d-%H%M%S");
  return oss.str();
}

} // namespace

auto export_to_rhino(const ExportScene &scene) -> std::unique_ptr<ONX_Model> {
  auto model = configure_rhino_model();

  // Root layer
  std::string root_name = "ReUseX-" + timestamp_string();
  ON_wString wroot(root_name.c_str());
  int root_idx = model->AddLayer(wroot, ON_Color::Black);
  const ON_Layer *root_layer =
      ON_Layer::Cast(model->LayerFromIndex(root_idx).ModelComponent());

  // --- Cloud ---
  if (scene.cloud) {
    int cloud_layer_idx = add_sublayer(*model, "cloud", root_layer);

    std::unique_ptr<ON_PointCloud> rhino_cloud;
    if (scene.cloud->normals)
      rhino_cloud =
          make_rhino_pointcloud(scene.cloud->cloud, scene.cloud->normals);
    else
      rhino_cloud = make_rhino_pointcloud(scene.cloud->cloud);

    auto *attr = new ON_3dmObjectAttributes();
    attr->m_name = L"cloud";
    attr->m_layer_index = cloud_layer_idx;
    model->AddManagedModelGeometryComponent(rhino_cloud.release(), attr);
    reusex::debug("Rhino: added cloud layer ({} points)",
                  scene.cloud->cloud->size());
  }

  // --- Semantic ---
  if (!scene.semantic.empty()) {
    int sem_layer_idx = add_sublayer(*model, "semantic", root_layer);
    const ON_Layer *sem_layer =
        ON_Layer::Cast(model->LayerFromIndex(sem_layer_idx).ModelComponent());

    for (const auto &cat : scene.semantic) {
      ON_Color cat_color(cat.color[0], cat.color[1], cat.color[2]);
      int cat_layer_idx = add_sublayer(*model, cat.name, sem_layer, cat_color);

      for (const auto &inst : cat.instances) {
        auto rhino_cloud = make_rhino_pointcloud(inst.points);

        auto *attr = new ON_3dmObjectAttributes();
        if (cat.instances.size() > 1)
          attr->m_name = ON_wString(
              fmt::format("{}_{}", cat.name, inst.instance_id).c_str());
        else
          attr->m_name = ON_wString(cat.name.c_str());
        attr->m_layer_index = cat_layer_idx;
        attr->m_color = ON_Color::UnsetColor;
        attr->SetUserString(L"type", ON_wString(cat.name.c_str()));

        model->AddManagedModelGeometryComponent(rhino_cloud.release(), attr);
      }
    }
    reusex::debug("Rhino: added semantic layer ({} categories)",
                  scene.semantic.size());
  }

  // --- Meshes ---
  if (!scene.meshes.empty()) {
    int mesh_layer_idx = add_sublayer(*model, "mesh", root_layer);

    for (const auto &entry : scene.meshes) {
      auto rhino_mesh = make_rhino_mesh(*entry.mesh);

      auto *attr = new ON_3dmObjectAttributes();
      attr->m_name = ON_wString(entry.name.c_str());
      attr->m_layer_index = mesh_layer_idx;

      model->AddManagedModelGeometryComponent(rhino_mesh.release(), attr);
    }
    reusex::debug("Rhino: added mesh layer ({} meshes)", scene.meshes.size());
  }

  // --- 360 Panoramas ---
  if (!scene.panoramas.empty()) {
    int pano_layer_idx = add_sublayer(*model, "360", root_layer);

    for (const auto &entry : scene.panoramas) {
      auto sphere = make_sphere_mesh(entry.x, entry.y, entry.z, 0.15);

      auto *attr = new ON_3dmObjectAttributes();
      attr->m_name = ON_wString(entry.image_name.c_str());
      attr->m_layer_index = pano_layer_idx;

      // User strings on attributes (per McNeel guidance)
      attr->SetUserString(L"Image Name", ON_wString(entry.image_name.c_str()));
      if (!entry.image_url.empty())
        attr->SetUserString(L"imageUrl", ON_wString(entry.image_url.c_str()));

      model->AddManagedModelGeometryComponent(sphere.release(), attr);
    }
    reusex::debug("Rhino: added 360 layer ({} panoramas)",
                  scene.panoramas.size());
  }

  // --- Materials ---
  if (!scene.materials.empty()) {
    int mat_layer_idx = add_sublayer(*model, "materials", root_layer);

    for (const auto &entry : scene.materials) {
      auto *dot = new ON_TextDot();
      dot->SetCenterPoint(ON_3dPoint(entry.x, entry.y, entry.z));
      dot->SetPrimaryText(ON_wString(entry.name.c_str()));

      auto *attr = new ON_3dmObjectAttributes();
      attr->m_name = ON_wString(entry.name.c_str());
      attr->m_layer_index = mat_layer_idx;

      // Store all properties as user strings, except "images": the
      // passport's base64 JPEG blob (the Speckle export skips it too), which
      // made up ~40 % of a demo .3dm as UTF-16 user text (reusex-f97m).
      for (const auto &[key, value] : entry.properties)
        if (key != "images")
          attr->SetUserString(ON_wString(key.c_str()),
                              ON_wString(value.c_str()));

      model->AddManagedModelGeometryComponent(dot, attr);
    }
    reusex::debug("Rhino: added materials layer ({} passports)",
                  scene.materials.size());
  }

  // --- Building Components ---
  if (!scene.components.empty()) {
    int comp_layer_idx = add_sublayer(*model, "components", root_layer);
    const ON_Layer *comp_layer =
        ON_Layer::Cast(model->LayerFromIndex(comp_layer_idx).ModelComponent());

    std::map<std::string, int> type_layers;
    for (const auto &entry : scene.components) {
      std::string type_name = entry.properties.count("type")
                                  ? entry.properties.at("type")
                                  : "unknown";
      if (auto [it, inserted] = type_layers.try_emplace(type_name); inserted)
        it->second = add_sublayer(*model, type_name, comp_layer);

      const auto &verts = entry.boundary.vertices;
      if (verts.size() < 3)
        continue;

      int vc = static_cast<int>(verts.size());
      int fc = vc - 2; // triangle fan
      auto rhino_mesh = std::make_unique<ON_Mesh>(fc, vc, false, false);
      for (int i = 0; i < vc; ++i)
        rhino_mesh->SetVertex(
            i, ON_3dPoint(verts[i].x(), verts[i].y(), verts[i].z()));
      for (int i = 0; i < fc; ++i)
        rhino_mesh->SetTriangle(i, 0, i + 1, i + 2);
      rhino_mesh->ComputeVertexNormals();

      auto *attr = new ON_3dmObjectAttributes();
      attr->m_name = ON_wString(entry.name.c_str());
      attr->m_layer_index = type_layers[type_name];
      for (const auto &[k, v] : entry.properties)
        attr->SetUserString(ON_wString(k.c_str()), ON_wString(v.c_str()));

      model->AddManagedModelGeometryComponent(rhino_mesh.release(), attr);
    }
    reusex::debug("Rhino: added components layer ({} components)",
                  scene.components.size());
  }

  // --- Typed elements: <storey>/<Walls|Slabs|Roof|Spaces|Doors|Windows|
  // Other openings>, one named mesh per element with its key data as user
  // text (W6.2, reusex-z1v.8.2) ---
  if (!scene.elements.empty()) {
    const auto storey_names = storey_layer_names(scene);
    std::map<int, int> storey_layer; // idx -> layer index
    std::map<std::pair<int, std::string>, int> type_layer;
    const auto layer_for = [&](const ExportScene::ElementEntry &e) {
      const int idx = storey_names.count(e.storey) ? e.storey : -1;
      auto [sit, fresh] = storey_layer.try_emplace(idx, -1);
      if (fresh)
        sit->second = add_sublayer(*model,
                                   idx >= 0 ? storey_names.at(idx)
                                            : std::string(kNoStoreyLayerName),
                                   root_layer);
      auto [tit, tfresh] = type_layer.try_emplace({idx, e.category}, -1);
      if (tfresh) {
        const ON_Layer *parent =
            ON_Layer::Cast(model->LayerFromIndex(sit->second).ModelComponent());
        tit->second = add_sublayer(*model, element_layer_name(e.category),
                                   parent, element_color(e.category));
      }
      return tit->second;
    };
    // Create the layers in a fixed order (storey, then category), so the
    // layer panel reads the same for every project.
    std::vector<const ExportScene::ElementEntry *> ordered;
    for (const auto &e : scene.elements)
      ordered.push_back(&e);
    const auto &cats = element_categories();
    const auto rank = [&](const ExportScene::ElementEntry *e) {
      const int s = storey_names.count(e->storey) ? e->storey : 1 << 30;
      const auto c = std::find(cats.begin(), cats.end(), e->category);
      return std::pair<int, long>(s, c - cats.begin());
    };
    std::stable_sort(
        ordered.begin(), ordered.end(),
        [&](const auto *a, const auto *b) { return rank(a) < rank(b); });
    for (const auto *e : ordered) {
      if (!e->mesh || e->mesh->polygons.empty())
        continue;
      auto rhino_mesh = make_rhino_mesh(*e->mesh);
      auto *attr = new ON_3dmObjectAttributes();
      attr->m_name = ON_wString(element_object_name(*e).c_str());
      attr->m_layer_index = layer_for(*e);
      for (const auto &[k, v] : e->properties)
        attr->SetUserString(ON_wString(k.c_str()), ON_wString(v.c_str()));
      model->AddManagedModelGeometryComponent(rhino_mesh.release(), attr);
    }
    reusex::debug("Rhino: added {} typed elements on {} storey layer(s)",
                  scene.elements.size(), storey_layer.size());
  }

  return model;
}

} // namespace reusex::io
