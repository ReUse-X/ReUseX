// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Smoke test for the packaged web-ifc C++ core (pkgs/web-ifc, reusex-3iy8.10):
// parse a hand-written IFC4 file and get its storey and a wall mesh. This only
// proves the dependency builds, links and works natively; `rux import ifc` is
// a separate task and gets its own tests.

#include <catch2/catch_test_macros.hpp>

#ifdef REUSEX_HAVE_WEB_IFC

#include "../../support/source_dir.hpp"

#include <glm/glm.hpp>
#include <modelmanager/ModelManager.h>
#include <schema/ifc-schema.h>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

namespace {

std::filesystem::path fixture() {
  return reusex::test_support::source_dir() / "tests" / "fixtures" / "ifc" /
         "minimal_wall.ifc";
}

// web-ifc emits meshes Y-up for three.js (GetFlatMesh pre-multiplies
// NormalizeIFC). Its inverse as the processor transformation gives the IFC
// file's own Z-up world coordinates instead.
std::array<double, 16> ifc_z_up() {
  const glm::dmat4 inv = glm::inverse(webifc::geometry::NormalizeIFC);
  std::array<double, 16> flat{};
  for (int c = 0; c < 4; ++c)
    for (int r = 0; r < 4; ++r)
      flat[static_cast<size_t>(c * 4 + r)] = inv[c][r];
  return flat;
}

} // namespace

TEST_CASE("web-ifc parses a storey and meshes a wall", "[io][ifc]") {
  webifc::manager::ModelManager manager(false);
  // Not SetLogLevel(): that re-levels and re-patterns spdlog's *default*
  // logger, which the host application owns. A sink-less logger of its own
  // silences the version banner CreateModel prints.
  manager.SetLogger(std::make_shared<spdlog::logger>("web-ifc"));
  const uint32_t model = manager.CreateModel({});

  std::ifstream in(fixture(), std::ios::binary);
  REQUIRE(in);
  auto *loader = manager.GetIfcLoader(model);
  REQUIRE(loader->LoadFile(in));

  const auto storeys =
      loader->GetExpressIDsWithType(webifc::schema::IFCBUILDINGSTOREY);
  REQUIRE(storeys.size() == 1);
  loader->MoveToLineArgument(storeys[0], 2); // IfcRoot.Name
  CHECK(loader->GetDecodedStringArgument() == "Ground Floor");

  const auto walls = loader->GetExpressIDsWithType(webifc::schema::IFCWALL);
  REQUIRE(walls.size() == 1);

  auto *geometry = manager.GetGeometryProcessor(model);
  geometry->SetTransformation(ifc_z_up());
  const auto mesh = geometry->GetFlatMesh(walls[0]);
  REQUIRE(mesh.geometries.size() == 1);

  const auto &placed = mesh.geometries[0];
  const auto &g = geometry->GetGeometry(placed.geometryExpressID);
  // vertexData is interleaved position + normal, 6 doubles per vertex.
  REQUIRE(g.vertexData.size() % 6 == 0);
  REQUIRE(g.vertexData.size() >= 8 * 6);
  REQUIRE(g.indexData.size() >= 12 * 3); // a box: at least 12 triangles

  constexpr double inf = std::numeric_limits<double>::infinity();
  glm::dvec3 lo(inf), hi(-inf);
  for (size_t i = 0; i < g.vertexData.size(); i += 6) {
    const glm::dvec3 p =
        placed.transformation * glm::dvec4(g.vertexData[i], g.vertexData[i + 1],
                                           g.vertexData[i + 2], 1.0);
    lo = glm::min(lo, p);
    hi = glm::max(hi, p);
  }

  // 4 x 0.2 m profile extruded 3 m, placed at (1, 2, 0) on the storey.
  constexpr double eps = 1e-6;
  CHECK(std::abs(lo.x - 1.0) < eps);
  CHECK(std::abs(lo.y - 2.0) < eps);
  CHECK(std::abs(lo.z - 0.0) < eps);
  CHECK(std::abs(hi.x - 5.0) < eps);
  CHECK(std::abs(hi.y - 2.2) < eps);
  CHECK(std::abs(hi.z - 3.0) < eps);
}

#else

TEST_CASE("web-ifc parses a storey and meshes a wall", "[io][ifc]") {
  SKIP("built without web-ifc (REUSEX_HAVE_WEB_IFC off)");
}

#endif
