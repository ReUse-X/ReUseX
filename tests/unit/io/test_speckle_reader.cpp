// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Offline tests for the read-only Speckle receive path. The object trees here
// are hand-written synthetic data mirroring the v3 wire format (reference
// stubs, @-detached keys, DataChunks, __closure); no customer data.

#include <reusex/io/speckle_reader.hpp>

#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace reusex::io::speckle;
using nlohmann::json;

namespace {

json ref(const std::string &id) {
  return {{"speckle_type", "reference"},
          {"referencedId", id},
          {"__closure", nullptr}};
}

json chunk(const std::string &id, json data) {
  return {{"id", id},
          {"speckle_type", "Speckle.Core.Models.DataChunk"},
          {"data", std::move(data)}};
}

/// root (Collection, inline `elements` + detached `@elements`)
///  ├─ elements:  [ref(level)]                       inline list of stubs
///  ├─ @elements: [ref(mesh)]                        detached list
///  └─ instanceDefinitionProxies: [ { objects: ["app.mesh"] } ]  embedded
/// mesh: vertices chunked as @(3)vertices -> [chunk1, chunk2], faces inline
json synthetic_array() {
  json root = {
      {"id", "root"},
      {"speckle_type", "Speckle.Core.Models.Collections.Collection"},
      {"version", "3"},
      {"name", "Synthetic"},
      {"elements", json::array({ref("level")})},
      {"@extra", json::array({ref("mesh")})},
      {"instanceDefinitionProxies",
       json::array({{{"speckle_type",
                      "Speckle.Core.Models.Instances.InstanceDefinitionProxy"},
                     {"applicationId", "def.1"},
                     {"objects", json::array({"app.mesh"})}}})},
      {"__closure", {{"level", 1}, {"mesh", 1}, {"c1", 2}, {"c2", 2}}}};
  json level = {{"id", "level"},
                {"speckle_type", "Speckle.Core.Models.Collections.Collection"},
                {"name", "Level 1"},
                {"elements", json::array()}};
  json mesh = {{"id", "mesh"},
               {"speckle_type", "Objects.Geometry.Mesh"},
               {"applicationId", "app.mesh"},
               {"@(3)vertices", json::array({ref("c1"), ref("c2")})},
               {"faces", json::array({0, 0, 1, 2})},
               {"units", "m"},
               {"__closure", {{"c1", 1}, {"c2", 1}}}};
  return json::array({level, mesh, chunk("c1", {0.0, 0.0, 0.0}),
                      chunk("c2", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0}), root});
}

ObjectTree tree_from(const json &arr, const std::string &root_id = "root") {
  return ObjectTree::parse(arr.dump(), root_id);
}

} // namespace

TEST_CASE("ObjectTreeParse_JsonArray_IndexesEveryObjectById", "[io][speckle]") {
  auto tree = tree_from(synthetic_array());
  CHECK(tree.size() == 5);
  CHECK(tree.root_id() == "root");
  CHECK(tree.contains("mesh"));
  CHECK_FALSE(tree.contains("nope"));
  CHECK(tree.find("nope") == nullptr);
  CHECK(tree.root()["name"] == "Synthetic");

  auto hist = tree.type_histogram();
  CHECK(hist["Speckle.Core.Models.Collections.Collection"] == 2);
  CHECK(hist["Speckle.Core.Models.DataChunk"] == 2);
  CHECK(hist["Objects.Geometry.Mesh"] == 1);
}

TEST_CASE("ObjectTreeParse_TabSeparatedLines_MatchesArrayFormat",
          "[io][speckle]") {
  std::string body;
  for (const auto &obj : synthetic_array())
    body += obj["id"].get<std::string>() + "\t" + obj.dump() + "\r\n";
  body += "\n"; // trailing blank line is tolerated
  auto lines = ObjectTree::parse(body, "root");
  auto array = tree_from(synthetic_array());
  CHECK(lines.size() == array.size());
  CHECK(lines.resolve_root() == array.resolve_root());
}

TEST_CASE("ObjectTreeParse_BareJsonLines_UsesEmbeddedIds", "[io][speckle]") {
  std::string body;
  for (const auto &obj : synthetic_array())
    body += obj.dump() + "\n";
  auto tree = ObjectTree::parse(body, "root");
  CHECK(tree.size() == 5);
  CHECK(tree.contains("c2"));
}

TEST_CASE("ObjectTreeParse_BadInput_Throws", "[io][speckle]") {
  SECTION("root not in download") {
    CHECK_THROWS_AS(tree_from(synthetic_array(), "absent"), std::runtime_error);
  }
  SECTION("malformed line") {
    CHECK_THROWS_AS(ObjectTree::parse("a\t{\"id\": \"a\"\n", "a"),
                    std::runtime_error);
  }
  SECTION("object without id") {
    CHECK_THROWS_AS(ObjectTree::parse("[{\"x\": 1}]", "a"), std::runtime_error);
  }
  SECTION("empty body") {
    CHECK_THROWS_AS(ObjectTree::parse("", "a"), std::runtime_error);
  }
}

TEST_CASE("ObjectTreeResolve_ReferencesAndDetachedKeys_InlinesTargets",
          "[io][speckle]") {
  auto tree = tree_from(synthetic_array());
  ResolveReport report;
  json root = tree.resolve_root(&report);

  CHECK(report.missing_ids.empty());
  CHECK(report.cyclic_ids.empty());
  CHECK_FALSE(root.contains("__closure"));

  // Inline list of stubs -> resolved objects.
  REQUIRE(root["elements"].size() == 1);
  CHECK(root["elements"][0]["name"] == "Level 1");

  // `@extra` lost its `@`.
  CHECK_FALSE(root.contains("@extra"));
  REQUIRE(root["extra"].size() == 1);
  const json &mesh = root["extra"][0];
  CHECK(mesh["speckle_type"] == "Objects.Geometry.Mesh");
  CHECK_FALSE(mesh.contains("__closure"));

  // Embedded proxies are untouched.
  CHECK(root["instanceDefinitionProxies"][0]["objects"][0] == "app.mesh");
}

TEST_CASE("ObjectTreeResolve_ChunkedProperty_ConcatenatesChunkData",
          "[io][speckle]") {
  auto tree = tree_from(synthetic_array());
  ResolveReport report;
  json mesh = tree.resolve(*tree.find("mesh"), &report);

  CHECK_FALSE(mesh.contains("@(3)vertices"));
  REQUIRE(mesh.contains("vertices"));
  CHECK(mesh["vertices"] ==
        json({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0}));
  CHECK(mesh["faces"] == json({0, 0, 1, 2}));
  CHECK(report.chunk_arrays_joined == 1);
  CHECK(report.references_resolved == 2);
}

TEST_CASE("ObjectTreeResolve_InlineChunks_AreJoinedToo", "[io][speckle]") {
  // Chunks embedded rather than detached, and a v2-namespaced chunk type.
  json obj = {{"id", "o"},
              {"colors",
               json::array({{{"speckle_type", "Objects.Geometry.DataChunk"},
                             {"data", {1, 2}}},
                            {{"speckle_type", "Speckle.Core.Models.DataChunk"},
                             {"data", {3}}}})}};
  ObjectTree tree("o", {{"o", obj}});
  CHECK(tree.resolve_root()["colors"] == json({1, 2, 3}));
}

TEST_CASE("ObjectTreeResolve_MixedArray_IsNotFlattened", "[io][speckle]") {
  json obj = {
      {"id", "o"},
      {"items", json::array({chunk("x", {1}), {{"speckle_type", "Other"}}})}};
  ObjectTree tree("o", {{"o", obj}});
  json r = tree.resolve_root();
  REQUIRE(r["items"].size() == 2);
  CHECK(r["items"][0]["data"] == json({1}));
}

TEST_CASE("ObjectTreeResolve_MissingId_LeavesStubAndReportsIt",
          "[io][speckle]") {
  json root = {{"id", "root"},
               {"@elements", json::array({ref("gone"), ref("here")})}};
  json here = {{"id", "here"}, {"speckle_type", "Base"}};
  ObjectTree tree("root", {{"root", root}, {"here", here}});

  ResolveReport report;
  json r;
  REQUIRE_NOTHROW(r = tree.resolve_root(&report));
  REQUIRE(report.missing_ids == std::vector<std::string>{"gone"});
  CHECK(r["elements"][0]["referencedId"] == "gone");
  CHECK(r["elements"][0]["speckle_type"] == "reference");
  CHECK(r["elements"][1]["speckle_type"] == "Base");
}

TEST_CASE("ObjectTreeResolve_Cycle_BreaksWithStubAndReportsIt",
          "[io][speckle]") {
  json a = {{"id", "a"}, {"@next", ref("b")}};
  json b = {{"id", "b"}, {"@next", ref("a")}};
  ObjectTree tree("a", {{"a", a}, {"b", b}});

  ResolveReport report;
  json r;
  REQUIRE_NOTHROW(r = tree.resolve_root(&report));
  CHECK(r["next"]["id"] == "b");
  CHECK(r["next"]["next"]["referencedId"] == "a");
  CHECK(report.cyclic_ids == std::vector<std::string>{"a"});
}

TEST_CASE("ObjectTreeResolve_SharedTarget_ResolvesEachUse", "[io][speckle]") {
  json root = {
      {"id", "root"}, {"left", ref("shared")}, {"right", ref("shared")}};
  json shared = {{"id", "shared"}, {"value", 7}};
  ObjectTree tree("root", {{"root", root}, {"shared", shared}});
  json r = tree.resolve_root();
  CHECK(r["left"]["value"] == 7);
  CHECK(r["right"]["value"] == 7);
}

TEST_CASE("ObjectTreeResolve_BothKeySpellings_KeepsDetachedKey",
          "[io][speckle]") {
  json root = {{"id", "root"},
               {"elements", json::array({1})},
               {"@elements", json::array({ref("x")})}};
  json x = {{"id", "x"}, {"v", 2}};
  ObjectTree tree("root", {{"root", root}, {"x", x}});
  json r = tree.resolve_root();
  CHECK(r["elements"] == json({1}));
  CHECK(r["@elements"][0]["v"] == 2);
}

TEST_CASE("ObjectTreeRoot_EmptyTree_Throws", "[io][speckle]") {
  ObjectTree tree;
  CHECK(tree.size() == 0);
  CHECK_THROWS_AS(tree.root(), std::runtime_error);
}

TEST_CASE("ParseProjectInfo_GraphqlData_ReadsModels", "[io][speckle]") {
  json data = json::parse(R"({"project": {"id": "p1", "name": "Demo",
      "models": {"cursor": null, "totalCount": 2, "items": [
        {"id": "m1", "name": "main"}, {"id": "m2", "name": "plans/first"}]}}})");
  auto info = parse_project_info(data);
  CHECK(info.id == "p1");
  CHECK(info.name == "Demo");
  REQUIRE(info.models.size() == 2);
  CHECK(info.models[1].name == "plans/first");

  CHECK_THROWS_AS(parse_project_info(json::parse(R"({"project": null})")),
                  std::runtime_error);
}

TEST_CASE("ParseVersions_GraphqlData_ReadsFieldsAndNullMessage",
          "[io][speckle]") {
  json data = json::parse(R"({"project": {"id": "p1", "model": {"id": "m1",
      "versions": {"items": [
        {"id": "v2", "referencedObject": "abc", "sourceApplication": "rvt",
         "createdAt": "2026-10-01T10:00:00.000Z", "message": null},
        {"id": "v1", "referencedObject": "def", "sourceApplication": "rvt",
         "createdAt": "2026-09-01T10:00:00.000Z", "message": "first"}]}}}})");
  auto versions = parse_versions(data);
  REQUIRE(versions.size() == 2);
  CHECK(versions[0].id == "v2");
  CHECK(versions[0].referenced_object == "abc");
  CHECK(versions[0].source_application == "rvt");
  CHECK(versions[0].created_at == "2026-10-01T10:00:00.000Z");
  CHECK(versions[0].message.empty());
  CHECK(versions[1].message == "first");

  CHECK_THROWS_AS(
      parse_versions(json::parse(R"({"project": {"id": "p", "model": null}})")),
      std::runtime_error);
}

TEST_CASE("SpeckleReader_EmptyServerUrl_Throws", "[io][speckle]") {
  CHECK_THROWS_AS(SpeckleReader("", "t"), std::runtime_error);
  CHECK(SpeckleReader("https://example.org///", "t").server_url() ==
        "https://example.org");
}

// Live, read-only check against a real server. Hidden by default; runs only
// with `"[.network]"` and SPECKLE_TOKEN, SPECKLE_TEST_PROJECT and
// SPECKLE_TEST_MODEL set (SPECKLE_SERVER defaults to app.speckle.systems).
// Issues GraphQL queries and GETs only.
TEST_CASE("SpeckleReader_LiveServer_QueriesAndFetchesLatestVersion",
          "[.network][io][speckle]") {
  const char *token = std::getenv("SPECKLE_TOKEN");
  const char *project = std::getenv("SPECKLE_TEST_PROJECT");
  const char *model = std::getenv("SPECKLE_TEST_MODEL");
  if (!token || !project || !model)
    SKIP("set SPECKLE_TOKEN, SPECKLE_TEST_PROJECT and SPECKLE_TEST_MODEL");
  const char *server_env = std::getenv("SPECKLE_SERVER");
  SpeckleReader reader(server_env ? server_env : "https://app.speckle.systems",
                       token);

  auto info = reader.query_project(project);
  CHECK_FALSE(info.models.empty());
  auto version = reader.latest_version(project, model);
  REQUIRE_FALSE(version.referenced_object.empty());

  auto single = reader.fetch_object(project, version.referenced_object);
  CHECK(single["id"] == version.referenced_object);

  auto tree = reader.fetch_object_tree(project, version.referenced_object);
  ResolveReport report;
  json root = tree.resolve_root(&report);
  CHECK(report.missing_ids.empty());

  std::cout << "project '" << info.name << "': " << info.models.size()
            << " models\nversion " << version.id << " ("
            << version.source_application << ", " << version.created_at
            << ") root " << version.referenced_object << "\n"
            << tree.size() << " objects, " << report.references_resolved
            << " references resolved, " << report.chunk_arrays_joined
            << " chunk arrays joined, " << report.missing_ids.size()
            << " missing, " << report.cyclic_ids.size() << " cyclic\n";
  for (const auto &[type, n] : tree.type_histogram())
    std::cout << "  " << n << "  " << type << "\n";
}
