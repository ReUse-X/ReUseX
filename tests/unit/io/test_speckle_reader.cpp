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
#include <unordered_map>
#include <vector>

#include <sys/resource.h>

using namespace reusex::io::speckle;
using nlohmann::json;

namespace {

json stub(const std::string &id) {
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
///  ├─ elements:  [stub(level)]                       inline list of stubs
///  ├─ @elements: [stub(mesh)]                        detached list
///  └─ instanceDefinitionProxies: [ { objects: ["app.mesh"] } ]  embedded
/// mesh: vertices chunked as @(3)vertices -> [chunk1, chunk2], faces inline
json synthetic_array() {
  json root = {
      {"id", "root"},
      {"speckle_type", "Speckle.Core.Models.Collections.Collection"},
      {"version", "3"},
      {"name", "Synthetic"},
      {"elements", json::array({stub("level")})},
      {"@extra", json::array({stub("mesh")})},
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
               {"@(3)vertices", json::array({stub("c1"), stub("c2")})},
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
               {"@elements", json::array({stub("gone"), stub("here")})}};
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
  json a = {{"id", "a"}, {"@next", stub("b")}};
  json b = {{"id", "b"}, {"@next", stub("a")}};
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
      {"id", "root"}, {"left", stub("shared")}, {"right", stub("shared")}};
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
               {"@elements", json::array({stub("x")})}};
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

TEST_CASE("ObjectTreeResolve_SharedTarget_ServesSecondUseFromCache",
          "[io][speckle]") {
  json root = {{"id", "root"},
               {"a", stub("shared")},
               {"b", stub("shared")},
               {"c", stub("single")}};
  json shared = {{"id", "shared"}, {"value", 7}};
  json single = {{"id", "single"}, {"value", 8}};
  ObjectTree tree("root",
                  {{"root", root}, {"shared", shared}, {"single", single}});
  ResolveReport report;
  json r = tree.resolve_root(&report);
  CHECK(r["a"]["value"] == 7);
  CHECK(r["b"]["value"] == 7);
  CHECK(r["c"]["value"] == 8);
  CHECK(report.references_resolved == 3);
  CHECK(report.cache_hits == 1); // only the second use of "shared"
}

TEST_CASE("ObjectTreeResolve_SharedDataChunk_IsNotCached", "[io][speckle]") {
  json root = {{"id", "root"},
               {"@(2)xs", json::array({stub("c")})},
               {"@(2)ys", json::array({stub("c")})}};
  ObjectTree tree("root", {{"root", root}, {"c", chunk("c", {1, 2})}});
  ResolveReport report;
  json r = tree.resolve_root(&report);
  CHECK(r["xs"] == json({1, 2}));
  CHECK(r["ys"] == json({1, 2}));
  CHECK(report.references_resolved == 2);
  CHECK(report.cache_hits == 0);
  CHECK(report.chunk_arrays_joined == 2);
}

TEST_CASE("ObjectTreeResolve_StubWithoutReferenceType_IsLeftAlone",
          "[io][speckle]") {
  json root = {{"id", "root"}, {"link", {{"referencedId", "x"}}}};
  ObjectTree tree("root", {{"root", root}, {"x", {{"id", "x"}, {"v", 1}}}});
  ResolveReport report;
  json r = tree.resolve_root(&report);
  CHECK(r["link"] == json({{"referencedId", "x"}}));
  CHECK(report.references_resolved == 0);
}

TEST_CASE("ObjectTreeResolve_ReferenceChainTooDeep_Throws", "[io][speckle]") {
  // o0 -> o1 -> ... -> o599 through references: 600 objects, > 512 levels.
  std::unordered_map<std::string, json> objects;
  constexpr int n = 600;
  for (int i = 0; i < n; ++i) {
    json o = {{"id", "o" + std::to_string(i)}};
    if (i + 1 < n)
      o["@next"] = stub("o" + std::to_string(i + 1));
    objects["o" + std::to_string(i)] = o;
  }
  ObjectTree tree("o0", objects);
  CHECK_THROWS_AS(tree.resolve_root(), std::runtime_error);

  ResolveOptions loose;
  loose.max_depth = 2 * n;
  ResolveReport report;
  REQUIRE_NOTHROW(tree.resolve_root(&report, loose));
  CHECK(report.references_resolved == n - 1);
  CHECK(report.max_depth_reached >= n);
}

TEST_CASE("ObjectTreeResolve_InlineNestingTooDeep_Throws", "[io][speckle]") {
  json v = json::array();
  for (int i = 0; i < 600; ++i)
    v = json::array({v});
  ObjectTree tree("root", {{"root", {{"id", "root"}, {"deep", v}}}});
  CHECK_THROWS_AS(tree.resolve_root(), std::runtime_error);
}

TEST_CASE("ObjectTreeResolve_DiamondBlowUp_ThrowsOnNodeBudget",
          "[io][speckle]") {
  // dK references d(K+1) twice: the inlined form doubles per level (2^40).
  std::unordered_map<std::string, json> objects;
  constexpr int levels = 40;
  for (int i = 0; i < levels; ++i) {
    const std::string next = "d" + std::to_string(i + 1);
    objects["d" + std::to_string(i)] = {
        {"id", "d" + std::to_string(i)}, {"l", stub(next)}, {"r", stub(next)}};
  }
  objects["d" + std::to_string(levels)] = {
      {"id", "d" + std::to_string(levels)}};
  ObjectTree tree("d0", objects);
  ResolveOptions opts;
  opts.max_nodes = 100'000;
  CHECK_THROWS_AS(tree.resolve_root(nullptr, opts), std::runtime_error);
}

TEST_CASE("IsValidSpeckleId_VariousInputs_AcceptsOnlyAlphanumeric",
          "[io][speckle]") {
  CHECK(is_valid_speckle_id("63142b1e80"));
  CHECK(is_valid_speckle_id("155e126736a671b12c02d4e3482ae3f6"));
  CHECK_FALSE(is_valid_speckle_id(""));
  CHECK_FALSE(is_valid_speckle_id("../graphql"));
  CHECK_FALSE(is_valid_speckle_id("abc?x=1"));
  CHECK_FALSE(is_valid_speckle_id("abc def"));
  CHECK_FALSE(is_valid_speckle_id(std::string(129, 'a')));
}

TEST_CASE("IsQueryDocument_VariousDocuments_AcceptsOnlyQueries",
          "[io][speckle]") {
  CHECK(is_query_document("query { project(id: \"x\") { id } }"));
  CHECK(is_query_document(
      "  query Named($p: String!) { project(id: $p) { id } }"));
  CHECK(is_query_document("query A { a { ...F } } fragment F on T { b }"));
  CHECK(is_query_document("# mutation in a comment\nquery { a }"));
  CHECK(is_query_document("query { a(s: \"mutation { x }\") }"));
  CHECK(is_query_document("query { a(s: \"\"\"subscription\"\"\") }"));
  CHECK(is_query_document("query { versionMutations }"));

  CHECK_FALSE(is_query_document(""));
  CHECK_FALSE(is_query_document("{ project { id } }")); // shorthand
  CHECK_FALSE(is_query_document("mutation { branchCreate }"));
  CHECK_FALSE(is_query_document("subscription { s }"));
  CHECK_FALSE(is_query_document("query { a } mutation { b }"));
  CHECK_FALSE(is_query_document("query { a } { b }"));
  CHECK_FALSE(is_query_document("query { mutation }"));
  CHECK_FALSE(is_query_document("query { a"));           // unbalanced
  CHECK_FALSE(is_query_document("query { a(s: \"x) }")); // open string
}

TEST_CASE("HttpErrorMessage_KnownStatuses_ExplainTheCause", "[io][speckle]") {
  using detail::http_error_message;
  CHECK(http_error_message(401, "GET", "p1", "").find("not authenticated") !=
        std::string::npos);
  CHECK(http_error_message(403, "GET", "p1", "")
            .find("token invalid or no access to project 'p1'") !=
        std::string::npos);
  CHECK(http_error_message(404, "GET", "p1", "").find("not found") !=
        std::string::npos);
  auto msg = http_error_message(500, "GET", "p1", std::string(1000, 'x'));
  CHECK(msg.find("HTTP 500") != std::string::npos);
  CHECK(msg.size() < 600); // body truncated
}

TEST_CASE("GraphqlDataOrThrow_ErrorsAndData_MapsToResultOrException",
          "[io][speckle]") {
  using detail::graphql_data_or_throw;
  CHECK(graphql_data_or_throw(json::parse(R"({"data": {"a": 1}})"), "p") ==
        json({{"a", 1}}));
  try {
    (void)graphql_data_or_throw(json::parse(R"({"errors": [{"message": "no",
                     "extensions": {"code": "FORBIDDEN"}}], "data": null})"),
                                "p1");
    FAIL("expected an exception");
  } catch (const std::runtime_error &e) {
    CHECK(std::string(e.what()).find("no access to project 'p1'") !=
          std::string::npos);
  }
  CHECK_THROWS_AS(graphql_data_or_throw(
                      json::parse(R"({"errors": [{"message": "bad"}]})"), "p"),
                  std::runtime_error);
  CHECK_THROWS_AS(graphql_data_or_throw(json::parse(R"({"data": null})"), "p"),
                  std::runtime_error);
}

TEST_CASE("IsAllowedServerUrl_Schemes_HttpsOrLocalHttpOnly", "[io][speckle]") {
  using detail::is_allowed_server_url;
  CHECK(is_allowed_server_url("https://app.speckle.systems"));
  CHECK(is_allowed_server_url("HTTPS://app.speckle.systems"));
  CHECK(is_allowed_server_url("http://localhost:3000"));
  CHECK(is_allowed_server_url("http://127.0.0.1"));
  CHECK(is_allowed_server_url("http://[::1]:8080/"));
  CHECK_FALSE(is_allowed_server_url("http://app.speckle.systems"));
  CHECK_FALSE(is_allowed_server_url("http://localhost.evil.com"));
  CHECK_FALSE(is_allowed_server_url("http://localhost@evil.com"));
  CHECK_FALSE(is_allowed_server_url("ftp://localhost"));
  CHECK_FALSE(is_allowed_server_url("file:///etc/passwd"));
  CHECK_FALSE(is_allowed_server_url("https://"));
}

TEST_CASE("SpeckleReader_BadUrlOrIds_ThrowsBeforeAnyRequest", "[io][speckle]") {
  CHECK_THROWS_AS(SpeckleReader("http://app.speckle.systems", "t"),
                  std::runtime_error);
  SpeckleReader reader("https://example.invalid", "t");
  CHECK(reader.max_download_bytes() == (std::size_t{4} << 30));
  reader.set_max_download_bytes(1024);
  CHECK(reader.max_download_bytes() == 1024);
  CHECK_THROWS_AS(reader.query_project("../x"), std::invalid_argument);
  CHECK_THROWS_AS(reader.query_versions("p1", "m/1"), std::invalid_argument);
  CHECK_THROWS_AS(reader.fetch_object_tree("p1", ""), std::invalid_argument);
  CHECK_THROWS_AS(reader.fetch_object("p 1", "abc"), std::invalid_argument);
}

TEST_CASE("CollectProjectPages_Pagination_FollowsCursorUntilDone",
          "[io][speckle]") {
  auto page = [](json cursor, int total, std::vector<std::string> ids) {
    json items = json::array();
    for (const auto &id : ids)
      items.push_back({{"id", id}, {"name", "n" + id}});
    return json{
        {"project",
         {{"id", "p"},
          {"name", "P"},
          {"models",
           {{"cursor", cursor}, {"totalCount", total}, {"items", items}}}}}};
  };

  SECTION("two pages") {
    std::vector<json> seen;
    auto info = collect_project_pages([&](const json &cursor) {
      seen.push_back(cursor);
      return seen.size() == 1 ? page("c1", 3, {"a", "b"})
                              : page(nullptr, 3, {"c"});
    });
    CHECK(info.name == "P");
    REQUIRE(info.models.size() == 3);
    CHECK(info.models[2].id == "c");
    REQUIRE(seen.size() == 2);
    CHECK(seen[0].is_null());
    CHECK(seen[1] == "c1");
  }
  SECTION("totalCount reached stops even with a cursor") {
    int calls = 0;
    auto info = collect_project_pages([&](const json &) {
      ++calls;
      return page("more", 2, {"a", "b"});
    });
    CHECK(calls == 1);
    CHECK(info.models.size() == 2);
  }
  SECTION("null totalCount and null models are tolerated") {
    int calls = 0;
    auto info = collect_project_pages([&](const json &) {
      ++calls;
      if (calls == 1) {
        json p = page("c1", 0, {"a"});
        p["project"]["models"]["totalCount"] = nullptr;
        return p;
      }
      json p = page(nullptr, 0, {});
      p["project"]["models"] = nullptr;
      return p;
    });
    CHECK(calls == 2);
    CHECK(info.models.size() == 1);
  }
  SECTION("page cap bounds a server that never stops") {
    int calls = 0;
    auto info = collect_project_pages(
        [&](const json &) {
          ++calls;
          json p = page("again", 0, {"x"});
          p["project"]["models"]["totalCount"] = nullptr;
          return p;
        },
        5);
    CHECK(calls == 5);
    CHECK(info.models.size() == 5);
  }
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
            << " missing, " << report.cyclic_ids.size() << " cyclic, "
            << report.cache_hits << " cache hits, " << report.nodes
            << " nodes, depth " << report.max_depth_reached << "\n";
  for (const auto &[type, n] : tree.type_histogram())
    std::cout << "  " << n << "  " << type << "\n";
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::cout << "peak RSS " << usage.ru_maxrss / 1024 << " MiB\n";
}
