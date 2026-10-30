// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The solid-model JSON document (reusex-z1v.2.5): one format for authored
// reference plans and detected models.

#include "../../support/source_dir.hpp"
#include "../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Catch::Matchers::ContainsSubstring;
using reusex::core::SolidModel;
using reusex::io::solid_model_from_json;
using reusex::io::solid_model_to_json;

namespace {

std::string read_file(const std::filesystem::path &p) {
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::filesystem::path fixture(const char *name) {
  return reusex::test_support::source_dir() / "tests" / "fixtures" /
         "solid_model" / name;
}

/// A minimal valid document; tests mutate it to provoke one error each.
nlohmann::json minimal_doc() {
  return nlohmann::json::parse(R"({
    "model_version": 1,
    "storeys": [{"idx": 0, "name": "Ground", "z_floor": 0, "z_ceiling": 2.6}],
    "walls": [
      {"guid": "w1", "storey": 0, "axis": [[0, 0], [4, 0]], "thickness": 0.2},
      {"guid": "w2", "storey": 0, "axis": [[4, 0], [4, 3]], "thickness": 0.1}
    ],
    "spaces": [
      {"guid": "s1", "storey": 0, "name": "Kitchen",
       "polygon": {"outer": [[0, 0], [4, 0], [4, 3], [0, 3]]}}
    ],
    "openings": [
      {"guid": "o1", "wall": "w1", "kind": "door", "s_start": 1.0,
       "s_end": 1.9, "z_sill": 0.0, "z_head": 2.1}
    ]
  })");
}

} // namespace

TEST_CASE("SolidModelFromJson_MinimalDocument_ParsesEveryElement",
          "[io][solid_model]") {
  const SolidModel m = solid_model_from_json(minimal_doc().dump());
  REQUIRE(m.storeys.size() == 1);
  CHECK(m.storeys[0].name == "Ground");
  CHECK(m.storeys[0].z_ceiling == 2.6);
  REQUIRE(m.walls.size() == 2);
  CHECK(m.walls[0].guid == "w1");
  CHECK(m.walls[0].x1 == 4.0);
  CHECK(m.walls[0].thickness == 0.2);
  CHECK(m.walls[1].y1 == 3.0);
  REQUIRE(m.spaces.size() == 1);
  CHECK(m.spaces[0].polygon.outer.size() == 4);
  CHECK(m.spaces[0].polygon.holes.empty());
  REQUIRE(m.openings.size() == 1);
  CHECK(m.openings[0].wall == "w1");
  CHECK(m.openings[0].kind == "door");
  CHECK(m.openings[0].z_head == 2.1);
  CHECK(m.has_walls());
  CHECK(m.has_spaces());
  CHECK(m.has_openings());
}

TEST_CASE("SolidModelFromJson_OmittedOptionalFields_TakeRecordDefaults",
          "[io][solid_model]") {
  const SolidModel m = solid_model_from_json(minimal_doc().dump());
  const auto &w = m.walls[0];
  CHECK(w.source == "detected");
  CHECK_FALSE(w.locked);
  CHECK(w.confidence == -1.0);
  CHECK(w.seen_share == -1.0);
  CHECK_FALSE(w.assumed);
  CHECK(m.storeys[0].level_transform[0] == 1.0);
  CHECK(m.storeys[0].level_transform[1] == 0.0);
}

TEST_CASE("SolidModelJson_RoundTrip_IsLossless", "[io][solid_model]") {
  const std::string first = solid_model_to_json(
      solid_model_from_json(read_file(fixture("two_rooms.plan.json"))));
  const std::string second = solid_model_to_json(solid_model_from_json(first));
  CHECK(first == second);

  const auto j = nlohmann::json::parse(first);
  CHECK(j.at("model_version") == reusex::io::kSolidModelJsonVersion);
  CHECK(j.at("walls").size() == 7);
  CHECK(j.at("spaces").size() == 2);
  CHECK(j.at("openings").size() == 3);
}

TEST_CASE("SolidModelJson_ProvidedButEmpty_SurvivesRoundTrip",
          "[io][solid_model]") {
  // "walls": [] means "this model has no walls" (scored as a miss), which is
  // not the same as omitting the key ("this model says nothing about walls").
  const SolidModel m =
      solid_model_from_json(R"({"model_version": 1, "walls": []})");
  CHECK(m.has_walls());
  CHECK_FALSE(m.has_spaces());
  const auto j = nlohmann::json::parse(solid_model_to_json(m));
  CHECK(j.contains("walls"));
  CHECK(j.at("walls").empty());
  CHECK_FALSE(j.contains("spaces"));
}

TEST_CASE("SolidModelFromJson_MetadataObject_RoundTripsAsJson",
          "[io][solid_model]") {
  auto doc = minimal_doc();
  doc["walls"][0]["metadata"] = {{"author", "agent"}, {"pass", 2}};
  const SolidModel m = solid_model_from_json(doc.dump());
  CHECK(nlohmann::json::parse(m.walls[0].metadata) ==
        doc["walls"][0]["metadata"]);
  const auto out = nlohmann::json::parse(solid_model_to_json(m));
  CHECK(out["walls"][0]["metadata"] == doc["walls"][0]["metadata"]);
}

TEST_CASE("SolidModelFromJson_InvalidDocuments_ThrowNamingTheElement",
          "[io][solid_model]") {
  SECTION("not JSON") {
    CHECK_THROWS_AS(solid_model_from_json("{nope"), std::invalid_argument);
  }
  SECTION("missing model_version") {
    auto doc = minimal_doc();
    doc.erase("model_version");
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("model_version"));
  }
  SECTION("future model_version") {
    auto doc = minimal_doc();
    doc["model_version"] = 99;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("99"));
  }
  SECTION("unknown key is a typo, not silently ignored") {
    auto doc = minimal_doc();
    doc["walls"][1]["thicknes"] = 0.3;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[1]") &&
                          ContainsSubstring("thicknes"));
  }
  SECTION("zero-length wall axis") {
    auto doc = minimal_doc();
    doc["walls"][0]["axis"] = {{1, 1}, {1, 1}};
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[0]") && ContainsSubstring("w1"));
  }
  SECTION("wrong type") {
    auto doc = minimal_doc();
    doc["walls"][0]["thickness"] = "thick";
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[0]") &&
                          ContainsSubstring("thickness"));
  }
  SECTION("polygon with two points") {
    auto doc = minimal_doc();
    doc["spaces"][0]["polygon"]["outer"] = {{0, 0}, {1, 0}};
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("spaces[0]"));
  }
  SECTION("opening on a wall that does not exist") {
    auto doc = minimal_doc();
    doc["openings"][0]["wall"] = "w9";
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("openings[0]") &&
                          ContainsSubstring("w9"));
  }
  SECTION("opening with s_end before s_start") {
    auto doc = minimal_doc();
    doc["openings"][0]["s_end"] = 0.5;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("openings[0]"));
  }
  SECTION("duplicate wall guid") {
    auto doc = minimal_doc();
    doc["walls"][1]["guid"] = "w1";
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("w1"));
  }
  SECTION("wall on a storey that does not exist") {
    auto doc = minimal_doc();
    doc["walls"][1]["storey"] = 3;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[1]") &&
                          ContainsSubstring("storey"));
  }
  SECTION("level_transform with the wrong length") {
    auto doc = minimal_doc();
    doc["storeys"][0]["level_transform"] = {1, 0, 0};
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("storeys[0]") &&
                          ContainsSubstring("level_transform"));
  }
}

TEST_CASE("SolidModelFromJson_EmptyDocument_ProvidesNothing",
          "[io][solid_model]") {
  const SolidModel m = solid_model_from_json(R"({"model_version": 1})");
  CHECK_FALSE(m.has_storeys());
  CHECK_FALSE(m.has_walls());
  CHECK_FALSE(m.has_spaces());
  CHECK_FALSE(m.has_openings());
}

TEST_CASE("LoadSolidModel_ProjectWithoutSolidModelTables_ProvidesNothing",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_load_solid_model");
  reusex::ProjectDB db(tmp.path);
  const SolidModel m = reusex::io::load_solid_model(db);
  CHECK_FALSE(m.has_storeys());
  CHECK_FALSE(m.has_walls());
  CHECK_FALSE(m.has_spaces());
  CHECK_FALSE(m.has_openings());
}

TEST_CASE("SolidModelFromJson_InvalidSpacePolygons_ThrowNamingTheSpace",
          "[io][solid_model]") {
  auto doc = minimal_doc();
  auto &poly = doc["spaces"][0]["polygon"];
  SECTION("self-intersecting outer ring") {
    poly["outer"] = {{0, 0}, {4, 3}, {4, 0}, {0, 3}};
  }
  SECTION("collinear outer ring") { poly["outer"] = {{0, 0}, {1, 0}, {2, 0}}; }
  SECTION("hole outside the outer ring") {
    poly["holes"] = {{{5, 5}, {6, 5}, {6, 6}, {5, 6}}};
  }
  SECTION("hole crossing the outer ring") {
    poly["holes"] = {{{3, 1}, {5, 1}, {5, 2}, {3, 2}}};
  }
  SECTION("overlapping holes") {
    poly["holes"] = {{{1, 1}, {2, 1}, {2, 2}, {1, 2}},
                     {{1.5, 1.5}, {2.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}}};
  }
  CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                    ContainsSubstring("spaces[0]"));
}

TEST_CASE("SolidModelFromJson_ValidHoles_AreKept", "[io][solid_model]") {
  auto doc = minimal_doc();
  doc["spaces"][0]["polygon"]["holes"] = {{{1, 1}, {2, 1}, {2, 2}, {1, 2}},
                                          {{3, 1}, {3.5, 1}, {3.5, 2}}};
  const SolidModel m = solid_model_from_json(doc.dump());
  CHECK(m.spaces[0].polygon.holes.size() == 2);
}

TEST_CASE("SolidModelFromJson_RepeatedClosingPoint_IsDropped",
          "[io][solid_model]") {
  // GeoJSON-style closed rings are accepted and stored open, so the writer
  // never emits the closing point and a second round trip is a fixed point.
  auto doc = minimal_doc();
  doc["spaces"][0]["polygon"]["outer"] = {
      {0, 0}, {4, 0}, {4, 3}, {0, 3}, {0, 0}};
  const SolidModel m = solid_model_from_json(doc.dump());
  CHECK(m.spaces[0].polygon.outer.size() == 4);
  const std::string once = solid_model_to_json(m);
  CHECK(solid_model_to_json(solid_model_from_json(once)) == once);
}

TEST_CASE("SolidModelJson_ProvidedButEmptyStoreys_SurviveRoundTrip",
          "[io][solid_model]") {
  const SolidModel m =
      solid_model_from_json(R"({"model_version": 1, "storeys": []})");
  CHECK(m.has_storeys());
  const auto j = nlohmann::json::parse(solid_model_to_json(m));
  REQUIRE(j.contains("storeys"));
  CHECK(j.at("storeys").empty());
  CHECK(solid_model_from_json(j.dump()).has_storeys());
}

TEST_CASE("SolidModelToJson_MetadataThatIsNotJson_Throws",
          "[io][solid_model]") {
  // metadata is JSON text by contract; writing anything else would not read
  // back as the same record.
  SolidModel m = solid_model_from_json(minimal_doc().dump());
  m.walls[0].metadata = "not json";
  CHECK_THROWS_WITH(solid_model_to_json(m),
                    ContainsSubstring("w1") && ContainsSubstring("metadata"));
}

TEST_CASE("SolidModelFromJson_StringMetadata_IsAFixedPoint",
          "[io][solid_model]") {
  auto doc = minimal_doc();
  doc["walls"][0]["metadata"] = "123";
  const std::string once =
      solid_model_to_json(solid_model_from_json(doc.dump()));
  CHECK(nlohmann::json::parse(once)["walls"][0]["metadata"] == "123");
  CHECK(solid_model_to_json(solid_model_from_json(once)) == once);
}

TEST_CASE("SolidModelFromJson_OutOfRangeNumbers_Throw", "[io][solid_model]") {
  auto doc = minimal_doc();
  SECTION("storey beyond int") { doc["walls"][0]["storey"] = 4294967296LL; }
  SECTION("negative idx beyond int") {
    doc["storeys"][0]["idx"] = -4294967296LL;
  }
  SECTION("face_points beyond int") {
    doc["walls"][0]["face_points"] = {1, 4294967296LL};
  }
  SECTION("level_transform overflow") {
    std::string text = doc.dump();
    const std::string key = R"("z_floor":0)";
    text.replace(
        text.find(key), key.size(),
        R"("level_transform":[1e400,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],"z_floor":0)");
    // The JSON parser itself refuses a number that overflows a double; it
    // must still surface as std::invalid_argument, not a json exception.
    CHECK_THROWS_AS(solid_model_from_json(text), std::invalid_argument);
    return;
  }
  SECTION("thickness_ci overflow") {
    std::string text = doc.dump();
    const std::string key = R"("thickness":0.2)";
    text.replace(text.find(key), key.size(),
                 R"("thickness":0.2,"thickness_ci":[0.1,1e400])");
    // The JSON parser itself refuses a number that overflows a double; it
    // must still surface as std::invalid_argument, not a json exception.
    CHECK_THROWS_AS(solid_model_from_json(text), std::invalid_argument);
    return;
  }
  CHECK_THROWS_AS(solid_model_from_json(doc.dump()), std::invalid_argument);
}

// ── Files and the project import path (reusex-z1v.2.6, rux edit plan) ───────

TEST_CASE("SolidModelFile_RoundTrip_IsAFixedPoint", "[io][solid_model]") {
  const SolidModel m =
      reusex::io::solid_model_from_file(fixture("two_rooms.plan.json"));
  CHECK(m.walls.size() == 7);
  reusex::test_support::TempPath tmp("test_solid_model_file", ".json");
  reusex::io::solid_model_to_file(m, tmp.path);
  const SolidModel back = reusex::io::solid_model_from_file(tmp.path);
  CHECK(solid_model_to_json(back) == solid_model_to_json(m));
}

TEST_CASE("SolidModelFile_Unreadable_IsAnIoErrorNotAnInvalidDocument",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_solid_model_missing", ".json");
  // A missing file is not "an invalid document": callers map the two to
  // different exit codes.
  CHECK_THROWS_AS(reusex::io::solid_model_from_file(tmp.path),
                  std::runtime_error);
  try {
    (void)reusex::io::solid_model_from_file(tmp.path);
  } catch (const std::invalid_argument &) {
    FAIL("a missing file must not throw std::invalid_argument");
  } catch (const std::runtime_error &e) {
    CHECK_THAT(e.what(), ContainsSubstring(tmp.path.string()));
  }
  {
    std::ofstream out(tmp.path);
    out << R"({"model_version": 1, "walls": [{"guid": "w1", "bogus": 1}]})";
  }
  CHECK_THROWS_AS(reusex::io::solid_model_from_file(tmp.path),
                  std::invalid_argument);
}

TEST_CASE("MarkImported_SetsSourceAndLockOnEveryElement", "[io][solid_model]") {
  SolidModel m = solid_model_from_json(minimal_doc().dump());
  reusex::io::mark_imported(m);
  for (const auto &s : m.storeys)
    CHECK((s.source == "imported" && s.locked));
  for (const auto &w : m.walls)
    CHECK((w.source == "imported" && w.locked));
  for (const auto &s : m.spaces)
    CHECK((s.source == "imported" && s.locked));
  for (const auto &o : m.openings)
    CHECK((o.source == "imported" && o.locked));
}

// ── store_solid_model: the table-backed import (reusex-3iy8.8) ────────────

namespace {

/// The model as JSON with every element array sorted: the tables list
/// records in their own order (walls by storey, openings by host wall, ...),
/// and name / description have no table, so "lossless" is per record.
nlohmann::json canonical(SolidModel m) {
  m.name.clear();
  m.description.clear();
  auto doc = nlohmann::json::parse(solid_model_to_json(m));
  for (auto &item : doc.items())
    if (auto &value = item.value(); value.is_array())
      std::sort(value.begin(), value.end(),
                [](const nlohmann::json &a, const nlohmann::json &b) {
                  return a.dump() < b.dump();
                });
  return doc;
}

reusex::core::StoreyRecord detected_storey(const std::string &guid, int idx,
                                           bool locked = false) {
  reusex::core::StoreyRecord s;
  s.guid = guid;
  s.idx = idx;
  s.name = "detected " + guid;
  s.z_floor = 3.0 * idx;
  s.z_ceiling = s.z_floor + 2.5;
  s.locked = locked;
  return s;
}

reusex::core::WallRecord detected_wall(const std::string &guid, int storey,
                                       bool locked = false) {
  reusex::core::WallRecord w;
  w.guid = guid;
  w.storey = storey;
  w.x1 = 5.0;
  w.height = 2.5;
  w.thickness = 0.15;
  w.locked = locked;
  return w;
}

std::vector<std::string> wall_guids(const reusex::ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &w : db.walls())
    out.push_back(w.guid);
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

TEST_CASE("StoreSolidModel_Fixture_RoundTripsThroughTheTables",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_store_solid_roundtrip");
  SolidModel m =
      reusex::io::solid_model_from_file(fixture("two_rooms.plan.json"));
  reusex::io::mark_imported(m);
  REQUIRE(!m.wall_joins.empty());
  REQUIRE(!m.space_boundaries.empty());
  {
    reusex::ProjectDB db(tmp.path);
    const auto r = reusex::io::store_solid_model(db, m);
    CHECK(r.storeys == m.storeys.size());
    CHECK(r.walls == m.walls.size());
    CHECK(r.spaces == m.spaces.size());
    CHECK(r.openings == m.openings.size());
    CHECK(r.not_stored.empty());
    CHECK(r.replaced_imported == 0);
    CHECK(r.taken_over == 0);
    CHECK(r.skipped_locked == 0);
  }
  // JSON -> tables -> JSON, through a fresh connection.
  const reusex::ProjectDB db(tmp.path, /*readOnly*/ true);
  const SolidModel back = reusex::io::load_solid_model(db);
  CHECK(canonical(back) == canonical(m));
}

TEST_CASE("StoreSolidModel_ReImport_ReplacesImportedRowsOnly",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_store_solid_reimport");
  reusex::ProjectDB db(tmp.path);
  // A detected storey on another idx, a detected wall with an opening, and
  // a detected space: none of them is the import's business.
  db.save_storeys({detected_storey("det-s1", 1)});
  db.save_walls({detected_wall("det-w1", 1)}, {});
  reusex::core::OpeningRecord det_door;
  det_door.guid = "det-o1";
  det_door.wall = "det-w1";
  det_door.kind = "door";
  det_door.s_start = 1.0;
  det_door.s_end = 1.9;
  det_door.z_head = 2.0;
  db.save_openings({det_door});
  reusex::core::SpaceRecord det_space;
  det_space.guid = "det-sp1";
  det_space.storey = 1;
  det_space.polygon.outer = {{0, 0}, {5, 0}, {5, 4}};
  db.save_spaces({det_space}, {});

  SolidModel a = solid_model_from_json(minimal_doc().dump());
  reusex::io::mark_imported(a);
  const auto r1 = reusex::io::store_solid_model(db, a);
  CHECK(r1.stored() == 1 + 2 + 1 + 1);
  CHECK(wall_guids(db) == std::vector<std::string>{"det-w1", "w1", "w2"});
  // The storey had no guid in the document: one was minted, the same one
  // on every import of it.
  const auto storeys1 = db.storeys();
  REQUIRE(storeys1.size() == 2);
  CHECK_FALSE(storeys1[0].guid.empty());
  CHECK(storeys1[1].guid == "det-s1");

  SECTION("the same document again: same rows, nothing piles up") {
    const auto r2 = reusex::io::store_solid_model(db, a);
    CHECK(r2.replaced_imported == 1 + 2 + 1 + 1);
    CHECK(r2.stored() == r1.stored());
    CHECK(db.storeys().size() == 2);
    CHECK(db.storeys()[0].guid == storeys1[0].guid);
    CHECK(wall_guids(db) == std::vector<std::string>{"det-w1", "w1", "w2"});
    CHECK(db.openings().size() == 2);
    CHECK(db.spaces().size() == 2);
  }

  SECTION("a walls-only model replaces the imported walls, nothing else") {
    SolidModel b;
    b.walls.push_back(detected_wall("w9", 0));
    reusex::io::mark_imported(b);
    const auto r2 = reusex::io::store_solid_model(db, b);
    CHECK(r2.walls == 1);
    CHECK(r2.replaced_imported == 2);
    CHECK(wall_guids(db) == std::vector<std::string>{"det-w1", "w9"});
    CHECK(db.storeys().size() == 2); // the imported storey stays
    CHECK(db.spaces().size() == 2);
    // The imported door's host is gone; the table keeps it, the model
    // leaves it out, and the detected door is untouched.
    CHECK(db.openings().size() == 2);
    const auto model = reusex::io::load_solid_model(db);
    REQUIRE(model.openings.size() == 1);
    CHECK(model.openings[0].guid == "det-o1");
  }

  SECTION("a provided-but-empty kind clears the imported rows of it") {
    SolidModel none;
    none.provided.spaces = true;
    const auto r2 = reusex::io::store_solid_model(db, none);
    CHECK(r2.stored() == 0);
    CHECK(r2.replaced_imported == 1);
    REQUIRE(db.spaces().size() == 1);
    CHECK(db.spaces()[0].guid == "det-sp1");
    CHECK(wall_guids(db) == std::vector<std::string>{"det-w1", "w1", "w2"});
  }

  SECTION("a detection re-run keeps the imported (locked) rows") {
    db.save_walls({detected_wall("det-w2", 1)}, {});
    CHECK(wall_guids(db) == std::vector<std::string>{"det-w2", "w1", "w2"});
    db.save_storeys(
        {detected_storey("det-s0", 0), detected_storey("det-s1", 1)});
    // idx 0 is held by the locked imported storey: the detected one gives way.
    const auto st = db.storeys();
    REQUIRE(st.size() == 2);
    CHECK(st[0].guid == storeys1[0].guid);
    CHECK(st[0].source == "imported");
  }
}

TEST_CASE("StoreSolidModel_Collisions_UnlockedTakenOverLockedWins",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_store_solid_collide");
  reusex::ProjectDB db(tmp.path);
  db.save_storeys({detected_storey("det-s0", 0),
                   detected_storey("det-s1", 1, /*locked*/ true)});
  db.save_walls({detected_wall("w1", 0, /*locked*/ true),
                 detected_wall("w2", 0), detected_wall("w3", 0)},
                {});

  SolidModel m;
  m.storeys = {detected_storey("imp-s0", 0), detected_storey("imp-s1", 1)};
  m.walls = {detected_wall("w1", 0), detected_wall("w2", 0)};
  m.walls[1].thickness = 0.3;
  reusex::io::mark_imported(m);
  const auto r = reusex::io::store_solid_model(db, m);
  CHECK(r.storeys == 1); // imp-s0 took idx 0 over; idx 1 is locked
  CHECK(r.walls == 1);   // w2 taken over; w1 is locked
  CHECK(r.taken_over == 2);
  CHECK(r.skipped_locked == 2);

  const auto st = db.storeys();
  REQUIRE(st.size() == 2);
  CHECK(st[0].guid == "imp-s0");
  CHECK(st[0].source == "imported");
  CHECK(st[1].guid == "det-s1");
  CHECK(st[1].source == "detected");

  const auto w1 = db.wall_by_guid("w1");
  REQUIRE(w1);
  CHECK(w1->source == "detected");
  CHECK(w1->locked);
  const auto w2 = db.wall_by_guid("w2");
  REQUIRE(w2);
  CHECK(w2->source == "imported");
  CHECK(w2->thickness == 0.3);
  CHECK(db.wall_by_guid("w3")); // not named by the import: untouched
}

TEST_CASE("StoreSolidModel_RefusedRecord_LeavesEveryTableAsItWas",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_store_solid_atomic");
  reusex::ProjectDB db(tmp.path);
  SolidModel a = solid_model_from_json(minimal_doc().dump());
  reusex::io::mark_imported(a);
  reusex::io::store_solid_model(db, a);

  // A model built in C++ (no JSON validator) whose opening names a wall
  // nothing holds: refused after its storeys and walls were written, so
  // those writes roll back too.
  SolidModel bad;
  bad.storeys = {detected_storey("other-s0", 0)};
  bad.walls = {detected_wall("wx", 0)};
  reusex::core::OpeningRecord o;
  o.guid = "ox";
  o.wall = "no-such-wall";
  o.s_end = 1.0;
  o.z_head = 2.0;
  bad.openings = {o};
  reusex::io::mark_imported(bad);
  CHECK_THROWS_AS(reusex::io::store_solid_model(db, bad),
                  std::invalid_argument);
  CHECK(wall_guids(db) == std::vector<std::string>{"w1", "w2"});
  REQUIRE(db.storeys().size() == 1);
  CHECK(db.storeys()[0].guid != "other-s0");
  REQUIRE(db.openings().size() == 1);
  CHECK(db.openings()[0].guid == "o1");
}

TEST_CASE("StoreSolidModel_PlaneClassesAndRelations_AreReportedNotStored",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_store_solid_notstored");
  reusex::ProjectDB db(tmp.path);
  SolidModel m;
  m.provided.plane_classes = true;
  m.scene_relations.push_back({"a", "b", "adjacent_to", 1.0, ""});
  const auto r = reusex::io::store_solid_model(db, m);
  CHECK(r.stored() == 0);
  CHECK(r.not_stored ==
        std::vector<std::string>{"plane_classes", "scene_relations"});
}

TEST_CASE("SolidModelFixture_NewOffice_IsAValidDraftReference",
          "[io][solid_model]") {
  // Authored with docs/guides/solid-model-reference-authoring.md as
  // hand-written, commented YAML (the single source, reusex-yf4.3); a draft
  // until the maintainer has reviewed it (reusex-z1v.2.6).
  const SolidModel m =
      reusex::io::solid_model_from_file(fixture("newoffice.plan.yaml"));
  CHECK(m.storeys.size() == 1);
  CHECK(m.walls.size() >= 10);
  CHECK(m.spaces.size() >= 5);
  CHECK(m.openings.size() >= 10);
  CHECK_THAT(m.description, ContainsSubstring("DRAFT"));
  CHECK_THAT(m.storeys[0].metadata,
             ContainsSubstring("draft - needs maintainer review"));
}

TEST_CASE("SolidModelFixture_House2_IsAManhattanDraftOnThePoseReference",
          "[io][solid_model]") {
  // Re-fitted on the house2 pose reference (DRAFT v3, reusex-ghy): one
  // detected level frame for both storeys, whose axes are the Manhattan grid
  // the reference was solved on, so every wall runs along x or y. The
  // re-fit kept every element of the old plan under its guid (only geometry
  // moved), dropped one opening the corrected cloud does not show and added
  // three it does.
  const SolidModel m =
      reusex::io::solid_model_from_file(fixture("house2.plan.yaml"));
  REQUIRE(m.storeys.size() == 2);
  CHECK(m.storeys[0].level_transform == m.storeys[1].level_transform);
  CHECK(m.walls.size() == 41);
  CHECK(m.spaces.size() == 17);
  CHECK(m.openings.size() == 9);
  for (const auto &w : m.walls) {
    INFO(w.guid);
    CHECK((w.x0 == w.x1 || w.y0 == w.y1));
  }
  std::vector<std::string> openings;
  for (const auto &o : m.openings)
    openings.push_back(o.guid);
  std::sort(openings.begin(), openings.end());
  CHECK(openings ==
        std::vector<std::string>{"o0-hall-bath", "o0-liv-cons", "o0-ne-door",
                                 "o0-util-door", "o0-w-win", "o1-corr-e",
                                 "o1-corr-e-2", "o1-corr-w-1", "o1-gable-win"});
  CHECK_THAT(m.description, ContainsSubstring("DRAFT"));
  CHECK_THAT(m.description, ContainsSubstring("pose reference"));
  CHECK_THAT(m.storeys[0].metadata,
             ContainsSubstring("draft - needs maintainer review"));
}

TEST_CASE("SolidModelFixture_NewOffice_FollowsTheMaintainersRoomLayout",
          "[io][solid_model]") {
  // The maintainer's description of NewOffice (2026-10-15, reusex-z1v.2.6):
  // one open office, two meeting rooms at its end, a closed office, a
  // bathroom with 3 stalls, a printer and server room, a kitchen / assembly
  // space and a meeting room / storage. Updated 2026-10-16 (reusex-mdu): the
  // maintainer decided the 3 WC stalls are fully enclosed rooms and should be
  // detected (and referenced) as separate spaces, so the single "Bathroom"
  // space is now split into "WC 1"/"WC 2"/"WC 3" plus a shared
  // "Bathroom lobby" (washbasin/entrance) space. Same session, a second
  // maintainer decision: the printer room (front, entered from the
  // corridor) and the server room (behind it, entered only through the
  // printer room) are separate spaces too, split by the new
  // p-printer-server wall placed from the cloud (see its comment in
  // newoffice.plan.yaml), so "Printer/server room" is now "Printer room"
  // plus "Server room".
  const SolidModel m =
      reusex::io::solid_model_from_file(fixture("newoffice.plan.yaml"));
  std::vector<std::string> names;
  for (const auto &s : m.spaces)
    names.push_back(s.name);
  std::sort(names.begin(), names.end());
  CHECK(names == std::vector<std::string>{
                     "Bathroom lobby", "Closed office", "Kitchen/assembly",
                     "Meeting room 1", "Meeting room 2", "Meeting room/storage",
                     "Open office", "Printer room", "Server room", "WC 1",
                     "WC 2", "WC 3"});
  CHECK_THAT(m.description, ContainsSubstring("maintainer's description"));
  CHECK_THAT(m.storeys[0].metadata, ContainsSubstring("room_layout"));
}
