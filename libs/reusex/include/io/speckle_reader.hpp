// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/io/speckle_transport.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

/// @file
/// Read-only Speckle client: GraphQL *queries* and HTTP GETs only.
///
/// This is the receive half of the Speckle integration, kept apart from the
/// upload-only `SpeckleClient` (`io/speckle.hpp`) so that nothing reachable
/// from here can create, modify or upload anything on a server: there is no
/// POST to `/objects`, and the GraphQL helper refuses any document that is
/// not a `query`.

namespace reusex::io::speckle {

/// A model (formerly "branch") of a Speckle project.
struct ModelInfo {
  std::string id;
  std::string name;
};

/// A Speckle project and its models.
struct ProjectInfo {
  std::string id;
  std::string name;
  std::vector<ModelInfo> models;
};

/// One version (formerly "commit") of a model.
struct VersionInfo {
  std::string id;
  /// Hash of the version's root object — the argument to
  /// `SpeckleReader::fetch_object_tree`.
  std::string referenced_object;
  std::string source_application;
  std::string created_at; ///< ISO 8601, as the server reports it
  std::string message;
};

/// What `ObjectTree::resolve` could not resolve, and what it did.
struct ResolveReport {
  /// Referenced ids that are not in the tree (sorted, unique). Their
  /// reference stubs are left in place.
  std::vector<std::string> missing_ids;
  /// Ids reached again while being resolved (sorted, unique). The inner
  /// reference stub is left in place to break the cycle.
  std::vector<std::string> cyclic_ids;
  std::size_t references_resolved = 0; ///< reference stubs replaced
  std::size_t chunk_arrays_joined = 0; ///< DataChunk lists concatenated
  /// References served from the cache of multi-use targets (a target used
  /// once is resolved once and moved into place, never cached).
  std::size_t cache_hits = 0;
  /// JSON nodes produced (the quantity `ResolveOptions::max_nodes` bounds).
  std::size_t nodes = 0;
  /// Deepest JSON nesting reached (the quantity `ResolveOptions::max_depth`
  /// bounds).
  std::size_t max_depth_reached = 0;
};

/// Bounds for `ObjectTree::resolve` (STANDARDS §5: no unbounded work on
/// input from a server).
struct ResolveOptions {
  /// Maximum JSON nesting depth of the resolved value (objects and arrays,
  /// counted through references).
  std::size_t max_depth = 512;
  /// Maximum number of JSON nodes the resolved value may hold. Guards
  /// against a "diamond" tree (each level referencing the next twice)
  /// whose inlined form grows exponentially.
  std::size_t max_nodes = 50'000'000;
};

/// The flat set of objects a version's root object closes over, as the
/// `GET /objects/{project}/{object}` endpoint returns it (id -> object).
class ObjectTree {
    public:
  ObjectTree() = default;
  ObjectTree(std::string root_id,
             std::unordered_map<std::string, nlohmann::json> objects);

  /// Parse a `GET /objects/{project}/{object}` response body. Accepts both
  /// download formats the server produces: a JSON array of objects
  /// (`Accept: application/json`) and newline-delimited `id<TAB>json`
  /// lines (`text/plain`; a line without a tab is read as bare JSON with
  /// its own `id`). Blank lines are skipped.
  /// @throws std::runtime_error on malformed JSON, an object without an id,
  ///         or when `root_id` is not among the parsed objects.
  static ObjectTree parse(std::string_view body, std::string root_id);

  [[nodiscard]] const std::string &root_id() const { return root_id_; }
  [[nodiscard]] std::size_t size() const { return objects_.size(); }
  [[nodiscard]] bool contains(const std::string &id) const;
  /// The stored (unresolved) object, or nullptr.
  [[nodiscard]] const nlohmann::json *find(const std::string &id) const;
  /// The stored (unresolved) root object.
  /// @throws std::runtime_error on an empty tree.
  [[nodiscard]] const nlohmann::json &root() const;
  [[nodiscard]] const std::unordered_map<std::string, nlohmann::json> &
  objects() const {
    return objects_;
  }

  /// Number of stored objects per `speckle_type` (objects without one count
  /// under the empty string).
  [[nodiscard]] std::map<std::string, std::size_t> type_histogram() const;

  /// Return a copy of `value` with every reference stub
  /// (`{"speckle_type": "reference", "referencedId": ...}`) replaced by its
  /// resolved target, recursively, and with these wire-format details
  /// undone:
  ///  - detached keys lose their `@` prefix, including the chunked-property
  ///    form `@(N)name` (`@elements` -> `elements`, `@(31250)vertices` ->
  ///    `vertices`); when both spellings exist the `@` one keeps its key;
  ///  - an array whose elements all resolve to DataChunks
  ///    (`...DataChunk` with a `data` array) becomes the concatenation of
  ///    their `data`;
  ///  - `__closure` is dropped (it only describes the detached layout).
  /// A missing or cyclic reference is logged at warn with its id and left
  /// as a stub.
  /// @throws std::runtime_error when the result would nest deeper than
  ///         `options.max_depth` or hold more than `options.max_nodes`
  ///         nodes.
  [[nodiscard]] nlohmann::json
  resolve(const nlohmann::json &value, ResolveReport *report = nullptr,
          const ResolveOptions &options = {}) const;

  /// `resolve(root(), report, options)`.
  [[nodiscard]] nlohmann::json
  resolve_root(ResolveReport *report = nullptr,
               const ResolveOptions &options = {}) const;

    private:
  std::string root_id_;
  std::unordered_map<std::string, nlohmann::json> objects_;
};

/// Parse the `data` of a project query response (see
/// `SpeckleReader::query_project`). Exposed for offline tests.
/// @throws std::runtime_error when the project is absent.
ProjectInfo parse_project_info(const nlohmann::json &data);

/// Collect a project's models over the paginated `models(cursor:)` query.
/// `fetch_page(cursor)` returns one response `data` (cursor is null for the
/// first page). Stops at a null/absent cursor, an empty page, a reached
/// (non-null) `totalCount`, a null `models`, or after `max_pages` pages.
/// Exposed for offline tests; `SpeckleReader::query_project` uses it.
ProjectInfo collect_project_pages(
    const std::function<nlohmann::json(const nlohmann::json &cursor)>
        &fetch_page,
    int max_pages = 100);

/// Parse the `data` of a versions query response (see
/// `SpeckleReader::query_versions`). Exposed for offline tests.
/// @throws std::runtime_error when the project or model is absent.
std::vector<VersionInfo> parse_versions(const nlohmann::json &data);

/// True for a usable Speckle id (project, model, version or object):
/// 1..128 characters of [A-Za-z0-9]. Ids are checked before they are put
/// into a URL or a query.
bool is_valid_speckle_id(std::string_view id);

/// True when `document` holds only GraphQL `query` operations (and
/// fragments): no `mutation` or `subscription` anywhere (outside strings and
/// comments) and no anonymous `{ ... }` shorthand. This is what keeps
/// `SpeckleReader` read-only.
bool is_query_document(std::string_view document);

namespace detail {
/// The message for a non-2xx response (401, 403, 404 are spelled out).
/// Exposed for offline tests.
std::string http_error_message(long status, const std::string &what,
                               const std::string &project,
                               const std::string &body);

/// The `data` of a parsed GraphQL response, or a `std::runtime_error` for
/// its first error (FORBIDDEN / UNAUTHENTICATED / UNAUTHORIZED read as
/// "token invalid or no access to project"). Exposed for offline tests.
nlohmann::json graphql_data_or_throw(const nlohmann::json &response,
                                     const std::string &project);

/// Whether `server_url` may be used: `https://`, or `http://` to localhost
/// / 127.0.0.1 / [::1] only.
bool is_allowed_server_url(const std::string &server_url);
} // namespace detail

/// Read-only Speckle client. Every id argument is checked with
/// `is_valid_speckle_id` (`std::invalid_argument` otherwise). Every method is a
/// GraphQL query or an HTTP GET.
class SpeckleReader {
    public:
  /// @param server_url  e.g. "https://app.speckle.systems"
  /// @param token       Personal access token (falls back to SPECKLE_TOKEN);
  ///                    never logged.
  /// @throws std::runtime_error for an empty URL, or one that is neither
  ///         `https://` nor `http://` to localhost.
  explicit SpeckleReader(std::string server_url, std::string token = "");

  [[nodiscard]] const std::string &server_url() const { return server_url_; }

  /// Abort any single download larger than this (default 4 GiB).
  void set_max_download_bytes(std::size_t bytes) {
    max_download_bytes_ = bytes;
  }
  [[nodiscard]] std::size_t max_download_bytes() const {
    return max_download_bytes_;
  }

  /// Abort any request in flight once `*cancel` is set (checked by curl's
  /// progress callback, about once a second even while stalled); the call
  /// then throws std::runtime_error ("... cancelled"). nullptr = never.
  /// The flag must outlive the reader's calls.
  void set_cancel_token(const std::atomic_bool *cancel) { cancel_ = cancel; }

  /// Overall time limit for one request, the object download included
  /// (0 = none: the download is bounded by a low-speed abort only, queries
  /// by their own 60 s).
  void set_timeout_seconds(long seconds) { timeout_s_ = seconds; }

  /// Vet every connection by its resolved peer address (see ConnectFilter,
  /// `io/speckle_transport.hpp`); also disables proxies. Empty = none.
  void set_connect_filter(ConnectFilter filter) {
    connect_filter_ = std::move(filter);
  }
  /// Configure every curl handle after the reader's own options (see
  /// CurlSetup, `io/speckle_transport.hpp`). Empty = none.
  void set_curl_setup(CurlSetup setup) { curl_setup_ = std::move(setup); }

  /// Project name and all of its models.
  [[nodiscard]] ProjectInfo query_project(const std::string &project) const;

  /// The newest `limit` versions of a model, newest first.
  [[nodiscard]] std::vector<VersionInfo>
  query_versions(const std::string &project, const std::string &model,
                 int limit = 10) const;

  /// One version of a model, by id (any age; a GraphQL query).
  /// @throws std::runtime_error when the version is not in the model.
  [[nodiscard]] VersionInfo query_version(const std::string &project,
                                          const std::string &model,
                                          const std::string &version) const;

  /// The newest version of a model.
  /// @throws std::runtime_error when the model has no versions.
  [[nodiscard]] VersionInfo latest_version(const std::string &project,
                                           const std::string &model) const;

  /// Download a root object and its whole closure in one GET
  /// (`/objects/{project}/{object_id}`), streamed into memory.
  [[nodiscard]] ObjectTree
  fetch_object_tree(const std::string &project,
                    const std::string &object_id) const;

  /// Download just one object (`/objects/{project}/{object_id}/single`),
  /// unresolved.
  [[nodiscard]] nlohmann::json fetch_object(const std::string &project,
                                            const std::string &object_id) const;

    private:
  [[nodiscard]] nlohmann::json graphql_query(const std::string &query,
                                             const nlohmann::json &variables,
                                             const std::string &project) const;

  std::string server_url_;
  std::string token_;
  std::size_t max_download_bytes_ = std::size_t{4} << 30; // 4 GiB
  const std::atomic_bool *cancel_ = nullptr;
  long timeout_s_ = 0;
  ConnectFilter connect_filter_;
  CurlSetup curl_setup_;
};

} // namespace reusex::io::speckle
