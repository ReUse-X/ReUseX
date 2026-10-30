// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/speckle_reader.hpp"

#include "core/logging.hpp"

#include <curl/curl.h>
#include <fmt/format.h>

#include <algorithm>
#include <cstdlib>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace reusex::io::speckle {

using nlohmann::json;

// ============================================================
// HTTP (read-only: GET, and POST of a GraphQL *query* document)
// ============================================================
namespace {

constexpr std::size_t progress_step_bytes = 8u << 20; // 8 MiB

struct Sink {
  std::string body;
  std::string label;
  std::size_t next_progress = progress_step_bytes;
};

size_t sink_write(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *sink = static_cast<Sink *>(userdata);
  sink->body.append(ptr, size * nmemb);
  if (sink->body.size() >= sink->next_progress) {
    core::debug("Speckle: {} — {:.1f} MiB received", sink->label,
                static_cast<double>(sink->body.size()) / (1 << 20));
    sink->next_progress += progress_step_bytes;
  }
  return size * nmemb;
}

/// Turn a non-2xx status into an exception with a message a user can act on.
[[noreturn]] void throw_http_error(long code, const std::string &what,
                                   const std::string &project,
                                   const std::string &body) {
  std::string reason;
  if (code == 401)
    reason = "not authenticated — the token is missing or invalid";
  else if (code == 403)
    reason = fmt::format("token invalid or no access to project '{}'", project);
  else if (code == 404)
    reason = "not found — check the project and object ids";
  else
    reason = "unexpected response";
  constexpr std::size_t max_body = 300;
  throw std::runtime_error(fmt::format(
      "Speckle {} failed (HTTP {}): {}. Server said: {}", what, code, reason,
      body.size() > max_body ? body.substr(0, max_body) + "…" : body));
}

struct HttpRequest {
  std::string url;
  std::string token;
  std::string what;    ///< for messages, e.g. "object download"
  std::string project; ///< for the 403 message
  std::string accept = "application/json";
  const std::string *post_json = nullptr; ///< GraphQL query body, or GET
  bool long_transfer = false; ///< no total timeout, low-speed abort instead
};

std::string http_perform(const HttpRequest &req) {
  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("curl_easy_init failed");

  Sink sink;
  sink.label = req.what;
  struct curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, ("Accept: " + req.accept).c_str());
  if (!req.token.empty())
    headers = curl_slist_append(headers,
                                ("Authorization: Bearer " + req.token).c_str());
  if (req.post_json)
    headers = curl_slist_append(headers, "Content-Type: application/json");

  // Required for signal-based timeout to work in multithreaded programs.
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  if (req.long_transfer) {
    // A full object tree can be hundreds of MB: bound stalls, not size.
    curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1024L);
    curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 60L);
  } else {
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
  }
  curl_easy_setopt(curl, CURLOPT_URL, req.url.c_str());
  curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, ""); // gzip/deflate/br
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, sink_write);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sink);
  if (req.post_json) {
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, req.post_json->c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                     static_cast<long>(req.post_json->size()));
  } else {
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
  }

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK)
    throw std::runtime_error(fmt::format("Speckle {}: curl error: {} ({})",
                                         req.what, curl_easy_strerror(res),
                                         req.url));
  if (http_code < 200 || http_code >= 300)
    throw_http_error(http_code, req.what, req.project, sink.body);
  return std::move(sink.body);
}

std::string http_get(const std::string &url, const std::string &token,
                     const std::string &what, const std::string &project,
                     bool long_transfer) {
  HttpRequest req;
  req.url = url;
  req.token = token;
  req.what = what;
  req.project = project;
  req.long_transfer = long_transfer;
  return http_perform(req);
}

std::string_view trim_left(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' ||
                        s.front() == '\n' || s.front() == '\r'))
    s.remove_prefix(1);
  return s;
}

std::string string_or_empty(const json &obj, const char *key) {
  auto it = obj.find(key);
  if (it == obj.end() || !it->is_string())
    return {};
  return it->get<std::string>();
}

// ---- Object-tree helpers ----

bool is_reference(const json &v) {
  if (!v.is_object())
    return false;
  auto ref = v.find("referencedId");
  if (ref == v.end() || !ref->is_string())
    return false;
  auto type = v.find("speckle_type");
  return type == v.end() || (type->is_string() && *type == "reference");
}

bool is_data_chunk(const json &v) {
  if (!v.is_object())
    return false;
  auto type = v.find("speckle_type");
  auto data = v.find("data");
  if (type == v.end() || !type->is_string() || data == v.end() ||
      !data->is_array())
    return false;
  const auto &t = type->get_ref<const std::string &>();
  static constexpr std::string_view suffix = "DataChunk";
  return t.size() >= suffix.size() &&
         t.compare(t.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/// `@elements` -> `elements`, `@(31250)vertices` -> `vertices`.
std::string strip_detach_prefix(const std::string &key) {
  if (key.size() < 2 || key.front() != '@')
    return key;
  std::string rest = key.substr(1);
  if (rest.front() == '(') {
    auto close = rest.find(')');
    if (close != std::string::npos && close + 1 < rest.size())
      return rest.substr(close + 1);
    return key;
  }
  return rest;
}

class Resolver {
    public:
  explicit Resolver(const std::unordered_map<std::string, json> &objects)
      : objects_(objects) {}

  json walk(const json &v) {
    if (v.is_object()) {
      if (is_reference(v))
        return deref(v["referencedId"].get<std::string>(), v);
      json out = json::object();
      for (const auto &[key, val] : v.items()) {
        if (key == "__closure")
          continue;
        std::string name = strip_detach_prefix(key);
        if (name != key && (v.contains(name) || out.contains(name)))
          name = key; // both spellings present: keep the detached one apart
        out[name] = walk(val);
      }
      return out;
    }
    if (v.is_array()) {
      json out = json::array();
      bool all_chunks = !v.empty();
      for (const auto &el : v) {
        out.push_back(walk(el));
        all_chunks = all_chunks && is_data_chunk(out.back());
      }
      if (!all_chunks)
        return out;
      json joined = json::array();
      for (auto &chunk : out)
        for (auto &d : chunk["data"])
          joined.push_back(std::move(d));
      ++report_.chunk_arrays_joined;
      return joined;
    }
    return v;
  }

  /// Resolve a top-level value. When it is a stored object itself (it
  /// carries the `id` of one), that id counts as in progress, so a
  /// reference back to it is reported as a cycle rather than expanded once.
  json walk_top(const json &v) {
    if (v.is_object() && !is_reference(v)) {
      auto id = v.find("id");
      if (id != v.end() && id->is_string() &&
          objects_.count(id->get<std::string>())) {
        const std::string key = id->get<std::string>();
        in_progress_.insert(key);
        json out = walk(v);
        in_progress_.erase(key);
        return out;
      }
    }
    return walk(v);
  }

  ResolveReport finish() {
    report_.missing_ids.assign(missing_.begin(), missing_.end());
    report_.cyclic_ids.assign(cyclic_.begin(), cyclic_.end());
    return std::move(report_);
  }

    private:
  json deref(const std::string &id, const json &stub) {
    if (auto hit = cache_.find(id); hit != cache_.end()) {
      ++report_.references_resolved;
      return hit->second;
    }
    if (in_progress_.count(id)) {
      cyclic_.insert(id);
      return stub;
    }
    auto it = objects_.find(id);
    if (it == objects_.end()) {
      missing_.insert(id);
      return stub;
    }
    in_progress_.insert(id);
    json resolved = walk(it->second);
    in_progress_.erase(id);
    ++report_.references_resolved;
    return cache_.emplace(id, std::move(resolved)).first->second;
  }

  const std::unordered_map<std::string, json> &objects_;
  std::unordered_map<std::string, json> cache_;
  std::unordered_set<std::string> in_progress_;
  std::set<std::string> missing_;
  std::set<std::string> cyclic_;
  ResolveReport report_;
};

std::string preview_ids(const std::vector<std::string> &ids) {
  constexpr std::size_t shown = 5;
  std::string s;
  for (std::size_t i = 0; i < ids.size() && i < shown; ++i)
    s += (i ? ", " : "") + ids[i];
  if (ids.size() > shown)
    s += fmt::format(", … ({} more)", ids.size() - shown);
  return s;
}

} // anonymous namespace

// ============================================================
// ObjectTree
// ============================================================

ObjectTree::ObjectTree(std::string root_id,
                       std::unordered_map<std::string, json> objects)
    : root_id_(std::move(root_id)), objects_(std::move(objects)) {}

ObjectTree ObjectTree::parse(std::string_view body, std::string root_id) {
  std::unordered_map<std::string, json> objects;
  auto add = [&](std::string id, json obj, std::size_t where) {
    if (!obj.is_object())
      throw std::runtime_error(fmt::format(
          "Speckle object download: entry {} is not a JSON object", where));
    if (id.empty())
      id = string_or_empty(obj, "id");
    if (id.empty())
      throw std::runtime_error(
          fmt::format("Speckle object download: entry {} has no id", where));
    objects.insert_or_assign(std::move(id), std::move(obj));
  };

  std::string_view rest = trim_left(body);
  if (!rest.empty() && rest.front() == '[') {
    json arr;
    try {
      arr = json::parse(rest.begin(), rest.end());
    } catch (const json::parse_error &e) {
      throw std::runtime_error(fmt::format(
          "Speckle object download: invalid JSON array: {}", e.what()));
    }
    std::size_t i = 0;
    for (auto &obj : arr)
      add({}, std::move(obj), i++);
  } else {
    std::size_t line_no = 0;
    while (!body.empty()) {
      auto nl = body.find('\n');
      std::string_view line = body.substr(0, nl);
      body.remove_prefix(nl == std::string_view::npos ? body.size() : nl + 1);
      ++line_no;
      if (!line.empty() && line.back() == '\r')
        line.remove_suffix(1);
      if (trim_left(line).empty())
        continue;
      std::string id;
      std::string_view text = line;
      if (auto tab = line.find('\t'); tab != std::string_view::npos) {
        id = std::string(line.substr(0, tab));
        text = line.substr(tab + 1);
      }
      json obj;
      try {
        obj = json::parse(text);
      } catch (const json::parse_error &e) {
        throw std::runtime_error(
            fmt::format("Speckle object download: invalid JSON on line {}: {}",
                        line_no, e.what()));
      }
      add(std::move(id), std::move(obj), line_no);
    }
  }

  if (!objects.count(root_id))
    throw std::runtime_error(fmt::format(
        "Speckle object download: root object '{}' is not among the {} "
        "objects received",
        root_id, objects.size()));
  return ObjectTree(std::move(root_id), std::move(objects));
}

bool ObjectTree::contains(const std::string &id) const {
  return objects_.count(id) != 0;
}

const json *ObjectTree::find(const std::string &id) const {
  auto it = objects_.find(id);
  return it == objects_.end() ? nullptr : &it->second;
}

const json &ObjectTree::root() const {
  const json *r = find(root_id_);
  if (!r)
    throw std::runtime_error(
        fmt::format("Speckle object tree has no root object '{}'", root_id_));
  return *r;
}

std::map<std::string, std::size_t> ObjectTree::type_histogram() const {
  std::map<std::string, std::size_t> hist;
  for (const auto &[id, obj] : objects_)
    ++hist[string_or_empty(obj, "speckle_type")];
  return hist;
}

json ObjectTree::resolve(const json &value, ResolveReport *report) const {
  Resolver resolver(objects_);
  json out = resolver.walk_top(value);
  ResolveReport r = resolver.finish();
  if (!r.missing_ids.empty())
    core::warn("Speckle: {} referenced object(s) missing from the downloaded "
               "tree (root {}), left as reference stubs: {}",
               r.missing_ids.size(), root_id_, preview_ids(r.missing_ids));
  if (!r.cyclic_ids.empty())
    core::warn("Speckle: {} cyclic reference(s) in the object tree (root {}), "
               "left as reference stubs: {}",
               r.cyclic_ids.size(), root_id_, preview_ids(r.cyclic_ids));
  if (report)
    *report = std::move(r);
  return out;
}

json ObjectTree::resolve_root(ResolveReport *report) const {
  return resolve(root(), report);
}

// ============================================================
// GraphQL response parsing
// ============================================================

ProjectInfo parse_project_info(const json &data) {
  const json *project = data.contains("project") ? &data["project"] : nullptr;
  if (!project || !project->is_object())
    throw std::runtime_error("Speckle: project not found in response");
  ProjectInfo info;
  info.id = string_or_empty(*project, "id");
  info.name = string_or_empty(*project, "name");
  if (auto models = project->find("models");
      models != project->end() && models->is_object()) {
    for (const auto &m : models->value("items", json::array()))
      info.models.push_back(
          {string_or_empty(m, "id"), string_or_empty(m, "name")});
  }
  return info;
}

std::vector<VersionInfo> parse_versions(const json &data) {
  const json *project = data.contains("project") ? &data["project"] : nullptr;
  if (!project || !project->is_object())
    throw std::runtime_error("Speckle: project not found in response");
  auto model = project->find("model");
  if (model == project->end() || !model->is_object())
    throw std::runtime_error("Speckle: model not found in response");
  std::vector<VersionInfo> out;
  auto versions = model->find("versions");
  if (versions == model->end() || !versions->is_object())
    return out;
  for (const auto &v : versions->value("items", json::array())) {
    VersionInfo vi;
    vi.id = string_or_empty(v, "id");
    vi.referenced_object = string_or_empty(v, "referencedObject");
    vi.source_application = string_or_empty(v, "sourceApplication");
    vi.created_at = string_or_empty(v, "createdAt");
    vi.message = string_or_empty(v, "message");
    out.push_back(std::move(vi));
  }
  return out;
}

// ============================================================
// SpeckleReader
// ============================================================

SpeckleReader::SpeckleReader(std::string server_url, std::string token)
    : server_url_(std::move(server_url)), token_(std::move(token)) {
  while (!server_url_.empty() && server_url_.back() == '/')
    server_url_.pop_back();
  if (server_url_.empty())
    throw std::runtime_error("SpeckleReader: empty server URL");
  if (token_.empty()) {
    if (const char *env = std::getenv("SPECKLE_TOKEN"))
      token_ = env;
  }
  if (token_.empty())
    core::warn("SpeckleReader: no auth token (set SPECKLE_TOKEN or pass one); "
               "only public projects will be readable");
}

json SpeckleReader::graphql_query(const std::string &query,
                                  const json &variables,
                                  const std::string &project) const {
  // Read-only by construction: refuse anything but a query operation.
  std::string_view doc = trim_left(query);
  if (doc.rfind("query", 0) != 0)
    throw std::logic_error("SpeckleReader: only GraphQL queries are allowed");

  const std::string body =
      json{{"query", query}, {"variables", variables}}.dump();
  HttpRequest req;
  req.url = server_url_ + "/graphql";
  req.token = token_;
  req.what = "GraphQL query";
  req.project = project;
  req.post_json = &body;
  const std::string response = http_perform(req);

  json parsed;
  try {
    parsed = json::parse(response);
  } catch (const json::parse_error &e) {
    throw std::runtime_error(
        fmt::format("Speckle GraphQL: invalid JSON response: {}", e.what()));
  }
  if (auto errors = parsed.find("errors");
      errors != parsed.end() && errors->is_array() && !errors->empty()) {
    const json &first = (*errors)[0];
    std::string message = string_or_empty(first, "message");
    std::string code;
    if (auto ext = first.find("extensions");
        ext != first.end() && ext->is_object())
      code = string_or_empty(*ext, "code");
    if (code == "FORBIDDEN" || code == "UNAUTHENTICATED" ||
        code == "UNAUTHORIZED")
      throw std::runtime_error(fmt::format(
          "Speckle GraphQL: token invalid or no access to project '{}' ({}: "
          "{})",
          project, code, message));
    throw std::runtime_error(fmt::format("Speckle GraphQL error{}: {}",
                                         code.empty() ? "" : " " + code,
                                         message));
  }
  auto data = parsed.find("data");
  if (data == parsed.end() || !data->is_object())
    throw std::runtime_error("Speckle GraphQL: response has no data");
  return *data;
}

ProjectInfo SpeckleReader::query_project(const std::string &project) const {
  static const std::string query =
      "query ProjectModels($project: String!, $cursor: String) { "
      "project(id: $project) { id name "
      "models(limit: 100, cursor: $cursor) { cursor totalCount "
      "items { id name } } } }";
  ProjectInfo info;
  json cursor = nullptr;
  // Bounded pagination (STANDARDS §5): 100 pages of 100 models.
  for (int page = 0; page < 100; ++page) {
    json data = graphql_query(query, {{"project", project}, {"cursor", cursor}},
                              project);
    if (data["project"].is_null())
      throw std::runtime_error(fmt::format(
          "Speckle: project '{}' not found, or no access to it", project));
    ProjectInfo page_info = parse_project_info(data);
    if (page == 0) {
      info.id = page_info.id;
      info.name = page_info.name;
    }
    const bool empty_page = page_info.models.empty();
    for (auto &m : page_info.models)
      info.models.push_back(std::move(m));
    const json &models = data["project"]["models"];
    const std::size_t total = models.value("totalCount", std::size_t{0});
    cursor = models.contains("cursor") ? models["cursor"] : json(nullptr);
    if (empty_page || cursor.is_null() || info.models.size() >= total)
      break;
  }
  core::debug("Speckle: project '{}' ({}) has {} model(s)", info.name, project,
              info.models.size());
  return info;
}

std::vector<VersionInfo>
SpeckleReader::query_versions(const std::string &project,
                              const std::string &model, int limit) const {
  if (limit < 1)
    throw std::invalid_argument("SpeckleReader::query_versions: limit < 1");
  static const std::string query =
      "query ModelVersions($project: String!, $model: String!, $limit: Int!) "
      "{ project(id: $project) { id model(id: $model) { id name "
      "versions(limit: $limit) { items { id referencedObject "
      "sourceApplication createdAt message } } } } }";
  json data = graphql_query(
      query, {{"project", project}, {"model", model}, {"limit", limit}},
      project);
  if (data["project"].is_null())
    throw std::runtime_error(fmt::format(
        "Speckle: project '{}' not found, or no access to it", project));
  if (data["project"]["model"].is_null())
    throw std::runtime_error(fmt::format(
        "Speckle: model '{}' not found in project '{}'", model, project));
  return parse_versions(data);
}

VersionInfo SpeckleReader::latest_version(const std::string &project,
                                          const std::string &model) const {
  auto versions = query_versions(project, model, 1);
  if (versions.empty())
    throw std::runtime_error(fmt::format(
        "Speckle: model '{}' in project '{}' has no versions", model, project));
  return std::move(versions.front());
}

ObjectTree
SpeckleReader::fetch_object_tree(const std::string &project,
                                 const std::string &object_id) const {
  const std::string url =
      fmt::format("{}/objects/{}/{}", server_url_, project, object_id);
  core::debug("Speckle: downloading object tree {} from project {}", object_id,
              project);
  stopwatch sw;
  std::string body = http_get(url, token_, "object download", project, true);
  core::debug("Speckle: received {:.1f} MiB in {:.1f} s, parsing",
              static_cast<double>(body.size()) / (1 << 20), sw.elapsed());
  ObjectTree tree = ObjectTree::parse(body, object_id);
  core::info("Speckle: loaded {} objects for root {} ({:.1f} MiB)", tree.size(),
             object_id, static_cast<double>(body.size()) / (1 << 20));
  return tree;
}

json SpeckleReader::fetch_object(const std::string &project,
                                 const std::string &object_id) const {
  const std::string url =
      fmt::format("{}/objects/{}/{}/single", server_url_, project, object_id);
  std::string body = http_get(url, token_, "object fetch", project, false);
  try {
    return json::parse(body);
  } catch (const json::parse_error &e) {
    throw std::runtime_error(fmt::format(
        "Speckle object fetch: invalid JSON for {}: {}", object_id, e.what()));
  }
}

} // namespace reusex::io::speckle
