// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/speckle_reader.hpp"

#include "core/logging.hpp"

#include <arpa/inet.h>
#include <curl/curl.h>
#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace reusex::io::speckle {

using nlohmann::json;

// ============================================================
// Small helpers
// ============================================================
namespace {

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

void require_id(std::string_view id, const char *what) {
  if (!is_valid_speckle_id(id))
    throw std::invalid_argument(fmt::format(
        "Speckle: invalid {} id '{}' (expected 1-128 characters [A-Za-z0-9])",
        what, id.substr(0, 64)));
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

} // anonymous namespace

bool is_valid_speckle_id(std::string_view id) {
  if (id.empty() || id.size() > 128)
    return false;
  return std::all_of(id.begin(), id.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
           (c >= 'A' && c <= 'Z');
  });
}

bool is_query_document(std::string_view doc) {
  // A small GraphQL lexer: enough to find operation keywords and the
  // brace depth they sit at, skipping comments and (block) strings.
  int depth = 0;
  bool expect_definition = true; // at depth 0, before a definition starts
  bool saw_operation = false;
  std::size_t i = 0;
  const std::size_t n = doc.size();
  auto is_name_char = [](char c) {
    return c == '_' || (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
           (c >= 'A' && c <= 'Z');
  };
  while (i < n) {
    const char c = doc[i];
    if (c == '#') { // comment to end of line
      while (i < n && doc[i] != '\n' && doc[i] != '\r')
        ++i;
      continue;
    }
    if (c == '"') {
      if (doc.substr(i, 3) == "\"\"\"") { // block string
        auto end = doc.find("\"\"\"", i + 3);
        if (end == std::string_view::npos)
          return false;
        i = end + 3;
        continue;
      }
      ++i;
      while (i < n && doc[i] != '"') {
        if (doc[i] == '\\')
          ++i;
        ++i;
      }
      if (i >= n)
        return false; // unterminated string
      ++i;
      continue;
    }
    if (c == '{') {
      if (depth == 0 && expect_definition)
        return false; // anonymous `{ ... }` shorthand
      ++depth;
      ++i;
      continue;
    }
    if (c == '}') {
      if (--depth < 0)
        return false;
      if (depth == 0)
        expect_definition = true;
      ++i;
      continue;
    }
    if (is_name_char(c)) {
      std::size_t start = i;
      while (i < n && is_name_char(doc[i]))
        ++i;
      std::string_view word = doc.substr(start, i - start);
      if (word == "mutation" || word == "subscription")
        return false; // anywhere outside strings/comments
      if (depth == 0 && expect_definition) {
        if (word == "query")
          saw_operation = true;
        else if (word != "fragment")
          return false;
        expect_definition = false;
      }
      continue;
    }
    ++i; // punctuation, whitespace, commas, ( ) $ : ! = @ [ ] ...
  }
  return depth == 0 && saw_operation;
}

// ============================================================
// HTTP (read-only: GET, and POST of a GraphQL *query* document)
// ============================================================
namespace detail {

std::string http_error_message(long status, const std::string &what,
                               const std::string &project,
                               const std::string &body) {
  std::string reason;
  if (status == 401)
    reason = "not authenticated — the token is missing or invalid";
  else if (status == 403)
    reason = fmt::format("token invalid or no access to project '{}'", project);
  else if (status == 404)
    reason = "not found — check the project and object ids";
  else
    reason = "unexpected response";
  constexpr std::size_t max_body = 300;
  return fmt::format(
      "Speckle {} failed (HTTP {}): {}. Server said: {}", what, status, reason,
      body.size() > max_body ? body.substr(0, max_body) + "…" : body);
}

json graphql_data_or_throw(const json &parsed, const std::string &project) {
  if (!parsed.is_object())
    throw std::runtime_error("Speckle GraphQL: response is not an object");
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

bool is_allowed_server_url(const std::string &server_url) {
  const std::string url = lower(server_url);
  if (url.rfind("https://", 0) == 0)
    return url.size() > 8;
  if (url.rfind("http://", 0) != 0)
    return false;
  std::string_view rest = std::string_view(url).substr(7);
  std::string_view host;
  if (!rest.empty() && rest.front() == '[') {
    auto close = rest.find(']');
    if (close == std::string_view::npos)
      return false;
    host = rest.substr(0, close + 1);
    rest.remove_prefix(close + 1);
  } else {
    auto end = rest.find_first_of(":/?#");
    host = rest.substr(0, end);
    rest.remove_prefix(end == std::string_view::npos ? rest.size() : end);
  }
  if (!rest.empty() && rest.front() != ':' && rest.front() != '/')
    return false;
  return host == "localhost" || host == "127.0.0.1" || host == "[::1]";
}

} // namespace detail

namespace {

constexpr std::size_t progress_step_bytes = 8u << 20; // 8 MiB

struct Sink {
  std::string body;
  std::string label;
  std::size_t max_bytes = 0;
  bool over_limit = false;
  std::size_t next_progress = progress_step_bytes;
};

size_t sink_write(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *sink = static_cast<Sink *>(userdata);
  const std::size_t bytes = size * nmemb;
  if (sink->body.size() + bytes > sink->max_bytes) {
    sink->over_limit = true;
    return 0; // makes curl abort with CURLE_WRITE_ERROR
  }
  sink->body.append(ptr, bytes);
  if (sink->body.size() >= sink->next_progress) {
    core::debug("Speckle: {} — {:.1f} MiB received", sink->label,
                static_cast<double>(sink->body.size()) / (1 << 20));
    sink->next_progress += progress_step_bytes;
  }
  return bytes;
}

void ensure_curl_global_init() {
  static std::once_flag once;
  static CURLcode result = CURLE_OK;
  std::call_once(once, [] { result = curl_global_init(CURL_GLOBAL_DEFAULT); });
  if (result != CURLE_OK)
    throw std::runtime_error(
        fmt::format("curl_global_init failed: {}", curl_easy_strerror(result)));
}

struct CurlDeleter {
  void operator()(CURL *c) const { curl_easy_cleanup(c); }
};
using CurlHandle = std::unique_ptr<CURL, CurlDeleter>;

/// Owns a curl header list; append() throws when curl cannot allocate.
class HeaderList {
    public:
  HeaderList() = default;
  HeaderList(const HeaderList &) = delete;
  HeaderList &operator=(const HeaderList &) = delete;
  ~HeaderList() { curl_slist_free_all(list_); }

  void append(const std::string &header) {
    curl_slist *next = curl_slist_append(list_, header.c_str());
    if (!next)
      throw std::runtime_error("curl_slist_append failed");
    list_ = next;
  }
  [[nodiscard]] curl_slist *get() const { return list_; }

    private:
  curl_slist *list_ = nullptr;
};

struct HttpRequest {
  std::string url;
  std::string token;
  std::string what;    ///< for messages, e.g. "object download"
  std::string project; ///< for the 403 message
  std::string accept = "application/json";
  const std::string *post_json = nullptr; ///< GraphQL query body, or GET
  bool long_transfer = false; ///< no total timeout, low-speed abort instead
  std::size_t max_bytes = 0;
  const std::atomic_bool *cancel = nullptr;      ///< abort when set
  long timeout_s = 0;                            ///< overall limit; 0 = default
  const ConnectFilter *connect_filter = nullptr; ///< unset/empty = none
  const CurlSetup *curl_setup = nullptr;         ///< unset/empty = none
};

/// "1.2.3.4:443" / "[::1]:80" for a refusal message.
std::string format_address(const sockaddr *addr, std::size_t len) {
  char text[INET6_ADDRSTRLEN] = {};
  if (addr->sa_family == AF_INET && len >= sizeof(sockaddr_in)) {
    const auto *in = reinterpret_cast<const sockaddr_in *>(addr);
    if (::inet_ntop(AF_INET, &in->sin_addr, text, sizeof text))
      return fmt::format("{}:{}", text, ntohs(in->sin_port));
  } else if (addr->sa_family == AF_INET6 && len >= sizeof(sockaddr_in6)) {
    const auto *in6 = reinterpret_cast<const sockaddr_in6 *>(addr);
    if (::inet_ntop(AF_INET6, &in6->sin6_addr, text, sizeof text))
      return fmt::format("[{}]:{}", text, ntohs(in6->sin6_port));
  }
  return fmt::format("<address family {}>", addr->sa_family);
}

/// CURLOPT_OPENSOCKETDATA of a request with a ConnectFilter.
struct ConnectGuard {
  const ConnectFilter *filter = nullptr;
  std::vector<std::string> refused; ///< formatted peer addresses
};

/// CURLOPT_OPENSOCKETFUNCTION: create the socket only for an address the
/// filter allows. Returning CURL_SOCKET_BAD makes curl try the next address
/// and, with none left, fail with CURLE_COULDNT_CONNECT.
curl_socket_t guarded_open_socket(void *clientp, curlsocktype /*purpose*/,
                                  curl_sockaddr *address) {
  auto *guard = static_cast<ConnectGuard *>(clientp);
  const auto len = static_cast<std::size_t>(address->addrlen);
  bool allowed = false;
  try {
    allowed = (*guard->filter)(&address->addr, len);
  } catch (const std::exception &e) {
    core::warn("Speckle: connect filter threw: {}", e.what());
  } catch (...) {
    core::warn("Speckle: connect filter threw a non-standard exception");
  }
  if (!allowed) {
    try {
      guard->refused.push_back(format_address(&address->addr, len));
    } catch (...) { // NOLINT(bugprone-empty-catch): no throw into C
    }
    return CURL_SOCKET_BAD;
  }
  // As curl's own: never leak the socket into a child process.
  return ::socket(address->family, address->socktype | SOCK_CLOEXEC,
                  address->protocol);
}

int cancel_progress(void *clientp, curl_off_t, curl_off_t, curl_off_t,
                    curl_off_t) {
  const auto *cancel = static_cast<const std::atomic_bool *>(clientp);
  return cancel && cancel->load(std::memory_order_acquire) ? 1 : 0;
}

std::string http_perform(const HttpRequest &req) {
  ensure_curl_global_init();
  CurlHandle curl(curl_easy_init());
  if (!curl)
    throw std::runtime_error("curl_easy_init failed");
  CURL *c = curl.get();

  Sink sink;
  sink.label = req.what;
  sink.max_bytes = req.max_bytes;
  HeaderList headers;
  headers.append("Accept: " + req.accept);
  if (!req.token.empty())
    headers.append("Authorization: Bearer " + req.token);
  if (req.post_json)
    headers.append("Content-Type: application/json");

  // The server URL was vetted by is_allowed_server_url: plain http only to
  // localhost. Redirects may only go to https.
  const bool plain_http = lower(req.url).rfind("http://", 0) == 0;
  curl_easy_setopt(c, CURLOPT_PROTOCOLS_STR, plain_http ? "http" : "https");
  curl_easy_setopt(c, CURLOPT_REDIR_PROTOCOLS_STR, "https");
  curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(c, CURLOPT_MAXREDIRS, 5L);
  // Required for signal-based timeout to work in multithreaded programs.
  curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 10L);
  if (req.long_transfer) {
    // A full object tree can be hundreds of MB: bound stalls, not time
    // (unless the caller set an overall limit).
    curl_easy_setopt(c, CURLOPT_LOW_SPEED_LIMIT, 1024L);
    curl_easy_setopt(c, CURLOPT_LOW_SPEED_TIME, 60L);
    if (req.timeout_s > 0)
      curl_easy_setopt(c, CURLOPT_TIMEOUT, req.timeout_s);
  } else {
    curl_easy_setopt(c, CURLOPT_TIMEOUT,
                     req.timeout_s > 0 ? std::min(req.timeout_s, 60L) : 60L);
  }
  if (req.cancel) {
    curl_easy_setopt(c, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(c, CURLOPT_XFERINFOFUNCTION, cancel_progress);
    curl_easy_setopt(c, CURLOPT_XFERINFODATA,
                     const_cast<std::atomic_bool *>(req.cancel));
  }
  curl_easy_setopt(c, CURLOPT_URL, req.url.c_str());
  curl_easy_setopt(c, CURLOPT_ACCEPT_ENCODING, ""); // gzip/deflate/br
  curl_easy_setopt(c, CURLOPT_HTTPHEADER, headers.get());
  curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, sink_write);
  curl_easy_setopt(c, CURLOPT_WRITEDATA, &sink);
  if (req.post_json) {
    curl_easy_setopt(c, CURLOPT_POSTFIELDS, req.post_json->c_str());
    curl_easy_setopt(c, CURLOPT_POSTFIELDSIZE,
                     static_cast<long>(req.post_json->size()));
  } else {
    curl_easy_setopt(c, CURLOPT_HTTPGET, 1L);
  }
  ConnectGuard guard;
  if (req.connect_filter && *req.connect_filter) {
    guard.filter = req.connect_filter;
    curl_easy_setopt(c, CURLOPT_OPENSOCKETFUNCTION, guarded_open_socket);
    curl_easy_setopt(c, CURLOPT_OPENSOCKETDATA, &guard);
    // Through a proxy the filter would vet the proxy, not the server.
    curl_easy_setopt(c, CURLOPT_PROXY, "");
  }
  // Last, so the front end's configurator can override any of the above.
  if (req.curl_setup && *req.curl_setup)
    (*req.curl_setup)(static_cast<void *>(c));

  const CURLcode res = curl_easy_perform(c);
  long http_code = 0;
  curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &http_code);

  if (sink.over_limit)
    throw std::runtime_error(fmt::format(
        "Speckle {}: response exceeds the download limit of {} MiB ({}); "
        "raise SpeckleReader::set_max_download_bytes if this is expected",
        req.what, req.max_bytes >> 20, req.url));
  if (res == CURLE_ABORTED_BY_CALLBACK)
    throw std::runtime_error(fmt::format("Speckle {}: cancelled", req.what));
  if (res != CURLE_OK && !guard.refused.empty()) {
    std::string refused;
    for (const auto &a : guard.refused)
      refused += (refused.empty() ? "" : ", ") + a;
    throw std::runtime_error(
        fmt::format("Speckle {}: connection to {} refused by policy ({})",
                    req.what, refused, req.url));
  }
  if (res != CURLE_OK)
    throw std::runtime_error(fmt::format("Speckle {}: curl error: {} ({})",
                                         req.what, curl_easy_strerror(res),
                                         req.url));
  if (http_code < 200 || http_code >= 300)
    throw std::runtime_error(detail::http_error_message(
        http_code, req.what, req.project, sink.body));
  return std::move(sink.body);
}

// ---- Object-tree helpers ----

bool is_reference(const json &v) {
  if (!v.is_object())
    return false;
  auto type = v.find("speckle_type");
  if (type == v.end() || !type->is_string() || *type != "reference")
    return false;
  auto ref = v.find("referencedId");
  return ref != v.end() && ref->is_string();
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

/// Number of times each id is referenced, over every stored object and the
/// value being resolved. Iterative, so a deep tree cannot overflow the stack.
std::unordered_map<std::string, std::size_t>
count_reference_uses(const std::unordered_map<std::string, json> &objects,
                     const json &value) {
  std::unordered_map<std::string, std::size_t> uses;
  std::vector<const json *> stack;
  stack.reserve(1024);
  stack.push_back(&value);
  for (const auto &[id, obj] : objects)
    stack.push_back(&obj);
  while (!stack.empty()) {
    const json *v = stack.back();
    stack.pop_back();
    if (v->is_object()) {
      if (is_reference(*v)) {
        ++uses[(*v)["referencedId"].get<std::string>()];
        continue;
      }
      for (const auto &el : *v)
        stack.push_back(&el);
    } else if (v->is_array()) {
      for (const auto &el : *v)
        stack.push_back(&el);
    }
  }
  return uses;
}

class Resolver {
    public:
  Resolver(const std::unordered_map<std::string, json> &objects,
           const ResolveOptions &options, const json &value)
      : objects_(objects), options_(options),
        uses_(count_reference_uses(objects, value)) {}

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
        path_.push_back(key);
        json out = walk(v, 1);
        path_.pop_back();
        in_progress_.erase(key);
        return out;
      }
    }
    return walk(v, 1);
  }

  ResolveReport finish() {
    report_.missing_ids.assign(missing_.begin(), missing_.end());
    report_.cyclic_ids.assign(cyclic_.begin(), cyclic_.end());
    return std::move(report_);
  }

    private:
  struct Cached {
    json value;
    std::size_t nodes = 0;
    std::size_t depth = 0; ///< nesting of the subtree itself
  };

  void add_nodes(std::size_t n) {
    report_.nodes += n;
    if (report_.nodes > options_.max_nodes)
      throw std::runtime_error(fmt::format(
          "Speckle: resolved object tree exceeds {} JSON nodes (inside object "
          "{}); a target shared by many parents inflates when inlined — "
          "resolve a subtree, or raise ResolveOptions::max_nodes",
          options_.max_nodes, current_id()));
  }

  void check_depth(std::size_t depth) {
    report_.max_depth_reached = std::max(report_.max_depth_reached, depth);
    local_max_depth_ = std::max(local_max_depth_, depth);
    if (depth > options_.max_depth)
      throw std::runtime_error(fmt::format(
          "Speckle: object tree nests deeper than {} levels (inside object "
          "{}); refusing to resolve further",
          options_.max_depth, current_id()));
  }

  [[nodiscard]] std::string current_id() const {
    return path_.empty() ? std::string("<value>") : path_.back();
  }

  json walk(const json &v, std::size_t depth) {
    if (v.is_object() && is_reference(v))
      return deref(v["referencedId"].get<std::string>(), v, depth);
    check_depth(depth);
    add_nodes(1);
    if (v.is_object()) {
      json out = json::object();
      for (const auto &[key, val] : v.items()) {
        if (key == "__closure")
          continue;
        std::string name = strip_detach_prefix(key);
        if (name != key && (v.contains(name) || out.contains(name)))
          name = key; // both spellings present: keep the detached one apart
        out[name] = walk(val, depth + 1);
      }
      return out;
    }
    if (v.is_array()) {
      json out = json::array();
      bool all_chunks = !v.empty();
      for (const auto &el : v) {
        out.push_back(walk(el, depth + 1));
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

  json deref(const std::string &id, const json &stub, std::size_t depth) {
    // Uses left after this one; a target used once is never cached.
    std::size_t &left = uses_[id];
    if (left > 0)
      --left;

    if (auto hit = cache_.find(id); hit != cache_.end()) {
      ++report_.references_resolved;
      ++report_.cache_hits;
      // The cached subtree was built at some depth; this use may sit
      // deeper, so its own nesting counts again from here.
      check_depth(depth + hit->second.depth);
      add_nodes(hit->second.nodes);
      if (left == 0) { // last use: hand the cached value over
        json out = std::move(hit->second.value);
        cache_.erase(hit);
        return out;
      }
      return hit->second.value;
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
    path_.push_back(id);
    const std::size_t nodes_before = report_.nodes;
    const std::size_t outer_max_depth = local_max_depth_;
    local_max_depth_ = depth;
    json resolved = walk(it->second, depth);
    const std::size_t subtree_depth = local_max_depth_ - depth;
    local_max_depth_ = std::max(outer_max_depth, local_max_depth_);
    path_.pop_back();
    in_progress_.erase(id);
    ++report_.references_resolved;

    if (left > 0 && !is_data_chunk(it->second)) {
      auto &entry = cache_[id];
      entry.nodes = report_.nodes - nodes_before;
      entry.depth = subtree_depth;
      entry.value = resolved; // one copy kept for the remaining uses
    }
    return resolved;
  }

  const std::unordered_map<std::string, json> &objects_;
  const ResolveOptions &options_;
  std::unordered_map<std::string, std::size_t> uses_;
  std::unordered_map<std::string, Cached> cache_;
  std::unordered_set<std::string> in_progress_;
  std::vector<std::string> path_;
  std::size_t local_max_depth_ = 0;
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

json ObjectTree::resolve(const json &value, ResolveReport *report,
                         const ResolveOptions &options) const {
  Resolver resolver(objects_, options, value);
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

json ObjectTree::resolve_root(ResolveReport *report,
                              const ResolveOptions &options) const {
  return resolve(root(), report, options);
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
    if (auto items = models->find("items");
        items != models->end() && items->is_array())
      for (const auto &m : *items)
        if (m.is_object())
          info.models.push_back(
              {string_or_empty(m, "id"), string_or_empty(m, "name")});
  }
  return info;
}

ProjectInfo
collect_project_pages(const std::function<json(const json &cursor)> &fetch_page,
                      int max_pages) {
  ProjectInfo info;
  json cursor = nullptr;
  for (int page = 0; page < max_pages; ++page) {
    const json data = fetch_page(cursor);
    ProjectInfo page_info = parse_project_info(data);
    if (page == 0) {
      info.id = page_info.id;
      info.name = page_info.name;
    }
    const bool empty_page = page_info.models.empty();
    for (auto &m : page_info.models)
      info.models.push_back(std::move(m));

    const json &project = data["project"];
    auto models = project.find("models");
    if (models == project.end() || !models->is_object())
      break; // null / absent models: nothing more to page through
    auto total = models->find("totalCount");
    const bool reached_total = total != models->end() &&
                               total->is_number_integer() &&
                               info.models.size() >= total->get<std::size_t>();
    auto next = models->find("cursor");
    const bool has_cursor = next != models->end() && next->is_string() &&
                            !next->get_ref<const std::string &>().empty();
    if (empty_page || reached_total || !has_cursor)
      break;
    if (page + 1 == max_pages)
      core::warn("Speckle: stopped listing models of project '{}' after {} "
                 "pages ({} models); more may exist",
                 info.name, max_pages, info.models.size());
    cursor = *next;
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
  auto items = versions->find("items");
  if (items == versions->end() || !items->is_array())
    return out;
  for (const auto &v : *items) {
    if (!v.is_object())
      continue;
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
  if (!detail::is_allowed_server_url(server_url_))
    throw std::runtime_error(fmt::format(
        "SpeckleReader: refusing server URL '{}': use https:// (plain http "
        "is only allowed to localhost)",
        server_url_));
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
  // Read-only by construction: refuse anything but query operations.
  if (!is_query_document(query))
    throw std::logic_error("SpeckleReader: only GraphQL queries are allowed");

  const std::string body =
      json{{"query", query}, {"variables", variables}}.dump();
  HttpRequest req;
  req.url = server_url_ + "/graphql";
  req.token = token_;
  req.what = "GraphQL query";
  req.project = project;
  req.post_json = &body;
  req.max_bytes = max_download_bytes_;
  req.cancel = cancel_;
  req.timeout_s = timeout_s_;
  req.connect_filter = &connect_filter_;
  req.curl_setup = &curl_setup_;
  const std::string response = http_perform(req);

  json parsed;
  try {
    parsed = json::parse(response);
  } catch (const json::parse_error &e) {
    throw std::runtime_error(
        fmt::format("Speckle GraphQL: invalid JSON response: {}", e.what()));
  }
  return detail::graphql_data_or_throw(parsed, project);
}

ProjectInfo SpeckleReader::query_project(const std::string &project) const {
  require_id(project, "project");
  static const std::string query =
      "query ProjectModels($project: String!, $cursor: String) { "
      "project(id: $project) { id name "
      "models(limit: 100, cursor: $cursor) { cursor totalCount "
      "items { id name } } } }";
  // Bounded pagination (STANDARDS §5): 100 pages of 100 models.
  ProjectInfo info = collect_project_pages(
      [&](const json &cursor) {
        json data = graphql_query(
            query, {{"project", project}, {"cursor", cursor}}, project);
        if (!data.contains("project") || data["project"].is_null())
          throw std::runtime_error(fmt::format(
              "Speckle: project '{}' not found, or no access to it", project));
        return data;
      },
      100);
  core::debug("Speckle: project '{}' ({}) has {} model(s)", info.name, project,
              info.models.size());
  return info;
}

std::vector<VersionInfo>
SpeckleReader::query_versions(const std::string &project,
                              const std::string &model, int limit) const {
  require_id(project, "project");
  require_id(model, "model");
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
  if (!data.contains("project") || data["project"].is_null())
    throw std::runtime_error(fmt::format(
        "Speckle: project '{}' not found, or no access to it", project));
  if (!data["project"].contains("model") || data["project"]["model"].is_null())
    throw std::runtime_error(fmt::format(
        "Speckle: model '{}' not found in project '{}'", model, project));
  return parse_versions(data);
}

VersionInfo SpeckleReader::query_version(const std::string &project,
                                         const std::string &model,
                                         const std::string &version) const {
  require_id(project, "project");
  require_id(model, "model");
  require_id(version, "version");
  static const std::string query =
      "query ModelVersion($project: String!, $model: String!, $version: "
      "String!) { project(id: $project) { id model(id: $model) { id "
      "version(id: $version) { id referencedObject sourceApplication "
      "createdAt message } } } }";
  json data = graphql_query(
      query, {{"project", project}, {"model", model}, {"version", version}},
      project);
  const json *v = nullptr;
  if (data.contains("project") && data["project"].is_object() &&
      data["project"].contains("model") &&
      data["project"]["model"].is_object() &&
      data["project"]["model"].contains("version") &&
      data["project"]["model"]["version"].is_object())
    v = &data["project"]["model"]["version"];
  if (!v)
    throw std::runtime_error(
        fmt::format("Speckle: version '{}' not found in model '{}' of "
                    "project '{}', or no access to it",
                    version, model, project));
  // Reuse the list parser for the field mapping.
  json wrapped = {
      {"project", {{"model", {{"versions", {{"items", json::array({*v})}}}}}}}};
  return parse_versions(wrapped).front();
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
  require_id(project, "project");
  require_id(object_id, "object");
  HttpRequest req;
  req.url = fmt::format("{}/objects/{}/{}", server_url_, project, object_id);
  req.token = token_;
  req.what = "object download";
  req.project = project;
  req.long_transfer = true;
  req.max_bytes = max_download_bytes_;
  req.cancel = cancel_;
  req.timeout_s = timeout_s_;
  req.connect_filter = &connect_filter_;
  req.curl_setup = &curl_setup_;
  core::debug("Speckle: downloading object tree {} from project {}", object_id,
              project);
  stopwatch sw;
  std::string body = http_perform(req);
  core::debug("Speckle: received {:.1f} MiB in {:.1f} s, parsing",
              static_cast<double>(body.size()) / (1 << 20), sw.elapsed());
  ObjectTree tree = ObjectTree::parse(body, object_id);
  core::info("Speckle: loaded {} objects for root {} ({:.1f} MiB)", tree.size(),
             object_id, static_cast<double>(body.size()) / (1 << 20));
  return tree;
}

json SpeckleReader::fetch_object(const std::string &project,
                                 const std::string &object_id) const {
  require_id(project, "project");
  require_id(object_id, "object");
  HttpRequest req;
  req.url =
      fmt::format("{}/objects/{}/{}/single", server_url_, project, object_id);
  req.token = token_;
  req.what = "object fetch";
  req.project = project;
  req.max_bytes = max_download_bytes_;
  req.cancel = cancel_;
  req.timeout_s = timeout_s_;
  req.connect_filter = &connect_filter_;
  req.curl_setup = &curl_setup_;
  std::string body = http_perform(req);
  try {
    return json::parse(body);
  } catch (const json::parse_error &e) {
    throw std::runtime_error(fmt::format(
        "Speckle object fetch: invalid JSON for {}: {}", object_id, e.what()));
  }
}

} // namespace reusex::io::speckle
