// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The import_speckle stage over its REAL network path
// (io::speckle::import_speckle_revit -> SpeckleReader -> curl), against a
// tiny local HTTP server on 127.0.0.1 that answers the one GraphQL query and
// the object download with the synthetic Revit model. Covers what the
// injected-download tests cannot: the token reaching the Authorization
// header (and nowhere else), the caller's pipeline_log row threaded through
// import_speckle_revit (exactly one row), and the cancel token / overall
// timeout aborting a stalled download, and the transport hooks a multi-user
// front end guards the reads with (StageExecutorOptions::
// speckle_connect_filter / speckle_curl_setup).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>

#include "../../support/revit_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace reusex;
using namespace reusex::pipeline;
using nlohmann::json;
using reusex::test_support::TempPath;

namespace {

constexpr const char *kToken = "http-token-5c1e";

/// One-thread HTTP/1.1 server: POST /graphql -> the model's latest version,
/// GET /objects/proj01/root01 -> the model (or a stalled trickle).
class FakeSpeckleServer {
    public:
  explicit FakeSpeckleServer(bool stall) : stall_(stall) {
    json root = test_support::RevitFixture{}.model();
    root["id"] = "root01";
    objects_ = json::array({root}).dump();

    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd_ >= 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    REQUIRE(::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    REQUIRE(::listen(fd_, 8) == 0);
    socklen_t len = sizeof addr;
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    thread_ = std::thread([this] { loop(); });
  }
  ~FakeSpeckleServer() {
    stop_ = true;
    thread_.join();
    ::close(fd_);
  }

  std::string url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }
  std::vector<std::string> requests() {
    std::lock_guard lock(mutex_);
    return requests_;
  }
  /// TCP connections accepted so far.
  int connections() const { return connections_.load(); }
  std::vector<std::string> authorizations() {
    std::lock_guard lock(mutex_);
    return auth_;
  }

    private:
  void loop() {
    while (!stop_) {
      pollfd p{fd_, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0)
        continue;
      const int c = ::accept(fd_, nullptr, nullptr);
      if (c < 0)
        continue;
      ++connections_;
      serve(c);
      ::close(c);
    }
  }

  static std::string header(const std::string &head, const std::string &name) {
    std::string lower = head;
    for (auto &ch : lower)
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    const auto at = lower.find("\r\n" + name + ":");
    if (at == std::string::npos)
      return {};
    const auto start = at + 2 + name.size() + 1;
    const auto end = head.find("\r\n", start);
    std::string v = head.substr(start, end - start);
    while (!v.empty() && v.front() == ' ')
      v.erase(v.begin());
    return v;
  }

  void send_all(int c, const std::string &data) {
    std::size_t off = 0;
    while (off < data.size()) {
      const auto n =
          ::send(c, data.data() + off, data.size() - off, MSG_NOSIGNAL);
      if (n <= 0)
        return;
      off += static_cast<std::size_t>(n);
    }
  }

  void serve(int c) {
    std::string in;
    char buf[4096];
    std::size_t head_end = std::string::npos;
    while ((head_end = in.find("\r\n\r\n")) == std::string::npos) {
      const auto n = ::recv(c, buf, sizeof buf, 0);
      if (n <= 0)
        return;
      in.append(buf, static_cast<std::size_t>(n));
    }
    const std::string head = in.substr(0, head_end);
    const std::string length = header(head, "content-length");
    const std::size_t body_len = length.empty() ? 0 : std::stoul(length);
    while (in.size() < head_end + 4 + body_len) {
      const auto n = ::recv(c, buf, sizeof buf, 0);
      if (n <= 0)
        return;
      in.append(buf, static_cast<std::size_t>(n));
    }
    const std::string line = head.substr(0, head.find("\r\n"));
    {
      std::lock_guard lock(mutex_);
      requests_.push_back(line);
      auth_.push_back(header(head, "authorization"));
    }

    if (line.rfind("POST /graphql", 0) == 0) {
      const json body = {
          {"data",
           {{"project",
             {{"id", "proj01"},
              {"model",
               {{"id", "model01"},
                {"name", "m"},
                {"versions",
                 {{"items", json::array({{{"id", "ver01"},
                                          {"referencedObject", "root01"},
                                          {"sourceApplication", "Revit"},
                                          {"createdAt", "2026-10-30T00:00:00Z"},
                                          {"message", ""}}})}}}}}}}}}};
      respond(c, "200 OK", body.dump());
    } else if (line.rfind("GET /objects/proj01/root01", 0) == 0) {
      if (!stall_) {
        respond(c, "200 OK", objects_);
        return;
      }
      // Promise a large body, then trickle until the client gives up.
      send_all(c, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                  "Content-Length: 100000000\r\nConnection: close\r\n\r\n[");
      for (int i = 0; i < 400 && !stop_; ++i) { // at most ~20 s
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (::send(c, " ", 1, MSG_NOSIGNAL) <= 0)
          return;
      }
    } else {
      respond(c, "404 Not Found", "{}");
    }
  }

  void respond(int c, const std::string &status, const std::string &body) {
    send_all(c, "HTTP/1.1 " + status +
                    "\r\nContent-Type: application/json\r\nContent-Length: " +
                    std::to_string(body.size()) +
                    "\r\nConnection: close\r\n\r\n" + body);
  }

  bool stall_;
  std::string objects_;
  int fd_ = -1;
  uint16_t port_ = 0;
  std::atomic_bool stop_{false};
  std::atomic_int connections_{0};
  std::thread thread_;
  std::mutex mutex_;
  std::vector<std::string> requests_;
  std::vector<std::string> auth_;
};

StageExecutorOptions local_options() {
  StageExecutorOptions o;
  o.allow_insecure_localhost = true;
  o.speckle_token = [](const StageContext &, const std::string &) {
    return std::optional<std::string>(kToken);
  };
  return o;
}

StageContext context(const TempPath &project, const std::string &server) {
  // The loopback server must not be sent through a proxy from the
  // environment.
  ::setenv("no_proxy", "127.0.0.1,localhost", 1);
  ::setenv("NO_PROXY", "127.0.0.1,localhost", 1);
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::import_speckle;
  ctx.parameters =
      json{{"server", server}, {"project", "proj01"}, {"model", "model01"}}
          .dump();
  ctx.job_id = "job-http-1";
  return ctx;
}

} // namespace

TEST_CASE("ImportSpeckleHttp_RealReader_OneLogRowAndTokenOnlyInTheHeader",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/false);
  const StageResult r =
      run_stage(context(project, server.url()), local_options());
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("ver01") != std::string::npos);

  const auto reqs = server.requests();
  REQUIRE(reqs.size() == 2);
  CHECK(reqs[0].rfind("POST /graphql", 0) == 0);
  CHECK(reqs[1].rfind("GET /objects/proj01/root01", 0) == 0);
  for (const auto &auth : server.authorizations())
    CHECK(auth == std::string("Bearer ") + kToken);

  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().size() == 1);
  // import_speckle_revit ran under the stage's row: exactly one.
  const auto runs = db.pipeline_log();
  REQUIRE(runs.size() == 1);
  CHECK(runs.front().stage == "import_speckle");
  CHECK(runs.front().status == "success");
  CHECK(json::parse(runs.front().parameters).at("job_id") == "job-http-1");
  CHECK(runs.front().parameters.find(kToken) == std::string::npos);
  // The meshes carry the resolved version.
  const auto md =
      json::parse(db.mesh_metadata("imported/revit/walls").parameters);
  CHECK(md.at("version") == "ver01");
  CHECK(md.dump().find(kToken) == std::string::npos);
}

TEST_CASE("ImportSpeckleHttp_CancelAbortsAStalledDownload",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/true);
  std::atomic_bool cancel{false};
  auto ctx = context(project, server.url());
  ctx.cancel_token = &cancel;
  std::thread canceller([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    cancel = true;
  });
  const auto t0 = std::chrono::steady_clock::now();
  const StageResult r = run_stage(ctx, local_options());
  canceller.join();
  INFO(r.message);
  CHECK(r.cancelled);
  CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10));

  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().empty());
  const auto runs = db.pipeline_log();
  REQUIRE(runs.size() == 1);
  CHECK(runs.front().status == "cancelled");
}

TEST_CASE("ImportSpeckleHttp_TimeoutAbortsAStalledDownload",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/true);
  auto options = local_options();
  options.speckle_timeout_s = 1;
  const auto t0 = std::chrono::steady_clock::now();
  const StageResult r = run_stage(context(project, server.url()), options);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK_FALSE(r.cancelled);
  CHECK(r.message.find("object download") != std::string::npos);
  CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10));
  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().empty());
}

namespace {
bool is_loopback(const sockaddr *addr, std::size_t len) {
  if (addr->sa_family == AF_INET && len >= sizeof(sockaddr_in))
    return ntohl(
               reinterpret_cast<const sockaddr_in *>(addr)->sin_addr.s_addr) >>
               24 ==
           127;
  if (addr->sa_family == AF_INET6 && len >= sizeof(sockaddr_in6)) {
    const auto &a = reinterpret_cast<const sockaddr_in6 *>(addr)->sin6_addr;
    return IN6_IS_ADDR_LOOPBACK(&a) != 0;
  }
  return false;
}
} // namespace

TEST_CASE("ImportSpeckleHttp_ConnectFilterRefusingLoopback_NoConnection",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/false);
  auto options = local_options();
  std::atomic_int calls{0};
  options.speckle_connect_filter = [&](const sockaddr *addr, std::size_t len) {
    ++calls;
    return !is_loopback(addr, len);
  };
  const StageResult r = run_stage(context(project, server.url()), options);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK_FALSE(r.cancelled);
  CHECK(r.message.find("refused by policy") != std::string::npos);
  CHECK(r.message.find("127.0.0.1:") != std::string::npos);
  CHECK(r.message.find(kToken) == std::string::npos);
  CHECK(calls >= 1);
  // Refused before the socket existed: the server never saw a connection.
  CHECK(server.connections() == 0);
  CHECK(server.requests().empty());
  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().empty());
}

TEST_CASE("ImportSpeckleHttp_ConnectFilterThrowing_IsARefusal",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/false);
  auto options = local_options();
  options.speckle_connect_filter = [](const sockaddr *, std::size_t) -> bool {
    throw std::runtime_error("resolver policy unavailable");
  };
  const StageResult r = run_stage(context(project, server.url()), options);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.message.find("refused by policy") != std::string::npos);
  CHECK(server.connections() == 0);
}

TEST_CASE("ImportSpeckleHttp_ConnectFilterAllowing_SeesEveryConnection",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/false);
  auto options = local_options();
  std::mutex mutex;
  std::vector<std::string> seen;
  options.speckle_connect_filter = [&](const sockaddr *addr, std::size_t len) {
    REQUIRE(addr->sa_family == AF_INET);
    REQUIRE(len >= sizeof(sockaddr_in));
    const auto *in = reinterpret_cast<const sockaddr_in *>(addr);
    char text[INET_ADDRSTRLEN] = {};
    ::inet_ntop(AF_INET, &in->sin_addr, text, sizeof text);
    std::lock_guard lock(mutex);
    seen.push_back(std::string(text) + ":" +
                   std::to_string(ntohs(in->sin_port)));
    return true;
  };
  const StageResult r = run_stage(context(project, server.url()), options);
  INFO(r.message);
  REQUIRE(r.ok);
  // One connection per request (the server closes each): GraphQL + objects.
  REQUIRE(server.connections() == 2);
  REQUIRE(seen.size() == 2);
  const std::string expected =
      "127.0.0.1:" + server.url().substr(server.url().rfind(':') + 1);
  for (const auto &s : seen)
    CHECK(s == expected);
  ProjectDB db(project.path, /*readOnly=*/true);
  CHECK(db.walls().size() == 1);
}

TEST_CASE("ImportSpeckleHttp_CurlSetupRunsOnEveryHandle",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/false);
  auto options = local_options();
  std::vector<void *> handles;
  options.speckle_curl_setup = [&](void *curl) { handles.push_back(curl); };
  const StageResult r = run_stage(context(project, server.url()), options);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(handles.size() == 2); // GraphQL query + object download
  for (void *h : handles)
    CHECK(h != nullptr);
  CHECK(server.connections() == 2);
}

TEST_CASE("ImportSpeckleHttp_CurlSetupThrowing_AbortsBeforeConnecting",
          "[pipeline][stages][import][speckle][http]") {
  TempPath project("test_import_speckle_http");
  FakeSpeckleServer server(/*stall=*/false);
  auto options = local_options();
  options.speckle_curl_setup = [](void *) {
    throw std::runtime_error("transport policy: refused");
  };
  const StageResult r = run_stage(context(project, server.url()), options);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.message.find("transport policy: refused") != std::string::npos);
  CHECK(server.connections() == 0);
}
