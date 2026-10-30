// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <functional>

/// @file
/// Transport policy hooks for the read-only Speckle client
/// (`io/speckle_reader.hpp`), kept curl-free and socket-header-free
/// (STANDARDS §2) so the pipeline's StageExecutorOptions can carry them
/// without pulling in either.

// Declared at global scope so the alias below names ::sockaddr, the POSIX
// type, and not a new reusex::io::speckle::sockaddr.
struct sockaddr; // NOLINT(readability-identifier-naming)

namespace reusex::io::speckle {

/// Decides, per connection attempt, whether SpeckleReader may connect to
/// @p addr (@p len bytes: a `sockaddr_in` or `sockaddr_in6`). Called with
/// the RESOLVED peer address of every socket curl opens, redirects and
/// happy-eyeballs fallbacks included, before the socket exists. Return
/// false to refuse: no socket is created, and when no address of a request
/// was allowed the request throws std::runtime_error "... connection to
/// <addr> refused by policy". An exception thrown here counts as a refusal.
///
/// Installing a filter also disables proxies for the reader's requests
/// (CURLOPT_PROXY ""): through a proxy the filter would only ever see the
/// proxy's address, never the server's.
///
/// May be called from the thread running the request; must not block for
/// long. This is the hook a multi-user front end (ruxd) uses to keep
/// imports to public addresses only (SSRF guard).
using ConnectFilter =
    std::function<bool(const ::sockaddr *addr, std::size_t len)>;

/// Raw curl configurator: called with the `CURL *` easy handle of every
/// request, after SpeckleReader has set all of its own options and just
/// before the transfer. It may tighten or override transport options
/// (CURLOPT_PROXY, CURLOPT_FOLLOWLOCATION / CURLOPT_MAXREDIRS,
/// CURLOPT_OPENSOCKETFUNCTION, CURLOPT_RESOLVE, TLS settings, ...). It must
/// NOT touch CURLOPT_URL, CURLOPT_WRITEFUNCTION / CURLOPT_WRITEDATA,
/// CURLOPT_HTTPHEADER, CURLOPT_POSTFIELDS or CURLOPT_XFERINFOFUNCTION /
/// CURLOPT_XFERINFODATA, which point at SpeckleReader's own state.
/// Overriding CURLOPT_OPENSOCKETFUNCTION replaces an installed ConnectFilter
/// (set CURLOPT_OPENSOCKETDATA along with it: the reader's own points at its
/// filter state). An exception thrown here aborts the request before any
/// connection is made.
///
/// Prefer ConnectFilter; this exists for policies it cannot express.
using CurlSetup = std::function<void(void *curl_easy_handle)>;

} // namespace reusex::io::speckle
