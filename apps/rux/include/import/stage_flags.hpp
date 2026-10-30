// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The `rux import speckle|ifc` flags, generated from the import stages'
// parameter tables (pipeline/import_parameters.hpp), so flag names, defaults
// and bounds are the stage's own (STANDARDS §4) and the Qt client's
// "Kopiér som rux-kommando" round-trips (test_cli_command_roundtrip.cpp).

#include <reusex/pipeline/import_parameters.hpp>

#include <CLI/CLI.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

namespace rux {

/// Add one option / flag per row of @p table to @p sub, writing into
/// @p options (which must outlive the parse). The row keyed @p positional_key
/// is also accepted as the positional argument @p positional_name.
template <typename O>
void add_import_stage_flags(
    CLI::App &sub, O &options,
    const std::vector<reusex::pipeline::ImportParameter<O>> &table,
    std::string_view positional_key = {},
    std::string_view positional_name = {}) {
  const O defaults{};
  for (const auto &p : table) {
    std::string name(p.flag);
    if (!positional_key.empty() && p.key == positional_key)
      name = std::string(positional_name) + "," + name;
    const std::string help(p.description);
    std::visit(
        [&](auto get) {
          auto &field = get(options);
          using T = std::remove_reference_t<decltype(field)>;
          if constexpr (std::is_same_v<T, bool>) {
            // The flag sets the opposite of the default.
            O copy = defaults;
            const bool def = get(copy);
            sub.add_flag_callback(
                name, [&field, def] { field = !def; },
                help + (def ? " (this flag turns it off)"
                            : " (this flag turns it on)"));
          } else if constexpr (std::is_same_v<T, std::string>) {
            auto *o = sub.add_option(name, field, help);
            if (!field.empty())
              o->default_val(field);
          } else {
            auto *o = sub.add_option(name, field, help);
            if constexpr (std::is_same_v<T, double>)
              o->default_val(field);
            const double lo =
                p.minimum.value_or(-std::numeric_limits<double>::max());
            const double hi = p.maximum && std::isfinite(*p.maximum)
                                  ? *p.maximum
                                  : std::numeric_limits<double>::max();
            o->check(CLI::Range(lo, hi));
          }
        },
        p.field);
  }
}

} // namespace rux
