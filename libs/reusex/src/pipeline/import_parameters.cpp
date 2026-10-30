// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/pipeline/import_parameters.hpp>

#include <reusex/io/speckle_reader.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>

namespace reusex::pipeline {
namespace {

using json = nlohmann::json;

constexpr double kInf = std::numeric_limits<double>::infinity();

// Accessors decayed to plain function pointers so they fit
// ImportParameter::Field.
#define SPECKLE(f)                                                             \
  +[](SpeckleStageOptions &o) -> decltype(auto) { return (o.import.f); }
#define REVIT(f)                                                               \
  +[](SpeckleStageOptions &o) -> decltype(auto) { return (o.import.mapping.f); }
#define IFC(f)                                                                 \
  +[](IfcStageOptions &o) -> decltype(auto) { return (o.import.f); }

/// The descriptors of @p table, defaults from a default-constructed @p O.
template <typename O>
std::vector<ParameterDescriptor>
descriptors_of(const std::vector<ImportParameter<O>> &table) {
  O d{};
  std::vector<ParameterDescriptor> out;
  for (const auto &p : table) {
    ParameterDescriptor desc;
    desc.key = std::string(p.key);
    desc.label = std::string(p.label);
    desc.description = std::string(p.description);
    desc.required = p.required;
    if (p.minimum && std::isfinite(*p.minimum))
      desc.minimum = p.minimum;
    if (p.maximum && std::isfinite(*p.maximum))
      desc.maximum = p.maximum;
    std::visit(
        [&](auto get) {
          using T = std::remove_reference_t<decltype(get(d))>;
          if constexpr (std::is_same_v<T, double>) {
            desc.type = ParameterType::number;
            desc.default_value = get(d);
          } else if constexpr (std::is_same_v<T, bool>) {
            desc.type = ParameterType::boolean;
            desc.default_value = get(d);
            desc.minimum.reset();
            desc.maximum.reset();
          } else if constexpr (std::is_same_v<T, std::string>) {
            desc.type = ParameterType::string;
            desc.minimum.reset();
            desc.maximum.reset();
            // An empty default is "no default" (project, model, version,
            // path): there is nothing to pre-fill a form with.
            if (!get(d).empty())
              desc.default_value = get(d);
          } else {
            // std::optional<double>: unset by default, and setting it at
            // all overrides what the importer would measure, so a form
            // must omit it untouched (#214's presence semantics). Its
            // neutral value is 0.
            desc.type = ParameterType::number;
            desc.default_value = get(d).value_or(0.0);
            desc.presence_sensitive = true;
          }
        },
        p.field);
    out.push_back(std::move(desc));
  }
  return out;
}

template <typename O>
O options_from(std::string_view stage,
               const std::vector<ImportParameter<O>> &table,
               std::string_view parameters) {
  json params = json::object();
  if (!parameters.empty()) {
    params = json::parse(parameters, nullptr, /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument(
          fmt::format("{} parameters must be a JSON object", stage));
  }
  if (params.contains(std::string(kSecretParameterKey)))
    throw std::invalid_argument(fmt::format(
        "{} parameters must not carry a credential: '{}' is refused (job "
        "parameters are stored in the job history and pipeline_log; the "
        "Speckle token comes from the executor's token provider, "
        "SPECKLE_TOKEN by default)",
        stage, kSecretParameterKey));

  O o{};
  for (const auto &p : table) {
    const auto bad = [&](std::string_view why) {
      return std::invalid_argument(
          fmt::format("{} parameter '{}': {}", stage, p.key, why));
    };
    const auto it = params.find(std::string(p.key));
    const bool absent = it == params.end() || it->is_null();
    if (absent) {
      if (p.required)
        throw bad("is required");
      continue;
    }
    const auto check_range = [&](double v) {
      if (!std::isfinite(v) || (p.minimum && v < *p.minimum) ||
          (p.maximum && v > *p.maximum))
        throw bad(fmt::format("{} is outside [{}, {}]", v,
                              p.minimum.value_or(-kInf),
                              p.maximum.value_or(kInf)));
    };
    std::visit(
        [&](auto get) {
          using T = std::remove_reference_t<decltype(get(o))>;
          if constexpr (std::is_same_v<T, bool>) {
            if (!it->is_boolean())
              throw bad("expected true or false");
            get(o) = it->template get<bool>();
          } else if constexpr (std::is_same_v<T, std::string>) {
            if (!it->is_string())
              throw bad("expected a string");
            auto v = it->template get<std::string>();
            if (p.required && v.empty())
              throw bad("is required");
            get(o) = std::move(v);
          } else {
            if (!it->is_number())
              throw bad("expected a number");
            const double v = it->template get<double>();
            check_range(v);
            get(o) = v;
          }
        },
        p.field);
  }
  return o;
}

template <typename O>
std::string json_of(const std::vector<ImportParameter<O>> &table,
                    const O &options) {
  O o = options;
  json out = json::object();
  for (const auto &p : table)
    std::visit(
        [&](auto get) {
          using T = std::remove_reference_t<decltype(get(o))>;
          const std::string key(p.key);
          if constexpr (std::is_same_v<T, double>) {
            // shortest round-trip decimal, as rux::StageParams writes it
            out[key] = json::parse(fmt::format("{}", get(o)));
          } else if constexpr (std::is_same_v<T, bool>) {
            out[key] = get(o);
          } else if constexpr (std::is_same_v<T, std::string>) {
            if (!get(o).empty())
              out[key] = get(o);
          } else {
            if (get(o))
              out[key] = json::parse(fmt::format("{}", *get(o)));
          }
        },
        p.field);
  return out.dump();
}

} // namespace

// --- import_speckle ---------------------------------------------------------

const std::vector<SpeckleParameter> &speckle_parameter_table() {
  // Order = what a form asks first (where), then the mapping options in the
  // RevitImportOptions field order (io/speckle_revit.hpp documents each).
  static const std::vector<SpeckleParameter> table = {
      {"server", "--server", "Speckle server",
       "Speckle server URL (https://, or http:// to localhost only).",
       SPECKLE(server_url), std::nullopt, std::nullopt},
      {"project", "--project", "Speckle project", "Speckle project id.",
       SPECKLE(project), std::nullopt, std::nullopt,
       /*required=*/true},
      {"model", "--model", "Speckle model", "Speckle model id.", SPECKLE(model),
       std::nullopt, std::nullopt, /*required=*/true},
      {"version", "--version", "Model version",
       "Model version id; empty = the latest version.", SPECKLE(version),
       std::nullopt, std::nullopt},
      {"meshes", "--no-meshes", "Category meshes",
       "Store the per-category meshes; off, only the solid-model records "
       "(no mesh is written or deleted).",
       REVIT(meshes), std::nullopt, std::nullopt},
      {"storey_height", "--storey-height", "Top storey height [m]",
       "Height of the top storey (no level above it).",
       REVIT(default_storey_height_m), 0.5, 100.0},
      {"min_wall_thickness", "--min-wall-thickness", "Thinnest wall [m]",
       "Thinner walls (finish layers) get no wall record; their geometry "
       "stays in the walls mesh.",
       REVIT(min_wall_thickness_m), 0.0, 2.0},
      {"min_wall_fill", "--min-wall-fill", "Straight wall fill",
       "A wall is straight when its footprint's hull fills this share of its "
       "minimum-area rectangle.",
       REVIT(min_wall_fill), 0.0, 1.0},
      {"opening_host_tolerance", "--opening-host-tolerance",
       "Opening host tolerance [m]",
       "How far a door / window centre may lie outside its host wall's faces "
       "when the host is found geometrically.",
       REVIT(opening_host_tolerance_m), 0.0, 10.0},
      {"elevation_offset", "--elevation-offset", "Elevation offset [m]",
       "Revit internal z minus level elevation; absent = measured from the "
       "walls.",
       REVIT(elevation_offset_m), -1e5, 1e5},
      {"recenter", "--no-recenter", "Recentre far models",
       "Recentre a model far from the origin by a whole-km x/y offset.",
       REVIT(recenter), std::nullopt, std::nullopt},
      {"recenter_threshold", "--recenter-threshold", "Recentre beyond [m]",
       "Recentre a model whose plan centre is farther than this from the "
       "origin.",
       REVIT(recenter_threshold_m), 0.0, kInf},
  };
  return table;
}

const std::vector<ParameterDescriptor> &speckle_stage_parameters() {
  static const std::vector<ParameterDescriptor> d =
      descriptors_of(speckle_parameter_table());
  return d;
}

SpeckleStageOptions
speckle_options_from_parameters(std::string_view parameters) {
  auto o =
      options_from("import_speckle", speckle_parameter_table(), parameters);
  // Checked before anything is sent anywhere (SpeckleReader checks again).
  if (!io::speckle::detail::is_allowed_server_url(o.import.server_url))
    throw std::invalid_argument(fmt::format(
        "import_speckle parameter 'server': '{}' is not an https:// URL (or "
        "http:// to localhost)",
        o.import.server_url));
  for (const auto *id : {&o.import.project, &o.import.model, &o.import.version})
    if (!id->empty() && !io::speckle::is_valid_speckle_id(*id))
      throw std::invalid_argument(
          fmt::format("import_speckle: '{}' is not a Speckle id (1-128 "
                      "letters and digits)",
                      *id));
  o.import.token.clear(); // never from parameters
  return o;
}

std::string speckle_parameters_json(const SpeckleStageOptions &options) {
  return json_of(speckle_parameter_table(), options);
}

// --- import_ifc -------------------------------------------------------------

const std::vector<IfcParameter> &ifc_parameter_table() {
  static const std::vector<IfcParameter> table = {
      {"path", "--path", "IFC file",
       "The IFC file: absolute, or relative to the project's directory.",
       +[](IfcStageOptions &o) -> std::string & { return o.path; },
       std::nullopt, std::nullopt, /*required=*/true},
      {"meshes", "--no-meshes", "Class meshes",
       "Store the per-class meshes; off, only the solid-model records (no "
       "mesh is written or deleted).",
       IFC(meshes), std::nullopt, std::nullopt},
      {"storey_height", "--storey-height", "Top storey height [m]",
       "Height of the top storey (no storey above it).",
       IFC(default_storey_height_m), 0.5, 100.0},
      {"min_wall_thickness", "--min-wall-thickness", "Thinnest wall [m]",
       "Thinner walls (finish layers) get no wall record (mesh only).",
       IFC(min_wall_thickness_m), 0.0, 2.0},
      {"min_wall_fill", "--min-wall-fill", "Straight wall fill",
       "A wall is straight when its footprint's hull fills this share of its "
       "minimum-area rectangle.",
       IFC(min_wall_fill), 0.0, 1.0},
      {"opening_host_tolerance", "--opening-host-tolerance",
       "Opening host tolerance [m]",
       "How far a door / window centre may lie outside its host wall's faces "
       "when the host is found geometrically.",
       IFC(opening_host_tolerance_m), 0.0, 10.0},
      {"recenter", "--no-recenter", "Recentre far models",
       "Recentre a model far from the origin (georeferenced) by a whole-km "
       "x/y offset.",
       IFC(recenter), std::nullopt, std::nullopt},
      {"recenter_threshold", "--recenter-threshold", "Recentre beyond [m]",
       "Recentre a model whose plan centre is farther than this from the "
       "origin.",
       IFC(recenter_threshold_m), 0.0, kInf},
  };
  return table;
}

#undef SPECKLE
#undef REVIT
#undef IFC

const std::vector<ParameterDescriptor> &ifc_stage_parameters() {
  static const std::vector<ParameterDescriptor> d =
      descriptors_of(ifc_parameter_table());
  return d;
}

IfcStageOptions ifc_options_from_parameters(std::string_view parameters) {
  return options_from("import_ifc", ifc_parameter_table(), parameters);
}

std::string ifc_parameters_json(const IfcStageOptions &options) {
  return json_of(ifc_parameter_table(), options);
}

std::filesystem::path resolve_ifc_path(const std::filesystem::path &project,
                                       const std::string &path) {
  namespace fs = std::filesystem;
  if (path.empty())
    throw std::invalid_argument("import_ifc parameter 'path' is required");
  fs::path file(path);
  if (file.is_relative()) {
    const fs::path rel = file.lexically_normal();
    if (rel.empty() || *rel.begin() == "..")
      throw std::invalid_argument(fmt::format(
          "import_ifc: relative path '{}' leaves the project's directory",
          path));
    fs::path base = project.parent_path();
    if (base.empty())
      base = fs::current_path();
    file = base / rel;
  }
  std::error_code ec;
  if (!fs::is_regular_file(file, ec))
    throw std::invalid_argument(
        fmt::format("import_ifc: '{}' is not an existing file", path));
  return file;
}

} // namespace reusex::pipeline
