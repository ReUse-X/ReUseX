// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/// @file
/// The display meshes a BIM import (Revit via Speckle, IFC) brings along
/// with its solid-model records: one merged triangle mesh per category,
/// stored as `imported/<slug>`.

namespace reusex::io {

/// One merged triangle mesh (world frame, metres).
struct ImportedMesh {
  std::string name;     ///< "imported/<category slug>"
  std::string category; ///< source category / class, as received
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<std::uint32_t, 3>> triangles;
  std::size_t elements = 0; ///< elements that contributed geometry
};

/// The ASCII slug of a category name used in mesh names
/// ("Plumbing Fixtures" -> "plumbing_fixtures"; "" -> "uncategorized").
std::string mesh_slug(const std::string &category);

} // namespace reusex::io
