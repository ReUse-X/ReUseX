// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Controls auto-discovery and import of companion 360° panoramas.
struct ImportPanoramasOptions {
  /// When true, skip panorama discovery entirely (--no-panoramas CLI flag).
  bool skip = false;

  /// Subdirectory names searched first for companion panoramas.
  /// If any of these directories exist under the capture root, only those
  /// directories are scanned (non-recursively). If none exist, the root
  /// directory itself is scanned one level deep as a fallback.
  std::vector<std::string> subdir_hints = {"360", "panoramas", "pano"};

  /// When true (default), stitch .insp dual-fisheye files using the static
  /// Insta360 X4 calibration (see io/insta360_x4.hpp). Set to false to skip
  /// .insp files entirely (e.g. if stitch quality needs manual verification
  /// before import, or for benchmarking without stitching overhead).
  bool stitch_insp = true;
};

/// Auto-discover and import companion 360° equirect JPEGs from a capture
/// directory into the project database.
///
/// Discovery order: for each name in @p opts.subdir_hints, check whether
/// @p dir/<name> is a directory and collect it; if any hint directories are
/// found those are scanned exclusively. If none exist, @p dir itself is
/// scanned one level deep as a fallback (non-recursive, to avoid picking up
/// RGB-D frame images that live in known subdirectories).
///
/// Equirect guard: only files whose JPEG header reports width == 2 × height
/// are treated as panoramas. Non-equirect JPEGs are skipped at debug level.
///
/// Each candidate is matched to the nearest sensor frame by EXIF timestamp.
/// If sensor frames carry no timestamps, or a file has no EXIF timestamp, the
/// panorama is imported with node_id=NULL and a warn-level log is emitted —
/// such panoramas are stored in the DB but are skipped by `rux align 360`
/// (which requires a linked sensor frame as an alignment seed).
///
/// @param db   Open, writable project database.
/// @param dir  Capture root directory to search.
/// @param opts Discovery and filtering options.
/// @returns    Number of panoramas newly imported (0 if none found).
std::size_t import_panoramas(ProjectDB &db, const std::filesystem::path &dir,
                             const ImportPanoramasOptions &opts = {});

/// Options of register_panorama() for a file.
struct RegisterPanoramaOptions {
  /// Stitch an Insta360 X4 .insp (the static calibration); false refuses one.
  bool stitch_insp = true;
  /// Name to store it under; empty = the file name (a .insp is stored as
  /// `<stem>.jpg`, as `rux import 360` does).
  std::string filename;
};

/// Register one new 360 panorama with a manual initial pose (reusex-3iy8.5):
/// the decode/store path of `rux import 360` (equirect guard, .insp
/// stitching, EXIF timestamp) without timestamp matching or content
/// alignment. The row is stored unlinked (node_id NULL) with
/// `pose_source = "manual"`, image and pose in one transaction.
///
/// @param pose row-major 4x4 panorama-optical -> world (see
///        core/panorama_pose.hpp; core::upright_panorama_pose() builds one
///        from a position and a yaw).
/// @return the new panorama's id.
/// @throws std::invalid_argument for an invalid pose, a name already
///         stored, or an image that is not an equirect (width == 2*height);
///         std::runtime_error when the file cannot be read or stitched.
///         Nothing is written on a throw.
int register_panorama(ProjectDB &db, const std::filesystem::path &file,
                      const std::array<double, 16> &pose,
                      const RegisterPanoramaOptions &opts = {});

/// The same for an equirect JPEG already in memory (an HTTP upload).
/// @p timestamp epoch seconds, < 0 unknown.
int register_panorama(ProjectDB &db, const std::string &filename,
                      const std::vector<uint8_t> &jpeg,
                      const std::array<double, 16> &pose,
                      double timestamp = -1.0);

} // namespace reusex::io
