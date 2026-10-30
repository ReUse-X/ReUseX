# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# web-ifc (That Open Company, MPL-2.0): the C++ IFC STEP parser and geometry
# engine behind IFC.js / That Open, built natively as a static library for
# `rux import ifc` (reusex-3iy8). Installs headers, libweb-ifc-library.a and
# a CMake package: find_package(web-ifc CONFIG) -> web-ifc::web-ifc.
#
# Upstream fetches its (header-only) dependencies with FetchContent, which
# cannot download in the sandbox; each is prefetched here at the exact commit
# upstream pins and handed over through FETCHCONTENT_SOURCE_DIR_<NAME>.
# spdlog is the exception: its types appear in ModelManager.h, and a second,
# header-only spdlog compiled into a static library would clash (ODR) with
# the compiled nixpkgs spdlog `rux` links, so it builds against that one
# (find_package, PUBLIC dependency, re-found by web-ifc-config.cmake).
#
# native-install.patch (CMake only, no source changes):
#   - WEB_IFC_BUILD_TESTS=OFF skips the test executables and TinyCppTest;
#   - WEB_IFC_SYSTEM_SPDLOG=ON uses find_package(spdlog) as described above;
#   - installs every header (upstream's install leaves out boolean-utils/,
#     which IfcGeometry.h includes) and the header-only dependencies the
#     headers include (unordered_dense, earcut, ...) under
#     include/web-ifc/third_party, which joins the target's include dirs;
#   - exports the target as web-ifc::web-ifc.
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  ninja,
  spdlog,
  ...
}: let
  dep = owner: repo: rev: hash: fetchFromGitHub {inherit owner repo rev hash;};
  deps = {
    FASTFLOAT = dep "fastfloat" "fast_float" "b0ab987b3dfdde13fa1915f65ef2a5c068d9208c" "sha256-Hfnoet/FMCqm5Qx7A6/2NnSlmWgTFJrOqHvXh1TDd6U=";
    TINYNURBS = dep "QuimMoya" "tinynurbs" "47115cd9b6e922b27bbc4ab01fdeac2e9ea597a4" "sha256-7LchmxZ/oN7mRMi9yFajGUYu4puJbRCSTUbPzSVZGhQ=";
    GLM = dep "g-truc" "glm" "8d1fd52e5ab5590e2c81768ace50c72bae28f2ed" "sha256-6WnVvFiTe1/OYj/oTGpCjZKNFurR9MxJ4zf0nDg0Alk=";
    EARCUT = dep "mapbox" "earcut.hpp" "c68c8835ccff2b7532d31d8fa8dfcf398f629498" "sha256-Q0BCfujUWNOg0mB6ZZUFOvisCHDKplnB5gAjmec8ZaM=";
    CDT = dep "artem-ogre" "CDT" "2068d015b9db3c92481e869b0c1f669b96a1d70a" "sha256-VXM70mWzZF40xfiQOveOeKMFfnpoNgUBuNVEp2KPx6k=";
    STDUUID = dep "mariusbancila" "stduuid" "3afe7193facd5d674de709fccc44d5055e144d7a" "sha256-MhpKv+gH3QxiaQMx5ImiQjDGrbKUFaaoBLj5Voh78vg=";
    UNORDERED_DENSE = dep "martinus" "unordered_dense" "1c8636e810d7e4a485d17bb9cbd6dccb5fe4e284" "sha256-dnPbVlQOcwD7p5uuN56vEin+/+d83xh6szW+Uth/HHk=";
  };
in
  stdenv.mkDerivation (finalAttrs: {
    pname = "web-ifc";
    version = "0.78";

    src = fetchFromGitHub {
      owner = "ThatOpen";
      repo = "engine_web-ifc";
      tag = finalAttrs.version;
      hash = "sha256-sHwUku3fR58KPckGNr8xAFQsWxcV+RWFZ/BX3ybOs+0=";
    };

    sourceRoot = "${finalAttrs.src.name}/src/cpp";

    patches = [./native-install.patch];

    nativeBuildInputs = [cmake ninja];

    # PUBLIC link dependency of web-ifc::web-ifc (see above).
    propagatedBuildInputs = [spdlog];

    cmakeFlags =
      [
        (lib.cmakeBool "WEB_IFC_BUILD_TESTS" false)
        (lib.cmakeBool "WEB_IFC_SYSTEM_SPDLOG" true)
        (lib.cmakeBool "BUILD_SHARED_LIBS" false)
        # The archive ends up in rux and in the Python extension module.
        (lib.cmakeBool "CMAKE_POSITION_INDEPENDENT_CODE" true)
        (lib.cmakeBool "FETCHCONTENT_FULLY_DISCONNECTED" true)
      ]
      ++ lib.mapAttrsToList (name: src: lib.cmakeFeature "FETCHCONTENT_SOURCE_DIR_${name}" "${src}") deps;

    meta = {
      description = "Native C++ core of web-ifc: IFC STEP parsing and geometry (That Open)";
      homepage = "https://github.com/ThatOpen/engine_web-ifc";
      license = lib.licenses.mpl20;
      platforms = lib.platforms.linux;
    };
  })
