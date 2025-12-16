# Standalone IVP Build Changes

This repository now carries the minimum set of helpers and build-system
adjustments needed to compile the IvP components outside of the full
`moos-ivp` tree. The most notable updates are:

## Build configuration

- `src/ivp/src/CMakeLists.txt` automatically disables GUI targets when the
  FLTK headers cannot be located, preventing configuration failures on systems
  where FLTK is not installed.
- The same file now filters the IVP library and application lists to include
  only directories that actually exist in this trimmed-down source tree. Missing
  entries trigger a warning instead of a hard error, so CMake can continue.
- Several command-line utilities (`app_alogeval`, `app_alogpick`,
  `app_manifest_test`) no longer link against the unused `apputil` target,
  keeping their dependency set aligned with the libraries that are available
  in this repo subset.

## Restored utility classes

- `lib_mbutil` once again ships the `ACTable` helper (`ACTable.h/.cpp`), so
  reporting tools that produce aligned tables can build without pulling in
  additional upstream sources. The target's `CMakeLists.txt` now exports the new
  files.
- `lib_realm` adds a lightweight `InfoCastSettings` class (and registers it with
  the library's `CMakeLists.txt`) so the realm utilities have the configuration
  accessors they expect.
- `lib_geometry` includes a minimal `MOOSGeodesy` stub that satisfies the
  `bhv2graphviz` dependency chain when the real MOOS geodesy library is absent.
