# Scenery3D

A flight-simulator-optimized terrain system for Godot 4.6+, built as a C++ GDExtension.

Scenery3D replaces Terrain3D for flight simulation use cases, solving the key architectural problems that cause visible pauses during flight:

- **Double-precision Godot** — designed to run on a Godot fork with `precision = "double"`, eliminating the need for floating-origin rebasing entirely
- **Async tile streaming** — all terrain I/O and mesh generation happens on background threads; the main thread only does sub-millisecond tile swaps
- **Per-tile independence** — tiles are added and removed individually, with no global rebuild step
- **O(1) elevation queries** — bilinear interpolation over loaded heightmaps, shared between rendering and flight dynamics

## Architecture

```
Scenery3D (Node3D)              — Main node, configuration and coordination
├── S3DTileManager (Node3D)     — Background thread pool, tile lifecycle
│   └── S3DTile (MeshInstance3D)— Individual terrain tile with heightmap mesh
└── S3DElevationDB (RefCounted) — Fast elevation lookup across all loaded tiles
```

## Data Sources

Designed for SwissTopo data:
- **Terrain elevation:** swissALTI3D (0.5m resolution, R16 format)
- **Orthophotos:** SWISSIMAGE (JPEG tiles)
- **Buildings:** swissBUILDINGS3D (GLB)

Tile size: 1024m x 1024m, aligned to Swiss LV95 coordinate grid.

## Building

Requires the godot-cpp submodule (included) and SCons.

```bash
# macOS (Apple Silicon, debug)
scons platform=macos target=template_debug arch=arm64

# macOS (release)
scons platform=macos target=template_release arch=arm64
```

Output: `demo/bin/libscenery3d.macos.template_{debug,release}.arm64.dylib`

## Godot Engine Fork

Scenery3D is designed to work with a double-precision Godot build:

```bash
cd godot
# custom.py sets: precision="double", metal=True, vulkan=False, optimize="speed", lto="thin"
scons platform=macos target=editor arch=arm64
```

Branch `4.6-flightsim` on `git@gitlab.md80.ch:gery/godot.git`.

## Related Projects

- **godotjbsim** — JSBSim flight dynamics integration for Godot
- **ProVPilot** — Full flight simulator (uses Terrain3D, predecessor architecture)
