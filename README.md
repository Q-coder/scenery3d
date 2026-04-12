# Scenery3D

A flight-simulator-optimized terrain system for Godot 4.6+, built as a C++ GDExtension.

Scenery3D replaces Terrain3D for flight simulation use cases, solving the key architectural problems that cause visible pauses during flight:

- **Double-precision Godot** — designed to run on a Godot fork with `precision = "double"`, eliminating the need for floating-origin rebasing entirely
- **Async tile streaming** — all terrain I/O happens on background threads (4-thread pool); the main thread only does budgeted mesh generation
- **Distance-based LOD** — tiles get progressively coarser with distance, enabling 50km+ visibility with manageable vertex counts
- **Far terrain chunks** — beyond the individual tile radius, 8×8 tile groups are composited into low-res meshes for 200km+ visibility (Alps panorama)
- **Seamless handoffs** — chunks and tiles overlap during loading; nothing is removed until its replacement is fully loaded
- **Per-tile independence** — tiles are added and removed individually, with no global rebuild step
- **O(1) elevation queries** — bilinear interpolation over loaded heightmaps, shared between rendering and flight dynamics

## Architecture

```
Scenery3D (Node3D)              — Main node, configuration and coordination
├── S3DTileManager (Node3D)     — Background thread pool, tile lifecycle, LOD rings
│   └── S3DTile (MeshInstance3D)— Individual terrain tile with heightmap mesh
├── S3DElevationDB (RefCounted) — Fast elevation lookup across all loaded tiles
└── S3DCoords (RefCounted)      — WGS84 ↔ LV95/CH1903+ ↔ Godot world conversion
```

### LOD Ring System

Tiles are assigned LOD levels based on Chebyshev distance from the camera tile:

| Distance (tiles) | LOD Level | Vertices per tile | Use case |
|-------------------|-----------|-------------------|----------|
| ≤ 3               | 3         | 129 × 129         | Close detail |
| ≤ 8               | 4         | 65 × 65           | Medium distance |
| ≤ 25              | 6         | 17 × 17           | Far terrain |
| ≤ load_radius     | 8         | 5 × 5             | Horizon silhouettes |

Vertex budget: 200K vertices/frame to prevent stalls. Distant tiles discard their
heightmap after meshing to save memory.

### Far Terrain Chunks

Beyond `load_radius`, terrain is rendered as low-resolution chunks. Each chunk
composites an 8×8 group of tiles into a single 33×33 vertex mesh, covering
8192×8192 meters. This extends visibility to `far_radius` (default 200 tiles = 200km).

Chunk lifecycle uses seamless handoffs:
- A chunk is only removed when ALL individual tiles covering it are fully loaded
- An individual tile beyond `load_radius` is only removed when its covering chunk is loaded
- Hysteresis margin (`unload_margin`, default 2 tiles) prevents thrashing at boundaries

### Coordinate Convention

Matches the ProVPilot terrain convention:

```
Godot X = origin_east - LV95_East     (+X = West)
Godot Z = LV95_North - origin_north   (+Z = North)
Godot Y = Elevation ASL (meters)
```

Default origin: E=2,600,000 / N=1,200,000 (Bern area).

## Data Sources

Designed for SwissTopo data:
- **Terrain elevation:** swissALTI3D (0.5m resolution)
- **Orthophotos:** SWISSIMAGE (JPEG tiles, planned)
- **Buildings:** swissBUILDINGS3D (GLB, planned)

### Terrain Tile Format

Headerless 1024×1024 float32 little-endian raw files (4 MB each).
Naming: `tile_{lv95_east}_{lv95_north}.raw`

Pixel(0,0) = SE corner (cols run E→W, rows run S→N). This matches the
ProVPilot conversion from SwissTopo GeoTIFFs (flipud + fliplr applied).

Pre-converted tiles for all of Switzerland (~42,400 files) are available
from the ProVPilot scenery pipeline.

## Building

Requires the godot-cpp submodule and SCons.

The extension must be built with `precision=double` to match the Godot editor:

```bash
# macOS (Apple Silicon, debug)
scons precision=double arch=arm64 -j$(sysctl -n hw.ncpu)
```

Output: `demo/bin/libscenery3d.macos.template_debug.double.arm64.dylib`

To regenerate the API dump from a new Godot build:

```bash
godot.macos.editor.double.arm64 --headless --dump-extension-api
cp extension_api.json gdextension/extension_api.double.json
```

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
