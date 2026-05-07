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
├── S3DBuildingManager (Node3D) — Background GLB loading, LOD, per-building visibility
├── S3DVegetationManager (Node3D) — Per-tile forest streaming via MultiMesh instancing
├── S3DWaterManager (Node3D)    — Water surface tiles (rivers, lakes, streams)
├── S3DRoadManager (Node3D)     — Road and railway tiles with lane markings
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

Chunk compositing block-averages raw source pixels into each of the 33×33 cells
and treats the tile's zero pixels as nodata, gap-filling them first — so a
chunk is classified as empty only when literally every underlying pixel is zero.

Chunk lifecycle:
- A chunk is dropped only once every individual tile in its footprint that is
  inside `load_radius` has finished loading (or is marked `no_data`). This
  prevents a hole in the world while the streamer is still catching up after
  the camera moves into a new chunk's area.
- An individual tile beyond `load_radius` is only removed when its covering chunk
  is loaded.
- Chunk meshes are rendered at a constant Y offset (`CHUNK_Y_BIAS = -30 m`) so
  wherever they overlap an individual tile the near terrain wins the depth test.
  This eliminates ridgeline Z-fighting caused by the 250 m chunk sampling poking
  above the 1 m tile mesh.
- Hysteresis margin (`unload_margin`, default 2 tiles) prevents thrashing at the
  far-range boundary.

### Main-Thread Budgets

Crossing into a new chunk can demand 64 fresh tiles at once, each requiring a
JPEG decode and texture upload on the main thread. Three caps spread that work
across many frames so the simulation never stalls:

- `VERTEX_BUDGET_PER_FRAME = 200000` — mesh build cost.
- `ORTHO_DECODES_PER_FRAME = 1` — tile results that include a JPEG ortho are
  deferred once the per-frame quota is hit. Kept intentionally conservative
  on macOS/Metal to avoid `timeout waiting for fence` upload stalls.
- `CHUNKS_PER_FRAME = 1` — each chunk decodes up to `chunk_size²` mip3 JPEGs
  and builds a composite texture, so one per frame is plenty.

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
- **Orthophotos:** SWISSIMAGE (JPEG tiles, 1m/px full res with mip levels for LOD)
- **Buildings:** swissBUILDINGS3D 3.0 (converted to GLB per tile)
- **Vegetation:** swissTLM3D forest cover and isolated trees (converted to per-tile point clouds)
- **Water:** swissTLM3D rivers, lakes, and streams (ribbon meshes with vertex colors)
- **Roads:** swissTLM3D roads and railways (ribbon meshes with white lane markings)

### Orthophoto Support

Aerial imagery is loaded as per-tile JPEG textures mapped via UV coordinates:

| LOD Level | Texture Source | Resolution | Use case |
|-----------|---------------|------------|----------|
| 3 (close) | mip0 | 1024×1024 px (1 m/px) | Close-up detail |
| 4+ (far) | mip2 | 64×64 px (16 m/px) | Distant tiles |
| Chunks | mip3 composite | 128×128 px (8×8 tiles) | Far terrain (200km+) |

Worker threads read JPEG files alongside heightmaps. Tiles without orthophotos
fall back to flat green. Chunk composites are assembled from 16×16 px mip3 tiles
using `blit_rect` on the main thread.

Naming: `ortho_{lv95_east}_{lv95_north}.jpg` with mip subdirectories
(`mip2/`, `mip3/`, `mip4/`) and a `manifest.json` index.

**Multi-region support.** The `orthophoto_paths` PackedStringArray accepts
multiple roots (e.g. one for SWISSIMAGE and one for LGL DOP20). The tile
manager probes them in order and uses the first JPEG it finds for each LV95
tile, so Switzerland and Baden-Württemberg can be served from independent
directories without renaming files. The legacy singular `orthophoto_path`
property is still honoured and is treated as a one-element array.

### Terrain Tile Format

Headerless 1024×1024 float32 little-endian raw files (4 MB each).
Naming: `tile_{lv95_east}_{lv95_north}.raw`

Pixel(0,0) = SE corner (cols run E→W, rows run S→N). This matches the
ProVPilot conversion from SwissTopo GeoTIFFs (flipud + fliplr applied).

Pre-converted tiles for all of Switzerland (~42,400 files) are available
from the ProVPilot scenery pipeline.

### Building Tile Format

Each building tile is a GLB file containing:
- One glTF node per building (named by UUID), each with 4 primitives:
  `wall_detail`, `roof_detail`, `wall_box`, `roof_box`
- One `_far_lod` node with 2 primitives: merged `wall` + `roof` boxes for all buildings

Naming: `buildings_{tile_id}.glb` with a `manifest.json` index.

Faces are split into **wall** (normal Y ≤ 0.5) and **roof** (normal Y > 0.5) for
separate coloring. All geometry uses flat normals (3 unique vertices per triangle)
and outward-oriented faces.

#### Building LOD System

| Distance        | Rendering                                  |
|-----------------|--------------------------------------------|
| ≤ detail_radius | Individual building meshes (hide/show per UUID) |
| > detail_radius | Single merged mesh per tile (detail geometry) |

Far tiles only create 1 MeshInstance3D each (merged mesh). Individual building
meshes are freed when leaving detail range, reclaiming GPU RIDs. When
re-entering detail range, the merged mesh stays visible as a placeholder until
the detail meshes finish loading (no blink).

### Water Tile Format

GLB files with a single mesh containing POSITION, NORMAL, COLOR_0, and indices.
Vertex colors distinguish rivers (blue), lakes (darker blue), and streams (lighter).

- Rivers use Catmull-Rom resampled ribbon meshes for smooth curves
- Streams overlapping river polygons are filtered via STRtree spatial indexing
- `vertical_offset_m = 1.5` lifts water above terrain to prevent z-fighting
- `far_radius_m = 50000` for tiles with lake polygons (visible at distance)
- `load_radius_m = 12000` for stream-only tiles

Naming: `water_{lv95_east}_{lv95_north}.glb` with `manifest.json`.

### Road and Railway Tile Format

GLB files with POSITION, NORMAL, COLOR_0, and indices. Vertex colors encode
road classification:

| Type | Color | Width |
|------|-------|-------|
| Motorway | Dark blue-grey | 14 m |
| Main road | Medium grey | 5–10 m |
| Secondary | Light grey | 3–6 m |
| Minor road | Warm light grey | 1.5–4 m |
| Railway | Dark brown | 2–4 m |

Visual enhancements:
- **White edge stripes** (24 cm wide) on motorway, main, and secondary roads
- **White center line** on motorways (median marker)
- **Silver rail lines** on railways (two parallel rails at 1.44 m gauge)
- Stripe geometry offset by 5 cm above road surface to avoid z-fighting

Naming: `roads_{lv95_east}_{lv95_north}.glb` with `manifest.json`.

### Vegetation Tile Format

Vegetation is streamed as one binary point file per LV95 tile and rendered as
`MultiMeshInstance3D` using a shared tree mesh:

- File name: `vegetation_{lv95_east}_{lv95_north}.bin`
- Header: tile origin plus record count
- Per-tree record: local east/north offset, sampled ground elevation, tree
  height, random yaw, and crown radius

`S3DVegetationManager` loads these files on background threads and instantiates
them tile-by-tile around the camera. The demo scene uses `demo/vegetation_setup.gd`
to apply foliage-safe materials to the imported tree mesh: two-sided alpha
scissor rendering, a small emissive lift for readability, and
`FLAG_DONT_RECEIVE_SHADOWS` so dense tree cards do not go unnaturally dark over
loaded orthophotos. The current demo also disables tree shadow casting on the
vegetation `MultiMeshInstance3D` as a temporary visual workaround.

#### Vegetation Extraction

`tools/extract_tlm_forests.py` converts swissTLM3D forest polygons and isolated
tree layers into the per-tile vegetation format. It samples ground elevation
from existing `tile_{E}_{N}.raw` heightmaps so runtime placement does not need
an elevation lookup.

```bash
python3 tools/extract_tlm_forests.py \
  --tlm-gpkg /path/to/swissTLM3D.gpkg \
  --alti-dir /path/to/scenery/terrain \
  --output /path/to/scenery/vegetation
```

#### Conversion Script

`scripts/convert_buildings.py` downloads swissBUILDINGS3D 3.0 tiles from the
SwissTopo STAC API and converts them to GLB:

```bash
# Convert a region (LV95 bounding box)
python3 scripts/convert_buildings.py --download \
  --bbox 2595000 1195000 2610000 1210000 \
  -o /path/to/buildings/

# Convert all of Switzerland
python3 scripts/convert_buildings.py --download \
  --bbox 2485000 1074000 2834000 1296000 \
  -o /path/to/buildings/ --cache /path/to/cache/
```

The script requires `fiona`, `numpy`, `requests`, and `pyproj`.
Full Switzerland: ~3,218 tiles, ~2.5M buildings, ~300 MB GLB.

#### Baden-Württemberg Terrain

`tools/convert_lgl_bw.py` converts LGL Baden-Württemberg DGM GeoTIFFs (EPSG:25832)
to the scenery3d tile format (LV95, headerless float32). Handles NaN-aware merging
for border tiles where Swiss and German data overlap. Tiles whose real-source
coverage is below `--min-valid-fraction` (default 0.5) are skipped rather than
being streak-filled from a few pixels, which prevents visible seams against
fully-covered neighbours.

`tools/xyz_to_tif.py` is a helper for bulk pre-conversion of the raw LGL ASCII
XYZ source files to GeoTIFF before running `convert_lgl_bw.py`. This avoids the
high open cost of GDAL's XYZ driver when scanning very large source corpora.

```bash
python3 tools/convert_lgl_bw.py \
  --input /path/to/bw_dgm/ \
  --output /path/to/scenery/Germany/Baden-Wuertemberg/terrain \
  --src-crs EPSG:25832
```

```bash
python3 tools/xyz_to_tif.py \
  --input /path/to/bw_dgm_xyz/ \
  --workers 8
```

`tools/download_lgl_bw.py` automates the full pipeline: HEAD-probe the LGL Open
GeoData Portal, download every DGM1 ZIP in a UTM32 bbox, extract the XYZ files
and invoke the converter. Idempotent — already-present ZIPs and extracted
tiles are skipped.

```bash
python3 tools/download_lgl_bw.py \
  --download-dir /path/to/scenery_in \
  --output       /path/to/scenery/Germany/Baden-Wuertemberg/terrain
```

`tools/find_missing_bw_tiles.py` finds gaps in an existing BW terrain directory
and maps them back to the LGL 2 km source cells that would cover them. Useful
after you've deleted the source data to identify exactly which ZIPs still need
to be re-downloaded. Writes a cell list that `download_lgl_bw.py --cells-file`
can consume directly. Also has a `--check-quality` mode that flags (and
optionally deletes) existing `.raw` tiles whose non-zero pixel coverage is
below a threshold — these are the partial tiles that cause streaking seams in
the rendered terrain.

```bash
# 1. Delete bad / streaky tiles
python3 tools/find_missing_bw_tiles.py \
  --terrain-dir /path/to/terrain \
  --shrink 2 --check-quality --delete-bad

# 2. Find gaps and probe the LGL server
python3 tools/find_missing_bw_tiles.py \
  --terrain-dir /path/to/terrain \
  --shrink 2 --probe-server --output-cells /tmp/missing.txt

# 3. Re-download only the missing cells
python3 tools/download_lgl_bw.py \
  --cells-file /tmp/missing.txt \
  --download-dir /path/to/scenery_in \
  --output      /path/to/terrain
```

#### Baden-Württemberg Orthophoto

`tools/convert_lgl_dop.py` converts LGL Baden-Württemberg DOP20 RGB GeoTIFFs
(20 cm resolution, EPSG:25832, delivered as 2 km × 2 km tiles) to the
scenery3d ortho tile format on the LV95 1024 m grid. For every covered LV95
tile it emits a 1024×1024 JPEG (mip0, 1 m/px) plus 64×64 (mip2) and 16×16
(mip3) downsamples, matching the SWISSIMAGE pipeline. The same
`--min-valid-fraction` filter as the DGM converter prevents streaky edge tiles.

```bash
python3 tools/convert_lgl_dop.py \
  --input  /path/to/bw_dop20/ \
  --output /path/to/scenery/Germany/Baden-Wuertemberg/orthophoto \
  --src-crs EPSG:25832
```

`tools/download_lgl_dop.py` automates the full pipeline: HEAD-probe the LGL
Open GeoData Portal for available 2 km DOP20 cells in a UTM32 bbox, download
them in parallel, extract the GeoTIFFs and invoke the converter. Like the DGM
downloader it is idempotent.

```bash
python3 tools/download_lgl_dop.py \
  --download-dir /path/to/scenery_in_dop \
  --output       /path/to/scenery/Germany/Baden-Wuertemberg/orthophoto
```

DOP20 source data is large (~225 MB per 2 km ZIP, full BW ~1.6 TB); plan
storage accordingly or restrict via `--extent`.

`tools/process_lgl_dop.py` is the streaming alternative to the
extract-then-convert pipeline. It reads each TIFF straight out of the ZIP via
GDAL's `/vsizip/` virtual filesystem, so it never extracts to disk. With
`--delete-zips` it removes each ZIP as soon as every LV95 tile it covers has
been written, which lets you process the full BW corpus on a disk that
doesn't have room to hold all the source data at once.

```bash
python3 tools/process_lgl_dop.py \
  --zip-dir /Volumes/Data1/scenery_in_dop \
  --output  /Volumes/Data1/scenery/Germany/Baden-Wuertemberg/orthophoto \
  --jobs 8 --delete-zips
```

The tool is resumable: re-running on a partially-deleted ZIP set just covers
the remaining area, and existing JPEGs are skipped unless `--overwrite` is
given. Safe to run alongside an in-progress `download_lgl_dop.py`.

License: dl-de/by-2-0 — required attribution: *“Datenquelle: LGL,
www.lgl-bw.de, dl-de/by-2-0”*.

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
