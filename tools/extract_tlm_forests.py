#!/usr/bin/env python3
"""
Extract Swiss tree positions from swissTLM3D and write per-LV95-tile
vegetation point files for the Scenery3D `S3DVegetationManager`.

Inputs
------
* swissTLM3D GeoPackage (free download from swisstopo, EPSG:2056).
* Existing terrain heightmaps (`tile_{E}_{N}.raw`, float32 1024×1024,
  produced by tools/convert_swisstopo.py). These provide ground Z so
  trees can be placed at runtime without an elevation lookup.

Output
------
For each LV95 1024 m tile intersecting forest / isolated-tree features,
one binary file `vegetation_{E}_{N}.bin`:

    header  (32 bytes, little-endian)
        magic[4] = "VEG1"
        uint32   version          (=1)
        uint32   count            (number of trees)
        uint32   reserved         (=0)
        float64  tile_east        (LV95 east of SW corner)
        float64  tile_north       (LV95 north of SW corner)

    records (24 bytes each, repeated `count` times)
        float32  dx               (east offset from SW corner, m)
        float32  dz               (north offset from SW corner, m)
        float32  ground_z         (m ASL, sampled from heightmap)
        float32  height_m         (tree height, m)
        float32  yaw_rad          (random rotation around Y)
        float32  crown_radius_m   (m, ~0.3 * height by default)

Layers used (TLM3D 2024 schema)
-------------------------------
* `tlm_bb_bodenbedeckung` — polygons; filter `objektart` ∈ {`Wald`,
  `Gehoelz`, `Wald_offen`} for forest interiors.
* `tlm_einzelbaum_gebuesch` (or `tlm_einzelbaum`) — points; isolated
  notable trees.

Layer names vary by release; pass --forest-layer / --tree-layer to
override.

Heuristics
----------
* Forest polygons are sampled with Poisson-disk-like distribution at a
  configurable density (default 1 tree / 25 m², matching realistic
  Central-European stand densities of 400–600 trees / ha).
* Tree height is drawn from a clipped normal centred at 22 m for
  forest, 14 m for isolated trees.
* Conifer / deciduous classification: random per tile (TLM3D doesn't
  carry species; later you can layer in BfS / WSL data if available).

Requires: `pip install geopandas shapely numpy pyproj rasterio`.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    import geopandas as gpd
    from shapely.geometry import box as shp_box, Point
    from shapely.prepared import prep
    from shapely.strtree import STRtree
except ImportError:
    print("Error: geopandas + shapely required. Install with:\n"
          "  pip install geopandas shapely", file=sys.stderr)
    sys.exit(1)

LV95_CRS = "EPSG:2056"
TILE_SIZE = 1024
HEIGHTMAP_SAMPLES = 1024  # one sample per metre

VEG_MAGIC = b"VEG1"
VEG_VERSION = 1
HEADER_FMT = "<4sIII dd"          # 32 bytes
RECORD_FMT = "<ffffff"             # 24 bytes


# ── heightmap lookup ────────────────────────────────────────────────────────

class HeightmapDB:
    """Lazy loader for terrain heightmaps. Caches the most recent N tiles."""

    def __init__(self, dirs: list[Path], cache_size: int = 32):
        self.dirs = [Path(d).expanduser().resolve() for d in dirs]
        self.cache: dict[tuple[int, int], np.ndarray | None] = {}
        self.cache_size = cache_size

    def _load(self, e: int, n: int) -> np.ndarray | None:
        for d in self.dirs:
            p = d / f"tile_{e}_{n}.raw"
            if p.is_file():
                arr = np.fromfile(p, dtype=np.float32)
                if arr.size != HEIGHTMAP_SAMPLES * HEIGHTMAP_SAMPLES:
                    print(f"  WARN: {p.name} has {arr.size} samples", file=sys.stderr)
                    return None
                # The .raw layout is row 0 = south, col 0 = east (see
                # convert_lgl_bw.py header). Reshape and remember that.
                return arr.reshape(HEIGHTMAP_SAMPLES, HEIGHTMAP_SAMPLES)
        return None

    def get(self, e: int, n: int) -> np.ndarray | None:
        key = (e, n)
        if key in self.cache:
            return self.cache[key]
        arr = self._load(e, n)
        if len(self.cache) >= self.cache_size:
            self.cache.pop(next(iter(self.cache)))
        self.cache[key] = arr
        return arr

    def sample(self, east: float, north: float) -> float | None:
        """Bilinear sample at LV95 (east, north). Returns None if no tile."""
        te = int(math.floor(east / TILE_SIZE)) * TILE_SIZE
        tn = int(math.floor(north / TILE_SIZE)) * TILE_SIZE
        arr = self.get(te, tn)
        if arr is None:
            return None
        # Local coordinates inside tile.
        lx = east - te        # 0..1024 east-from-west
        ly = north - tn       # 0..1024 north-from-south
        # arr[row][col]: row 0 = south, col 0 = east.
        col_f = (TILE_SIZE - 1) - lx  # east-from-east → col index (col 0 = east)
        row_f = ly                     # north-from-south → row index (row 0 = south)
        c = int(np.clip(col_f, 0, HEIGHTMAP_SAMPLES - 1))
        r = int(np.clip(row_f, 0, HEIGHTMAP_SAMPLES - 1))
        v = float(arr[r, c])
        return v if math.isfinite(v) and v != 0.0 else None


# ── point generation ────────────────────────────────────────────────────────

def poisson_disk_samples_in_bbox(
    minx: float, miny: float, maxx: float, maxy: float,
    radius: float,
    rng: np.random.Generator,
    k: int = 20,
) -> np.ndarray:
    """Bridson Poisson-disk sampling on a 2D rectangle. Returns Nx2 array."""
    cell = radius / math.sqrt(2.0)
    gw = max(1, int(math.ceil((maxx - minx) / cell)))
    gh = max(1, int(math.ceil((maxy - miny) / cell)))
    grid: list[list[tuple[float, float] | None]] = [[None] * gh for _ in range(gw)]

    samples: list[tuple[float, float]] = []
    active: list[int] = []

    def gridpos(p):
        return (int((p[0] - minx) / cell), int((p[1] - miny) / cell))

    def fits(p):
        gx, gy = gridpos(p)
        for ix in range(max(0, gx - 2), min(gw, gx + 3)):
            for iy in range(max(0, gy - 2), min(gh, gy + 3)):
                q = grid[ix][iy]
                if q is not None:
                    if (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 < radius * radius:
                        return False
        return True

    # Seed.
    p0 = (rng.uniform(minx, maxx), rng.uniform(miny, maxy))
    samples.append(p0)
    active.append(0)
    gx, gy = gridpos(p0)
    grid[gx][gy] = p0

    while active:
        i = rng.integers(0, len(active))
        idx = active[i]
        base = samples[idx]
        found = False
        for _ in range(k):
            theta = rng.uniform(0.0, 2.0 * math.pi)
            r = rng.uniform(radius, 2.0 * radius)
            cand = (base[0] + r * math.cos(theta), base[1] + r * math.sin(theta))
            if not (minx <= cand[0] < maxx and miny <= cand[1] < maxy):
                continue
            if fits(cand):
                samples.append(cand)
                active.append(len(samples) - 1)
                gx, gy = gridpos(cand)
                grid[gx][gy] = cand
                found = True
                break
        if not found:
            active.pop(i)

    return np.asarray(samples, dtype=np.float64) if samples else np.zeros((0, 2))


def sample_forest_points(
    forest_geoms: list,
    tile_minx: float, tile_miny: float, tile_maxx: float, tile_maxy: float,
    spacing_m: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Generate tree points inside the union of forest polygons within tile."""
    candidates = poisson_disk_samples_in_bbox(
        tile_minx, tile_miny, tile_maxx, tile_maxy, spacing_m, rng
    )
    if candidates.size == 0:
        return candidates

    keep = np.zeros(len(candidates), dtype=bool)
    for g in forest_geoms:
        prepared = prep(g)
        for i, (x, y) in enumerate(candidates):
            if not keep[i] and prepared.contains(Point(x, y)):
                keep[i] = True
    return candidates[keep]


# ── tile writer ─────────────────────────────────────────────────────────────

def write_tile(
    out_path: Path,
    tile_east: int,
    tile_north: int,
    points: np.ndarray,        # Nx2 (LV95 east, north)
    heights: np.ndarray,       # N
    ground_zs: np.ndarray,     # N (m ASL, may be NaN)
    yaws: np.ndarray,          # N (rad)
    crown_radii: np.ndarray,   # N (m)
):
    valid = np.isfinite(ground_zs)
    if not valid.any():
        return 0

    points = points[valid]
    heights = heights[valid]
    ground_zs = ground_zs[valid]
    yaws = yaws[valid]
    crown_radii = crown_radii[valid]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(struct.pack(
            HEADER_FMT,
            VEG_MAGIC, VEG_VERSION, len(points), 0,
            float(tile_east), float(tile_north),
        ))
        rec = struct.Struct(RECORD_FMT)
        for (e, n), h, gz, yaw, cr in zip(points, heights, ground_zs, yaws, crown_radii):
            f.write(rec.pack(
                float(e - tile_east),
                float(n - tile_north),
                float(gz),
                float(h),
                float(yaw),
                float(cr),
            ))
    return len(points)


# ── main ────────────────────────────────────────────────────────────────────

FOREST_OBJEKTART = {"Wald", "Gehoelz", "Wald_offen", "Wald offen"}


def iter_tiles(geom_bounds: tuple[float, float, float, float]) -> Iterable[tuple[int, int]]:
    minx, miny, maxx, maxy = geom_bounds
    e0 = int(math.floor(minx / TILE_SIZE)) * TILE_SIZE
    n0 = int(math.floor(miny / TILE_SIZE)) * TILE_SIZE
    e1 = int(math.floor((maxx - 1) / TILE_SIZE)) * TILE_SIZE
    n1 = int(math.floor((maxy - 1) / TILE_SIZE)) * TILE_SIZE
    for n in range(n0, n1 + 1, TILE_SIZE):
        for e in range(e0, e1 + 1, TILE_SIZE):
            yield e, n


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--tlm-gpkg", required=True,
                   help="Path to swissTLM3D GeoPackage.")
    p.add_argument("--forest-layer", default="tlm_bb_bodenbedeckung",
                   help="Layer with forest polygons (default: tlm_bb_bodenbedeckung).")
    p.add_argument("--tree-layer", default="tlm_einzelbaum_gebuesch",
                   help="Layer with isolated trees (default: tlm_einzelbaum_gebuesch).")
    p.add_argument("--alti-dir", action="append", required=True,
                   help="Directory with tile_{E}_{N}.raw heightmaps. Repeatable.")
    p.add_argument("--output", required=True,
                   help="Output directory for vegetation_{E}_{N}.bin files.")
    p.add_argument("--extent", type=float, nargs=4,
                   metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                   help="LV95 bbox to process (default: full TLM extent).")
    p.add_argument("--forest-spacing", type=float, default=5.0,
                   help="Min spacing between trees inside forests, m "
                        "(default 5 m → ~400 trees/ha).")
    p.add_argument("--forest-mean-height", type=float, default=22.0)
    p.add_argument("--forest-height-sd", type=float, default=4.0)
    p.add_argument("--isolated-mean-height", type=float, default=14.0)
    p.add_argument("--isolated-height-sd", type=float, default=3.0)
    p.add_argument("--seed", type=int, default=20260501)
    args = p.parse_args()

    print(f"Reading forest polygons from {args.forest_layer} ...", flush=True)
    forests = gpd.read_file(args.tlm_gpkg, layer=args.forest_layer)
    if "objektart" in forests.columns:
        forests = forests[forests["objektart"].isin(FOREST_OBJEKTART)]
    forests = forests.to_crs(LV95_CRS)
    print(f"  {len(forests)} forest polygons", flush=True)

    print(f"Reading isolated trees from {args.tree_layer} ...", flush=True)
    try:
        trees = gpd.read_file(args.tlm_gpkg, layer=args.tree_layer)
        trees = trees.to_crs(LV95_CRS)
    except Exception as e:
        print(f"  (no isolated tree layer: {e})", flush=True)
        trees = gpd.GeoDataFrame(geometry=[], crs=LV95_CRS)
    print(f"  {len(trees)} isolated trees", flush=True)

    if args.extent:
        bbox = tuple(args.extent)
        forests = forests.cx[bbox[0]:bbox[2], bbox[1]:bbox[3]]
        if len(trees):
            trees = trees.cx[bbox[0]:bbox[2], bbox[1]:bbox[3]]
    elif not forests.empty:
        bbox = tuple(forests.total_bounds)
    else:
        print("Nothing to process.", file=sys.stderr)
        return 1

    print(f"LV95 extent: {bbox}", flush=True)

    # Spatial index of forest polygons for fast tile-vs-forest lookup.
    forest_geoms = list(forests.geometry.values)
    forest_index = STRtree(forest_geoms)

    # Spatial index of isolated trees keyed by point.
    tree_geoms = list(trees.geometry.values) if len(trees) else []
    tree_index = STRtree(tree_geoms) if tree_geoms else None

    heightmap = HeightmapDB(args.alti_dir)
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)

    tiles_done = trees_total = tiles_with_data = 0
    for tile_e, tile_n in iter_tiles(bbox):
        tiles_done += 1
        tile_box = shp_box(tile_e, tile_n, tile_e + TILE_SIZE, tile_n + TILE_SIZE)

        # Forest polygons that touch this tile, intersected to the tile bbox.
        idxs = forest_index.query(tile_box)
        local_forests = []
        for i in idxs:
            g = forest_geoms[int(i)]
            if not g.intersects(tile_box):
                continue
            clipped = g.intersection(tile_box)
            if clipped.is_empty:
                continue
            local_forests.append(clipped)

        forest_pts = (sample_forest_points(local_forests,
                                           tile_e, tile_n,
                                           tile_e + TILE_SIZE, tile_n + TILE_SIZE,
                                           args.forest_spacing, rng)
                      if local_forests else np.zeros((0, 2)))

        isolated_pts = np.zeros((0, 2))
        if tree_index is not None:
            ti_idxs = tree_index.query(tile_box)
            xs, ys = [], []
            for i in ti_idxs:
                g = tree_geoms[int(i)]
                if g.geom_type != "Point":
                    g = g.centroid
                if tile_box.contains(g):
                    xs.append(g.x); ys.append(g.y)
            if xs:
                isolated_pts = np.column_stack([xs, ys])

        if forest_pts.size == 0 and isolated_pts.size == 0:
            continue

        # Heights, yaws, crown radii.
        f_h = np.clip(rng.normal(args.forest_mean_height,
                                 args.forest_height_sd,
                                 len(forest_pts)),
                      4.0, 45.0)
        i_h = np.clip(rng.normal(args.isolated_mean_height,
                                 args.isolated_height_sd,
                                 len(isolated_pts)),
                      4.0, 35.0)

        all_pts = np.vstack([forest_pts, isolated_pts]) if (
            forest_pts.size and isolated_pts.size
        ) else (forest_pts if forest_pts.size else isolated_pts)
        all_h = np.concatenate([f_h, i_h])
        yaws = rng.uniform(0.0, 2.0 * math.pi, len(all_pts)).astype(np.float32)
        crown = (all_h * 0.30).astype(np.float32)

        # Sample ground Z from heightmaps.
        gz = np.array([heightmap.sample(x, y) or float("nan")
                       for x, y in all_pts], dtype=np.float32)

        n_written = write_tile(
            output_dir / f"vegetation_{tile_e}_{tile_n}.bin",
            tile_e, tile_n,
            all_pts.astype(np.float64), all_h.astype(np.float32),
            gz, yaws, crown,
        )
        if n_written:
            tiles_with_data += 1
            trees_total += n_written
        if tiles_done % 50 == 0:
            print(f"  {tiles_done} tiles processed, "
                  f"{tiles_with_data} with data, "
                  f"{trees_total} trees written", flush=True)

    print(f"Done. {tiles_with_data} tiles, {trees_total} trees → {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
