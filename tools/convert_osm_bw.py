#!/usr/bin/env python3
"""
Convert an OpenStreetMap PBF for Baden-Württemberg into Scenery3D
road and water GLB tile sets.

Pipeline
--------
1. Stream the PBF with pyosmium, collecting:
     * highway= linestrings (with class + tags),
     * waterway=river|stream|canal|ditch linestrings,
     * water polygons (natural=water, landuse=reservoir|basin) and
       multipolygon relations of the same.
2. Reproject all coordinates from WGS84 (EPSG:4326) to LV95 (EPSG:2056)
   so that the BW dataset shares the same Cartesian frame as the
   existing Swiss tiles.
3. Bin features into a 4096-m grid in LV95 coordinates, splitting
   linestrings at tile boundaries and clipping polygons via
   Sutherland-Hodgman.
4. For each tile:
       roads:  extrude every road centerline into a flat ribbon
               whose width depends on the highway class, color each
               vertex by class.
       water:  triangulate polygons with mapbox_earcut; treat
               waterway lines as thin ribbons.
   Vertex y is 0 at tile time — the C++ S3DRoadManager / S3DWaterManager
   re-drapes onto the elevation DB at runtime (roads) and applies a
   fixed vertical offset (water).
5. Emit one binary glTF (.glb) per tile containing a single primitive
   with attributes POSITION (vec3 float), NORMAL (vec3 float),
   COLOR_0 (vec4 ubyte normalised), and either UNSIGNED_SHORT or
   UNSIGNED_INT indices.
6. Write a manifest.json listing each tile with its center and counts.

   Vertex coordinate convention (matching the C++ parsers):
    vertex.x = conv_origin_e - east
    vertex.z = north - conv_origin_n
    vertex.y = terrain_elevation + vertical_offset  (when --terrain-dir is given)
               0                                    (legacy, runtime-draped)

OSM data © OpenStreetMap contributors, ODbL 1.0
https://www.openstreetmap.org/copyright

Usage
-----
    python3 tools/convert_osm_bw.py \\
        --pbf /Volumes/Data1/scenery_in/osm_bw/baden-wuerttemberg-latest.osm.pbf \\
        --roads-out  /Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/roads \\
        --water-out  /Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/water \\
        --terrain-dir /Users/gery/provpilot/scenery/Germany/terrain
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import gc
import time
from collections import defaultdict, OrderedDict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    import osmium
except ImportError:
    print("ERROR: pyosmium is required (pip install --user osmium)", file=sys.stderr)
    sys.exit(1)

try:
    from pyproj import Transformer
except ImportError:
    print("ERROR: pyproj is required (pip install --user pyproj)", file=sys.stderr)
    sys.exit(1)

try:
    import mapbox_earcut as earcut
except ImportError:
    print("ERROR: mapbox_earcut is required (pip install --user mapbox_earcut)",
          file=sys.stderr)
    sys.exit(1)

try:
    from shapely.geometry import Polygon, MultiPolygon, Point
    from shapely.prepared import prep
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False


def parse_geofabrik_poly(path: Path):
    """Parse the Geofabrik .poly format and return a shapely (Multi)Polygon
    in WGS84 lon/lat. Returns None if shapely is not available or file is
    empty / unparsable.

    Format (simplified):
        <name>
        <ring_name>          ! prefix means hole
            lon lat
            ...
        END
        ...
        END
    """
    if not HAS_SHAPELY:
        return None
    with open(path, "r") as f:
        toks = [ln.strip() for ln in f if ln.strip()]
    if not toks:
        return None
    i = 1  # skip header name line
    outers: List[List[Tuple[float, float]]] = []
    holes: List[List[Tuple[float, float]]] = []
    while i < len(toks):
        if toks[i].upper() == "END":
            break
        is_hole = toks[i].startswith("!")
        i += 1
        pts: List[Tuple[float, float]] = []
        while i < len(toks) and toks[i].upper() != "END":
            parts = toks[i].split()
            if len(parts) >= 2:
                try:
                    pts.append((float(parts[0]), float(parts[1])))
                except ValueError:
                    pass
            i += 1
        if i < len(toks) and toks[i].upper() == "END":
            i += 1
        if len(pts) >= 3:
            (holes if is_hole else outers).append(pts)
    if not outers:
        return None
    # Pair each outer with any holes it contains; simple approach: build one
    # MultiPolygon where the first outer gets all holes.  Geofabrik BW has a
    # single outer ring, so this is sufficient.
    if len(outers) == 1:
        poly = Polygon(outers[0], holes)
    else:
        poly = MultiPolygon([Polygon(outers[0], holes)]
                            + [Polygon(o) for o in outers[1:]])
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly


# ── Configuration ───────────────────────────────────────────────────────────

TILE_SIZE_M = 4096
# A LV95 origin near the centre of Baden-Württemberg so per-vertex offsets
# stay small. Anywhere reasonable works; the C++ manager picks this up
# from manifest.json["conversion_origin_e"/"_n"].
DEFAULT_CONV_E = 2700000.0
DEFAULT_CONV_N = 1310000.0

# Densify long road segments so the rendered ribbon hugs terrain after
# per-vertex re-drape on the C++ side. OSM road nodes are often 50-200 m
# apart on highways, which causes long ribbons to span hill features and
# z-fight with the heightmap. 25 m is a good balance between mesh size
# and terrain conformance.
# 10 m: small enough to follow rolling terrain and large enough to keep mesh sizes
# reasonable. Lower values give smoother close-range silhouettes at the cost of
# more vertices.
DEFAULT_DENSIFY_M = 10.0
# Cap the miter offset to this multiple of the half-width to avoid huge spikes
# at sharp turns.
MITER_LIMIT = 2.5

# Vertical offset baked into GLB vertex Y (metres above terrain surface).
# Roads sit slightly above terrain to prevent z-fighting; water is raised
# more so it stays visually above the ground even on gentle slopes.
DEFAULT_ROAD_VERTICAL_OFFSET_M = 0.4
DEFAULT_WATER_VERTICAL_OFFSET_M = 1.5

# Road class → (width meters, sRGB tuple 0..1, priority).
# Only proper drivable classes are included. Tracks, paths, footways,
# cycleways, bridleways, steps and pedestrian zones are intentionally
# omitted: OSM coverage of those is extremely dense in rural BW and the
# result obscures the actual road network. Reintroduce them per-class
# with --include-class if needed.
ROAD_CLASSES: Dict[str, Tuple[float, Tuple[float, float, float], int]] = {
    "motorway":       (24.0, (0.55, 0.55, 0.57), 10),
    "motorway_link":  (10.0, (0.55, 0.55, 0.57), 9),
    "trunk":          (16.0, (0.62, 0.60, 0.58), 9),
    "trunk_link":     (8.0,  (0.62, 0.60, 0.58), 8),
    "primary":        (12.0, (0.72, 0.66, 0.58), 8),
    "primary_link":   (6.0,  (0.72, 0.66, 0.58), 7),
    "secondary":      (9.0,  (0.76, 0.72, 0.66), 7),
    "secondary_link": (5.0,  (0.76, 0.72, 0.66), 6),
    "tertiary":       (7.0,  (0.78, 0.74, 0.70), 6),
    "tertiary_link":  (4.0,  (0.78, 0.74, 0.70), 5),
    "unclassified":   (5.0,  (0.78, 0.76, 0.72), 5),
    "residential":    (5.0,  (0.80, 0.78, 0.74), 4),
    "living_street":  (4.0,  (0.82, 0.80, 0.76), 3),
    "service":        (3.5,  (0.74, 0.74, 0.74), 3),
}

# Railway classes rendered alongside roads (same road GLB tiles).
# Drawn as thin dark ribbons. subway/U-Bahn included where visible above ground.
RAILWAY_CLASSES: Dict[str, Tuple[float, Tuple[float, float, float], int]] = {
    "rail":           (3.0, (0.30, 0.30, 0.30), 8),
    "light_rail":     (2.5, (0.38, 0.38, 0.38), 7),
    "tram":           (2.0, (0.42, 0.42, 0.42), 6),
    "subway":         (2.5, (0.28, 0.32, 0.38), 7),
}

# Unified lookup for all road + railway classes.
ALL_ROAD_CLASSES: Dict[str, Tuple[float, Tuple[float, float, float], int]] = {
    **{k: v for k, v in ROAD_CLASSES.items()},
    **{f"railway:{k}": v for k, v in RAILWAY_CLASSES.items()},
}

# Water classes. Streams and ditches are omitted: OSM coverage in BW is
# extremely noisy and the per-vertex drape sits poorly on tight gullies.
WATER_LINE_CLASSES: Dict[str, float] = {
    "river": 12.0,
    "canal": 8.0,
}
WATER_COLOR = (0.28, 0.46, 0.62)


# ── Elevation sampler ───────────────────────────────────────────────────────

class ElevationSampler:
    """Loads scenery3d .raw terrain tiles on demand and returns bilinear-
    interpolated elevation in metres (LV95).

    Tile format: headerless 1024×1024 float32 little-endian, named
    ``tile_{tile_e}_{tile_n}.raw`` where tile_e/tile_n are the LV95
    coordinates of the SW corner (multiples of 1024 m).
    Pixel (0,0) = SE corner; columns run E→W, rows run S→N — the same
    convention used by S3DElevationDB::get_elevation in C++.
    """

    TILE_PX = 1024  # pixels per tile edge

    # Keep at most this many 4 MB terrain tiles in RAM at once (~128 MB).
    MAX_CACHE = 32

    def __init__(self, terrain_dir: str) -> None:
        self._dir = Path(terrain_dir)
        # OrderedDict used as an LRU cache: most-recently-used at the end.
        self._cache: OrderedDict = OrderedDict()
        # Separate set tracks tiles known to be absent so we skip disk I/O.
        self._missing: set = set()
        self.n_miss = 0    # tiles that could not be found on disk

    def _load(self, tile_e: int, tile_n: int) -> Optional[np.ndarray]:
        key = (tile_e, tile_n)
        if key in self._missing:
            return None
        if key in self._cache:
            # Move to end (most-recently-used).
            self._cache.move_to_end(key)
            return self._cache[key]
        path = self._dir / f"tile_{tile_e}_{tile_n}.raw"
        if not path.exists():
            self._missing.add(key)
            self.n_miss += 1
            return None
        data = np.frombuffer(path.read_bytes(), dtype="<f4").reshape(
            self.TILE_PX, self.TILE_PX
        )
        self._cache[key] = data
        # Evict least-recently-used entry when over the limit.
        if len(self._cache) > self.MAX_CACHE:
            self._cache.popitem(last=False)
        return data

    def sample(self, east: float, north: float) -> float:
        """Bilinear elevation at LV95 (east, north). Returns NaN if no tile."""
        px = self.TILE_PX
        tile_e = int(math.floor(east / px)) * px
        tile_n = int(math.floor(north / px)) * px
        data = self._load(tile_e, tile_n)
        if data is None:
            return float("nan")
        # Local coords within tile: local_x grows west→east, local_z south→north.
        local_x = east - tile_e   # [0, TILE_PX)
        local_z = north - tile_n  # [0, TILE_PX)
        # Pixel space: col 0 = east edge, col px-1 = west edge (mirrors C++ code).
        pcol = (1.0 - local_x / px) * (px - 1)
        prow = local_z / px * (px - 1)
        c0 = max(0, min(px - 2, int(pcol)))
        r0 = max(0, min(px - 2, int(prow)))
        c1, r1 = c0 + 1, r0 + 1
        fc, fr = pcol - c0, prow - r0
        h = (
            data[r0, c0] * (1 - fc) * (1 - fr)
            + data[r0, c1] * fc * (1 - fr)
            + data[r1, c0] * (1 - fc) * fr
            + data[r1, c1] * fc * fr
        )
        return float(h)


# ── Geometry helpers ────────────────────────────────────────────────────────

def make_transformer() -> Transformer:
    # always_xy=True so input is (lon, lat) and output is (east, north).
    return Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)


def tile_xy(e: float, n: float, conv_e: float, conv_n: float) -> Tuple[int, int]:
    """Return (tile_east_origin, tile_north_origin) in metres.

    Tile origins are integer multiples of TILE_SIZE_M counted from
    the LV95 origin (not from conv_origin). This keeps BW tiles
    on the same global LV95 grid as CH tiles.
    """
    tx = int(math.floor(e / TILE_SIZE_M))
    tn = int(math.floor(n / TILE_SIZE_M))
    return tx * TILE_SIZE_M, tn * TILE_SIZE_M


def densify_polyline(
    pts: Sequence[Tuple[float, float]],
    max_seg_m: float,
) -> List[Tuple[float, float]]:
    """Insert intermediate vertices so no segment exceeds max_seg_m.

    The road manager re-drapes one Y per ribbon vertex, so denser
    centerlines follow the terrain more closely and avoid z-fighting.
    """
    if max_seg_m <= 0 or len(pts) < 2:
        return list(pts)
    out: List[Tuple[float, float]] = [pts[0]]
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        dx = x1 - x0
        dy = y1 - y0
        d = math.hypot(dx, dy)
        if d > max_seg_m:
            n_sub = int(math.ceil(d / max_seg_m))
            for k in range(1, n_sub):
                t = k / n_sub
                out.append((x0 + dx * t, y0 + dy * t))
        out.append((x1, y1))
    return out


def split_polyline_into_tiles(
    pts: Sequence[Tuple[float, float]],
) -> Dict[Tuple[int, int], List[List[Tuple[float, float]]]]:
    """Split a polyline into sub-polylines that each lie in a single tile.

    Returns {(tile_east, tile_north): [polyline, polyline, ...]} with
    polyline = list of (east, north).
    """
    out: Dict[Tuple[int, int], List[List[Tuple[float, float]]]] = defaultdict(list)
    if len(pts) < 2:
        return out

    cur_tile = tile_xy(*pts[0], 0.0, 0.0)
    cur: List[Tuple[float, float]] = [pts[0]]

    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        # Walk from (x0,y0) to (x1,y1) clipping at tile boundaries.
        seg_start = (x0, y0)
        while True:
            dx = x1 - seg_start[0]
            dy = y1 - seg_start[1]
            tx, tn = cur_tile
            # Tile bounds.
            xa, xb = tx, tx + TILE_SIZE_M
            ya, yb = tn, tn + TILE_SIZE_M
            # Find smallest t in (0,1] where the segment exits the tile.
            t_best = 1.0
            exit_dir = None  # 'x+','x-','y+','y-'
            eps = 1e-9
            if dx > eps:
                t = (xb - seg_start[0]) / dx
                if t < t_best:
                    t_best, exit_dir = t, "x+"
            elif dx < -eps:
                t = (xa - seg_start[0]) / dx
                if t < t_best:
                    t_best, exit_dir = t, "x-"
            if dy > eps:
                t = (yb - seg_start[1]) / dy
                if t < t_best:
                    t_best, exit_dir = t, "y+"
            elif dy < -eps:
                t = (ya - seg_start[1]) / dy
                if t < t_best:
                    t_best, exit_dir = t, "y-"

            if exit_dir is None or t_best >= 1.0 - 1e-12:
                cur.append((x1, y1))
                seg_start = (x1, y1)
                break

            cut = (seg_start[0] + dx * t_best, seg_start[1] + dy * t_best)
            cur.append(cut)
            if len(cur) >= 2:
                out[cur_tile].append(cur)
            # Advance to the neighbouring tile.
            if exit_dir == "x+":
                cur_tile = (cur_tile[0] + TILE_SIZE_M, cur_tile[1])
                cut = (cur_tile[0], cut[1])
            elif exit_dir == "x-":
                cur_tile = (cur_tile[0] - TILE_SIZE_M, cur_tile[1])
                cut = (cur_tile[0] + TILE_SIZE_M, cut[1])
            elif exit_dir == "y+":
                cur_tile = (cur_tile[0], cur_tile[1] + TILE_SIZE_M)
                cut = (cut[0], cur_tile[1])
            else:  # y-
                cur_tile = (cur_tile[0], cur_tile[1] - TILE_SIZE_M)
                cut = (cut[0], cur_tile[1] + TILE_SIZE_M)
            cur = [cut]
            seg_start = cut

    if len(cur) >= 2:
        out[cur_tile].append(cur)
    return out


def sutherland_hodgman(
    poly: Sequence[Tuple[float, float]],
    xa: float, ya: float, xb: float, yb: float,
) -> List[Tuple[float, float]]:
    """Clip a (CCW or CW) polygon to the rectangle [xa,xb] x [ya,yb]."""
    if not poly:
        return []
    edges = [
        ("x>=", xa), ("x<=", xb), ("y>=", ya), ("y<=", yb),
    ]
    output = list(poly)
    for kind, val in edges:
        if not output:
            break
        input_pts = output
        output = []
        if input_pts[0] != input_pts[-1]:
            input_pts = list(input_pts) + [input_pts[0]]
        for i in range(len(input_pts) - 1):
            a = input_pts[i]
            b = input_pts[i + 1]

            def inside(p):
                if kind == "x>=": return p[0] >= val
                if kind == "x<=": return p[0] <= val
                if kind == "y>=": return p[1] >= val
                if kind == "y<=": return p[1] <= val

            def intersect(a, b):
                # Parametric intersection with the clip line.
                if kind in ("x>=", "x<="):
                    if b[0] == a[0]:
                        return a
                    t = (val - a[0]) / (b[0] - a[0])
                    return (val, a[1] + t * (b[1] - a[1]))
                else:
                    if b[1] == a[1]:
                        return a
                    t = (val - a[1]) / (b[1] - a[1])
                    return (a[0] + t * (b[0] - a[0]), val)

            in_a, in_b = inside(a), inside(b)
            if in_a and in_b:
                output.append(b)
            elif in_a and not in_b:
                output.append(intersect(a, b))
            elif not in_a and in_b:
                output.append(intersect(a, b))
                output.append(b)
        # Drop trailing duplicate.
        if len(output) >= 2 and output[0] == output[-1]:
            output.pop()
    return output


# ── Mesh builders ───────────────────────────────────────────────────────────

class MeshBuf:
    """Accumulates POSITION/NORMAL/COLOR/indices for one tile."""

    __slots__ = ("verts", "normals", "colors", "indices")

    def __init__(self) -> None:
        self.verts: List[Tuple[float, float, float]] = []
        self.normals: List[Tuple[float, float, float]] = []
        self.colors: List[Tuple[int, int, int, int]] = []
        self.indices: List[int] = []

    def empty(self) -> bool:
        return not self.indices


def extrude_ribbon(
    buf: MeshBuf,
    pts_xz: Sequence[Tuple[float, float]],
    width: float,
    color_u8: Tuple[int, int, int, int],
    pts_y: Optional[Sequence[float]] = None,
) -> None:
    """Append a flat ribbon following pts_xz with given width.

    pts_xz contains already-converted (vx, vz) tile-local coordinates.
    pts_y, when provided, gives the pre-baked terrain elevation (metres
    above sea level plus vertical offset) for each centerline point.
    When None, vertex Y is 0 (legacy — runtime drape handles it).
    """
    n = len(pts_xz)
    if n < 2 or width <= 0:
        return
    half = width * 0.5
    # Per-vertex miter offset (perpendicular * scale).
    # For two consecutive unit tangents t0, t1 with perpendiculars n0, n1
    # (90° CCW), the correct ribbon-edge offset along the bisector is
    #     miter = (n0 + n1) * half / (1 + t0 . t1)
    # which keeps the rendered ribbon at constant `width` through bends.
    # Cap by MITER_LIMIT * half to avoid spikes at near-U-turn vertices.
    base = len(buf.verts)
    perps: List[Tuple[float, float]] = []
    max_off = MITER_LIMIT * half
    for i in range(n):
        if i == 0:
            dx0 = dx1 = pts_xz[1][0] - pts_xz[0][0]
            dz0 = dz1 = pts_xz[1][1] - pts_xz[0][1]
        elif i == n - 1:
            dx0 = dx1 = pts_xz[-1][0] - pts_xz[-2][0]
            dz0 = dz1 = pts_xz[-1][1] - pts_xz[-2][1]
        else:
            dx0 = pts_xz[i][0] - pts_xz[i - 1][0]
            dz0 = pts_xz[i][1] - pts_xz[i - 1][1]
            dx1 = pts_xz[i + 1][0] - pts_xz[i][0]
            dz1 = pts_xz[i + 1][1] - pts_xz[i][1]
        l0 = math.hypot(dx0, dz0) or 1.0
        l1 = math.hypot(dx1, dz1) or 1.0
        # Unit tangents and perpendiculars (rotate 90° CCW: (-dz, dx)).
        tx0, tz0 = dx0 / l0, dz0 / l0
        tx1, tz1 = dx1 / l1, dz1 / l1
        nx0, nz0 = -tz0, tx0
        nx1, nz1 = -tz1, tx1
        denom = 1.0 + tx0 * tx1 + tz0 * tz1
        if denom < 1e-4:
            # Near U-turn; fall back to plain perpendicular of the previous
            # segment to avoid blowing up.
            px, pz = nx0 * half, nz0 * half
        else:
            px = (nx0 + nx1) * half / denom
            pz = (nz0 + nz1) * half / denom
            l = math.hypot(px, pz)
            if l > max_off:
                px *= max_off / l
                pz *= max_off / l
        perps.append((px, pz))

    for i, ((x, z), (px, pz)) in enumerate(zip(pts_xz, perps)):
        y = float(pts_y[i]) if pts_y is not None else 0.0
        buf.verts.append((x + px, y, z + pz))
        buf.verts.append((x - px, y, z - pz))
        buf.normals.append((0.0, 1.0, 0.0))
        buf.normals.append((0.0, 1.0, 0.0))
        buf.colors.append(color_u8)
        buf.colors.append(color_u8)

    for i in range(n - 1):
        a = base + i * 2
        b = a + 1
        c = a + 2
        d = a + 3
        buf.indices.extend([a, c, b, b, c, d])


def emit_polygon(
    buf: MeshBuf,
    rings_xz: Sequence[Sequence[Tuple[float, float]]],
    color_u8: Tuple[int, int, int, int],
    flat_y: float = 0.0,
) -> None:
    """Triangulate a polygon (outer + holes) using mapbox_earcut.

    flat_y is the pre-baked terrain elevation for all vertices of this
    polygon (using a per-polygon mean so water bodies stay visually flat).
    """
    if not rings_xz or len(rings_xz[0]) < 3:
        return
    flat: List[float] = []
    ring_ends: List[int] = []
    count = 0
    for ring in rings_xz:
        if len(ring) < 3:
            continue
        for x, z in ring:
            flat.append(x)
            flat.append(z)
        count += len(ring)
        ring_ends.append(count)
    if not ring_ends:
        return
    verts2d = np.asarray(flat, dtype=np.float64).reshape(-1, 2)
    rings_arr = np.asarray(ring_ends, dtype=np.uint32)
    tris = earcut.triangulate_float64(verts2d, rings_arr)
    if len(tris) == 0:
        return
    base = len(buf.verts)
    for x, z in verts2d:
        buf.verts.append((float(x), flat_y, float(z)))
        buf.normals.append((0.0, 1.0, 0.0))
        buf.colors.append(color_u8)
    for idx in tris:
        buf.indices.append(base + int(idx))


# ── GLB writer ──────────────────────────────────────────────────────────────

def color_to_u8(c: Tuple[float, float, float]) -> Tuple[int, int, int, int]:
    return (
        max(0, min(255, int(round(c[0] * 255.0)))),
        max(0, min(255, int(round(c[1] * 255.0)))),
        max(0, min(255, int(round(c[2] * 255.0)))),
        255,
    )


def write_glb(path: Path, buf: MeshBuf) -> None:
    n_verts = len(buf.verts)
    n_idx = len(buf.indices)
    use_u32 = n_verts > 65535

    pos = np.asarray(buf.verts, dtype=np.float32).tobytes()
    nrm = np.asarray(buf.normals, dtype=np.float32).tobytes()
    col = np.asarray(buf.colors, dtype=np.uint8).tobytes()
    if use_u32:
        idx_arr = np.asarray(buf.indices, dtype=np.uint32)
        idx_comp = 5125
    else:
        idx_arr = np.asarray(buf.indices, dtype=np.uint16)
        idx_comp = 5123
    idx_bytes = idx_arr.tobytes()

    def pad4(b: bytes) -> bytes:
        r = len(b) % 4
        return b if r == 0 else b + b"\x00" * (4 - r)

    pos_b = pad4(pos)
    nrm_b = pad4(nrm)
    col_b = pad4(col)
    idx_b = pad4(idx_bytes)

    offs = [0]
    for blk in (pos_b, nrm_b, col_b):
        offs.append(offs[-1] + len(blk))
    idx_off = offs[3]
    bin_total = idx_off + len(idx_b)

    # Compute bounds for POSITION accessor.
    pos_np = np.asarray(buf.verts, dtype=np.float32)
    mins = pos_np.min(axis=0).tolist()
    maxs = pos_np.max(axis=0).tolist()

    gltf = {
        "asset": {"version": "2.0", "generator": "scenery3d/convert_osm_bw.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{
            "primitives": [{
                "attributes": {"POSITION": 0, "NORMAL": 1, "COLOR_0": 2},
                "indices": 3,
                "mode": 4,
            }]
        }],
        "buffers": [{"byteLength": bin_total}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": offs[0], "byteLength": len(pos), "target": 34962},
            {"buffer": 0, "byteOffset": offs[1], "byteLength": len(nrm), "target": 34962},
            {"buffer": 0, "byteOffset": offs[2], "byteLength": len(col), "target": 34962},
            {"buffer": 0, "byteOffset": idx_off, "byteLength": len(idx_bytes), "target": 34963},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": n_verts,
             "type": "VEC3", "min": mins, "max": maxs},
            {"bufferView": 1, "componentType": 5126, "count": n_verts, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5121, "count": n_verts,
             "type": "VEC4", "normalized": True},
            {"bufferView": 3, "componentType": idx_comp, "count": n_idx, "type": "SCALAR"},
        ],
    }

    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    # JSON chunk must be 4-byte aligned, padded with spaces.
    r = len(json_bytes) % 4
    if r:
        json_bytes += b" " * (4 - r)
    bin_chunk = pos_b + nrm_b + col_b + idx_b
    # BIN chunk must be 4-byte aligned (already is via pad4 on each block).
    total = 12 + 8 + len(json_bytes) + 8 + len(bin_chunk)

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"glTF")
        f.write(struct.pack("<II", 2, total))
        f.write(struct.pack("<I", len(json_bytes)))
        f.write(b"JSON")
        f.write(json_bytes)
        f.write(struct.pack("<I", len(bin_chunk)))
        f.write(b"BIN\x00")
        f.write(bin_chunk)


# ── OSM extraction ──────────────────────────────────────────────────────────

class FeatureCollector(osmium.SimpleHandler):
    """Pass 1: stream ways and area-relations, collect classified features.

    pyosmium provides node locations automatically when reading a PBF
    via SimpleHandler with locations=True.

    We store features as raw (lon, lat) tuples to keep memory modest;
    reprojection happens lazily in pass 2.
    """

    def __init__(self, boundary_prep=None) -> None:
        super().__init__()
        # (highway_class, [(lon, lat), ...])
        self.roads: List[Tuple[str, List[Tuple[float, float]]]] = []
        # (waterway_class, [(lon, lat), ...])
        self.water_lines: List[Tuple[str, List[Tuple[float, float]]]] = []
        # Each polygon: outer ring + 0+ inner rings, each ring [(lon,lat),...]
        self.water_polys: List[List[List[Tuple[float, float]]]] = []
        self.n_way = 0
        self.n_area = 0
        self.n_skipped_boundary = 0
        # PreparedGeometry covering the desired region in WGS84 (lon, lat).
        # When set, features whose representative point is outside are
        # dropped — eliminates cross-border bleed from a bbox-clipped PBF.
        self._boundary = boundary_prep

    def _inside(self, lon: float, lat: float) -> bool:
        if self._boundary is None:
            return True
        try:
            return self._boundary.contains(Point(lon, lat))
        except Exception:
            return True

    def way(self, w) -> None:
        self.n_way += 1
        tags = w.tags
        hwy = tags.get("highway")
        if hwy in ROAD_CLASSES:
            try:
                pts = [(n.lon, n.lat) for n in w.nodes if n.location.valid()]
            except osmium.InvalidLocationError:
                return
            if len(pts) >= 2:
                mid = pts[len(pts) // 2]
                if not self._inside(mid[0], mid[1]):
                    self.n_skipped_boundary += 1
                    return
                self.roads.append((hwy, pts))
            return
        railway = tags.get("railway")
        rwy_key = f"railway:{railway}" if railway else None
        if rwy_key and rwy_key in ALL_ROAD_CLASSES:
            # Skip underground tunnels: they're underground and would poke
            # through the terrain surface.
            if tags.get("tunnel") in ("yes", "true", "1"):
                return
            try:
                pts = [(n.lon, n.lat) for n in w.nodes if n.location.valid()]
            except osmium.InvalidLocationError:
                return
            if len(pts) >= 2:
                mid = pts[len(pts) // 2]
                if not self._inside(mid[0], mid[1]):
                    self.n_skipped_boundary += 1
                    return
                self.roads.append((rwy_key, pts))
            return
        wway = tags.get("waterway")
        if wway in WATER_LINE_CLASSES:
            # Skip if the way is itself a polygon (riverbank): handled via area().
            if tags.get("area") == "yes":
                return
            try:
                pts = [(n.lon, n.lat) for n in w.nodes if n.location.valid()]
            except osmium.InvalidLocationError:
                return
            if len(pts) >= 2:
                mid = pts[len(pts) // 2]
                if not self._inside(mid[0], mid[1]):
                    self.n_skipped_boundary += 1
                    return
                self.water_lines.append((wway, pts))

    def area(self, a) -> None:
        self.n_area += 1
        tags = a.tags
        natural = tags.get("natural")
        landuse = tags.get("landuse")
        waterway = tags.get("waterway")
        is_water = (
            natural == "water" or
            landuse in ("reservoir", "basin") or
            waterway == "riverbank"
        )
        if not is_water:
            return
        # Iterate rings: pyosmium gives an iterator of outer and inner rings.
        for outer in a.outer_rings():
            try:
                ring_outer = [(n.lon, n.lat) for n in outer if n.location.valid()]
            except osmium.InvalidLocationError:
                continue
            if len(ring_outer) < 3:
                continue
            # Centroid-in-boundary test: drop water polygons outside BW.
            if self._boundary is not None:
                cx = sum(p[0] for p in ring_outer) / len(ring_outer)
                cy = sum(p[1] for p in ring_outer) / len(ring_outer)
                if not self._inside(cx, cy):
                    self.n_skipped_boundary += 1
                    continue
            rings: List[List[Tuple[float, float]]] = [ring_outer]
            for inner in a.inner_rings(outer):
                try:
                    ring_inner = [(n.lon, n.lat) for n in inner if n.location.valid()]
                except osmium.InvalidLocationError:
                    continue
                if len(ring_inner) >= 3:
                    rings.append(ring_inner)
            self.water_polys.append(rings)


# ── Driver ──────────────────────────────────────────────────────────────────

def project_pts(
    transformer: Transformer,
    pts: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    lons = [p[0] for p in pts]
    lats = [p[1] for p in pts]
    es, ns = transformer.transform(lons, lats)
    return list(zip(es, ns))


def tile_id(tx: int, tn: int) -> str:
    return f"{tx}_{tn}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pbf", required=True, help="Input .osm.pbf file.")
    ap.add_argument("--roads-out", required=True, help="Output dir for road tiles.")
    ap.add_argument("--water-out", required=True, help="Output dir for water tiles.")
    ap.add_argument("--conv-origin-e", type=float, default=DEFAULT_CONV_E,
                    help=f"LV95 east origin written to manifest (default {DEFAULT_CONV_E}).")
    ap.add_argument("--conv-origin-n", type=float, default=DEFAULT_CONV_N,
                    help=f"LV95 north origin written to manifest (default {DEFAULT_CONV_N}).")
    ap.add_argument("--no-roads", action="store_true", help="Skip road tiles.")
    ap.add_argument("--no-water", action="store_true", help="Skip water tiles.")
    ap.add_argument("--densify-m", type=float, default=DEFAULT_DENSIFY_M,
                    help=f"Max segment length in metres before splitting a road "
                         f"polyline (helps terrain drape, default {DEFAULT_DENSIFY_M}).")
    ap.add_argument("--terrain-dir", default=None,
                    help="Directory containing .raw terrain tiles "
                         "(tile_{e}_{n}.raw, 1024×1024 float32 LE). "
                         "When provided, vertex elevation is pre-sampled and "
                         "baked into the GLB so no runtime draping is needed.")
    ap.add_argument("--road-offset", type=float, default=DEFAULT_ROAD_VERTICAL_OFFSET_M,
                    help=f"Metres above terrain surface baked into road vertex Y "
                         f"(default {DEFAULT_ROAD_VERTICAL_OFFSET_M}).")
    ap.add_argument("--water-offset", type=float, default=DEFAULT_WATER_VERTICAL_OFFSET_M,
                    help=f"Metres above terrain surface baked into water vertex Y "
                         f"(default {DEFAULT_WATER_VERTICAL_OFFSET_M}).")
    ap.add_argument("--boundary-poly", default=None,
                    help="Optional Geofabrik .poly file (WGS84) used to clip "
                         "features to the political boundary. Defaults to "
                         "<pbf dir>/baden-wuerttemberg.poly if it exists.")
    args = ap.parse_args()

    pbf = Path(args.pbf)
    if not pbf.exists():
        print(f"ERROR: PBF not found: {pbf}", file=sys.stderr)
        return 1

    # Set up terrain elevation sampler if a terrain directory was given.
    sampler: Optional[ElevationSampler] = None
    if args.terrain_dir:
        terrain_path = Path(args.terrain_dir)
        if not terrain_path.is_dir():
            print(f"ERROR: terrain-dir not found: {terrain_path}", file=sys.stderr)
            return 1
        sampler = ElevationSampler(str(terrain_path))
        print(f"Terrain elevation sampler: {terrain_path}", flush=True)
    else:
        print("No --terrain-dir provided; vertex Y will be 0 "
              "(runtime draping still needed).", flush=True)

    road_offset = args.road_offset
    water_offset = args.water_offset

    print(f"Reading {pbf} ({pbf.stat().st_size / (1024 * 1024):.0f} MiB)…", flush=True)
    t0 = time.time()

    boundary_prep = None
    poly_path = args.boundary_poly
    if poly_path is None:
        cand = pbf.parent / "baden-wuerttemberg.poly"
        if cand.exists():
            poly_path = str(cand)
    if poly_path:
        if not HAS_SHAPELY:
            print(f"WARNING: shapely not installed; cannot clip to {poly_path}",
                  file=sys.stderr)
        else:
            poly = parse_geofabrik_poly(Path(poly_path))
            if poly is None:
                print(f"WARNING: could not parse {poly_path}", file=sys.stderr)
            else:
                boundary_prep = prep(poly)
                print(f"Boundary clip: {poly_path} "
                      f"(area={poly.area:.3f} deg², bbox={poly.bounds})",
                      flush=True)

    coll = FeatureCollector(boundary_prep=boundary_prep)
    # locations=True caches node locations to disk-backed in-memory store so
    # we get (lon, lat) on each way node without a second pass.
    coll.apply_file(str(pbf), locations=True, idx="flex_mem")
    print(f"  ways={coll.n_way} areas={coll.n_area}", flush=True)
    print(f"  roads={len(coll.roads)} water_lines={len(coll.water_lines)} "
          f"water_polys={len(coll.water_polys)}  "
          f"skipped_boundary={coll.n_skipped_boundary}  "
          f"({time.time() - t0:.1f}s)",
          flush=True)

    transformer = make_transformer()
    conv_e = args.conv_origin_e
    conv_n = args.conv_origin_n

    # ── Roads ───────────────────────────────────────────────────────────────
    if not args.no_roads:
        print("Building road tiles…", flush=True)
        # Tile bucket: tile_xy -> list of (hwy_class, [(e, n)...])
        road_tiles: Dict[Tuple[int, int], List[Tuple[str, List[Tuple[float, float]]]]] = \
            defaultdict(list)
        t1 = time.time()
        for i, (hwy, pts_ll) in enumerate(coll.roads):
            if i and i % 50000 == 0:
                print(f"  reprojecting roads {i}/{len(coll.roads)} "
                      f"({time.time() - t1:.1f}s)", flush=True)
            pts = project_pts(transformer, pts_ll)
            if args.densify_m > 0:
                pts = densify_polyline(pts, args.densify_m)
            for tile_key, pieces in split_polyline_into_tiles(pts).items():
                for piece in pieces:
                    road_tiles[tile_key].append((hwy, piece))

        # Free the raw OSM road list — no longer needed now that road_tiles
        # holds all reprojected pieces.  This cuts peak RSS by ~1–2 GB.
        coll.roads.clear()
        gc.collect()

        print(f"  building meshes for {len(road_tiles)} tiles…", flush=True)
        roads_out = Path(args.roads_out)
        manifest_tiles: Dict[str, dict] = {}
        kept = 0
        for (tx, tn) in sorted(road_tiles):
            feats = road_tiles.pop((tx, tn))
            buf = MeshBuf()
            seg_count = 0
            for hwy, pts in feats:
                width, color, _prio = ALL_ROAD_CLASSES[hwy]
                color_u8 = color_to_u8(color)
                pts_xz = [(conv_e - e, n - conv_n) for (e, n) in pts]
                # Pre-bake terrain elevation into vertex Y when a terrain
                # directory is available; avoids visible height-snapping pops
                # during runtime that occur when the elevation DB loads later.
                if sampler is not None:
                    baked_y: Optional[List[float]] = []
                    for e, n in pts:
                        h = sampler.sample(e, n)
                        baked_y.append((h if not math.isnan(h) else 0.0) + road_offset)
                else:
                    baked_y = None
                extrude_ribbon(buf, pts_xz, width, color_u8, baked_y)
                seg_count += len(pts) - 1
            if buf.empty():
                continue
            fname = f"roads_{tx}_{tn}.glb"
            write_glb(roads_out / fname, buf)
            # Compute centre in LV95.
            cx = sum(v[0] for v in buf.verts) / len(buf.verts)
            cz = sum(v[2] for v in buf.verts) / len(buf.verts)
            ce = conv_e - cx
            cn = cz + conv_n
            manifest_tiles[tile_id(tx, tn)] = {
                "file": fname,
                "tile_e": tx,
                "tile_n": tn,
                "center_e": round(ce),
                "center_n": round(cn),
                "segments": seg_count,
                "vertices": len(buf.verts),
            }
            kept += 1
        if sampler is not None:
            print(f"  terrain miss count during road baking: {sampler.n_miss}",
                  flush=True)
            sampler.n_miss = 0
        manifest = {
            "format": "glb",
            "source": "OpenStreetMap (Geofabrik baden-wuerttemberg)",
            "license": "ODbL 1.0 (© OpenStreetMap contributors)",
            "elevation_baked": sampler is not None,
            "tile_size_m": TILE_SIZE_M,
            "conversion_origin_e": conv_e,
            "conversion_origin_n": conv_n,
            "tiles": manifest_tiles,
        }
        roads_out.mkdir(parents=True, exist_ok=True)
        with open(roads_out / "manifest.json", "w") as f:
            json.dump(manifest, f, indent=2)
        print(f"  wrote {kept} road tiles in {time.time() - t1:.1f}s", flush=True)

    # ── Water ───────────────────────────────────────────────────────────────
    if not args.no_water:
        print("Building water tiles…", flush=True)
        # Lines first.
        water_line_tiles: Dict[Tuple[int, int], List[Tuple[str, List[Tuple[float, float]]]]] = \
            defaultdict(list)
        t1 = time.time()
        for i, (wway, pts_ll) in enumerate(coll.water_lines):
            if i and i % 50000 == 0:
                print(f"  reprojecting waterways {i}/{len(coll.water_lines)} "
                      f"({time.time() - t1:.1f}s)", flush=True)
            pts = project_pts(transformer, pts_ll)
            for tile_key, pieces in split_polyline_into_tiles(pts).items():
                for piece in pieces:
                    water_line_tiles[tile_key].append((wway, piece))

        # Polygons: reproject all rings once, then for each tile clip the polygon.
        proj_polys: List[List[List[Tuple[float, float]]]] = []
        bbox_polys: List[Tuple[float, float, float, float]] = []
        for rings_ll in coll.water_polys:
            rings_en = [project_pts(transformer, ring) for ring in rings_ll]
            if not rings_en or len(rings_en[0]) < 3:
                proj_polys.append([])
                bbox_polys.append((0.0, 0.0, 0.0, 0.0))
                continue
            outer = rings_en[0]
            xs = [p[0] for p in outer]
            ys = [p[1] for p in outer]
            proj_polys.append(rings_en)
            bbox_polys.append((min(xs), min(ys), max(xs), max(ys)))

        water_poly_tiles: Dict[Tuple[int, int], List[List[List[Tuple[float, float]]]]] = \
            defaultdict(list)
        for rings, bb in zip(proj_polys, bbox_polys):
            if not rings:
                continue
            x0, y0, x1, y1 = bb
            tx_a = int(math.floor(x0 / TILE_SIZE_M)) * TILE_SIZE_M
            tn_a = int(math.floor(y0 / TILE_SIZE_M)) * TILE_SIZE_M
            tx_b = int(math.floor(x1 / TILE_SIZE_M)) * TILE_SIZE_M
            tn_b = int(math.floor(y1 / TILE_SIZE_M)) * TILE_SIZE_M
            for tx in range(tx_a, tx_b + 1, TILE_SIZE_M):
                for tn in range(tn_a, tn_b + 1, TILE_SIZE_M):
                    clipped_rings: List[List[Tuple[float, float]]] = []
                    for ri, ring in enumerate(rings):
                        cr = sutherland_hodgman(ring, tx, tn,
                                                tx + TILE_SIZE_M,
                                                tn + TILE_SIZE_M)
                        if len(cr) >= 3:
                            clipped_rings.append(cr)
                        elif ri == 0:
                            clipped_rings = []
                            break
                    if clipped_rings:
                        water_poly_tiles[(tx, tn)].append(clipped_rings)

        # Free raw OSM water collections and reprojected polygon arrays.
        coll.water_lines.clear()
        coll.water_polys.clear()
        proj_polys.clear()
        bbox_polys.clear()
        gc.collect()

        all_water_tiles = set(water_line_tiles) | set(water_poly_tiles)
        print(f"  building meshes for {len(all_water_tiles)} tiles…", flush=True)
        water_out = Path(args.water_out)
        manifest_tiles = {}
        kept = 0
        color_u8 = color_to_u8(WATER_COLOR)
        for (tx, tn) in sorted(all_water_tiles):
            buf = MeshBuf()
            n_poly = 0
            n_stream = 0
            for rings in water_poly_tiles.pop((tx, tn), []):
                rings_xz = [
                    [(conv_e - e, n - conv_n) for (e, n) in ring]
                    for ring in rings
                ]
                # Polygons (lakes, reservoirs) are flat water bodies: use the
                # mean terrain elevation of the outer ring so the whole polygon
                # sits at a consistent height rather than being warped by
                # per-vertex terrain noise.
                if sampler is not None:
                    hs_poly = []
                    for vx, vz in rings_xz[0]:
                        h = sampler.sample(conv_e - vx, vz + conv_n)
                        if not math.isnan(h):
                            hs_poly.append(h)
                    flat_y = (sum(hs_poly) / len(hs_poly) if hs_poly else 0.0) + water_offset
                else:
                    flat_y = 0.0
                emit_polygon(buf, rings_xz, color_u8, flat_y)
                n_poly += 1
            for wway, pts in water_line_tiles.pop((tx, tn), []):
                width = WATER_LINE_CLASSES[wway]
                pts_xz = [(conv_e - e, n - conv_n) for (e, n) in pts]
                # Rivers/canals as ribbons: per-vertex elevation so the ribbon
                # follows the valley floor.
                if sampler is not None:
                    water_baked_y: Optional[List[float]] = []
                    for e, n in pts:
                        h = sampler.sample(e, n)
                        water_baked_y.append((h if not math.isnan(h) else 0.0) + water_offset)
                else:
                    water_baked_y = None
                extrude_ribbon(buf, pts_xz, width, color_u8, water_baked_y)
                n_stream += 1
            if buf.empty():
                continue
            fname = f"water_{tx}_{tn}.glb"
            write_glb(water_out / fname, buf)
            cx = sum(v[0] for v in buf.verts) / len(buf.verts)
            cz = sum(v[2] for v in buf.verts) / len(buf.verts)
            ce = conv_e - cx
            cn = cz + conv_n
            manifest_tiles[tile_id(tx, tn)] = {
                "file": fname,
                "tile_e": tx,
                "tile_n": tn,
                "center_e": round(ce),
                "center_n": round(cn),
                "polygons": n_poly,
                "streams": n_stream,
                "vertices": len(buf.verts),
            }
            kept += 1
        if sampler is not None:
            print(f"  terrain miss count during water baking: {sampler.n_miss}",
                  flush=True)
        manifest = {
            "format": "glb",
            "source": "OpenStreetMap (Geofabrik baden-wuerttemberg)",
            "license": "ODbL 1.0 (© OpenStreetMap contributors)",
            "elevation_baked": sampler is not None,
            "tile_size_m": TILE_SIZE_M,
            "conversion_origin_e": conv_e,
            "conversion_origin_n": conv_n,
            "tiles": manifest_tiles,
        }
        water_out.mkdir(parents=True, exist_ok=True)
        with open(water_out / "manifest.json", "w") as f:
            json.dump(manifest, f, indent=2)
        print(f"  wrote {kept} water tiles in {time.time() - t1:.1f}s", flush=True)

    print(f"Total: {time.time() - t0:.1f}s")
    return 0

if __name__ == "__main__":
    sys.exit(main())
