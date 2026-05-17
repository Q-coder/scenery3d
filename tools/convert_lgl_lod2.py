#!/usr/bin/env python3
"""
Convert LGL Baden-Württemberg LoD2 CityGML building tiles to scenery3d
building tiles (GLB in the ``scenery3d_buildings_v1`` format).

Each CityGML tile contains all buildings on a 2 km × 2 km UTM32 cell. We:

  * stream-parse the GML to enumerate :code:`bldg:Building` (and any nested
    :code:`bldg:BuildingPart`) elements,
  * collect their :code:`bldg:WallSurface`, :code:`bldg:RoofSurface` and
    :code:`bldg:GroundSurface` polygons (heights in NHN, planar in UTM32),
  * triangulate every polygon in the plane that best fits its vertices,
  * reproject the resulting vertices from UTM32 / ETRS89 (EPSG:25832) to
    LV95 / CH1903+ (EPSG:2056) so the buildings line up with the rest of
    the scenery3d data,
  * write a per-tile GLB containing one node per building (detail wall +
    detail roof + box wall + box roof primitives, exactly like the Swiss
    converter) plus a merged "_far_lod" mesh.

Usage::

    tools/convert_lgl_lod2.py path/to/LoD2_32_495_5288_2_bw.gml \
        -o /path/to/output/

    tools/convert_lgl_lod2.py --gml-dir /tmp/lod2_in -o /path/to/output/

The output directory matches the existing buildings layout consumed by
``S3DBuildingManager`` (``manifest.json`` + ``buildings_<tile_id>.glb``).

Requires: numpy, pyproj, mapbox_earcut.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

import numpy as np
from pyproj import Transformer

try:
    import mapbox_earcut as earcut
except ImportError:
    print("ERROR: mapbox_earcut is required (pip install --user mapbox_earcut)",
          file=sys.stderr)
    raise

# Reuse the existing GLB writer + box-mesh helpers from the Swiss converter
# so the output is byte-compatible with the runtime loader.
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "scripts"))
from convert_buildings import (  # noqa: E402
    DEFAULT_ORIGIN_E, DEFAULT_ORIGIN_N,
    make_box_mesh_split, split_wall_roof, transform_to_godot,
    flip_winding, orient_outward, write_glb, write_manifest,
)


# CityGML 1.0 namespaces (LGL BW LoD2 ships CityGML 1.0).
NS = {
    "core": "http://www.opengis.net/citygml/1.0",
    "bldg": "http://www.opengis.net/citygml/building/1.0",
    "gml":  "http://www.opengis.net/gml",
    "xlink": "http://www.w3.org/1999/xlink",
}

# Optional: keep buildings whose footprint area is below this many m² to
# avoid pathological micro-buildings that derail the box / far-LOD passes.
MIN_FOOTPRINT_AREA_M2 = 1.0


class DTMSampler:
    """Bilinear sampler over scenery3d .raw heightmap tiles (1024 m grid).

    Each ``tile_<EI>_<NI>.raw`` is ``size×size`` float32 little-endian. The
    on-disk layout (matching the runtime convention) is row 0 = south,
    col 0 = east, so increasing row goes north and increasing col goes
    west. Cells with value 0 are treated as no-data (the converter zeroes
    NaNs before writing, so true sea level samples are vanishingly rare in
    BW and mistaking them for missing data is harmless).

    ``sample(lv95_e, lv95_n)`` returns NaN if the covering tile is absent
    or every contributing sample is 0; otherwise the bilinearly
    interpolated NHN elevation in metres.
    """

    def __init__(self, dtm_dir: Path | str, tile_size: int = 1024):
        self.dtm_dir = Path(dtm_dir)
        self.tile_size = tile_size
        # tile_x, tile_z -> np.ndarray[size, size] | None (missing on disk)
        self._cache: dict[tuple[int, int], np.ndarray | None] = {}

    def _load(self, tile_x: int, tile_z: int) -> np.ndarray | None:
        key = (tile_x, tile_z)
        if key in self._cache:
            return self._cache[key]
        path = self.dtm_dir / f"tile_{tile_x * self.tile_size}_{tile_z * self.tile_size}.raw"
        if not path.is_file():
            self._cache[key] = None
            return None
        try:
            raw = np.fromfile(path, dtype=np.float32)
        except OSError:
            self._cache[key] = None
            return None
        side = int(round(np.sqrt(len(raw))))
        if side * side != len(raw):
            self._cache[key] = None
            return None
        arr = raw.reshape(side, side)
        self._cache[key] = arr
        return arr

    def sample(self, lv95_e: float, lv95_n: float) -> float:
        tile_x = int(np.floor(lv95_e / self.tile_size))
        tile_z = int(np.floor(lv95_n / self.tile_size))
        arr = self._load(tile_x, tile_z)
        if arr is None:
            return float("nan")
        h, w = arr.shape
        local_x = lv95_e - tile_x * self.tile_size
        local_z = lv95_n - tile_z * self.tile_size
        # Matches runtime S3DElevationDB::get_elevation: col 0 = east,
        # row 0 = south.
        px = (1.0 - local_x / self.tile_size) * (w - 1)
        pz = (local_z / self.tile_size) * (h - 1)
        x0 = int(np.floor(px)); z0 = int(np.floor(pz))
        x1 = min(x0 + 1, w - 1); z1 = min(z0 + 1, h - 1)
        x0 = max(0, min(x0, w - 1)); z0 = max(0, min(z0, h - 1))
        fx = px - np.floor(px); fz = pz - np.floor(pz)
        h00 = float(arr[z0, x0]); h10 = float(arr[z0, x1])
        h01 = float(arr[z1, x0]); h11 = float(arr[z1, x1])
        # Treat all-zero quad as no-data (fillnodata leaves true gaps at 0).
        if h00 == 0.0 and h10 == 0.0 and h01 == 0.0 and h11 == 0.0:
            return float("nan")
        return (h00 * (1.0 - fx) * (1.0 - fz)
              + h10 * fx * (1.0 - fz)
              + h01 * (1.0 - fx) * fz
              + h11 * fx * fz)


def _qn(prefix_local: str) -> str:
    """Resolve ``"bldg:Building"`` to ``{ns}Building``."""
    prefix, local = prefix_local.split(":")
    return f"{{{NS[prefix]}}}{local}"


# Pre-resolved tag names (avoid building them inside hot loops).
_BUILDING_TAG = _qn("bldg:Building")
_BUILDING_PART_TAG = _qn("bldg:BuildingPart")
_BOUNDED_BY_TAG = _qn("bldg:boundedBy")
_WALL_TAG = _qn("bldg:WallSurface")
_ROOF_TAG = _qn("bldg:RoofSurface")
_GROUND_TAG = _qn("bldg:GroundSurface")
_POLYGON_TAG = _qn("gml:Polygon")
_EXTERIOR_TAG = _qn("gml:exterior")
_INTERIOR_TAG = _qn("gml:interior")
_LINEAR_RING_TAG = _qn("gml:LinearRing")
_POS_LIST_TAG = _qn("gml:posList")
_POS_TAG = _qn("gml:pos")


def _parse_pos_list(text: str) -> np.ndarray:
    """Parse a gml:posList into an (N, 3) array of (X, Y, Z) doubles."""
    vals = text.split()
    n = len(vals)
    if n < 9 or n % 3 != 0:
        return np.zeros((0, 3), dtype=np.float64)
    arr = np.fromiter((float(v) for v in vals), dtype=np.float64, count=n)
    return arr.reshape(-1, 3)


def _parse_ring(ring_elem: ET.Element) -> np.ndarray:
    """Extract the (N, 3) point array from a gml:LinearRing."""
    pos_list = ring_elem.find(_POS_LIST_TAG)
    if pos_list is not None and pos_list.text:
        return _parse_pos_list(pos_list.text)
    # Fall back to individual gml:pos children.
    pts = []
    for pos in ring_elem.findall(_POS_TAG):
        if pos.text:
            coords = pos.text.split()
            if len(coords) >= 3:
                pts.append([float(coords[0]), float(coords[1]), float(coords[2])])
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 3), dtype=np.float64)


def _polygon_rings(poly: ET.Element):
    """Yield (exterior, [interior, ...]) ring arrays for a gml:Polygon."""
    exterior = np.zeros((0, 3), dtype=np.float64)
    interiors: list[np.ndarray] = []
    for ext in poly.findall(_EXTERIOR_TAG):
        ring = ext.find(_LINEAR_RING_TAG)
        if ring is not None:
            exterior = _parse_ring(ring)
    for inter in poly.findall(_INTERIOR_TAG):
        ring = inter.find(_LINEAR_RING_TAG)
        if ring is not None:
            ring_pts = _parse_ring(ring)
            if len(ring_pts) >= 4:
                interiors.append(ring_pts)
    return exterior, interiors


def _newell_normal(verts: np.ndarray) -> np.ndarray:
    """Robust polygon normal via Newell's method (closed or open ring)."""
    n = len(verts)
    if n < 3:
        return np.array([0.0, 0.0, 1.0])
    # Drop the closing duplicate point if any, for numerical stability.
    if np.allclose(verts[0], verts[-1]):
        verts = verts[:-1]
        n -= 1
        if n < 3:
            return np.array([0.0, 0.0, 1.0])
    nx = ny = nz = 0.0
    for i in range(n):
        x1, y1, z1 = verts[i]
        x2, y2, z2 = verts[(i + 1) % n]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    nrm = np.array([nx, ny, nz])
    mag = float(np.linalg.norm(nrm))
    if mag < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return nrm / mag


def _project_to_plane(verts: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """Return (N, 2) coordinates of *verts* in the plane whose normal is *normal*."""
    n = normal / max(float(np.linalg.norm(normal)), 1e-12)
    # Build an orthonormal basis (u, v) of the plane.
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    un = float(np.linalg.norm(u))
    if un < 1e-9:
        u = np.array([1.0, 0.0, 0.0])
    else:
        u /= un
    v = np.cross(n, u)
    p = verts - verts[0]
    return np.column_stack([p @ u, p @ v])


def _triangulate_polygon(exterior: np.ndarray,
                         interiors: list[np.ndarray]) -> np.ndarray:
    """Triangulate a planar (possibly non-convex) 3D polygon.

    Returns an (M, 3) array of indices into the combined vertex list
    ``np.vstack([exterior, *interiors])`` (closing duplicate stripped).
    """
    # Strip closing duplicate vertex from each ring.
    def strip(ring: np.ndarray) -> np.ndarray:
        if len(ring) >= 2 and np.allclose(ring[0], ring[-1]):
            return ring[:-1]
        return ring

    ext = strip(exterior)
    ints = [strip(r) for r in interiors]
    if len(ext) < 3:
        return np.zeros((0, 3), dtype=np.int64)

    all_rings = [ext] + ints
    all_verts = np.vstack(all_rings)
    # Compute normal from the exterior ring only (most reliable).
    normal = _newell_normal(ext)
    flat = _project_to_plane(all_verts, normal)

    # mapbox_earcut wants the *end* indices of every ring (cumulative),
    # NOT just the starts of holes. For a single polygon with no holes
    # this is just [len(ext)].
    ring_lengths = [len(r) for r in all_rings]
    ring_ends = np.cumsum(ring_lengths, dtype=np.uint32)
    verts2d = np.ascontiguousarray(flat, dtype=np.float64)
    try:
        indices = earcut.triangulate_float64(verts2d, ring_ends)
    except Exception:
        return np.zeros((0, 3), dtype=np.int64)
    if indices is None or len(indices) == 0 or len(indices) % 3 != 0:
        return np.zeros((0, 3), dtype=np.int64)
    return np.asarray(indices, dtype=np.int64).reshape(-1, 3)


def _polygon_surfaces(parent: ET.Element, results: dict[str, list]):
    """Collect all gml:Polygon elements under *parent*, tagged by surface type.

    *results* maps surface kind ("wall"|"roof"|"ground"|"other") to a list of
    ``(exterior, interiors)`` tuples.
    """
    for bb in parent.findall(_BOUNDED_BY_TAG):
        # The surface kind is the *single* child of bldg:boundedBy.
        kind = "other"
        target = None
        for child in bb:
            tag = child.tag
            if tag == _WALL_TAG:
                kind, target = "wall", child
            elif tag == _ROOF_TAG:
                kind, target = "roof", child
            elif tag == _GROUND_TAG:
                kind, target = "ground", child
            else:
                target = child
            break
        if target is None:
            continue
        # Polygons can be anywhere deeper inside the bounded surface.
        for poly in target.iter(_POLYGON_TAG):
            ext, ints = _polygon_rings(poly)
            if len(ext) >= 4:
                results[kind].append((ext, ints))


def _build_geometry_for_building(building: ET.Element):
    """Return ``(verts_utm, wall_tris, roof_tris)`` for a single building.

    ``verts_utm`` is an (N, 3) array in UTM32 with NHN heights; the triangle
    arrays index into it. Ground polygons are ignored — the runtime needs
    only walls + roofs (the ground sits inside terrain).
    """
    surfaces = {"wall": [], "roof": [], "ground": [], "other": []}
    _polygon_surfaces(building, surfaces)
    # BuildingParts contribute their own surfaces.
    for part in building.iter(_BUILDING_PART_TAG):
        if part is building:
            continue
        _polygon_surfaces(part, surfaces)

    if not surfaces["wall"] and not surfaces["roof"] and not surfaces["other"]:
        return None

    verts_list = []
    wall_tris_list = []
    roof_tris_list = []
    offset = 0

    def emit(polys, out_tris):
        nonlocal offset
        for ext, ints in polys:
            # Combined vertex list for the polygon, with closing duplicates stripped.
            def strip(r):
                return r[:-1] if len(r) >= 2 and np.allclose(r[0], r[-1]) else r
            ext_s = strip(ext)
            ints_s = [strip(r) for r in ints]
            combined = np.vstack([ext_s] + ints_s) if ints_s else ext_s
            tris = _triangulate_polygon(ext, ints)
            if len(tris) == 0:
                continue
            verts_list.append(combined)
            out_tris.append(tris + offset)
            offset += len(combined)

    emit(surfaces["wall"], wall_tris_list)
    emit(surfaces["roof"], roof_tris_list)
    # Treat semantically-unlabelled surfaces by orientation later — for now
    # lump them into walls and let the normal-based splitter sort it out.
    emit(surfaces["other"], wall_tris_list)
    # Ground surfaces are intentionally dropped.

    if not verts_list:
        return None

    verts = np.vstack(verts_list)
    wall_tris = np.vstack(wall_tris_list) if wall_tris_list else np.zeros((0, 3), dtype=np.int64)
    roof_tris = np.vstack(roof_tris_list) if roof_tris_list else np.zeros((0, 3), dtype=np.int64)
    return verts, wall_tris, roof_tris


def _building_uuid(building: ET.Element, fallback: str) -> str:
    gml_id = building.get(_qn("gml:id"))
    if gml_id:
        return gml_id
    return fallback


def _iter_buildings(gml_path: Path):
    """Streaming iterator over top-level :code:`bldg:Building` elements."""
    # iterparse with end events; clear processed elements to bound memory.
    context = ET.iterparse(str(gml_path), events=("end",))
    for _, elem in context:
        if elem.tag == _BUILDING_TAG:
            yield elem
            elem.clear()


def _tile_id_from_path(path: Path) -> str:
    """Derive a tile id like ``495_5288`` from ``LoD2_32_495_5288_2_bw.gml``."""
    stem = path.stem
    parts = stem.split("_")
    # Expected: LoD2 32 <E> <N> 2 bw
    if len(parts) >= 5 and parts[0].lower().startswith("lod"):
        return f"{parts[2]}_{parts[3]}"
    return stem


def convert_gml(gml_path: Path,
                output_dir: Path,
                origin_e: float,
                origin_n: float,
                tile_id: str | None = None,
                transformer: Transformer | None = None,
                dtm_sampler: "DTMSampler | None" = None,
                dtm_stats: dict | None = None):
    """Convert a single LoD2 GML tile to a scenery3d building GLB."""
    gml_path = Path(gml_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if tile_id is None:
        tile_id = _tile_id_from_path(gml_path)

    if transformer is None:
        transformer = Transformer.from_crs("EPSG:25832", "EPSG:2056", always_xy=True)

    buildings = []
    far_wall_verts: list[np.ndarray] = []
    far_wall_norms: list[np.ndarray] = []
    far_wall_tris: list[np.ndarray] = []
    far_wall_offset = 0
    far_roof_verts: list[np.ndarray] = []
    far_roof_norms: list[np.ndarray] = []
    far_roof_tris: list[np.ndarray] = []
    far_roof_offset = 0

    skipped_empty = 0
    skipped_tiny = 0

    for idx, building in enumerate(_iter_buildings(gml_path)):
        uuid = _building_uuid(building, f"building_{idx}")
        geom = _build_geometry_for_building(building)
        building.clear()
        if geom is None:
            skipped_empty += 1
            continue
        verts_utm, wall_tris, roof_tris = geom

        # Reproject UTM32 (E, N, H) -> LV95 (E', N', H). Height unchanged
        # (NHN vs LN02 differ <0.5 m in BW, irrelevant at building scale).
        e_lv95, n_lv95 = transformer.transform(verts_utm[:, 0], verts_utm[:, 1])
        verts_lv95 = np.column_stack([e_lv95, n_lv95, verts_utm[:, 2]])

        # Footprint area sanity check (avoid 0-volume buildings).
        e_span = verts_lv95[:, 0].max() - verts_lv95[:, 0].min()
        n_span = verts_lv95[:, 1].max() - verts_lv95[:, 1].min()
        if e_span * n_span < MIN_FOOTPRINT_AREA_M2:
            skipped_tiny += 1
            continue

        # LV95 -> Godot (X = origin - E, Y = H, Z = N - origin).
        verts = transform_to_godot(verts_lv95, origin_e, origin_n)

        # Optional Y-baseline: shift the whole building so its lowest vertex
        # sits exactly on the runtime DTM at the footprint centroid. This
        # cancels out (a) LoD2 foundation level ≠ visible terrain, (b)
        # NHN-vs-LN02 datum offset, (c) absolute-Y drift on slopes. If the
        # DTM has no coverage at this point we leave Y at the original NHN
        # elevation so distant un-streamed tiles still produce plausible
        # buildings.
        if dtm_sampler is not None:
            cx = float(verts_lv95[:, 0].mean())
            cy = float(verts_lv95[:, 1].mean())
            ground_y = dtm_sampler.sample(cx, cy)
            if np.isfinite(ground_y):
                building_min_y = float(verts[:, 1].min())
                verts[:, 1] += ground_y - building_min_y
                if dtm_stats is not None:
                    dtm_stats["shifted"] = dtm_stats.get("shifted", 0) + 1
            elif dtm_stats is not None:
                dtm_stats["no_dtm"] = dtm_stats.get("no_dtm", 0) + 1

        # Concatenate wall + roof tris into one triangle list, remember the
        # split so we can preserve semantic info if needed later. We rely on
        # the existing normal-based splitter to choose wall vs roof, which
        # also fixes any miscategorised surfaces from the CityGML semantics.
        tris = np.vstack([wall_tris, roof_tris]) if (len(wall_tris) or len(roof_tris)) else np.zeros((0, 3), dtype=np.int64)
        if len(tris) == 0:
            skipped_empty += 1
            continue
        tris = tris.astype(np.uint32)
        tris = flip_winding(tris)
        tris = orient_outward(verts, tris)

        wv, wn, wt, rv, rn, rt = split_wall_roof(verts, tris)

        v_min = verts.min(axis=0)
        v_max = verts.max(axis=0)
        bwv, bwn, bwt, brv, brn, brt = make_box_mesh_split(v_min, v_max)

        name = uuid.strip("{}").replace("-", "")
        buildings.append({
            "name": name,
            "wall_vertices": wv, "wall_normals": wn, "wall_triangles": wt,
            "roof_vertices": rv, "roof_normals": rn, "roof_triangles": rt,
            "box_wall_vertices": bwv, "box_wall_normals": bwn, "box_wall_triangles": bwt,
            "box_roof_vertices": brv, "box_roof_normals": brn, "box_roof_triangles": brt,
            "center_e": float(verts_lv95[:, 0].mean()),
            "center_n": float(verts_lv95[:, 1].mean()),
        })

        far_wall_verts.append(bwv)
        far_wall_norms.append(bwn)
        far_wall_tris.append(bwt + far_wall_offset)
        far_wall_offset += len(bwv)
        far_roof_verts.append(brv)
        far_roof_norms.append(brn)
        far_roof_tris.append(brt + far_roof_offset)
        far_roof_offset += len(brv)

    if not buildings:
        print(f"  {gml_path.name}: no buildings (empty={skipped_empty}, tiny={skipped_tiny})")
        return None

    far_lod = {
        "wall_vertices": np.vstack(far_wall_verts),
        "wall_normals":  np.vstack(far_wall_norms),
        "wall_triangles": np.vstack(far_wall_tris),
        "roof_vertices": np.vstack(far_roof_verts),
        "roof_normals":  np.vstack(far_roof_norms),
        "roof_triangles": np.vstack(far_roof_tris),
    }

    glb_name = f"buildings_{tile_id}.glb"
    glb_path = output_dir / glb_name
    write_glb(glb_path, buildings, far_lod)

    center_e = sum(b["center_e"] for b in buildings) / len(buildings)
    center_n = sum(b["center_n"] for b in buildings) / len(buildings)
    detail_tris = sum(len(b["wall_triangles"]) + len(b["roof_triangles"]) for b in buildings)
    far_tris = len(far_lod["wall_triangles"]) + len(far_lod["roof_triangles"])
    print(f"  {glb_path.name}: {len(buildings)} buildings, "
          f"{detail_tris} detail tris, {far_tris} far-LOD tris "
          f"(skipped empty={skipped_empty}, tiny={skipped_tiny})")

    return {
        "file": glb_name,
        "tile_id": tile_id,
        "center_e": int(center_e),
        "center_n": int(center_n),
        "building_count": len(buildings),
    }


def convert_inputs(inputs: Iterable[Path],
                   output_dir: Path,
                   origin_e: float,
                   origin_n: float,
                   dtm_sampler: "DTMSampler | None" = None) -> dict:
    transformer = Transformer.from_crs("EPSG:25832", "EPSG:2056", always_xy=True)
    entries: dict[str, dict] = {}
    inputs = list(inputs)
    dtm_stats: dict = {}
    for i, p in enumerate(inputs):
        print(f"[{i + 1}/{len(inputs)}] {p.name}")
        result = convert_gml(p, output_dir, origin_e, origin_n,
                             transformer=transformer,
                             dtm_sampler=dtm_sampler,
                             dtm_stats=dtm_stats)
        if result:
            entries[result["tile_id"]] = result
    if dtm_sampler is not None:
        print(f"DTM baseline: shifted={dtm_stats.get('shifted', 0)} "
              f"no_dtm={dtm_stats.get('no_dtm', 0)}")
    return entries


def main():
    parser = argparse.ArgumentParser(
        description="Convert LGL BW LoD2 CityGML tiles to scenery3d building tiles")
    parser.add_argument("input", nargs="*", help="GML file(s)")
    parser.add_argument("--gml-dir", help="Convert every *.gml under this directory")
    parser.add_argument("-o", "--output", required=True, help="Output directory")
    parser.add_argument("--origin-e", type=float, default=DEFAULT_ORIGIN_E,
                        help="LV95 origin east (default matches scenery3d)")
    parser.add_argument("--origin-n", type=float, default=DEFAULT_ORIGIN_N,
                        help="LV95 origin north (default matches scenery3d)")
    parser.add_argument("--dtm-dir",
                        help="Directory of scenery3d .raw heightmap tiles "
                             "(e.g. the BW terrain output). When given, each "
                             "building is shifted vertically so its lowest "
                             "vertex sits on the DTM at the footprint centroid.")
    args = parser.parse_args()

    paths: list[Path] = []
    for p in args.input:
        paths.append(Path(p))
    if args.gml_dir:
        paths.extend(sorted(Path(args.gml_dir).rglob("*.gml")))

    if not paths:
        parser.error("Provide GML files or --gml-dir")

    dtm_sampler = DTMSampler(args.dtm_dir) if args.dtm_dir else None
    entries = convert_inputs(paths, Path(args.output), args.origin_e, args.origin_n,
                             dtm_sampler=dtm_sampler)
    if entries:
        write_manifest(Path(args.output), entries, args.origin_e, args.origin_n)


if __name__ == "__main__":
    main()
