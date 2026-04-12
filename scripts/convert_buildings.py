#!/usr/bin/env python3
"""
Convert SwissTopo swissBUILDINGS3D 3.0 GDB files to scenery3d building tiles.

Output format per tile: a GLB with each building as a separate glTF node,
identified by its UUID. Each building has two mesh primitives:
  - LOD0: full-detail TIN mesh (close range)
  - LOD1: simplified box mesh (far range)

Additionally generates a far-LOD merged mesh per tile for distant rendering
(all buildings as boxes in a single draw call).

Usage:
    python3 convert_buildings.py /path/to/tile.gdb -o /path/to/output/
    python3 convert_buildings.py /path/to/gdb_dir/ -o /path/to/output/ --all
    python3 convert_buildings.py --download --bbox 2600000 1180000 2620000 1200000 -o /path/to/output/

Requires: fiona, shapely, numpy, requests (for --download)
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import fiona
import numpy as np
from shapely.geometry import shape


# Default origin matching scenery3d default
DEFAULT_ORIGIN_E = 2600000.0
DEFAULT_ORIGIN_N = 1200000.0


def parse_tin_geometry(geometry):
    """Parse TIN MultiPolygon Z into vertices and triangle indices."""
    vertices = []
    triangles = []
    vertex_map = {}

    geom = shape(geometry)

    polys = []
    if geom.geom_type == "MultiPolygon":
        polys = list(geom.geoms)
    elif geom.geom_type == "Polygon":
        polys = [geom]

    for polygon in polys:
        coords = list(polygon.exterior.coords)
        if len(coords) < 4:
            continue
        tri_indices = []
        for coord in coords[:3]:
            x, y = coord[0], coord[1]
            z = coord[2] if len(coord) > 2 else 0.0
            key = (round(x, 4), round(y, 4), round(z, 4))
            if key not in vertex_map:
                vertex_map[key] = len(vertices)
                vertices.append([x, y, z])
            tri_indices.append(vertex_map[key])
        if len(tri_indices) == 3 and len(set(tri_indices)) == 3:
            triangles.append(tri_indices)

    if not vertices:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint32)
    return np.array(vertices, dtype=np.float64), np.array(triangles, dtype=np.uint32)


def transform_to_godot(vertices, origin_e, origin_n):
    """LV95 (E, N, H) -> Godot (+X=West, +Y=Up, +Z=North)."""
    if len(vertices) == 0:
        return vertices.astype(np.float32)
    out = np.zeros_like(vertices, dtype=np.float32)
    out[:, 0] = origin_e - vertices[:, 0]  # X = origin - E (+X = West)
    out[:, 1] = vertices[:, 2]             # Y = height
    out[:, 2] = vertices[:, 1] - origin_n  # Z = N - origin (+Z = North)
    return out


def flip_winding(triangles):
    """Reverse winding order to correct for X-axis inversion."""
    if len(triangles) == 0:
        return triangles
    flipped = triangles.copy()
    flipped[:, 1], flipped[:, 2] = triangles[:, 2].copy(), triangles[:, 1].copy()
    return flipped


def orient_outward(vertices, triangles):
    """Ensure all face normals point away from the mesh centroid (vectorized)."""
    if len(triangles) == 0:
        return triangles
    centroid = vertices.mean(axis=0).astype(np.float64)
    v = vertices.astype(np.float64)
    v0, v1, v2 = v[triangles[:, 0]], v[triangles[:, 1]], v[triangles[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    fc = (v0 + v1 + v2) / 3.0
    dot = np.sum(fn * (fc - centroid), axis=1)
    # Flip triangles whose normal points inward.
    flip = dot < 0
    result = triangles.copy()
    result[flip, 1], result[flip, 2] = triangles[flip, 2], triangles[flip, 1]
    return result


def compute_normals(vertices, triangles):
    """Compute smooth per-vertex normals."""
    normals = np.zeros_like(vertices, dtype=np.float32)
    v = vertices.astype(np.float64)
    for tri in triangles:
        e1 = v[tri[1]] - v[tri[0]]
        e2 = v[tri[2]] - v[tri[0]]
        fn = np.cross(e1, e2)
        norm = np.linalg.norm(fn)
        if norm > 0:
            fn /= norm
            for idx in tri:
                normals[idx] += fn.astype(np.float32)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0] = 1
    normals /= norms
    return normals


def compute_face_normals(vertices, triangles):
    """Compute per-face normals. Returns array of shape (N, 3)."""
    v = vertices.astype(np.float64)
    fn = np.zeros((len(triangles), 3), dtype=np.float64)
    for i, tri in enumerate(triangles):
        e1 = v[tri[1]] - v[tri[0]]
        e2 = v[tri[2]] - v[tri[0]]
        n = np.cross(e1, e2)
        norm = np.linalg.norm(n)
        if norm > 0:
            fn[i] = n / norm
    return fn


def split_wall_roof(vertices, triangles):
    """
    Split triangles into wall and roof sets based on face normal Y component.
    Roof = face normal Y > 0.5 (upward-facing), Wall = everything else.
    Uses flat normals (per-face, not smooth) for correct building lighting.
    Returns (wall_verts, wall_norms, wall_tris, roof_verts, roof_norms, roof_tris).
    """
    face_normals = compute_face_normals(vertices, triangles)
    roof_mask = face_normals[:, 1] > 0.5
    wall_mask = ~roof_mask

    def extract_flat(mask):
        sub_tris = triangles[mask]
        sub_fn = face_normals[mask]
        if len(sub_tris) == 0:
            return (np.zeros((0, 3), dtype=np.float32),
                    np.zeros((0, 3), dtype=np.float32),
                    np.zeros((0, 3), dtype=np.uint32))
        n_tris = len(sub_tris)
        # Flat shading: 3 unique vertices per triangle, each with face normal
        flat_verts = np.zeros((n_tris * 3, 3), dtype=np.float32)
        flat_norms = np.zeros((n_tris * 3, 3), dtype=np.float32)
        flat_tris = np.zeros((n_tris, 3), dtype=np.uint32)
        for i in range(n_tris):
            base = i * 3
            flat_verts[base:base + 3] = vertices[sub_tris[i]]
            flat_norms[base:base + 3] = sub_fn[i].astype(np.float32)
            flat_tris[i] = [base, base + 1, base + 2]
        return flat_verts, flat_norms, flat_tris

    wall_v, wall_n, wall_t = extract_flat(wall_mask)
    roof_v, roof_n, roof_t = extract_flat(roof_mask)
    return wall_v, wall_n, wall_t, roof_v, roof_n, roof_t


def make_box_mesh_split(min_coords, max_coords):
    """Create box mesh split into wall (sides+bottom) and roof (top) parts."""
    x0, y0, z0 = min_coords
    x1, y1, z1 = max_coords

    # Wall: sides + bottom (20 verts, 10 tris)
    wall_verts = np.array([
        # Bottom face
        [x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1],
        # Front face (z0)
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        # Back face (z1)
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        # Left face (x0)
        [x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0],
        # Right face (x1)
        [x1, y0, z0], [x1, y0, z1], [x1, y1, z1], [x1, y1, z0],
    ], dtype=np.float32)
    wall_norms = np.array([
        [0, -1, 0], [0, -1, 0], [0, -1, 0], [0, -1, 0],
        [0, 0, -1], [0, 0, -1], [0, 0, -1], [0, 0, -1],
        [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1],
        [-1, 0, 0], [-1, 0, 0], [-1, 0, 0], [-1, 0, 0],
        [1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0],
    ], dtype=np.float32)
    wall_tris = np.array([
        [0, 1, 2], [0, 2, 3],      # bottom
        [4, 7, 6], [4, 6, 5],      # front
        [8, 9, 10], [8, 10, 11],   # back
        [12, 13, 14], [12, 14, 15],# left
        [16, 19, 18], [16, 18, 17],# right
    ], dtype=np.uint32)

    # Roof: top face only (4 verts, 2 tris)
    roof_verts = np.array([
        [x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1],
    ], dtype=np.float32)
    roof_norms = np.array([
        [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0],
    ], dtype=np.float32)
    roof_tris = np.array([
        [0, 3, 2], [0, 2, 1],
    ], dtype=np.uint32)

    return wall_verts, wall_norms, wall_tris, roof_verts, roof_norms, roof_tris


def align_to_4(n):
    """Align byte offset to 4-byte boundary."""
    return (n + 3) & ~3


def write_glb(path, nodes_data, far_lod_data=None):
    """
    Write a GLB file with individual building nodes + optional far-LOD merged mesh.

    nodes_data: list of dicts with keys:
        name,
        wall_vertices, wall_normals, wall_triangles,
        roof_vertices, roof_normals, roof_triangles,
        box_wall_vertices, box_wall_normals, box_wall_triangles,
        box_roof_vertices, box_roof_normals, box_roof_triangles
    far_lod_data: dict with keys:
        wall_vertices, wall_normals, wall_triangles,
        roof_vertices, roof_normals, roof_triangles
    """
    # Build glTF JSON and binary buffer
    buffer_data = b""
    buffer_views = []
    accessors = []
    meshes = []
    nodes = []
    scene_nodes = []

    def add_mesh_primitive(verts, norms, tris):
        """Add a mesh primitive's data to the buffer, return accessor indices."""
        nonlocal buffer_data

        # Vertices
        v_offset = len(buffer_data)
        v_bytes = verts.astype(np.float32).tobytes()
        buffer_data += v_bytes
        pad = align_to_4(len(buffer_data)) - len(buffer_data)
        buffer_data += b"\x00" * pad

        v_bv = len(buffer_views)
        buffer_views.append({
            "buffer": 0,
            "byteOffset": v_offset,
            "byteLength": len(v_bytes),
            "target": 34962,
        })
        v_acc = len(accessors)
        v_min = verts.min(axis=0).tolist()
        v_max = verts.max(axis=0).tolist()
        accessors.append({
            "bufferView": v_bv,
            "componentType": 5126,
            "count": len(verts),
            "type": "VEC3",
            "min": v_min,
            "max": v_max,
        })

        # Normals
        n_offset = len(buffer_data)
        n_bytes = norms.astype(np.float32).tobytes()
        buffer_data += n_bytes
        pad = align_to_4(len(buffer_data)) - len(buffer_data)
        buffer_data += b"\x00" * pad

        n_bv = len(buffer_views)
        buffer_views.append({
            "buffer": 0,
            "byteOffset": n_offset,
            "byteLength": len(n_bytes),
            "target": 34962,
        })
        n_acc = len(accessors)
        accessors.append({
            "bufferView": n_bv,
            "componentType": 5126,
            "count": len(norms),
            "type": "VEC3",
        })

        # Indices
        i_offset = len(buffer_data)
        flat_idx = tris.flatten()
        if len(verts) < 65536:
            i_bytes = flat_idx.astype(np.uint16).tobytes()
            i_comp = 5123  # UNSIGNED_SHORT
        else:
            i_bytes = flat_idx.astype(np.uint32).tobytes()
            i_comp = 5125  # UNSIGNED_INT
        buffer_data += i_bytes
        pad = align_to_4(len(buffer_data)) - len(buffer_data)
        buffer_data += b"\x00" * pad

        i_bv = len(buffer_views)
        buffer_views.append({
            "buffer": 0,
            "byteOffset": i_offset,
            "byteLength": len(i_bytes),
            "target": 34963,
        })
        i_acc = len(accessors)
        accessors.append({
            "bufferView": i_bv,
            "componentType": i_comp,
            "count": len(flat_idx),
            "type": "SCALAR",
        })

        return {"POSITION": v_acc, "NORMAL": n_acc}, i_acc

    for bld in nodes_data:
        primitives = []

        # Primitive 0: wall detail (LOD0)
        if len(bld["wall_vertices"]) > 0:
            attrs, idx = add_mesh_primitive(
                bld["wall_vertices"], bld["wall_normals"], bld["wall_triangles"])
            primitives.append({"attributes": attrs, "indices": idx})
        else:
            # Empty placeholder — 0 triangles
            attrs, idx = add_mesh_primitive(
                np.zeros((1, 3), dtype=np.float32),
                np.array([[0, 1, 0]], dtype=np.float32),
                np.zeros((0, 3), dtype=np.uint32))
            primitives.append({"attributes": attrs, "indices": idx})

        # Primitive 1: roof detail (LOD0)
        if len(bld["roof_vertices"]) > 0:
            attrs, idx = add_mesh_primitive(
                bld["roof_vertices"], bld["roof_normals"], bld["roof_triangles"])
            primitives.append({"attributes": attrs, "indices": idx})
        else:
            attrs, idx = add_mesh_primitive(
                np.zeros((1, 3), dtype=np.float32),
                np.array([[0, 1, 0]], dtype=np.float32),
                np.zeros((0, 3), dtype=np.uint32))
            primitives.append({"attributes": attrs, "indices": idx})

        # Primitive 2: wall box (LOD1)
        attrs, idx = add_mesh_primitive(
            bld["box_wall_vertices"], bld["box_wall_normals"], bld["box_wall_triangles"])
        primitives.append({"attributes": attrs, "indices": idx})

        # Primitive 3: roof box (LOD1)
        attrs, idx = add_mesh_primitive(
            bld["box_roof_vertices"], bld["box_roof_normals"], bld["box_roof_triangles"])
        primitives.append({"attributes": attrs, "indices": idx})

        mesh_idx = len(meshes)
        meshes.append({"name": bld["name"], "primitives": primitives})

        node_idx = len(nodes)
        nodes.append({"name": bld["name"], "mesh": mesh_idx})
        scene_nodes.append(node_idx)

    # Far-LOD merged mesh (2 primitives: wall + roof).
    if far_lod_data and len(far_lod_data["wall_vertices"]) > 0:
        far_primitives = []
        # Primitive 0: wall (sides + bottom)
        attrs_fw, idx_fw = add_mesh_primitive(
            far_lod_data["wall_vertices"], far_lod_data["wall_normals"],
            far_lod_data["wall_triangles"])
        far_primitives.append({"attributes": attrs_fw, "indices": idx_fw})
        # Primitive 1: roof (top)
        attrs_fr, idx_fr = add_mesh_primitive(
            far_lod_data["roof_vertices"], far_lod_data["roof_normals"],
            far_lod_data["roof_triangles"])
        far_primitives.append({"attributes": attrs_fr, "indices": idx_fr})

        mesh_idx = len(meshes)
        meshes.append({
            "name": "_far_lod",
            "primitives": far_primitives,
        })
        node_idx = len(nodes)
        nodes.append({"name": "_far_lod", "mesh": mesh_idx})
        scene_nodes.append(node_idx)

    gltf = {
        "asset": {"version": "2.0", "generator": "scenery3d convert_buildings"},
        "scene": 0,
        "scenes": [{"nodes": scene_nodes}],
        "nodes": nodes,
        "meshes": meshes,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(buffer_data)}],
    }

    # Write GLB
    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_pad = align_to_4(len(json_bytes)) - len(json_bytes)
    json_bytes += b" " * json_pad

    bin_pad = align_to_4(len(buffer_data)) - len(buffer_data)
    buffer_data += b"\x00" * bin_pad

    total = 12 + 8 + len(json_bytes) + 8 + len(buffer_data)

    with open(path, "wb") as f:
        # GLB header
        f.write(b"glTF")
        f.write(struct.pack("<II", 2, total))
        # JSON chunk
        f.write(struct.pack("<II", len(json_bytes), 0x4E4F534A))
        f.write(json_bytes)
        # Binary chunk
        f.write(struct.pack("<II", len(buffer_data), 0x004E4942))
        f.write(buffer_data)


def convert_gdb(gdb_path, output_dir, origin_e, origin_n, tile_id):
    """Convert a single GDB file to the scenery3d building tile format."""
    gdb_path = Path(gdb_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    layers = fiona.listlayers(str(gdb_path))
    if "Building_solid" not in layers:
        print(f"  WARNING: No Building_solid layer in {gdb_path.name}, skipping")
        return None

    buildings = []
    # Far-LOD: separate wall and roof merged meshes.
    far_wall_verts = []
    far_wall_norms = []
    far_wall_tris = []
    far_wall_offset = 0
    far_roof_verts = []
    far_roof_norms = []
    far_roof_tris = []
    far_roof_offset = 0

    with fiona.open(str(gdb_path), layer="Building_solid") as src:
        total = len(src)
        for i, feat in enumerate(src):
            geom = feat.get("geometry")
            if geom is None:
                continue

            uuid = feat["properties"].get("UUID", f"building_{i}")

            # Parse TIN geometry (in LV95 coords)
            verts_lv95, tris = parse_tin_geometry(geom)
            if len(verts_lv95) == 0 or len(tris) == 0:
                continue

            # Transform to Godot coords
            verts = transform_to_godot(verts_lv95, origin_e, origin_n)
            tris = flip_winding(tris)
            tris = orient_outward(verts, tris)

            # Split detail mesh into wall and roof by face normals
            wv, wn, wt, rv, rn, rt = split_wall_roof(verts, tris)

            # Compute bounding box for LOD1 (split into wall + roof)
            v_min = verts.min(axis=0)
            v_max = verts.max(axis=0)
            bwv, bwn, bwt, brv, brn, brt = make_box_mesh_split(v_min, v_max)

            # Clean UUID for use as node name
            name = uuid.strip("{}").replace("-", "")

            buildings.append({
                "name": name,
                # Detail: wall + roof surfaces
                "wall_vertices": wv, "wall_normals": wn, "wall_triangles": wt,
                "roof_vertices": rv, "roof_normals": rn, "roof_triangles": rt,
                # Box: wall + roof surfaces
                "box_wall_vertices": bwv, "box_wall_normals": bwn, "box_wall_triangles": bwt,
                "box_roof_vertices": brv, "box_roof_normals": brn, "box_roof_triangles": brt,
                "center_e": float(verts_lv95[:, 0].mean()),
                "center_n": float(verts_lv95[:, 1].mean()),
            })

            # Accumulate far-LOD merged mesh (wall sides + roof top from box)
            far_wall_verts.append(bwv)
            far_wall_norms.append(bwn)
            far_wall_tris.append(bwt + far_wall_offset)
            far_wall_offset += len(bwv)

            far_roof_verts.append(brv)
            far_roof_norms.append(brn)
            far_roof_tris.append(brt + far_roof_offset)
            far_roof_offset += len(brv)

            if (i + 1) % 1000 == 0:
                print(f"    {i + 1}/{total} buildings...")

    if not buildings:
        print(f"  No buildings found in {gdb_path.name}")
        return None

    # Build far-LOD merged data (2 primitives: wall + roof)
    far_lod = None
    if far_wall_verts:
        far_lod = {
            "wall_vertices": np.vstack(far_wall_verts),
            "wall_normals": np.vstack(far_wall_norms),
            "wall_triangles": np.vstack(far_wall_tris),
            "roof_vertices": np.vstack(far_roof_verts),
            "roof_normals": np.vstack(far_roof_norms),
            "roof_triangles": np.vstack(far_roof_tris),
        }

    # Write GLB
    glb_name = f"buildings_{tile_id}.glb"
    glb_path = output_dir / glb_name
    write_glb(glb_path, buildings, far_lod)

    # Compute tile center
    centers_e = [b["center_e"] for b in buildings]
    centers_n = [b["center_n"] for b in buildings]
    center_e = sum(centers_e) / len(centers_e)
    center_n = sum(centers_n) / len(centers_n)

    detail_tris = sum(len(b["wall_triangles"]) + len(b["roof_triangles"]) for b in buildings)
    far_tris = (len(far_lod["wall_triangles"]) + len(far_lod["roof_triangles"])) if far_lod else 0
    print(f"  {glb_path.name}: {len(buildings)} buildings, "
          f"{detail_tris} detail tris, {far_tris} far-LOD tris")

    return {
        "file": glb_name,
        "tile_id": tile_id,
        "center_e": int(center_e),
        "center_n": int(center_n),
        "building_count": len(buildings),
    }


def download_and_convert(bbox, output_dir, origin_e, origin_n, cache_dir=None):
    """Download GDB tiles from STAC API and convert them."""
    import requests
    import zipfile
    from pyproj import Transformer

    STAC_BASE = "https://data.geo.admin.ch/api/stac/v0.9"
    COLLECTION = "ch.swisstopo.swissbuildings3d_3_0"

    west, south, east, north = bbox
    output_dir = Path(output_dir)

    if cache_dir is None:
        cache_dir = output_dir / "_cache"
    else:
        cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)

    # Convert LV95 bbox to WGS84 for STAC query
    transformer = Transformer.from_crs("EPSG:2056", "EPSG:4326", always_xy=True)
    lon_w, lat_s = transformer.transform(west, south)
    lon_e, lat_n = transformer.transform(east, north)

    print(f"Querying STAC API for bbox {lon_w:.4f},{lat_s:.4f},{lon_e:.4f},{lat_n:.4f}")

    # Discover tiles
    tiles = []
    cursor = None
    while True:
        url = (f"{STAC_BASE}/collections/{COLLECTION}/items"
               f"?bbox={lon_w},{lat_s},{lon_e},{lat_n}&limit=50")
        if cursor:
            url += f"&cursor={cursor}"
        resp = requests.get(url, timeout=30)
        resp.raise_for_status()
        data = resp.json()

        for feature in data.get("features", []):
            fid = feature["id"]
            parts = fid.split("_")
            if len(parts) >= 5:
                year = parts[3]
                tile_id = parts[4]
                gdb_url = None
                for akey, asset in feature.get("assets", {}).items():
                    if akey.endswith(".gdb.zip"):
                        gdb_url = asset["href"]
                        break
                if gdb_url:
                    tiles.append({"tile_id": tile_id, "year": year, "url": gdb_url})

        next_link = None
        for link in data.get("links", []):
            if link.get("rel") == "next":
                import urllib.parse
                parsed = urllib.parse.urlparse(link["href"])
                params = urllib.parse.parse_qs(parsed.query)
                cursor = params.get("cursor", [None])[0]
                next_link = cursor
                break
        if not next_link or not data.get("features"):
            break

    # Deduplicate
    by_tile = {}
    for t in tiles:
        tid = t["tile_id"]
        if tid not in by_tile or t["year"] > by_tile[tid]["year"]:
            by_tile[tid] = t
    tiles = list(by_tile.values())
    print(f"Found {len(tiles)} building tiles")

    manifest_entries = {}
    for i, tile in enumerate(tiles):
        tile_id = tile["tile_id"]
        glb_check = output_dir / f"buildings_{tile_id}.glb"
        if glb_check.exists():
            print(f"[{i+1}/{len(tiles)}] {tile_id}: already exists, skipping")
            continue

        print(f"[{i+1}/{len(tiles)}] {tile_id}: downloading...")
        zip_path = cache_dir / f"{tile_id}.gdb.zip"
        if not zip_path.exists():
            r = requests.get(tile["url"], stream=True, timeout=120)
            r.raise_for_status()
            with open(zip_path, "wb") as f:
                for chunk in r.iter_content(65536):
                    f.write(chunk)

        # Extract
        gdb_path = None
        with zipfile.ZipFile(zip_path) as z:
            z.extractall(cache_dir)
        for p in cache_dir.iterdir():
            if p.suffix == ".gdb" and p.is_dir() and tile_id in p.name:
                gdb_path = p
                break

        if not gdb_path:
            print(f"  WARNING: Could not find GDB for {tile_id}")
            continue

        result = convert_gdb(gdb_path, output_dir, origin_e, origin_n, tile_id)
        if result:
            manifest_entries[tile_id] = result

    return manifest_entries


def convert_directory(gdb_dir, output_dir, origin_e, origin_n):
    """Convert all GDB directories found under gdb_dir."""
    gdb_dir = Path(gdb_dir)
    results = {}

    gdbs = sorted(gdb_dir.glob("*.gdb"))
    if not gdbs:
        # Maybe the path IS a single GDB
        if gdb_dir.suffix == ".gdb":
            gdbs = [gdb_dir]

    print(f"Found {len(gdbs)} GDB files")
    for i, gdb in enumerate(gdbs):
        # Extract tile ID from name: swissBUILDINGS3D_3-0_1166-24.gdb -> 1166-24
        name = gdb.stem
        parts = name.split("_")
        tile_id = parts[-1] if parts else name
        print(f"[{i+1}/{len(gdbs)}] Converting {tile_id}...")
        result = convert_gdb(gdb, output_dir, origin_e, origin_n, tile_id)
        if result:
            results[tile_id] = result

    return results


def write_manifest(output_dir, entries, origin_e, origin_n):
    """Write or update building manifest.json."""
    output_dir = Path(output_dir)
    manifest_path = output_dir / "manifest.json"

    manifest = {
        "format": "scenery3d_buildings_v1",
        "origin_e": origin_e,
        "origin_n": origin_n,
        "tiles": {},
    }

    if manifest_path.exists():
        with open(manifest_path) as f:
            existing = json.load(f)
            if "tiles" in existing:
                manifest["tiles"] = existing["tiles"]

    manifest["tiles"].update(entries)

    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nManifest: {manifest_path} ({len(manifest['tiles'])} tiles)")


def main():
    parser = argparse.ArgumentParser(description="Convert swissBUILDINGS3D to scenery3d format")
    parser.add_argument("input", nargs="?", help="GDB file/directory, or omit with --download")
    parser.add_argument("-o", "--output", required=True, help="Output directory")
    parser.add_argument("--origin-e", type=float, default=DEFAULT_ORIGIN_E)
    parser.add_argument("--origin-n", type=float, default=DEFAULT_ORIGIN_N)
    parser.add_argument("--download", action="store_true", help="Download from STAC API")
    parser.add_argument("--bbox", nargs=4, type=float, metavar=("W", "S", "E", "N"),
                        help="LV95 bounding box for --download")
    parser.add_argument("--cache", help="Cache directory for downloads")
    parser.add_argument("--all", action="store_true", help="Convert all GDBs in directory")
    args = parser.parse_args()

    if args.download:
        if not args.bbox:
            parser.error("--download requires --bbox")
        entries = download_and_convert(
            args.bbox, args.output, args.origin_e, args.origin_n, args.cache)
    elif args.all or (args.input and Path(args.input).is_dir()
                       and not Path(args.input).suffix == ".gdb"):
        entries = convert_directory(args.input, args.output, args.origin_e, args.origin_n)
    elif args.input:
        inp = Path(args.input)
        tile_id = inp.stem.split("_")[-1] if "_" in inp.stem else inp.stem
        result = convert_gdb(inp, args.output, args.origin_e, args.origin_n, tile_id)
        entries = {tile_id: result} if result else {}
    else:
        parser.error("Provide an input path or use --download")

    if entries:
        write_manifest(args.output, entries, args.origin_e, args.origin_n)


if __name__ == "__main__":
    main()
