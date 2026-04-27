#!/usr/bin/env python3
"""
Streaming LGL DOP20 ZIP → Scenery3D ortho tiles.

Reads RGB GeoTIFFs straight out of the LGL DOP20 ZIPs via GDAL's
``/vsizip/`` virtual filesystem (no extraction required) and emits per-LV95
1024 m JPEG tiles (mip0 / mip2 / mip3) on the same grid as the SwissTopo
SWISSIMAGE pipeline.

Why this exists
---------------
``convert_lgl_dop.py`` opens every raster up front, which only works on
already-extracted GeoTIFFs. The full BW DOP20 corpus is ~1.4 TB extracted —
not realistic if the source disk only has a few hundred GB free.

This tool walks one ZIP at a time, never extracts to disk, and can delete each
ZIP after every LV95 tile it contributes to has been written. Disk usage stays
bounded by the (small) JPEG output and the running ZIP index.

Usage
-----
  python tools/process_lgl_dop.py \\
      --zip-dir /Volumes/Data1/scenery_in_dop \\
      --output  /Volumes/Data1/scenery/Germany/Baden-Wuertemberg/orthophoto

Resume / overwrite:
  --overwrite           Re-encode tiles even if all three mip JPEGs exist.
  --extent E_min N_min E_max N_max
                        Restrict to an LV95 bbox.

Cleaning up sources to free space:
  --delete-zips         Delete each ZIP once every LV95 tile it covers has
                        been written to disk. Idempotent: re-running on a
                        partially deleted ZIP set just covers the remaining
                        area.

Parallelism:
  --jobs N              Worker processes (default: ncpu // 2). Each worker
                        handles a slab of LV95 tiles.

The script can be run alongside an in-progress ``download_lgl_dop.py`` —
ZIPs that arrive after the index was built are simply picked up on the next
invocation.
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import re
import sys
from pathlib import Path

import numpy as np

try:
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine
    from rasterio.vrt import WarpedVRT
    from rasterio.warp import reproject, transform_bounds
except ImportError:
    print("Error: rasterio is required. Install with: pip install rasterio",
          file=sys.stderr)
    sys.exit(1)

try:
    from PIL import Image
except ImportError:
    print("Error: Pillow is required. Install with: pip install Pillow",
          file=sys.stderr)
    sys.exit(1)

LV95_CRS = "EPSG:2056"
SRC_CRS = "EPSG:25832"  # UTM32N
TILE_SIZE_M = 1024
MIP0_PX = 1024
MIP2_PX = 64
MIP3_PX = 16

# Each LGL DOP20 ZIP covers a 2 km × 2 km UTM32 cell, named by its SW corner
# (E_km, N_km). Reprojected to LV95 the cell is a quadrilateral, slightly
# rotated. Pad by this many metres on every side when expanding to an LV95
# bbox so we never miss a contributing source for an LV95 tile.
UTM_TO_LV95_SAFETY_M = 200

ZIP_NAME_RE = re.compile(r"dop20rgb_32_(\d+)_(\d+)_2_bw\.zip$")


def parse_zip_name(path: Path) -> tuple[int, int] | None:
    m = ZIP_NAME_RE.match(path.name)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))


def list_inner_tifs(zip_path: Path) -> list[str]:
    """Return /vsizip/ URIs for the four inner TIFFs of a DOP20 ZIP."""
    import zipfile
    with zipfile.ZipFile(zip_path) as zf:
        return [
            f"/vsizip/{zip_path}/{name}"
            for name in zf.namelist()
            if name.lower().endswith(".tif")
        ]


def utm_cell_to_lv95_bbox(e_km: int, n_km: int) -> tuple[float, float, float, float]:
    """Reproject a 2 km UTM32 cell to an axis-aligned LV95 bbox (padded)."""
    e0 = e_km * 1000.0
    n0 = n_km * 1000.0
    e1 = e0 + 2000.0
    n1 = n0 + 2000.0
    le, ln, re_, rn = transform_bounds(SRC_CRS, LV95_CRS, e0, n0, e1, n1,
                                       densify_pts=21)
    return (le - UTM_TO_LV95_SAFETY_M, ln - UTM_TO_LV95_SAFETY_M,
            re_ + UTM_TO_LV95_SAFETY_M, rn + UTM_TO_LV95_SAFETY_M)


def lv95_bbox_to_tiles(bbox: tuple[float, float, float, float]
                       ) -> list[tuple[int, int]]:
    le, ln, re_, rn = bbox
    tx_min = int(np.floor(le / TILE_SIZE_M))
    tz_min = int(np.floor(ln / TILE_SIZE_M))
    tx_max = int(np.floor((re_ - 1e-3) / TILE_SIZE_M))
    tz_max = int(np.floor((rn - 1e-3) / TILE_SIZE_M))
    return [(tx, tz)
            for tz in range(tz_min, tz_max + 1)
            for tx in range(tx_min, tx_max + 1)]


def build_index(zip_dir: Path,
                extent: tuple[float, float, float, float] | None
                ) -> tuple[dict[tuple[int, int], list[Path]],
                           dict[Path, list[tuple[int, int]]]]:
    """Map each LV95 tile to the ZIPs that cover it and vice versa.

    Returns (tile_to_zips, zip_to_tiles).
    """
    zips = sorted(zip_dir.glob("dop20rgb_32_*_*_2_bw.zip"))
    print(f"  scanning {len(zips)} ZIP(s) in {zip_dir}")

    tile_to_zips: dict[tuple[int, int], list[Path]] = {}
    zip_to_tiles: dict[Path, list[tuple[int, int]]] = {}

    for zp in zips:
        cell = parse_zip_name(zp)
        if cell is None:
            continue
        bbox = utm_cell_to_lv95_bbox(*cell)
        if extent is not None:
            le, ln, re_, rn = bbox
            ee_min, en_min, ee_max, en_max = extent
            if (re_ <= ee_min or le >= ee_max
                    or rn <= en_min or ln >= en_max):
                continue
        tiles = lv95_bbox_to_tiles(bbox)
        if extent is not None:
            ee_min, en_min, ee_max, en_max = extent
            tx_lo = int(np.floor(ee_min / TILE_SIZE_M))
            tx_hi = int(np.floor((ee_max - 1e-3) / TILE_SIZE_M))
            tz_lo = int(np.floor(en_min / TILE_SIZE_M))
            tz_hi = int(np.floor((en_max - 1e-3) / TILE_SIZE_M))
            tiles = [(tx, tz) for (tx, tz) in tiles
                     if tx_lo <= tx <= tx_hi and tz_lo <= tz <= tz_hi]
        if not tiles:
            continue
        zip_to_tiles[zp] = tiles
        for t in tiles:
            tile_to_zips.setdefault(t, []).append(zp)

    return tile_to_zips, zip_to_tiles


def render_tile_from_zips(tx: int, tz: int, zip_paths: list[Path],
                          min_valid_fraction: float
                          ) -> np.ndarray | None:
    """Render an LV95 1024 m tile from the given DOP20 ZIPs.

    Returns HWC uint8 RGB in native Scenery3D layout (row 0 = south,
    col 0 = east) or None if coverage is below threshold.
    """
    e_min = tx * TILE_SIZE_M
    n_min = tz * TILE_SIZE_M
    e_max = e_min + TILE_SIZE_M
    n_max = n_min + TILE_SIZE_M

    pixel = TILE_SIZE_M / MIP0_PX  # = 1.0
    dst_transform = Affine(pixel, 0.0, e_min, 0.0, -pixel, n_max)

    dst = np.zeros((3, MIP0_PX, MIP0_PX), dtype=np.uint8)
    cover = np.zeros((MIP0_PX, MIP0_PX), dtype=bool)

    for zp in zip_paths:
        try:
            uris = list_inner_tifs(zp)
        except Exception as e:
            print(f"  WARN cannot list {zp.name}: {e}", file=sys.stderr)
            continue
        for uri in uris:
            try:
                src = rasterio.open(uri)
            except Exception as e:
                print(f"  WARN cannot open {uri}: {e}", file=sys.stderr)
                continue
            try:
                # Quick UTM-bounds check before warping.
                lb, bb, rb, tb = transform_bounds(
                    src.crs or SRC_CRS, LV95_CRS,
                    *src.bounds, densify_pts=11)
                if (rb <= e_min or lb >= e_max
                        or tb <= n_min or bb >= n_max):
                    continue
                if src.count < 3:
                    continue
                with WarpedVRT(src,
                               src_crs=src.crs or SRC_CRS,
                               crs=LV95_CRS,
                               resampling=Resampling.bilinear) as vrt:
                    tmp = np.zeros_like(dst)
                    for band_idx in range(3):
                        reproject(
                            source=rasterio.band(vrt, band_idx + 1),
                            destination=tmp[band_idx],
                            dst_transform=dst_transform,
                            dst_crs=LV95_CRS,
                            resampling=Resampling.bilinear,
                        )
                    tmp_cover = (tmp[0] | tmp[1] | tmp[2]).astype(bool)
                    need = ~cover & tmp_cover
                    if need.any():
                        for c in range(3):
                            dst[c][need] = tmp[c][need]
                        cover |= need
            finally:
                src.close()

    if not cover.any():
        return None
    if cover.sum() / cover.size < min_valid_fraction:
        return None

    out = dst[:, ::-1, ::-1]
    return np.ascontiguousarray(np.transpose(out, (1, 2, 0)))


def write_jpeg(arr_hwc: np.ndarray, path: Path, quality: int) -> int:
    img = Image.fromarray(arr_hwc, mode="RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path, format="JPEG", quality=quality, optimize=True,
             progressive=False)
    return path.stat().st_size


def downsample(arr_hwc: np.ndarray, target_px: int) -> np.ndarray:
    img = Image.fromarray(arr_hwc, mode="RGB")
    img = img.resize((target_px, target_px), Image.Resampling.LANCZOS)
    return np.asarray(img)


# ──────────────────────────────────────────────────────────────────────
# Worker
# ──────────────────────────────────────────────────────────────────────

_WORKER_CONFIG: dict = {}


def _worker_init(output_dir: str, q_mip0: int, q_mip_low: int,
                 min_valid_fraction: float, overwrite: bool):
    _WORKER_CONFIG["output_dir"] = Path(output_dir)
    _WORKER_CONFIG["q_mip0"] = q_mip0
    _WORKER_CONFIG["q_mip_low"] = q_mip_low
    _WORKER_CONFIG["min_valid_fraction"] = min_valid_fraction
    _WORKER_CONFIG["overwrite"] = overwrite


def _worker_render(args):
    tx, tz, zip_paths_str = args
    zip_paths = [Path(s) for s in zip_paths_str]
    output_dir: Path = _WORKER_CONFIG["output_dir"]
    overwrite: bool = _WORKER_CONFIG["overwrite"]

    lv95_e = tx * TILE_SIZE_M
    lv95_n = tz * TILE_SIZE_M
    fname = f"ortho_{lv95_e}_{lv95_n}.jpg"
    mip0 = output_dir / fname
    mip2 = output_dir / "mip2" / fname
    mip3 = output_dir / "mip3" / fname

    if not overwrite and mip0.exists() and mip2.exists() and mip3.exists():
        return ("skip-exists", tx, tz, lv95_e, lv95_n, fname, 0)

    arr = render_tile_from_zips(tx, tz, zip_paths,
                                _WORKER_CONFIG["min_valid_fraction"])
    if arr is None:
        return ("skip-empty", tx, tz, lv95_e, lv95_n, fname, 0)

    bytes_w = 0
    bytes_w += write_jpeg(arr, mip0, _WORKER_CONFIG["q_mip0"])
    bytes_w += write_jpeg(downsample(arr, MIP2_PX), mip2,
                          _WORKER_CONFIG["q_mip_low"])
    bytes_w += write_jpeg(downsample(arr, MIP3_PX), mip3,
                          _WORKER_CONFIG["q_mip_low"])
    return ("write", tx, tz, lv95_e, lv95_n, fname, bytes_w)


# ──────────────────────────────────────────────────────────────────────
# Driver
# ──────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Stream LGL DOP20 ZIPs into Scenery3D ortho JPEGs.",
    )
    parser.add_argument("--zip-dir", required=True,
                        help="Directory containing dop20rgb_32_*_*_2_bw.zip")
    parser.add_argument("--output", required=True,
                        help="Output directory for ortho_*.jpg + mip2/ + mip3/")
    parser.add_argument("--extent", type=float, nargs=4,
                        metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        help="LV95 bbox to restrict processing to.")
    parser.add_argument("--quality-mip0", type=int, default=85)
    parser.add_argument("--quality-mip-low", type=int, default=80)
    parser.add_argument("--min-valid-fraction", type=float, default=0.5)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--jobs", type=int,
                        default=max(1, (os.cpu_count() or 4) // 2),
                        help="Worker processes (default: ncpu/2).")
    parser.add_argument("--delete-zips", action="store_true",
                        help="Delete each ZIP once every LV95 tile it covers "
                             "has been processed (write or skip-exists).")
    args = parser.parse_args()

    zip_dir = Path(args.zip_dir).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    if not zip_dir.is_dir():
        print(f"Error: {zip_dir} is not a directory", file=sys.stderr)
        return 1
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "mip2").mkdir(exist_ok=True)
    (output_dir / "mip3").mkdir(exist_ok=True)

    print(f"[1/3] Indexing ZIPs")
    extent = tuple(args.extent) if args.extent else None
    tile_to_zips, zip_to_tiles = build_index(zip_dir, extent)
    if not tile_to_zips:
        print("No tiles intersect the requested area. Nothing to do.")
        return 0
    print(f"      {len(tile_to_zips)} LV95 tile(s) covered "
          f"by {len(zip_to_tiles)} ZIP(s)")

    # Process in (tz, tx) order for spatial locality.
    work = sorted(((tx, tz, [str(p) for p in zips])
                   for (tx, tz), zips in tile_to_zips.items()),
                  key=lambda w: (w[1], w[0]))

    print(f"[2/3] Rendering tiles with {args.jobs} worker(s)")
    written = 0
    skipped_empty = 0
    skipped_exists = 0
    bytes_written = 0
    manifest_path = output_dir / "manifest.json"
    if manifest_path.exists():
        try:
            with manifest_path.open() as f:
                manifest = json.load(f)
            manifest_tiles = manifest.get("tiles", {})
        except Exception:
            manifest_tiles = {}
    else:
        manifest_tiles = {}

    # Track which LV95 tiles still need processing per ZIP.
    pending_per_zip: dict[Path, set[tuple[int, int]]] = {
        zp: set(tiles) for zp, tiles in zip_to_tiles.items()
    }

    init_args = (str(output_dir), args.quality_mip0, args.quality_mip_low,
                 args.min_valid_fraction, args.overwrite)
    ctx = mp.get_context("spawn")
    total = len(work)
    done = 0

    if args.jobs <= 1:
        _worker_init(*init_args)
        results_iter = (_worker_render(w) for w in work)
    else:
        pool = ctx.Pool(processes=args.jobs, initializer=_worker_init,
                        initargs=init_args)
        results_iter = pool.imap_unordered(_worker_render, work, chunksize=4)

    try:
        for status, tx, tz, lv95_e, lv95_n, fname, nbytes in results_iter:
            done += 1
            if status == "write":
                written += 1
                bytes_written += nbytes
                manifest_tiles[f"{tx}_{tz}"] = {
                    "lv95_e": lv95_e, "lv95_n": lv95_n,
                    "tile_ei": tx, "tile_ni": tz,
                    "file": fname,
                }
            elif status == "skip-exists":
                skipped_exists += 1
                manifest_tiles.setdefault(f"{tx}_{tz}", {
                    "lv95_e": lv95_e, "lv95_n": lv95_n,
                    "tile_ei": tx, "tile_ni": tz,
                    "file": fname,
                })
            else:
                skipped_empty += 1

            # Mark tile complete for every contributing ZIP, then drop ZIPs
            # whose pending set is now empty.
            for zp in tile_to_zips[(tx, tz)]:
                s = pending_per_zip.get(zp)
                if s is None:
                    continue
                s.discard((tx, tz))
                if not s:
                    pending_per_zip.pop(zp, None)
                    if args.delete_zips:
                        try:
                            zp.unlink()
                        except FileNotFoundError:
                            pass
                        except Exception as e:
                            print(f"  WARN cannot delete {zp.name}: {e}",
                                  file=sys.stderr)

            if done % 50 == 0 or done == total:
                mb = bytes_written / (1024 * 1024)
                print(f"  {done}/{total}  written={written}  "
                      f"skipped_exists={skipped_exists}  "
                      f"skipped_empty={skipped_empty}  ({mb:.1f} MB)")
    finally:
        if args.jobs > 1:
            pool.close()
            pool.join()

    print(f"[3/3] Writing manifest")
    manifest = {
        "tile_size_m": TILE_SIZE_M,
        "pixels_per_tile": MIP0_PX,
        "pixel_resolution_m": float(TILE_SIZE_M / MIP0_PX),
        "format": "jpeg_rgb",
        "mips": [
            {"name": "mip0", "pixels": MIP0_PX, "subdir": ""},
            {"name": "mip2", "pixels": MIP2_PX, "subdir": "mip2"},
            {"name": "mip3", "pixels": MIP3_PX, "subdir": "mip3"},
        ],
        "tile_count": len(manifest_tiles),
        "tiles": manifest_tiles,
    }
    with manifest_path.open("w") as f:
        json.dump(manifest, f, indent=2)

    mb = bytes_written / (1024 * 1024)
    print(f"Done: written={written}, skipped_exists={skipped_exists}, "
          f"skipped_empty={skipped_empty}, bytes={mb:.1f} MB. → {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
