#!/usr/bin/env python3
"""
Convert LGL BW DGM XYZ tiles to GeoTIFF, in parallel.

Why:
  GDAL's XYZ driver reads the entire ASCII file on rasterio.open() to
  determine the grid, which makes opening tens of thousands of XYZ files
  at once (as convert_lgl_bw.py does) impractical. GeoTIFF only reads the
  header on open(), so converting once up-front is much cheaper.

Behaviour:
  * Recursively finds *.xyz files under --input.
  * For each one, writes a sibling .tif with the same basename.
  * Skips files where an up-to-date .tif already exists.
  * On success, deletes the .xyz (override with --keep-xyz).
  * Parallelised with a process pool.

The XYZ files written by LGL BW are 1 m float32 grids in UTM32 (EPSG:25832)
with no embedded CRS, so the CRS is stamped onto the GeoTIFF via --src-crs.
"""

from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

try:
    import numpy as np
    import rasterio
    from rasterio.crs import CRS
except ImportError:
    print("Error: rasterio is required. Install with: pip install rasterio numpy",
          file=sys.stderr)
    sys.exit(1)


def convert_one(xyz_path_str: str, src_crs: str, keep_xyz: bool) -> tuple[str, str]:
    """Convert a single XYZ file. Returns (status, path)."""
    xyz_path = Path(xyz_path_str)
    tif_path = xyz_path.with_suffix(".tif")

    # Skip if an up-to-date TIF already exists.
    if tif_path.exists():
        try:
            if tif_path.stat().st_mtime >= xyz_path.stat().st_mtime:
                if not keep_xyz:
                    xyz_path.unlink(missing_ok=True)
                return ("skip", xyz_path_str)
        except FileNotFoundError:
            pass

    try:
        with rasterio.open(xyz_path) as src:
            data = src.read(1)
            transform = src.transform
            width = src.width
            height = src.height
        profile = {
            "driver": "GTiff",
            "dtype": "float32",
            "count": 1,
            "width": width,
            "height": height,
            "transform": transform,
            "crs": CRS.from_string(src_crs),
            "compress": "deflate",
            "predictor": 3,         # float predictor
            "tiled": True,
            "blockxsize": 256,
            "blockysize": 256,
            "BIGTIFF": "IF_SAFER",
        }
        tmp_path = tif_path.with_suffix(".tif.tmp")
        with rasterio.open(tmp_path, "w", **profile) as dst:
            dst.write(data.astype(np.float32), 1)
        tmp_path.replace(tif_path)
    except Exception as e:
        return ("fail", f"{xyz_path_str}: {e}")

    if not keep_xyz:
        xyz_path.unlink(missing_ok=True)
    return ("ok", xyz_path_str)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert LGL BW XYZ DGM tiles to GeoTIFF (in-place, parallel).",
    )
    parser.add_argument("--input", required=True,
                        help="Directory to scan recursively for *.xyz files.")
    parser.add_argument("--src-crs", default="EPSG:25832",
                        help="CRS to stamp on the GeoTIFFs (default: EPSG:25832).")
    parser.add_argument("--workers", type=int, default=8,
                        help="Parallel worker processes (default: 8).")
    parser.add_argument("--keep-xyz", action="store_true",
                        help="Do not delete .xyz files after successful conversion.")
    args = parser.parse_args()

    root = Path(args.input).expanduser().resolve()
    if not root.is_dir():
        print(f"Error: {root} is not a directory.", file=sys.stderr)
        return 1

    xyz_files = sorted(str(p) for p in root.rglob("*.xyz"))
    if not xyz_files:
        print(f"No .xyz files found under {root}.")
        return 0
    print(f"Found {len(xyz_files)} .xyz files under {root}", flush=True)

    ok = skip = fail = 0
    failures: list[str] = []
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(convert_one, p, args.src_crs, args.keep_xyz)
                for p in xyz_files]
        for i, fut in enumerate(as_completed(futs), 1):
            status, info = fut.result()
            if status == "ok":
                ok += 1
            elif status == "skip":
                skip += 1
            else:
                fail += 1
                failures.append(info)
            if i % 500 == 0 or i == len(futs):
                print(f"  progress {i}/{len(futs)} "
                      f"(ok={ok} skip={skip} fail={fail})", flush=True)

    if failures:
        print(f"\n{fail} failures (showing first 20):", file=sys.stderr)
        for f in failures[:20]:
            print(f"  {f}", file=sys.stderr)
        return 2
    print(f"Done: {ok} converted, {skip} already up-to-date.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
