#!/usr/bin/env python3
"""
LGL Baden-Württemberg DGM → Scenery3D heightmap converter.

Reads one or more elevation rasters in any rasterio-supported format
(GeoTIFF, ASC, VRT, ...) from LGL BW (Landesamt für Geoinformation und
Landentwicklung Baden-Württemberg) — DGM1 (1 m, recommended, matches
swissALTI3D detail) or DGM25 in UTM32N (EPSG:25832) — and emits tiles
on the same LV95 (EPSG:2056) grid used by the existing SwissTopo pipeline,
so BW tiles can be streamed seamlessly alongside the Swiss terrain.

Output format (provpilot / Scenery3D "raw" tile):
  * headerless float32 little-endian, size * size samples
  * pixel(0, 0) = SE corner of the tile
  * columns go E → W (col 0 = east edge)
  * rows go S → N (row 0 = south edge)
  * filename: tile_{lv95_east}_{lv95_north}.raw
    where lv95_east  = tx * tile_size
          lv95_north = tz * tile_size
    (i.e. the SW / lower-left corner in LV95, matching the SwissTopo tiles)

Source data:
  LGL BW Open GeoData Portal — https://opengeodata.lgl-bw.de
  Open data products: DGM1 (1 m, recommended) and DGM25 (25 m, coarse).
  Tiles are published in UTM32N; if your downloads are in XYZ ASCII,
  convert them first, e.g.:
      gdal_translate -of GTiff input.xyz input.tif

Requirements:
  pip install numpy rasterio pyproj

Usage:
  python convert_lgl_bw.py \\
      --input  /path/to/dgm10_tiles \\
      --output /path/to/Germany/terrain \\
      --tile-size 1024

  # restrict to a bounding box (LV95):
  python convert_lgl_bw.py --input ... --output ... \\
      --extent 2560000 1250000 2700000 1330000
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.fill import fillnodata
    from rasterio.transform import Affine
    from rasterio.vrt import WarpedVRT
    from rasterio.warp import reproject
except ImportError:
    print("Error: rasterio is required. Install with: pip install rasterio",
          file=sys.stderr)
    sys.exit(1)

LV95_CRS = "EPSG:2056"
DEFAULT_SRC_CRS = "EPSG:25832"  # UTM32N, used by LGL BW


def open_sources(tif_paths: list[Path], src_crs_override: str | None):
    """Open rasters and return list of (WarpedVRT in LV95, lv95_bounds)."""
    opened = []
    for p in tif_paths:
        src = rasterio.open(p)
        src_crs = src.crs or src_crs_override
        if src_crs is None:
            src.close()
            raise RuntimeError(
                f"{p}: source CRS missing from file; pass --src-crs "
                f"(e.g. EPSG:25832 for LGL BW UTM32N)."
            )
        # DGM1 XYZ files encode water (Rhine, Bodensee, …) as nodata = 0.0.
        # Force that through the VRT so bilinear resampling doesn't blend
        # real elevations against 0 and produce edge cliffs along rivers.
        src_nodata = src.nodata if src.nodata is not None else 0.0
        # Lazy-reproject the source into LV95. WarpedVRT handles the
        # per-read bilinear resampling.
        vrt = WarpedVRT(
            src,
            src_crs=src_crs,
            crs=LV95_CRS,
            resampling=Resampling.bilinear,
            src_nodata=src_nodata,
            nodata=np.nan,
        )
        b = vrt.bounds  # already in LV95
        opened.append((vrt, src, b))
    return opened


def tile_bounds(tx: int, tz: int, tile_size: int) -> tuple[float, float, float, float]:
    """Return (e_min, n_min, e_max, n_max) for an LV95 grid tile."""
    e_min = tx * tile_size
    n_min = tz * tile_size
    return (e_min, n_min, e_min + tile_size, n_min + tile_size)


def render_tile(
    sources,
    tx: int,
    tz: int,
    tile_size: int,
    samples: int,
) -> np.ndarray | None:
    """Render a single LV95 tile by reading from all overlapping sources.

    Returns an (samples, samples) float32 array in the native Scenery3D
    orientation (row 0 = south edge, col 0 = east edge), or None if no
    source overlaps this tile with real data.
    """
    e_min, n_min, e_max, n_max = tile_bounds(tx, tz, tile_size)

    # Destination: standard north-up affine in LV95.
    # row 0 = north edge, col 0 = west edge, pixel size = tile_size / samples.
    pixel = tile_size / samples
    dst_transform = Affine(pixel, 0.0, e_min, 0.0, -pixel, n_max)
    dst = np.full((samples, samples), np.nan, dtype=np.float32)

    filled = False
    for vrt, _src, b in sources:
        if (e_max <= b.left or e_min >= b.right or
                n_max <= b.bottom or n_min >= b.top):
            continue
        # Reproject this source into a NaN-initialised scratch buffer.
        # Where the source is nodata (water, out-of-coverage) we get NaN,
        # which we treat as "no info" during merge and fill later.
        tmp = np.full_like(dst, np.nan)
        reproject(
            source=rasterio.band(vrt, 1),
            destination=tmp,
            dst_transform=dst_transform,
            dst_crs=LV95_CRS,
            resampling=Resampling.bilinear,
            src_nodata=np.nan,
            dst_nodata=np.nan,
            init_dest_nodata=False,
        )
        # Merge: fill still-empty (NaN) destination cells from tmp.
        need = np.isnan(dst) & ~np.isnan(tmp)
        if need.any():
            dst[need] = tmp[need]
            filled = True

    if not filled:
        return None

    # Inpaint any remaining NaN (water, small holes) from nearby valid
    # pixels. fillnodata uses GDAL's inverse-distance-weighted interpolation,
    # which on water surfaces gives an elevation close to the surrounding
    # shore — good enough for terrain rendering.
    mask = (~np.isnan(dst)).astype(np.uint8)
    if mask.all():
        pass  # no holes
    elif not mask.any():
        return None  # nothing valid at all
    else:
        dst = fillnodata(dst, mask=mask, max_search_distance=100.0,
                         smoothing_iterations=0)
        # After fillnodata, any still-NaN cell is beyond max_search_distance;
        # drop it to 0 so we don't emit NaNs into the heightmap.
        dst = np.where(np.isnan(dst), 0.0, dst).astype(np.float32)

    # Reorient to native Scenery3D layout:
    #   native[prow][pcol] where row 0 = south, col 0 = east
    # Our dst has row 0 = north, col 0 = west → flip both axes.
    return np.ascontiguousarray(dst[::-1, ::-1])


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert LGL BW DGM rasters to Scenery3D .raw tiles on the LV95 grid.",
    )
    parser.add_argument("--input", required=True,
                        help="File or directory with elevation rasters (tif/asc/vrt).")
    parser.add_argument("--output", required=True,
                        help="Output directory for tile_*.raw files.")
    parser.add_argument("--tile-size", type=int, default=1024,
                        help="LV95 tile size in metres and samples per side (default: 1024).")
    parser.add_argument("--samples", type=int, default=None,
                        help="Samples per tile side (default: same as --tile-size, i.e. 1 m/px).")
    parser.add_argument("--src-crs", default=DEFAULT_SRC_CRS,
                        help=f"Source CRS for files without embedded CRS (default: {DEFAULT_SRC_CRS}).")
    parser.add_argument("--extent", type=float, nargs=4,
                        metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        help="LV95 bounding box to convert (default: full source extent).")
    parser.add_argument("--pattern", default="*.tif,*.tiff,*.asc,*.xyz,*.vrt",
                        help="Comma-separated glob patterns when --input is a directory (recursive).")
    args = parser.parse_args()

    tile_size = args.tile_size
    samples = args.samples or tile_size

    input_path = Path(args.input)
    if input_path.is_dir():
        patterns = [p.strip() for p in args.pattern.split(",") if p.strip()]
        tif_files: list[Path] = []
        for pat in patterns:
            tif_files.extend(sorted(input_path.rglob(pat)))
        tif_files = sorted(set(tif_files))
    elif input_path.is_file():
        tif_files = [input_path]
    else:
        print(f"Error: {input_path} not found.", file=sys.stderr)
        return 1

    if not tif_files:
        print(f"Error: no raster files found in {input_path}", file=sys.stderr)
        return 1
    print(f"Found {len(tif_files)} source raster(s)")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    sources = open_sources(tif_files, args.src_crs)

    # Determine processing extent in LV95.
    if args.extent:
        e_min, n_min, e_max, n_max = args.extent
    else:
        e_min = min(b.left for _, _, b in sources)
        n_min = min(b.bottom for _, _, b in sources)
        e_max = max(b.right for _, _, b in sources)
        n_max = max(b.top for _, _, b in sources)
    print(f"LV95 extent: E [{e_min:.0f}, {e_max:.0f}]  N [{n_min:.0f}, {n_max:.0f}]")

    tx_min = int(np.floor(e_min / tile_size))
    tz_min = int(np.floor(n_min / tile_size))
    tx_max = int(np.floor((e_max - 1) / tile_size))
    tz_max = int(np.floor((n_max - 1) / tile_size))

    total = (tx_max - tx_min + 1) * (tz_max - tz_min + 1)
    print(f"Tile grid: tx [{tx_min}, {tx_max}]  tz [{tz_min}, {tz_max}]  ({total} candidate tiles)")

    written = 0
    skipped = 0
    for tz in range(tz_min, tz_max + 1):
        for tx in range(tx_min, tx_max + 1):
            data = render_tile(sources, tx, tz, tile_size, samples)
            if data is None:
                skipped += 1
                continue

            # Match SwissTopo filename: tile_{lv95_east}_{lv95_north}.raw
            lv95_east = tx * tile_size
            lv95_north = tz * tile_size
            fname = f"tile_{lv95_east}_{lv95_north}.raw"
            with open(output_dir / fname, "wb") as f:
                f.write(data.astype(np.float32).tobytes())
            written += 1

            if written % 100 == 0:
                print(f"  {written} tiles written, {skipped} empty skipped ...")

    # Close sources.
    for vrt, src, _ in sources:
        vrt.close()
        src.close()

    print(f"Done: {written} tiles written to {output_dir} ({skipped} empty tiles skipped).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
