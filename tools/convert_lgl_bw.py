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
import resource
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np


def _raise_fd_limit():
    """Try to raise RLIMIT_NOFILE to the hard cap. macOS default soft is 256."""
    try:
        soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
        target = hard if hard != resource.RLIM_INFINITY else 65536
        if soft < target:
            resource.setrlimit(resource.RLIMIT_NOFILE, (target, hard))
    except (ValueError, OSError):
        pass


_raise_fd_limit()

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


class SourceEntry:
    """Metadata about a source raster, with on-demand cached opener."""

    __slots__ = ("path", "src_crs", "src_nodata", "bounds")

    def __init__(self, path: Path, src_crs: str, src_nodata: float,
                 bounds: tuple[float, float, float, float]):
        self.path = path
        self.src_crs = src_crs
        self.src_nodata = src_nodata
        self.bounds = bounds  # LV95 (left, bottom, right, top)


class SourceCache:
    """LRU cache of (rasterio.DatasetReader, WarpedVRT) handles.

    Opening 35k+ files at once exhausts the OS file-descriptor limit; this
    class keeps at most `max_open` source pairs alive and lazily reopens
    others on demand.
    """

    def __init__(self, max_open: int = 128):
        self.max_open = max_open
        self._cache: "OrderedDict[Path, tuple]" = OrderedDict()

    def get(self, entry: SourceEntry):
        """Return (vrt, src) for `entry`, opening if necessary."""
        cached = self._cache.get(entry.path)
        if cached is not None:
            self._cache.move_to_end(entry.path)
            return cached
        while len(self._cache) >= self.max_open:
            _path, (old_vrt, old_src) = self._cache.popitem(last=False)
            try:
                old_vrt.close()
            finally:
                old_src.close()
        src = rasterio.open(entry.path)
        vrt = WarpedVRT(
            src,
            src_crs=entry.src_crs,
            crs=LV95_CRS,
            resampling=Resampling.bilinear,
            src_nodata=entry.src_nodata,
            nodata=np.nan,
        )
        self._cache[entry.path] = (vrt, src)
        return vrt, src

    def close_all(self):
        for _path, (vrt, src) in self._cache.items():
            try:
                vrt.close()
            finally:
                src.close()
        self._cache.clear()


def scan_sources(tif_paths: list[Path], src_crs_override: str | None,
                 progress_every: int = 500) -> list[SourceEntry]:
    """Open each raster briefly to capture its CRS, nodata and LV95 bounds.

    Files are closed immediately so we never hold more than one descriptor
    at a time during this scan.
    """
    entries: list[SourceEntry] = []
    for i, p in enumerate(tif_paths, start=1):
        with rasterio.open(p) as src:
            src_crs = src.crs.to_string() if src.crs else src_crs_override
            if src_crs is None:
                raise RuntimeError(
                    f"{p}: source CRS missing from file; pass --src-crs "
                    f"(e.g. EPSG:25832 for LGL BW UTM32N)."
                )
            src_nodata = src.nodata if src.nodata is not None else 0.0
            with WarpedVRT(
                src,
                src_crs=src_crs,
                crs=LV95_CRS,
                resampling=Resampling.bilinear,
                src_nodata=src_nodata,
                nodata=np.nan,
            ) as vrt:
                b = vrt.bounds
        entries.append(SourceEntry(p, src_crs, src_nodata,
                                   (b.left, b.bottom, b.right, b.top)))
        if progress_every and i % progress_every == 0:
            print(f"  scanned {i}/{len(tif_paths)} sources ...", flush=True)
    return entries


def tile_bounds(tx: int, tz: int, tile_size: int) -> tuple[float, float, float, float]:
    """Return (e_min, n_min, e_max, n_max) for an LV95 grid tile."""
    e_min = tx * tile_size
    n_min = tz * tile_size
    return (e_min, n_min, e_min + tile_size, n_min + tile_size)


def render_tile(
    sources: list[SourceEntry],
    cache: SourceCache,
    tx: int,
    tz: int,
    tile_size: int,
    samples: int,
    min_valid_fraction: float = 0.5,
) -> np.ndarray | None:
    """Render a single LV95 tile by reading from all overlapping sources.

    Returns an (samples, samples) float32 array in the native Scenery3D
    orientation (row 0 = south edge, col 0 = east edge), or None if no
    source overlaps this tile with real data, or if post-fill coverage
    is below `min_valid_fraction` (tile is mostly synthesised and would
    produce visible streaking at the seam with neighbouring tiles).
    """
    e_min, n_min, e_max, n_max = tile_bounds(tx, tz, tile_size)

    # Destination: standard north-up affine in LV95.
    # row 0 = north edge, col 0 = west edge, pixel size = tile_size / samples.
    pixel = tile_size / samples
    dst_transform = Affine(pixel, 0.0, e_min, 0.0, -pixel, n_max)
    dst = np.full((samples, samples), np.nan, dtype=np.float32)

    filled = False
    for entry in sources:
        left, bottom, right, top = entry.bounds
        if (e_max <= left or e_min >= right or
                n_max <= bottom or n_min >= top):
            continue
        vrt, _src = cache.get(entry)
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
        # Reject tiles whose real source coverage is too sparse: even with
        # inpainting, the result is a patchwork that creates visible seams
        # against neighbouring, fully-covered tiles.
        valid_fraction = float(mask.sum()) / mask.size
        if valid_fraction < min_valid_fraction:
            return None
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
    parser.add_argument("--max-open-sources", type=int, default=128,
                        help="Maximum number of source rasters kept open "
                             "simultaneously (LRU). Lower this if you hit "
                             "'Too many open files'. Default: 128.")
    parser.add_argument("--resume", action="store_true", default=True,
                        help="Skip tiles whose output .raw already exists. "
                             "Default: on. Use --no-resume to force.")
    parser.add_argument("--no-resume", dest="resume", action="store_false",
                        help="Re-render every tile, even if its output exists.")
    parser.add_argument("--min-valid-fraction", type=float, default=0.5,
                        help="Minimum fraction of a tile's pixels that must "
                             "come from real source data (before inpainting) "
                             "for the tile to be written. Tiles below this "
                             "would stretch sparse data across the whole "
                             "footprint and cause seams against full tiles. "
                             "Default: 0.5.")
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

    print("Scanning source bounds (one open at a time) ...", flush=True)
    sources = scan_sources(tif_files, args.src_crs)
    cache = SourceCache(max_open=args.max_open_sources)

    # Determine processing extent in LV95.
    if args.extent:
        e_min, n_min, e_max, n_max = args.extent
    else:
        e_min = min(s.bounds[0] for s in sources)
        n_min = min(s.bounds[1] for s in sources)
        e_max = max(s.bounds[2] for s in sources)
        n_max = max(s.bounds[3] for s in sources)
    print(f"LV95 extent: E [{e_min:.0f}, {e_max:.0f}]  N [{n_min:.0f}, {n_max:.0f}]")

    tx_min = int(np.floor(e_min / tile_size))
    tz_min = int(np.floor(n_min / tile_size))
    tx_max = int(np.floor((e_max - 1) / tile_size))
    tz_max = int(np.floor((n_max - 1) / tile_size))

    total = (tx_max - tx_min + 1) * (tz_max - tz_min + 1)
    print(f"Tile grid: tx [{tx_min}, {tx_max}]  tz [{tz_min}, {tz_max}]  ({total} candidate tiles)")

    # Build a per-row spatial index so we don't iterate all 35k sources
    # for every tile.
    sources_by_tz: dict[int, list[SourceEntry]] = {}
    for s in sources:
        left, bottom, right, top = s.bounds
        tz_lo = int(np.floor(bottom / tile_size))
        tz_hi = int(np.floor((top - 1) / tile_size))
        for tz in range(tz_lo, tz_hi + 1):
            sources_by_tz.setdefault(tz, []).append(s)

    written = 0
    skipped = 0
    resumed = 0
    for tz in range(tz_min, tz_max + 1):
        row_sources = sources_by_tz.get(tz, [])
        if not row_sources:
            continue
        for tx in range(tx_min, tx_max + 1):
            lv95_east = tx * tile_size
            lv95_north = tz * tile_size
            fname = f"tile_{lv95_east}_{lv95_north}.raw"
            out_path = output_dir / fname
            if args.resume and out_path.exists():
                resumed += 1
                continue
            data = render_tile(row_sources, cache, tx, tz, tile_size,
                               samples, min_valid_fraction=args.min_valid_fraction)
            if data is None:
                skipped += 1
                continue
            tmp_path = out_path.with_suffix(".raw.tmp")
            with open(tmp_path, "wb") as f:
                f.write(data.astype(np.float32).tobytes())
            tmp_path.replace(out_path)
            written += 1

            if written % 100 == 0:
                print(f"  {written} tiles written, {skipped} empty skipped, "
                      f"{resumed} resumed ...", flush=True)

    cache.close_all()

    print(f"Done: {written} tiles written to {output_dir} "
          f"({skipped} empty tiles skipped, {resumed} pre-existing).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
