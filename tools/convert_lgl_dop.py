#!/usr/bin/env python3
"""
LGL Baden-Württemberg DOP20 → Scenery3D orthophoto converter.

Reads RGB orthophoto rasters (LGL DOP20, 20 cm resolution, UTM32N /
EPSG:25832) and emits per-LV95-tile JPEGs on the same 1024 m grid as the
SwissTopo SWISSIMAGE pipeline so the runtime tile manager can stream BW
imagery alongside the Swiss orthos with no code changes.

Output layout (matches SWISSIMAGE):
  <output>/ortho_<E>_<N>.jpg              # mip0, 1024×1024 px (1 m/px)
  <output>/mip2/ortho_<E>_<N>.jpg         #  64×64 px (16 m/px)
  <output>/mip3/ortho_<E>_<N>.jpg         #  16×16 px (64 m/px)
  <output>/manifest.json                  # informational only
where E = tile_ei * 1024, N = tile_ni * 1024 (LV95 SW corner).

Source data:
  LGL BW Open GeoData Portal — https://opengeodata.lgl-bw.de
  Product `dop20rgb` (Digitales Orthophoto, 20 cm RGB), published as
  2 km × 2 km GeoTIFFs in UTM32N. Use tools/download_lgl_dop.py to
  fetch + extract the ZIPs first.

Requirements:
  pip install numpy rasterio Pillow

Usage:
  python tools/convert_lgl_dop.py \\
      --input  /path/to/dop20_extracted/ \\
      --output /path/to/Germany/Baden-Wuertemberg/orthophoto

  # restrict to an LV95 bounding box:
  python tools/convert_lgl_dop.py --input ... --output ... \\
      --extent 2560000 1250000 2700000 1330000
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

try:
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine
    from rasterio.vrt import WarpedVRT
    from rasterio.warp import reproject
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
DEFAULT_SRC_CRS = "EPSG:25832"  # UTM32N, used by LGL BW
TILE_SIZE_M = 1024              # LV95 grid spacing
MIP0_PX = 1024                  # 1 m/px
MIP2_PX = 64                    # 16 m/px (LOD ≥ 7)
MIP3_PX = 16                    # 64 m/px (chunks)


def open_sources(tif_paths: list[Path], src_crs_override: str | None):
    """Open RGB rasters and wrap each in an LV95 WarpedVRT."""
    opened = []
    for p in tif_paths:
        src = rasterio.open(p)
        src_crs = src.crs or src_crs_override
        if src_crs is None:
            src.close()
            raise RuntimeError(
                f"{p}: source CRS missing; pass --src-crs (e.g. EPSG:25832)."
            )
        if src.count < 3:
            src.close()
            print(f"  skip {p.name}: only {src.count} bands (RGB required)",
                  file=sys.stderr)
            continue
        # Bilinear resampling at the warp boundary; nearest would create
        # blocky artefacts at the LV95 grid cell edges.
        vrt = WarpedVRT(
            src,
            src_crs=src_crs,
            crs=LV95_CRS,
            resampling=Resampling.bilinear,
        )
        opened.append((vrt, src, vrt.bounds))
    return opened


def render_tile(sources, tx: int, tz: int,
                min_valid_fraction: float) -> np.ndarray | None:
    """Render a single LV95 1024 m tile as a 1024×1024 RGB uint8 array.

    Returns array in native Scenery3D layout (row 0 = south edge,
    col 0 = east edge), or None if no source overlaps the tile or
    coverage is below ``min_valid_fraction``.
    """
    e_min = tx * TILE_SIZE_M
    n_min = tz * TILE_SIZE_M
    e_max = e_min + TILE_SIZE_M
    n_max = n_min + TILE_SIZE_M

    pixel = TILE_SIZE_M / MIP0_PX  # = 1.0 m
    # Standard north-up affine in LV95: row 0 = north, col 0 = west.
    dst_transform = Affine(pixel, 0.0, e_min, 0.0, -pixel, n_max)

    # 3-band RGB scratch buffer.
    dst = np.zeros((3, MIP0_PX, MIP0_PX), dtype=np.uint8)
    cover = np.zeros((MIP0_PX, MIP0_PX), dtype=bool)

    for vrt, _src, b in sources:
        if (e_max <= b.left or e_min >= b.right or
                n_max <= b.bottom or n_min >= b.top):
            continue
        tmp = np.zeros_like(dst)
        for band_idx in range(3):
            reproject(
                source=rasterio.band(vrt, band_idx + 1),
                destination=tmp[band_idx],
                dst_transform=dst_transform,
                dst_crs=LV95_CRS,
                resampling=Resampling.bilinear,
            )
        # Coverage = any non-zero pixel across the three bands.
        tmp_cover = (tmp[0] | tmp[1] | tmp[2]).astype(bool)
        need = ~cover & tmp_cover
        if need.any():
            for c in range(3):
                dst[c][need] = tmp[c][need]
            cover |= need

    if not cover.any():
        return None
    valid_fraction = cover.sum() / cover.size
    if valid_fraction < min_valid_fraction:
        return None

    # Reorient to native layout: row 0 = south, col 0 = east → flip both.
    out = dst[:, ::-1, ::-1]
    # CHW → HWC for PIL.
    return np.ascontiguousarray(np.transpose(out, (1, 2, 0)))


def write_jpeg(arr_hwc: np.ndarray, path: Path, quality: int) -> int:
    """Write an HWC uint8 RGB array as JPEG. Returns bytes written."""
    img = Image.fromarray(arr_hwc, mode="RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path, format="JPEG", quality=quality, optimize=True,
             progressive=False)
    return path.stat().st_size


def downsample(arr_hwc: np.ndarray, target_px: int) -> np.ndarray:
    img = Image.fromarray(arr_hwc, mode="RGB")
    img = img.resize((target_px, target_px), Image.Resampling.LANCZOS)
    return np.asarray(img)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert LGL BW DOP20 RGB rasters to Scenery3D ortho JPEGs.",
    )
    parser.add_argument("--input", required=True,
                        help="File or directory with DOP20 GeoTIFFs.")
    parser.add_argument("--output", required=True,
                        help="Output directory for ortho_*.jpg + mip2/ + mip3/.")
    parser.add_argument("--src-crs", default=DEFAULT_SRC_CRS,
                        help=f"Source CRS for files without embedded CRS "
                             f"(default: {DEFAULT_SRC_CRS}).")
    parser.add_argument("--extent", type=float, nargs=4,
                        metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        help="LV95 bounding box (default: full source extent).")
    parser.add_argument("--pattern", default="*.tif,*.tiff,*.jp2",
                        help="Glob patterns when --input is a directory (recursive).")
    parser.add_argument("--quality-mip0", type=int, default=85,
                        help="JPEG quality for the 1024 px mip0 (default: 85).")
    parser.add_argument("--quality-mip-low", type=int, default=80,
                        help="JPEG quality for mip2/mip3 (default: 80).")
    parser.add_argument("--min-valid-fraction", type=float, default=0.5,
                        help="Skip tiles whose source coverage is below this "
                             "(default: 0.5). Avoids streaky edge tiles.")
    parser.add_argument("--overwrite", action="store_true",
                        help="Re-encode tiles that already exist on disk.")
    args = parser.parse_args()

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
    (output_dir / "mip2").mkdir(parents=True, exist_ok=True)
    (output_dir / "mip3").mkdir(parents=True, exist_ok=True)

    sources = open_sources(tif_files, args.src_crs)
    if not sources:
        print("Error: no usable RGB sources after CRS / band check.",
              file=sys.stderr)
        return 1

    if args.extent:
        e_min, n_min, e_max, n_max = args.extent
    else:
        e_min = min(b.left for _, _, b in sources)
        n_min = min(b.bottom for _, _, b in sources)
        e_max = max(b.right for _, _, b in sources)
        n_max = max(b.top for _, _, b in sources)
    print(f"LV95 extent: E [{e_min:.0f}, {e_max:.0f}]  N [{n_min:.0f}, {n_max:.0f}]")

    tx_min = int(np.floor(e_min / TILE_SIZE_M))
    tz_min = int(np.floor(n_min / TILE_SIZE_M))
    tx_max = int(np.floor((e_max - 1) / TILE_SIZE_M))
    tz_max = int(np.floor((n_max - 1) / TILE_SIZE_M))
    total = (tx_max - tx_min + 1) * (tz_max - tz_min + 1)
    print(f"Tile grid: tx [{tx_min}, {tx_max}]  tz [{tz_min}, {tz_max}]  "
          f"({total} candidate tiles)")

    written = 0
    skipped = 0
    bytes_written = 0
    manifest_tiles: dict[str, dict] = {}

    for tz in range(tz_min, tz_max + 1):
        for tx in range(tx_min, tx_max + 1):
            lv95_e = tx * TILE_SIZE_M
            lv95_n = tz * TILE_SIZE_M
            fname = f"ortho_{lv95_e}_{lv95_n}.jpg"
            mip0 = output_dir / fname
            mip2 = output_dir / "mip2" / fname
            mip3 = output_dir / "mip3" / fname

            if not args.overwrite and mip0.exists() and mip2.exists() and mip3.exists():
                skipped += 1
                manifest_tiles[f"{tx}_{tz}"] = {
                    "lv95_e": lv95_e, "lv95_n": lv95_n,
                    "tile_ei": tx, "tile_ni": tz,
                    "file": fname,
                }
                continue

            arr = render_tile(sources, tx, tz, args.min_valid_fraction)
            if arr is None:
                skipped += 1
                continue

            bytes_written += write_jpeg(arr, mip0, args.quality_mip0)
            bytes_written += write_jpeg(downsample(arr, MIP2_PX), mip2,
                                        args.quality_mip_low)
            bytes_written += write_jpeg(downsample(arr, MIP3_PX), mip3,
                                        args.quality_mip_low)
            manifest_tiles[f"{tx}_{tz}"] = {
                "lv95_e": lv95_e, "lv95_n": lv95_n,
                "tile_ei": tx, "tile_ni": tz,
                "file": fname,
            }
            written += 1
            if written % 50 == 0:
                mb = bytes_written / (1024 * 1024)
                print(f"  {written} tiles written ({mb:.1f} MB), {skipped} skipped …")

    for vrt, src, _ in sources:
        vrt.close()
        src.close()

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
    with open(output_dir / "manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)

    mb = bytes_written / (1024 * 1024)
    print(f"Done: {written} tiles written ({mb:.1f} MB), "
          f"{skipped} skipped. Output: {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
