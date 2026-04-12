#!/usr/bin/env python3
"""
SwissTopo GeoTIFF to Scenery3D heightmap converter.

Converts swissALTI3D GeoTIFF elevation tiles into the binary heightmap format
expected by the Scenery3D Godot plugin.

Output format (.heightmap):
  - 4 bytes: width  (uint32, little-endian)
  - 4 bytes: height (uint32, little-endian)
  - width * height * 4 bytes: elevation values (float32, little-endian)

Tile naming convention:
  tile_{tx}_{tz}.heightmap
  where tx = floor(east / tile_size), tz = floor(north / tile_size)
  relative to the configured LV95 origin.

Requirements:
  pip install numpy rasterio

Usage:
  python convert_swisstopo.py --input ./geotiff_dir --output ./heightmaps \\
      --origin-east 2600000 --origin-north 1200000 --tile-size 1000
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

try:
    import rasterio
    from rasterio.windows import Window
except ImportError:
    print("Error: rasterio is required. Install with: pip install rasterio", file=sys.stderr)
    sys.exit(1)


def extract_tile(src, east_min, north_min, tile_size, tile_resolution):
    """Extract a single tile from a rasterio dataset.

    Args:
        src: Open rasterio dataset.
        east_min: LV95 easting of tile's west edge.
        north_min: LV95 northing of tile's south edge.
        tile_size: Tile size in meters.
        tile_resolution: Output resolution in meters per pixel.

    Returns:
        2D numpy array of float32 elevation values, or None if no overlap.
    """
    east_max = east_min + tile_size
    north_max = north_min + tile_size

    # Check overlap with the dataset bounds.
    bounds = src.bounds
    if (east_min >= bounds.right or east_max <= bounds.left or
            north_min >= bounds.top or north_max <= bounds.bottom):
        return None

    # GeoTIFF row 0 is at the top (north), so north_max maps to the top row.
    col_off = (east_min - bounds.left) / src.res[0]
    row_off = (bounds.top - north_max) / src.res[1]
    col_size = tile_size / src.res[0]
    row_size = tile_size / src.res[1]

    window = Window(col_off, row_off, col_size, row_size)

    # Read with resampling to desired output size.
    out_pixels = int(tile_size / tile_resolution)
    data = src.read(
        1,
        window=window,
        out_shape=(out_pixels, out_pixels),
        boundless=True,
        fill_value=0.0,
    )

    return data.astype(np.float32)


def write_heightmap(path, data):
    """Write elevation data in the Scenery3D binary heightmap format."""
    h, w = data.shape
    with open(path, "wb") as f:
        f.write(struct.pack("<II", w, h))
        f.write(data.tobytes())


def main():
    parser = argparse.ArgumentParser(
        description="Convert SwissTopo GeoTIFF elevation data to Scenery3D heightmaps."
    )
    parser.add_argument("--input", required=True, help="Directory containing .tif GeoTIFF files.")
    parser.add_argument("--output", required=True, help="Output directory for .heightmap files.")
    parser.add_argument("--origin-east", type=float, default=2600000.0,
                        help="LV95 easting of the Godot world origin (default: 2600000).")
    parser.add_argument("--origin-north", type=float, default=1200000.0,
                        help="LV95 northing of the Godot world origin (default: 1200000).")
    parser.add_argument("--tile-size", type=int, default=1000,
                        help="Tile size in meters (default: 1000).")
    parser.add_argument("--resolution", type=float, default=2.0,
                        help="Output resolution in meters per pixel (default: 2.0).")
    parser.add_argument("--extent", type=float, nargs=4, metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        help="LV95 extent to convert. If omitted, uses the union of all input files.")

    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    tif_files = sorted(input_dir.glob("*.tif")) + sorted(input_dir.glob("*.tiff"))
    if not tif_files:
        print(f"No GeoTIFF files found in {input_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(tif_files)} GeoTIFF file(s)")

    # Determine processing extent.
    if args.extent:
        e_min, n_min, e_max, n_max = args.extent
    else:
        # Compute union of all input bounds.
        e_min = float("inf")
        n_min = float("inf")
        e_max = float("-inf")
        n_max = float("-inf")
        for tif in tif_files:
            with rasterio.open(tif) as src:
                b = src.bounds
                e_min = min(e_min, b.left)
                n_min = min(n_min, b.bottom)
                e_max = max(e_max, b.right)
                n_max = max(n_max, b.top)
        print(f"Input extent: E [{e_min:.0f}, {e_max:.0f}] N [{n_min:.0f}, {n_max:.0f}]")

    # Snap extent to tile grid.
    ts = args.tile_size
    tx_min = int(np.floor((e_min - args.origin_east) / ts))
    tz_min = int(np.floor((n_min - args.origin_north) / ts))
    tx_max = int(np.floor((e_max - args.origin_east) / ts))
    tz_max = int(np.floor((n_max - args.origin_north) / ts))

    total_tiles = (tx_max - tx_min + 1) * (tz_max - tz_min + 1)
    print(f"Tile grid: tx [{tx_min}, {tx_max}], tz [{tz_min}, {tz_max}] ({total_tiles} tiles)")

    # Open all source datasets.
    sources = [rasterio.open(tif) for tif in tif_files]

    manifest = {
        "origin_east": args.origin_east,
        "origin_north": args.origin_north,
        "tile_size": ts,
        "resolution": args.resolution,
        "tiles": [],
    }

    generated = 0
    for tz in range(tz_min, tz_max + 1):
        for tx in range(tx_min, tx_max + 1):
            east_min = args.origin_east + tx * ts
            north_min = args.origin_north + tz * ts

            # Try each source; use the first one that overlaps.
            tile_data = None
            for src in sources:
                tile_data = extract_tile(src, east_min, north_min, ts, args.resolution)
                if tile_data is not None:
                    break

            if tile_data is None:
                continue

            # Skip tiles that are entirely zero (no data).
            if np.all(tile_data == 0.0):
                continue

            filename = f"tile_{tx}_{tz}.heightmap"
            write_heightmap(output_dir / filename, tile_data)
            manifest["tiles"].append({"tx": tx, "tz": tz, "file": filename})
            generated += 1

            if generated % 50 == 0:
                print(f"  Generated {generated} tiles...")

    # Close sources.
    for src in sources:
        src.close()

    # Write manifest.
    manifest_path = output_dir / "manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"Done: {generated} tiles written to {output_dir}")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
