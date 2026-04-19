#!/usr/bin/env python3
"""
Locate gaps in the BW (Baden-Württemberg) Scenery3D terrain directory
and map each missing LV95 tile back to the LGL 2 km source ZIP cells
that would cover it.  Useful when the source XYZ data has been deleted
and you need to re-download only the missing bits.

Pipeline:
  1. Scan --terrain-dir for tile_{E}_{N}.raw files.
  2. Optionally restrict to an LV95 bounding box (--lv95-extent).
     Otherwise use the min/max of existing tiles as the interior bbox
     so border gaps (outside BW) are not reported.
  3. Report every missing (ei, ni) inside that bbox.
  4. For each missing tile, sample a grid of LV95 points, reproject to
     UTM32 (EPSG:25832), and collect the unique set of LGL 2 km source
     cells (E_km, N_km multiples of 2) that overlap the tile.
  5. Optionally HEAD-probe https://opengeodata.lgl-bw.de to show which
     cells are actually published (some UTM32 cells don't exist or are
     404).
  6. Print a UTM32 bbox you can pass to download_lgl_bw.py --extent,
     plus an explicit list of (E_km, N_km) cells.

Usage:
  python tools/find_missing_bw_tiles.py \\
      --terrain-dir /Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/terrain

  # Check the LGL server and write a cells file suitable for feeding
  # back into a targeted download:
  python tools/find_missing_bw_tiles.py --terrain-dir ... \\
      --probe-server --output-cells missing_cells.txt

  # Restrict to a specific LV95 region (e.g. Bodensee / north of CH):
  python tools/find_missing_bw_tiles.py --terrain-dir ... \\
      --lv95-extent 2640000 1280000 2780000 1330000
"""

from __future__ import annotations

import argparse
import concurrent.futures
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

try:
    from pyproj import Transformer
except ImportError:
    print("Error: pyproj is required. pip install pyproj", file=sys.stderr)
    sys.exit(1)

LV95_CRS = "EPSG:2056"
UTM32_CRS = "EPSG:25832"
LGL_BASE = "https://opengeodata.lgl-bw.de/data/dgm"
USER_AGENT = "scenery3d-missing-tile-finder/1.0"

TILE_RE = re.compile(r"^tile_(-?\d+)_(-?\d+)\.raw$")


def scan_tiles(terrain_dir: Path, tile_size: int) -> set[tuple[int, int]]:
    """Return set of (ei, ni) tile indices present on disk."""
    present: set[tuple[int, int]] = set()
    for p in terrain_dir.iterdir():
        m = TILE_RE.match(p.name)
        if not m:
            continue
        e_m = int(m.group(1))
        n_m = int(m.group(2))
        if e_m % tile_size or n_m % tile_size:
            continue
        present.add((e_m // tile_size, n_m // tile_size))
    return present


def find_missing(present: set[tuple[int, int]],
                 bbox: tuple[int, int, int, int]) -> list[tuple[int, int]]:
    """Return list of (ei, ni) inside bbox not in `present`."""
    ei_min, ni_min, ei_max, ni_max = bbox
    missing: list[tuple[int, int]] = []
    for ei in range(ei_min, ei_max + 1):
        for ni in range(ni_min, ni_max + 1):
            if (ei, ni) not in present:
                missing.append((ei, ni))
    return missing


def lv95_tile_to_utm_cells(ei: int, ni: int, tile_size: int,
                            transformer: Transformer,
                            grid: int = 5) -> set[tuple[int, int]]:
    """Map one LV95 tile to the set of UTM32 2 km cells it intersects.

    LGL BW anchors cells at (odd E km, even N km) — see
    download_lgl_bw.py, which probes every integer km for that reason.
    Snap each sample to (odd E, even N) cell anchor accordingly.

    Samples a `grid`×`grid` lattice across the tile footprint; for each
    sample, reprojects to UTM32, then snaps to the cell anchor.  Grid=5
    is generous for 1024 m tiles (≈256 m step).
    """
    cells: set[tuple[int, int]] = set()
    e_min = ei * tile_size
    n_min = ni * tile_size
    for gy in range(grid):
        for gx in range(grid):
            fx = gx / (grid - 1)
            fy = gy / (grid - 1)
            lv_e = e_min + fx * tile_size
            lv_n = n_min + fy * tile_size
            utm_e, utm_n = transformer.transform(lv_e, lv_n)
            e_km = int(utm_e // 1000)
            n_km = int(utm_n // 1000)
            # LGL anchors: E is odd, N is even (2 km cells).
            if e_km % 2 == 0:
                e_km -= 1
            if n_km % 2 != 0:
                n_km -= 1
            cells.add((e_km, n_km))
    return cells


def probe_server(cells: list[tuple[int, int]], workers: int = 32
                 ) -> dict[tuple[int, int], int]:
    """HEAD-probe each cell, return {cell: http_status}.

    Uses as_completed so one slow request doesn't stall progress reporting.
    Falls back to a ranged GET if HEAD times out (some CDN frontends
    throttle HEAD heavily).
    """
    import time

    def probe(cell):
        e, n = cell
        url = f"{LGL_BASE}/dgm1_32_{e}_{n}_2_bw.zip"
        for method in ("HEAD", "GET"):
            req = urllib.request.Request(
                url, method=method,
                headers={"User-Agent": USER_AGENT,
                         **({"Range": "bytes=0-0"} if method == "GET" else {})})
            try:
                with urllib.request.urlopen(req, timeout=8) as resp:
                    return cell, resp.status
            except urllib.error.HTTPError as e:
                return cell, e.code
            except (urllib.error.URLError, TimeoutError, OSError):
                continue
        return cell, 0

    result: dict[tuple[int, int], int] = {}
    total = len(cells)
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        futures = [ex.submit(probe, c) for c in cells]
        done = 0
        ok = 0
        nf = 0
        for fut in concurrent.futures.as_completed(futures):
            cell, code = fut.result()
            result[cell] = code
            done += 1
            if code == 200:
                ok += 1
            elif code == 404:
                nf += 1
            if done % 100 == 0 or done == total:
                elapsed = time.time() - t0
                rate = done / elapsed if elapsed > 0 else 0
                print(f"    probed {done}/{total}  200:{ok} 404:{nf}  "
                      f"{rate:.0f}/s", flush=True)
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--terrain-dir", required=True,
                    help="BW terrain dir with tile_{E}_{N}.raw files.")
    ap.add_argument("--tile-size", type=int, default=1024)
    ap.add_argument("--lv95-extent", type=float, nargs=4,
                    metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                    help="LV95 bbox to check (default: bbox of existing tiles).")
    ap.add_argument("--shrink", type=int, default=0,
                    help="Shrink the inferred bbox by N tiles on each side "
                         "(default: 0). Use this to ignore border stragglers.")
    ap.add_argument("--probe-server", action="store_true",
                    help="HEAD-probe LGL portal for each candidate source cell.")
    ap.add_argument("--probe-workers", type=int, default=32)
    ap.add_argument("--output-cells",
                    help="Write discovered UTM32 (E_km N_km) cells to a file.")
    ap.add_argument("--limit", type=int, default=0,
                    help="Stop after N missing tiles (debug).")
    ap.add_argument("--check-quality", action="store_true",
                    help="Also inspect existing .raw tiles for excessive "
                         "zero-elevation coverage (indicating sparse source "
                         "data that caused the tile manager to streak-fill). "
                         "Tiles below --min-valid-fraction of nonzero pixels "
                         "are reported.")
    ap.add_argument("--min-valid-fraction", type=float, default=0.5,
                    help="Threshold used by --check-quality (default: 0.5).")
    ap.add_argument("--delete-bad", action="store_true",
                    help="With --check-quality, delete the bad tiles so "
                         "they can be re-rendered from better source data "
                         "or remain as gaps.")
    args = ap.parse_args()

    terrain_dir = Path(args.terrain_dir).expanduser()
    if not terrain_dir.is_dir():
        print(f"Error: {terrain_dir} is not a directory.", file=sys.stderr)
        return 1

    present = scan_tiles(terrain_dir, args.tile_size)
    print(f"Scanned {terrain_dir}: {len(present)} tiles present")
    if not present:
        print("No tiles found — nothing to analyse.")
        return 1

    if args.lv95_extent:
        e_min, n_min, e_max, n_max = args.lv95_extent
        ei_min = int(e_min // args.tile_size)
        ni_min = int(n_min // args.tile_size)
        ei_max = int((e_max - 1) // args.tile_size)
        ni_max = int((n_max - 1) // args.tile_size)
    else:
        ei_all = [ei for ei, _ in present]
        ni_all = [ni for _, ni in present]
        ei_min, ei_max = min(ei_all) + args.shrink, max(ei_all) - args.shrink
        ni_min, ni_max = min(ni_all) + args.shrink, max(ni_all) - args.shrink

    bbox_str = (f"ei [{ei_min}, {ei_max}] × ni [{ni_min}, {ni_max}]  "
                f"→ LV95 E [{ei_min * args.tile_size}, {(ei_max + 1) * args.tile_size}]"
                f" × N [{ni_min * args.tile_size}, {(ni_max + 1) * args.tile_size}]")
    print(f"Scan area: {bbox_str}")

    missing = find_missing(present, (ei_min, ni_min, ei_max, ni_max))
    if args.limit:
        missing = missing[:args.limit]
    print(f"Missing tiles: {len(missing)}")

    # --- Quality check: find existing tiles that are mostly zero ---
    bad_tiles: list[tuple[int, int, float]] = []
    if args.check_quality:
        import numpy as np

        print(f"\nChecking tile quality (min_valid_fraction="
              f"{args.min_valid_fraction}) ...")
        expected = args.tile_size * args.tile_size * 4
        checked = 0
        for ei, ni in sorted(present):
            if not (ei_min <= ei <= ei_max and ni_min <= ni <= ni_max):
                continue
            p = terrain_dir / f"tile_{ei * args.tile_size}_{ni * args.tile_size}.raw"
            try:
                data = np.fromfile(p, dtype=np.float32)
            except OSError:
                continue
            if data.size * 4 != expected:
                continue
            valid = float(np.count_nonzero(data)) / data.size
            if valid < args.min_valid_fraction:
                bad_tiles.append((ei, ni, valid))
            checked += 1
            if checked % 2000 == 0:
                print(f"  checked {checked} tiles, {len(bad_tiles)} bad so far")
        print(f"  checked {checked} tiles: {len(bad_tiles)} below threshold")

        if bad_tiles:
            print("  worst 5:")
            for ei, ni, vf in sorted(bad_tiles, key=lambda x: x[2])[:5]:
                print(f"    tile_{ei * args.tile_size}_{ni * args.tile_size}.raw  "
                      f"valid={vf*100:.1f}%")

        if args.delete_bad and bad_tiles:
            removed = 0
            for ei, ni, _ in bad_tiles:
                p = terrain_dir / f"tile_{ei * args.tile_size}_{ni * args.tile_size}.raw"
                try:
                    p.unlink()
                    removed += 1
                except OSError:
                    pass
            print(f"  deleted {removed} bad tiles — re-run finder to get "
                  f"updated missing-cell list for re-download.")
            # Treat them as missing for the rest of this run.
            missing.extend((ei, ni) for ei, ni, _ in bad_tiles)

    if not missing:
        return 0

    # Map each missing tile to source cells.
    transformer = Transformer.from_crs(LV95_CRS, UTM32_CRS, always_xy=True)
    all_cells: set[tuple[int, int]] = set()
    for ei, ni in missing:
        all_cells |= lv95_tile_to_utm_cells(ei, ni, args.tile_size, transformer,
                                            grid=9)

    cells_sorted = sorted(all_cells)
    # UTM32 bbox enclosing all cells (km).
    e_km_min = min(c[0] for c in cells_sorted)
    n_km_min = min(c[1] for c in cells_sorted)
    e_km_max = max(c[0] for c in cells_sorted) + 2  # +2 km to cover cell extent
    n_km_max = max(c[1] for c in cells_sorted) + 2
    print(f"\nUTM32 coverage: {len(cells_sorted)} source 2 km cells")
    print(f"UTM32 bbox (km): E [{e_km_min}, {e_km_max}]  N [{n_km_min}, {n_km_max}]")
    print(f"\nFor a bulk re-download (will re-probe all intermediate cells):")
    print(f"  python tools/download_lgl_bw.py \\")
    print(f"      --extent {e_km_min} {n_km_min} {e_km_max} {n_km_max} \\")
    print(f"      --download-dir <ZIPDIR> \\")
    print(f"      --output {terrain_dir}")

    server_status: dict[tuple[int, int], int] = {}
    if args.probe_server:
        print(f"\nProbing {len(cells_sorted)} candidate cells on LGL server "
              f"(workers={args.probe_workers})...")
        server_status = probe_server(cells_sorted, workers=args.probe_workers)
        avail = [c for c, s in server_status.items() if s == 200]
        miss_404 = [c for c, s in server_status.items() if s == 404]
        err = [c for c, s in server_status.items() if s not in (200, 404)]
        print(f"  {len(avail)} cells AVAILABLE, {len(miss_404)} return 404, "
              f"{len(err)} network/other")
        if err:
            print(f"  (network errors — re-run to retry: "
                  f"{sorted(err)[:5]}{'...' if len(err) > 5 else ''})")

    if args.output_cells:
        outp = Path(args.output_cells)
        with outp.open("w") as f:
            f.write("# UTM32 2 km LGL source cells covering missing LV95 tiles\n")
            f.write("# columns: e_km n_km [http_status]\n")
            for cell in cells_sorted:
                s = server_status.get(cell)
                f.write(f"{cell[0]} {cell[1]}"
                        f"{' ' + str(s) if s is not None else ''}\n")
        print(f"\nWrote {len(cells_sorted)} cells to {outp}")

    # Helpful per-cell download URLs for available ones.
    if args.probe_server:
        avail_sorted = sorted(c for c, s in server_status.items() if s == 200)
        if avail_sorted:
            print(f"\nExample URLs to fetch missing source ZIPs:")
            for cell in avail_sorted[:5]:
                print(f"  {LGL_BASE}/dgm1_32_{cell[0]}_{cell[1]}_2_bw.zip")
            if len(avail_sorted) > 5:
                print(f"  ... and {len(avail_sorted) - 5} more")

    return 0


if __name__ == "__main__":
    sys.exit(main())
