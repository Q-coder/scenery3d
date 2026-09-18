#!/usr/bin/env python3
"""
Audit LGL Baden-Württemberg source coverage (DGM1 / DOP20 / LoD2).

Answers "which official 2 km LGL cells exist on the server but are missing
from my local staging dir and/or my converted Scenery3D output?" and writes a
cells file that can be fed straight back into the matching downloader:

  download_lgl_bw.py   --cells-file ...   (dgm)
  download_lgl_dop.py  --cells-file ...   (dop)
  download_lgl_lod2.py --cells-file ...   (lod2)

Why this exists
---------------
The downloaders used to default to a UTM32 bbox starting at E=432 km, which
clips the whole western Rhine valley (Freiburg / Breisach / Lörrach: BW
reaches E≈388). On top of that, a transient HEAD failure used to be treated
as "tile absent", so bursts of rate-limiting punched scattered holes into the
downloaded set. Both produced scenery with buildings but no terrain/ortho.

The probe result is cached in a JSON file so re-running is cheap: a full-BW
probe of one product is ~14 000 HEAD requests.

Usage
-----
  # What DGM1 (height model) cells are missing locally?
  python tools/audit_lgl_coverage.py --product dgm \
      --zip-dir /Volumes/Data1/scenery_in_bw \
      --output-cells /tmp/dgm_missing.txt

  # Same for DOP20 (aerial imagery), restricted to the western strip:
  python tools/audit_lgl_coverage.py --product dop \
      --zip-dir /Volumes/Data1/scenery_in_dop \
      --extent 388 5262 436 5518 \
      --output-cells /tmp/dop_missing.txt

  # Compare against the converted Scenery3D tiles instead of the ZIPs
  # (catches "downloaded but never processed"):
  python tools/audit_lgl_coverage.py --product dgm \
      --tile-dir /Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/terrain \
      --output-cells /tmp/dgm_unconverted.txt

  # Offline inventory only (no network), useful for a quick overview:
  python tools/audit_lgl_coverage.py --product dop --zip-dir ... --no-probe
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

try:
    from pyproj import Transformer
except ImportError:  # only needed for --tile-dir
    Transformer = None  # type: ignore[assignment]

USER_AGENT = "scenery3d-lgl-coverage-audit/1.0"
TILE_STEP_KM = 2

# BW reaches the Rhine at E≈388 km; the north-east corner is around
# E 612 / N 5518. 404s are cheap, so the box is deliberately generous.
DEFAULT_EXTENT = (388, 5262, 612, 5518)

PRODUCTS = {
    "dgm": {
        "base_url": "https://opengeodata.lgl-bw.de/data/dgm",
        "name": "dgm1_32_{e}_{n}_2_bw",
        "zip_re": re.compile(r"^dgm1_32_(\d+)_(\d+)_2_bw\.zip$"),
        "downloader": "download_lgl_bw.py",
    },
    "dop": {
        "base_url": "https://opengeodata.lgl-bw.de/data/dop20",
        "name": "dop20rgb_32_{e}_{n}_2_bw",
        "zip_re": re.compile(r"^dop20rgb_32_(\d+)_(\d+)_2_bw\.zip$"),
        "downloader": "download_lgl_dop.py",
    },
    "lod2": {
        "base_url": "https://opengeodata.lgl-bw.de/data/lod2",
        "name": "LoD2_32_{e}_{n}_2_bw",
        "zip_re": re.compile(r"^LoD2_32_(\d+)_(\d+)_2_bw\.zip$"),
        "downloader": "download_lgl_lod2.py",
    },
}

# Scenery3D output tile naming per product family.
TILE_PATTERNS = {
    "dgm": re.compile(r"^tile_(\d+)_(\d+)\.raw$"),
    "dop": re.compile(r"^ortho_(\d+)_(\d+)\.jpg$"),
}

LV95_CRS = "EPSG:2056"
UTM32_CRS = "EPSG:25832"


def tile_url(product: str, e_km: int, n_km: int) -> str:
    p = PRODUCTS[product]
    return f"{p['base_url']}/{p['name'].format(e=e_km, n=n_km)}.zip"


def enumerate_cells(extent: tuple[int, int, int, int]) -> list[tuple[int, int]]:
    """Anchored 2 km grid: odd E km, even N km."""
    e_min, n_min, e_max, n_max = extent
    e0 = int(e_min) + (0 if int(e_min) % 2 else 1)
    n0 = int(n_min) + (0 if int(n_min) % 2 == 0 else 1)
    return [(e, n)
            for e in range(e0, int(e_max) + 1, TILE_STEP_KM)
            for n in range(n0, int(n_max) + 1, TILE_STEP_KM)]


def scan_zips(zip_dir: Path, product: str) -> set[tuple[int, int]]:
    rx = PRODUCTS[product]["zip_re"]
    present: set[tuple[int, int]] = set()
    for entry in zip_dir.iterdir():
        m = rx.match(entry.name)
        if m:
            present.add((int(m.group(1)), int(m.group(2))))
    return present


def scan_output_cells(tile_dir: Path, product: str,
                      tile_size: int = 1024) -> set[tuple[int, int]]:
    """Map converted LV95 output tiles back onto LGL 2 km UTM cells.

    A cell counts as covered only when *every* LV95 tile it feeds exists, so
    partially converted cells are reported as missing.
    """
    if Transformer is None:
        raise SystemExit("pyproj is required for --tile-dir. pip install pyproj")
    rx = TILE_PATTERNS.get(product)
    if rx is None:
        raise SystemExit(f"--tile-dir is not supported for product '{product}'")

    present: set[tuple[int, int]] = set()
    for entry in tile_dir.iterdir():
        m = rx.match(entry.name)
        if m:
            present.add((int(m.group(1)) // tile_size, int(m.group(2)) // tile_size))

    fwd = Transformer.from_crs(LV95_CRS, UTM32_CRS, always_xy=True)
    covered: dict[tuple[int, int], bool] = {}
    for ei, ni in present:
        for cell in lv95_tile_to_cells(ei, ni, tile_size, fwd):
            covered.setdefault(cell, True)
    # Mark a cell incomplete if any LV95 tile inside it is absent.
    inv = Transformer.from_crs(UTM32_CRS, LV95_CRS, always_xy=True)
    for cell in list(covered):
        for ei, ni in cell_to_lv95_tiles(cell, tile_size, inv):
            if (ei, ni) not in present:
                covered[cell] = False
                break
    return {c for c, ok in covered.items() if ok}


def lv95_tile_to_cells(ei: int, ni: int, tile_size: int,
                       fwd: "Transformer", grid: int = 3) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for gy in range(grid):
        for gx in range(grid):
            e = ei * tile_size + gx / (grid - 1) * tile_size
            n = ni * tile_size + gy / (grid - 1) * tile_size
            ue, un = fwd.transform(e, n)
            cells.add(snap_cell(int(ue // 1000), int(un // 1000)))
    return cells


def cell_to_lv95_tiles(cell: tuple[int, int], tile_size: int,
                       inv: "Transformer", grid: int = 5) -> set[tuple[int, int]]:
    e_km, n_km = cell
    tiles: set[tuple[int, int]] = set()
    for gy in range(grid):
        for gx in range(grid):
            ue = (e_km + gx / (grid - 1) * TILE_STEP_KM) * 1000
            un = (n_km + gy / (grid - 1) * TILE_STEP_KM) * 1000
            le, ln = inv.transform(ue, un)
            tiles.add((int(le // tile_size), int(ln // tile_size)))
    return tiles


def snap_cell(e_km: int, n_km: int) -> tuple[int, int]:
    if e_km % 2 == 0:
        e_km -= 1
    if n_km % 2 != 0:
        n_km -= 1
    return e_km, n_km


def http_head(url: str, timeout: float = 15.0, retries: int = 4) -> int:
    """HTTP status for `url`; transient failures retried, 0 = gave up.

    Never conflate a transient error with 404 — that is exactly the bug that
    punched holes into the downloaded corpus.
    """
    req = urllib.request.Request(url, method="HEAD",
                                 headers={"User-Agent": USER_AGENT})
    backoff = 0.5
    for attempt in range(retries + 1):
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return resp.status
        except urllib.error.HTTPError as e:
            if e.code in (429, 500, 502, 503, 504) and attempt < retries:
                time.sleep(backoff)
                backoff *= 2
                continue
            return e.code
        except (urllib.error.URLError, TimeoutError, OSError):
            if attempt < retries:
                time.sleep(backoff)
                backoff *= 2
                continue
            return 0
    return 0


def probe(product: str, cells: list[tuple[int, int]], workers: int,
          cache: dict[str, int], delay: float,
          cache_path: Path | None = None) -> dict[tuple[int, int], int]:
    todo = [c for c in cells if cache.get(f"{c[0]}_{c[1]}", 0) == 0]
    print(f"  {len(cells) - len(todo)} cached, {len(todo)} to probe "
          f"with {workers} workers")
    t0 = time.time()
    done = 0

    def worker(cell: tuple[int, int]) -> tuple[tuple[int, int], int]:
        if delay:
            time.sleep(delay)
        return cell, http_head(tile_url(product, *cell))

    if todo:
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
            for cell, status in ex.map(worker, todo):
                # Never cache an unresolved probe (0): it must be retried,
                # otherwise a rate-limit burst permanently looks like "absent".
                if status:
                    cache[f"{cell[0]}_{cell[1]}"] = status
                done += 1
                if done % 250 == 0 or done == len(todo):
                    rate = done / max(time.time() - t0, 1e-6)
                    print(f"    probed {done}/{len(todo)}  {rate:.0f}/s",
                          flush=True)
                    if cache_path is not None:
                        cache_path.write_text(json.dumps(cache))
    return {c: cache.get(f"{c[0]}_{c[1]}", 0) for c in cells}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--product", required=True, choices=sorted(PRODUCTS))
    ap.add_argument("--extent", type=int, nargs=4, default=list(DEFAULT_EXTENT),
                    metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                    help=f"UTM32 km bbox (default: full BW {DEFAULT_EXTENT}).")
    ap.add_argument("--zip-dir", help="Local staging dir with the product ZIPs.")
    ap.add_argument("--tile-dir",
                    help="Converted Scenery3D output dir (terrain/ or "
                         "orthophoto/); reports cells whose LV95 tiles are "
                         "not all present.")
    ap.add_argument("--tile-size", type=int, default=1024)
    ap.add_argument("--reference-zips", action="append", default=[],
                    metavar="PRODUCT=DIR",
                    help="Use another product's downloaded ZIPs as an offline "
                         "reference for which cells exist (repeatable, e.g. "
                         "--reference-zips lod2=/Volumes/Data1/scenery_in/lgl_lod2). "
                         "The union is a lower bound on BW's true cell set, "
                         "but it needs no network and immediately exposes "
                         "cells one product has and another lacks.")
    ap.add_argument("--no-probe", action="store_true",
                    help="Skip the server probe; report local inventory only.")
    ap.add_argument("--probe-workers", type=int, default=4,
                    help="HEAD parallelism (default: 4 — the LGL portal "
                         "blocks aggressive bursts for several minutes).")
    ap.add_argument("--probe-delay", type=float, default=0.15,
                    help="Seconds each worker sleeps before a request "
                         "(default: 0.15, i.e. ~25 req/s at 4 workers).")
    ap.add_argument("--cache", help="JSON probe cache (default: "
                                    "~/.cache/lgl_probe_<product>.json).")
    ap.add_argument("--output-cells", help="Write missing cells here.")
    args = ap.parse_args()

    cells = enumerate_cells(tuple(args.extent))
    print(f"[1/3] {len(cells)} candidate {args.product} cells in "
          f"E {args.extent[0]}..{args.extent[2]} × N {args.extent[1]}..{args.extent[3]}")

    have: set[tuple[int, int]] = set()
    if args.zip_dir:
        have |= scan_zips(Path(args.zip_dir).expanduser(), args.product)
        print(f"      {len(have)} ZIPs present in {args.zip_dir}")
    converted: set[tuple[int, int]] | None = None
    if args.tile_dir:
        converted = scan_output_cells(Path(args.tile_dir).expanduser(),
                                      args.product, args.tile_size)
        print(f"      {len(converted)} cells fully converted in {args.tile_dir}")

    reference_extra: set[tuple[int, int]] = set()
    for spec in args.reference_zips:
        product, _, path = spec.partition("=")
        if product not in PRODUCTS or not path:
            raise SystemExit(f"--reference-zips expects PRODUCT=DIR, got {spec!r}")
        found = scan_zips(Path(path).expanduser(), product)
        reference_extra |= found
        print(f"      reference {product}: {len(found)} ZIPs in {path}")

    if args.no_probe:
        available = None
        print("[2/3] Probe skipped")
    else:
        cache_path = Path(args.cache).expanduser() if args.cache else (
            Path.home() / ".cache" / f"lgl_probe_{args.product}.json")
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        cache: dict[str, int] = {}
        if cache_path.exists():
            cache = json.loads(cache_path.read_text())
        print(f"[2/3] Probing {PRODUCTS[args.product]['base_url']} "
              f"(cache {cache_path})")
        try:
            status = probe(args.product, cells, args.probe_workers, cache,
                           args.probe_delay, cache_path)
        finally:
            cache_path.write_text(json.dumps(cache))
        available = {c for c, s in status.items() if s == 200}
        transient = sum(1 for s in status.values() if s == 0)
        print(f"      {len(available)} cells published on the server"
              + (f"  ({transient} UNRESOLVED after retries — re-run to "
                 f"finish; they are excluded from the report)"
                 if transient else ""))

    reference = (available if available is not None else have) | reference_extra
    reference &= set(cells)
    missing_zip = sorted(reference - have) if args.zip_dir else []
    missing_conv = sorted(reference - converted) if converted is not None else []

    print("[3/3] Result")
    if args.zip_dir:
        print(f"      {len(missing_zip)} cells never downloaded")
    if converted is not None:
        print(f"      {len(missing_conv)} cells not fully converted "
              f"(includes the never-downloaded ones)")

    report = missing_conv if converted is not None else missing_zip
    if report:
        by_col: dict[int, int] = {}
        for e, _ in report:
            by_col[e] = by_col.get(e, 0) + 1
        cols = ", ".join(f"E{e}:{c}" for e, c in sorted(by_col.items())[:24])
        print(f"      columns: {cols}{' ...' if len(by_col) > 24 else ''}")

    if args.output_cells:
        out = Path(args.output_cells).expanduser()
        with out.open("w") as f:
            f.write(f"# {args.product} cells missing locally "
                    f"({len(report)}), generated by audit_lgl_coverage.py\n")
            for e, n in report:
                f.write(f"{e} {n}\n")
        print(f"      wrote {len(report)} cells → {out}")
        print(f"      feed to: tools/{PRODUCTS[args.product]['downloader']} "
              f"--cells-file {out} ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
