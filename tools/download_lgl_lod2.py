#!/usr/bin/env python3
"""
Automated LGL BW LoD2 building downloader + extractor + converter.

Downloads every 2 km LoD2 CityGML tile from the LGL Open GeoData Portal
inside a bounding box, extracts the GML files into per-tile subdirectories
and (unless --no-convert is given) invokes tools/convert_lgl_lod2.py to
produce scenery3d building tiles (GLB + manifest.json) on the LV95 grid.

Portal URL template:
  https://opengeodata.lgl-bw.de/data/lod2/LoD2_32_{E}_{N}_2_bw.zip
with E in odd kilometres and N in even kilometres (same anchoring as DGM/DOP).

Usage (small Schaffhausen-border test):
  tools/download_lgl_lod2.py \\
      --extent 493 5282 503 5294 \\
      --download-dir /Users/gery/Documents/scenery_in/lod2 \\
      --output       /Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/buildings

Use --no-convert to download + extract only.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

BASE_URL = "https://opengeodata.lgl-bw.de/data/lod2"
USER_AGENT = "scenery3d-lgl-lod2-downloader/1.0"

# Default: full Baden-Württemberg (UTM32 km). Tile anchoring is odd-E/even-N,
# but the probe step harmlessly rejects mis-anchored candidates.
DEFAULT_EXTENT = (432, 5262, 612, 5518)


def tile_url(e_km: int, n_km: int) -> str:
    return f"{BASE_URL}/LoD2_32_{e_km}_{n_km}_2_bw.zip"


def tile_name(e_km: int, n_km: int) -> str:
    return f"LoD2_32_{e_km}_{n_km}_2_bw"


def http_head(url: str, timeout: float = 15.0) -> tuple[int, int]:
    req = urllib.request.Request(url, method="HEAD",
                                 headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            length = int(resp.headers.get("Content-Length", "0") or 0)
            return resp.status, length
    except urllib.error.HTTPError as e:
        return e.code, 0
    except (urllib.error.URLError, TimeoutError, OSError):
        return 0, 0


def http_download(url: str, dest: Path, expected_size: int = 0,
                  timeout: float = 60.0) -> bool:
    tmp = dest.with_suffix(dest.suffix + ".part")
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp, open(tmp, "wb") as out:
            shutil.copyfileobj(resp, out, length=1024 * 1024)
    except Exception as e:  # noqa: BLE001
        if tmp.exists():
            tmp.unlink(missing_ok=True)
        print(f"  ERROR downloading {url}: {e}", file=sys.stderr)
        return False
    if expected_size and tmp.stat().st_size != expected_size:
        print(f"  WARN size mismatch for {dest.name}: "
              f"got {tmp.stat().st_size}, expected {expected_size}",
              file=sys.stderr)
    tmp.rename(dest)
    return True


def enumerate_candidates(extent: tuple[int, int, int, int]) -> list[tuple[int, int]]:
    """Anchored 2 km grid: odd-E, even-N. Probe every matching km cell in bbox."""
    e_min, n_min, e_max, n_max = extent
    cells = []
    e_start = e_min if (e_min % 2 == 1) else e_min + 1
    n_start = n_min if (n_min % 2 == 0) else n_min + 1
    for e in range(int(e_start), int(e_max) + 1, 2):
        for n in range(int(n_start), int(n_max) + 1, 2):
            cells.append((e, n))
    return cells


def probe_and_index(candidates, workers: int):
    hits: list[tuple[int, int, int]] = []
    total = len(candidates)
    done = 0
    t0 = time.time()

    def worker(cell):
        e, n = cell
        status, size = http_head(tile_url(e, n))
        return (e, n, status, size)

    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        for e, n, status, size in ex.map(worker, candidates):
            done += 1
            if status == 200:
                hits.append((e, n, size))
            if done % 100 == 0 or done == total:
                elapsed = time.time() - t0
                rate = done / elapsed if elapsed > 0 else 0
                print(f"  probed {done}/{total} cells, {len(hits)} hits, {rate:.0f}/s")
    return hits


def download_all(hits, download_dir: Path, workers: int) -> list[Path]:
    download_dir.mkdir(parents=True, exist_ok=True)
    jobs = []
    skipped = 0
    for e, n, size in hits:
        dest = download_dir / f"{tile_name(e, n)}.zip"
        if dest.exists() and (size == 0 or dest.stat().st_size == size):
            skipped += 1
            continue
        jobs.append((e, n, size, dest))

    print(f"  {skipped} ZIPs already present, {len(jobs)} to download")

    def worker(job):
        e, n, size, dest = job
        ok = http_download(tile_url(e, n), dest, expected_size=size)
        return (dest, ok)

    ok_count = 0
    fail_count = 0
    total_bytes = 0
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        for idx, (dest, ok) in enumerate(ex.map(worker, jobs), start=1):
            if ok:
                ok_count += 1
                try:
                    total_bytes += dest.stat().st_size
                except FileNotFoundError:
                    pass
            else:
                fail_count += 1
            if idx % 10 == 0 or idx == len(jobs):
                elapsed = time.time() - t0
                mbps = (total_bytes / 1e6) / elapsed if elapsed > 0 else 0
                print(f"  downloaded {idx}/{len(jobs)} "
                      f"({ok_count} ok, {fail_count} failed, {mbps:.1f} MB/s)")

    return [download_dir / f"{tile_name(e, n)}.zip" for e, n, _ in hits
            if (download_dir / f"{tile_name(e, n)}.zip").exists()]


def extract_all(zips: list[Path], input_dir: Path) -> list[Path]:
    """Extract each ZIP into input_dir/<stem>/. Returns list of GML files."""
    input_dir.mkdir(parents=True, exist_ok=True)
    extracted = 0
    skipped = 0
    failed = 0
    gml_files: list[Path] = []
    for zpath in zips:
        out_dir = input_dir / zpath.stem
        existing = list(out_dir.rglob("*.gml")) if out_dir.is_dir() else []
        if existing:
            skipped += 1
            gml_files.extend(existing)
            continue
        try:
            out_dir.mkdir(exist_ok=True)
            with zipfile.ZipFile(zpath) as zf:
                zf.extractall(out_dir)
            extracted += 1
            # LGL ZIPs nest a same-named folder; use rglob to find GMLs.
            gml_files.extend(out_dir.rglob("*.gml"))
        except zipfile.BadZipFile as e:
            print(f"  bad zip {zpath.name}: {e}", file=sys.stderr)
            failed += 1
            zpath.unlink(missing_ok=True)
    print(f"  extracted {extracted}, already-present {skipped}, failed {failed}, "
          f"{len(gml_files)} GML file(s)")
    return gml_files


def run_converter(gml_files: list[Path], output_dir: Path,
                  origin_e: float, origin_n: float,
                  extra_args: list[str]):
    script = Path(__file__).resolve().parent / "convert_lgl_lod2.py"
    cmd = [sys.executable, str(script),
           "-o", str(output_dir),
           "--origin-e", str(origin_e),
           "--origin-n", str(origin_n)] + extra_args + [str(p) for p in gml_files]
    print("  $", " ".join(cmd[:8]), f"... ({len(gml_files)} files)")
    subprocess.run(cmd, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download, extract and convert LGL BW LoD2 building tiles.")
    parser.add_argument("--download-dir", required=True,
                        help="Where ZIPs are stored and extracted (e.g. /Users/gery/Documents/scenery_in/lod2).")
    parser.add_argument("--output",
                        help="Output directory for converted .glb tiles "
                             "(required unless --no-convert).")
    parser.add_argument("--extent", type=int, nargs=4,
                        metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        default=DEFAULT_EXTENT,
                        help=f"UTM32 bounding box in km (default: full BW {DEFAULT_EXTENT}).")
    parser.add_argument("--cells-file",
                        help="Restrict to (E_km N_km) cells listed one per line.")
    parser.add_argument("--probe-workers", type=int, default=8,
                        help="HEAD-probe parallelism (default: 8).")
    parser.add_argument("--download-workers", type=int, default=4,
                        help="Download parallelism (default: 4; LoD2 ZIPs are small).")
    parser.add_argument("--no-convert", action="store_true",
                        help="Download + extract only, skip the conversion step.")
    parser.add_argument("--origin-e", type=float, default=2600000.0,
                        help="LV95 origin east (default: 2600000).")
    parser.add_argument("--origin-n", type=float, default=1200000.0,
                        help="LV95 origin north (default: 1200000).")
    parser.add_argument("--convert-arg", action="append", default=[],
                        help="Extra arg forwarded to convert_lgl_lod2.py (repeatable).")
    args = parser.parse_args()

    if not args.no_convert and not args.output:
        parser.error("--output is required unless --no-convert is given")

    download_dir = Path(args.download_dir).expanduser().resolve()

    # 1) Enumerate candidates.
    if args.cells_file:
        cells: list[tuple[int, int]] = []
        with Path(args.cells_file).expanduser().open() as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 2:
                    cells.append((int(parts[0]), int(parts[1])))
        candidates = cells
        print(f"[1/4] {len(candidates)} cells from {args.cells_file}")
    else:
        candidates = enumerate_candidates(tuple(args.extent))
        print(f"[1/4] {len(candidates)} candidate cells in extent "
              f"{tuple(args.extent)} (odd-E/even-N grid)")

    # 2) Probe.
    print("[2/4] probing server for available tiles...")
    hits = probe_and_index(candidates, workers=args.probe_workers)
    total_mb = sum(s for *_e, s in hits) / 1e6
    print(f"  {len(hits)} tiles available, total ~{total_mb:.1f} MB")
    if not hits:
        print("No tiles found; aborting.")
        return 1

    # 3) Download.
    print("[3/4] downloading ZIPs...")
    zips = download_all(hits, download_dir, workers=args.download_workers)

    # 4) Extract.
    print("[4/4] extracting + collecting GML files...")
    gml_files = extract_all(zips, download_dir / "_extract")

    if args.no_convert:
        print(f"\nDone (no-convert). {len(gml_files)} GML file(s) at "
              f"{download_dir / '_extract'}")
        return 0

    if not gml_files:
        print("No GML files extracted; nothing to convert.")
        return 1

    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nConverting {len(gml_files)} GML file(s) -> {output_dir}")
    run_converter(gml_files, output_dir, args.origin_e, args.origin_n,
                  args.convert_arg)
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
