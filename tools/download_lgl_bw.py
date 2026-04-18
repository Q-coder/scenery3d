#!/usr/bin/env python3
"""
Automated LGL BW DGM1 downloader + extractor + converter.

Downloads every 2 km DGM1 tile from the LGL Open GeoData Portal inside a
bounding box (defaults to the full Baden-Württemberg extent), extracts the
XYZ files into per-tile subdirectories, and optionally invokes
convert_lgl_bw.py to produce Scenery3D LV95 .raw tiles.

The portal publishes each tile as:
  https://opengeodata.lgl-bw.de/data/dgm/dgm1_32_{E}_{N}_2_bw.zip
where E and N are UTM32 eastings/northings in kilometres, both multiples of 2.

Usage (full BW, same workflow as the swisstopo downloader):
  python tools/download_lgl_bw.py \\
      --download-dir /Users/gery/Documents/scenery_in \\
      --output       /Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/terrain

Restrict to a UTM32 km bounding box (E_min N_min E_max N_max):
  python tools/download_lgl_bw.py --extent 456 5282 468 5296 \\
      --download-dir ...  --output ...

Skip the conversion step (just download + extract):
  python tools/download_lgl_bw.py --no-convert --download-dir ...

Re-running is cheap: already-present ZIPs and extracted XYZ files are skipped.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import shutil
import subprocess
import sys
import time
import urllib.request
import urllib.error
import zipfile
from pathlib import Path

BASE_URL = "https://opengeodata.lgl-bw.de/data/dgm"
TILE_STEP_KM = 2
USER_AGENT = "scenery3d-lgl-bw-downloader/1.0 (https://github.com/)"

# Default: Baden-Württemberg bounding box in UTM32 km (generous; 404s skipped).
# BW extent roughly: E 432-612, N 5262-5518 km.
DEFAULT_EXTENT = (432, 5262, 612, 5518)


def tile_url(e_km: int, n_km: int) -> str:
    return f"{BASE_URL}/dgm1_32_{e_km}_{n_km}_2_bw.zip"


def tile_name(e_km: int, n_km: int) -> str:
    return f"dgm1_32_{e_km}_{n_km}_2_bw"


def http_head(url: str, timeout: float = 15.0) -> tuple[int, int]:
    """Return (status_code, content_length_or_0). Network errors → (0, 0)."""
    req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": USER_AGENT})
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
    """Download `url` → `dest` atomically. Returns True on success."""
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
    # LGL 2 km tiles don't align with even km boundaries in both axes
    # (observed grid: odd E km, even N km). Probe every integer km in the
    # bbox; non-existent cells just return 404 from the HEAD probe.
    e_min, n_min, e_max, n_max = extent
    cells = []
    for e in range(int(e_min), int(e_max)):
        for n in range(int(n_min), int(n_max)):
            cells.append((e, n))
    return cells


def probe_and_index(candidates, workers: int):
    """HEAD-probe every candidate; return list of (e, n, size_bytes) for hits."""
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
            if done % 200 == 0 or done == total:
                elapsed = time.time() - t0
                rate = done / elapsed if elapsed > 0 else 0
                print(f"  probed {done}/{total} cells, {len(hits)} hits, {rate:.0f}/s")
    return hits


def download_all(hits, download_dir: Path, workers: int) -> list[Path]:
    """Download every missing ZIP; return list of local ZIP paths (present after)."""
    download_dir.mkdir(parents=True, exist_ok=True)
    jobs: list[tuple[int, int, int, Path]] = []
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
                total_bytes += dest.stat().st_size
            else:
                fail_count += 1
            if idx % 20 == 0 or idx == len(jobs):
                elapsed = time.time() - t0
                mbps = (total_bytes / 1e6) / elapsed if elapsed > 0 else 0
                print(f"  downloaded {idx}/{len(jobs)} "
                      f"({ok_count} ok, {fail_count} failed, {mbps:.1f} MB/s)")

    return [download_dir / f"{tile_name(e, n)}.zip" for e, n, _ in hits
            if (download_dir / f"{tile_name(e, n)}.zip").exists()]


def extract_all(zips: list[Path], input_dir: Path):
    """Extract each ZIP into input_dir/<stem>/, skipping already-extracted ones."""
    input_dir.mkdir(parents=True, exist_ok=True)
    extracted = 0
    skipped = 0
    failed = 0
    for zpath in zips:
        out_dir = input_dir / zpath.stem
        # Heuristic: if the target dir exists and contains at least one .xyz,
        # trust it and skip. Cheap and idempotent.
        if out_dir.is_dir() and any(out_dir.glob("*.xyz")):
            skipped += 1
            continue
        try:
            out_dir.mkdir(exist_ok=True)
            with zipfile.ZipFile(zpath) as zf:
                zf.extractall(out_dir)
            extracted += 1
        except zipfile.BadZipFile as e:
            print(f"  bad zip {zpath.name}: {e}", file=sys.stderr)
            failed += 1
            # Corrupt: remove so next run re-downloads.
            zpath.unlink(missing_ok=True)
    print(f"  extracted {extracted}, already-present {skipped}, failed {failed}")


def run_converter(input_dir: Path, output_dir: Path, src_crs: str, extra_args: list[str]):
    script = Path(__file__).resolve().parent / "convert_lgl_bw.py"
    cmd = [sys.executable, str(script),
           "--input", str(input_dir),
           "--output", str(output_dir),
           "--src-crs", src_crs] + extra_args
    print("  $", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download, extract and convert LGL BW DGM1 tiles.",
    )
    parser.add_argument("--download-dir", required=True,
                        help="Where ZIPs are stored and extracted (e.g. /Users/gery/Documents/scenery_in).")
    parser.add_argument("--output",
                        help="Output directory for converted .raw tiles (required unless --no-convert).")
    parser.add_argument("--extent", type=int, nargs=4,
                        metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        default=DEFAULT_EXTENT,
                        help=f"UTM32 bounding box in km (default: full BW {DEFAULT_EXTENT}).")
    parser.add_argument("--probe-workers", type=int, default=32,
                        help="HEAD-probe parallelism (default: 32).")
    parser.add_argument("--download-workers", type=int, default=8,
                        help="Download parallelism (default: 8).")
    parser.add_argument("--no-convert", action="store_true",
                        help="Download + extract only, skip the conversion step.")
    parser.add_argument("--src-crs", default="EPSG:25832",
                        help="Source CRS override passed to the converter.")
    parser.add_argument("--convert-arg", action="append", default=[],
                        help="Extra arg forwarded to convert_lgl_bw.py (repeatable).")
    args = parser.parse_args()

    if not args.no_convert and not args.output:
        parser.error("--output is required unless --no-convert is given")

    download_dir = Path(args.download_dir).expanduser().resolve()

    # 1) Enumerate candidate tiles in the bounding box.
    print(f"[1/4] Enumerating 2 km cells in UTM32 extent "
          f"E [{args.extent[0]}, {args.extent[2]}] × N [{args.extent[1]}, {args.extent[3]}]")
    cells = enumerate_candidates(tuple(args.extent))
    print(f"      {len(cells)} candidate cells")

    # 2) HEAD-probe the portal to find the ones that actually exist.
    print(f"[2/4] Probing availability with {args.probe_workers} workers")
    hits = probe_and_index(cells, workers=args.probe_workers)
    if not hits:
        print("No available tiles found for this extent. Exiting.", file=sys.stderr)
        return 1
    total_mb = sum(s for _, _, s in hits) / 1e6
    print(f"      {len(hits)} tiles available (~{total_mb:.0f} MB total)")

    # 3) Download missing ZIPs.
    print(f"[3/4] Downloading missing ZIPs into {download_dir}")
    zips = download_all(hits, download_dir, workers=args.download_workers)

    # 4) Extract XYZ files.
    print(f"[4/4] Extracting ZIPs into per-tile subdirectories")
    extract_all(zips, download_dir)

    # 5) Optional: run the converter.
    if args.no_convert:
        print("Done (skipping conversion).")
        return 0

    output_dir = Path(args.output).expanduser().resolve()
    print(f"Running convert_lgl_bw.py → {output_dir}")
    run_converter(download_dir, output_dir, args.src_crs, args.convert_arg)

    print("All done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
