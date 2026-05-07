#!/usr/bin/env python3
"""
Automated LGL BW DOP20 RGB downloader + extractor + converter.

Downloads every 2 km DOP20 RGB tile from the LGL Open GeoData Portal inside a
bounding box (default: full Baden-Württemberg extent), extracts the GeoTIFFs
into per-tile subdirectories, and optionally invokes ``convert_lgl_dop.py`` to
produce Scenery3D LV95 ortho JPEGs (mip0 / mip2 / mip3).

Tile URL pattern (verified 2026-04):
  https://opengeodata.lgl-bw.de/data/dop20/dop20rgb_32_{E}_{N}_2_bw.zip
where E (km, odd) and N (km, even) are UTM32 km — same anchor as the DGM
grid. Each ZIP holds a 2 km × 2 km RGB GeoTIFF (~80–120 MB; full BW ~2.5 TB).

Usage (full BW):
  python tools/download_lgl_dop.py \\
      --download-dir /Volumes/Data1/scenery_in_dop \\
      --output       /Volumes/Data1/scenery/Germany/Baden-Wuertemberg/orthophoto

Restrict to a UTM32 km bbox (E_min N_min E_max N_max):
  python tools/download_lgl_dop.py --extent 484 5290 504 5310 \\
      --download-dir ...  --output ...

Skip the conversion step:
  python tools/download_lgl_dop.py --no-convert --download-dir ...

Re-running is cheap: already-present ZIPs and extracted TIFFs are skipped.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import random
import shutil
import subprocess
import sys
import time
import urllib.request
import urllib.error
import zipfile
from pathlib import Path

BASE_URL = "https://opengeodata.lgl-bw.de/data/dop20"
USER_AGENT = "scenery3d-lgl-dop-downloader/1.0"
TILE_STEP_KM = 2

# Default: Baden-Württemberg bounding box in UTM32 km (generous; 404s skipped).
# BW extent roughly: E 432-612, N 5262-5518 km.
DEFAULT_EXTENT = (432, 5262, 612, 5518)


def tile_url(e_km: int, n_km: int) -> str:
    return f"{BASE_URL}/dop20rgb_32_{e_km}_{n_km}_2_bw.zip"


def tile_name(e_km: int, n_km: int) -> str:
    return f"dop20rgb_32_{e_km}_{n_km}_2_bw"


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
                  timeout: float = 600.0,
                  retries: int = 4) -> bool:
    tmp = dest.with_suffix(dest.suffix + ".part")
    last_err: Exception | None = None
    for attempt in range(retries + 1):
        req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp, \
                 open(tmp, "wb") as out:
                shutil.copyfileobj(resp, out, length=4 * 1024 * 1024)

            if expected_size and tmp.stat().st_size != expected_size:
                raise IOError(
                    f"size mismatch for {dest.name}: got {tmp.stat().st_size}, "
                    f"expected {expected_size}"
                )

            tmp.rename(dest)
            return True
        except Exception as e:  # noqa: BLE001
            last_err = e
            if tmp.exists():
                tmp.unlink(missing_ok=True)
            if attempt < retries:
                # Exponential backoff with jitter to ride out transient
                # server-side throttling/reset bursts.
                sleep_s = min(60.0, (2.0 ** attempt) + random.random())
                time.sleep(sleep_s)
                continue
            print(f"  ERROR downloading {url} after {retries + 1} attempt(s): "
                  f"{e}", file=sys.stderr)
            return False

    if last_err is not None:
        print(f"  ERROR downloading {url}: {last_err}", file=sys.stderr)
    return False


def enumerate_candidates(extent: tuple[int, int, int, int]) -> list[tuple[int, int]]:
    # LGL DOP20 is anchored on odd E_km and even N_km with 2 km cell size.
    # Enumerating only anchored starts cuts probes by 4x and avoids server
    # throttling that can hide valid tiles behind transient network errors.
    e_min, n_min, e_max, n_max = extent

    e0 = int(e_min)
    if (e0 % 2) == 0:
        e0 += 1

    n0 = int(n_min)
    if (n0 % 2) != 0:
        n0 += 1

    cells = []
    for e in range(e0, int(e_max), TILE_STEP_KM):
        for n in range(n0, int(n_max), TILE_STEP_KM):
            cells.append((e, n))
    return cells


def probe_and_index(candidates, workers: int, retries: int = 3,
                    timeout: float = 15.0):
    hits: list[tuple[int, int, int]] = []
    total = len(candidates)
    done = 0
    t0 = time.time()
    transient = 0

    def worker(cell):
        e, n = cell
        status, size = 0, 0
        for attempt in range(retries + 1):
            status, size = http_head(tile_url(e, n), timeout=timeout)
            if status != 0:
                break
            if attempt < retries:
                # Gentle linear backoff (0.5s, 1.0s, 1.5s, ...)
                time.sleep(0.5 * (attempt + 1))
        return (e, n, status, size)

    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        for e, n, status, size in ex.map(worker, candidates):
            done += 1
            if status == 200:
                hits.append((e, n, size))
            elif status == 0:
                transient += 1
            if done % 200 == 0 or done == total:
                elapsed = time.time() - t0
                rate = done / elapsed if elapsed > 0 else 0
                print(f"  probed {done}/{total} cells, {len(hits)} hits, "
                      f"{rate:.0f}/s")
    if transient:
        print(f"  note: {transient} probe(s) had transient network errors "
              f"after retries")
    return hits


def download_all(hits, download_dir: Path, workers: int,
                 timeout: float, retries: int) -> list[Path]:
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
        ok = http_download(tile_url(e, n),
                           dest,
                           expected_size=size,
                           timeout=timeout,
                           retries=retries)
        return (dest, ok)

    ok_count = 0
    fail_count = 0
    total_bytes = 0
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        for idx, (dest, ok) in enumerate(ex.map(worker, jobs), start=1):
            if ok:
                ok_count += 1
                # The drain watchdog may delete a freshly-arrived ZIP
                # between download_all() finishing the download and us
                # stat()-ing it. Treat a missing file as size 0 — the
                # download itself succeeded.
                try:
                    total_bytes += dest.stat().st_size
                except FileNotFoundError:
                    pass
            else:
                fail_count += 1
            if idx % 5 == 0 or idx == len(jobs):
                elapsed = time.time() - t0
                mbps = (total_bytes / 1e6) / elapsed if elapsed > 0 else 0
                print(f"  downloaded {idx}/{len(jobs)} "
                      f"({ok_count} ok, {fail_count} failed, {mbps:.1f} MB/s)")

    return [download_dir / f"{tile_name(e, n)}.zip" for e, n, _ in hits
            if (download_dir / f"{tile_name(e, n)}.zip").exists()]


def extract_all(zips: list[Path], input_dir: Path):
    input_dir.mkdir(parents=True, exist_ok=True)
    extracted = 0
    skipped = 0
    failed = 0
    for zpath in zips:
        out_dir = input_dir / zpath.stem
        # Skip if a TIFF (or JP2) is already extracted.
        if out_dir.is_dir() and (any(out_dir.glob("*.tif"))
                                  or any(out_dir.glob("*.tiff"))
                                  or any(out_dir.glob("*.jp2"))):
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
            zpath.unlink(missing_ok=True)
    print(f"  extracted {extracted}, already-present {skipped}, failed {failed}")


def run_streaming_processor(zip_dir: Path, output_dir: Path,
                            extra_args: list[str]):
    script = Path(__file__).resolve().parent / "process_lgl_dop.py"
    cmd = [sys.executable, str(script),
           "--zip-dir", str(zip_dir),
           "--output", str(output_dir)] + extra_args
    print("  $", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download LGL BW DOP20 RGB tiles (and optionally process them).",
    )
    parser.add_argument("--download-dir", required=True,
                        help="Where ZIPs are stored.")
    parser.add_argument("--output",
                        help="Output directory for ortho JPEGs. Required "
                             "only if --process is given.")
    parser.add_argument("--extent", type=int, nargs=4,
                        metavar=("E_MIN", "N_MIN", "E_MAX", "N_MAX"),
                        default=DEFAULT_EXTENT,
                        help=f"UTM32 bbox in km (default: full BW {DEFAULT_EXTENT}).")
    parser.add_argument("--cells-file",
                        help="Restrict to (E_km N_km) cells listed one per "
                             "line. Same format as for download_lgl_bw.py.")
    parser.add_argument("--probe-workers", type=int, default=8,
                        help="HEAD-probe parallelism (default: 8). LGL "
                             "rate-limits aggressive probing.")
    parser.add_argument("--probe-retries", type=int, default=3,
                        help="Retries for transient probe network failures "
                            "(status 0). Default: 3.")
    parser.add_argument("--probe-timeout", type=float, default=15.0,
                        help="HEAD probe timeout in seconds (default: 15).")
    parser.add_argument("--download-workers", type=int, default=4,
                        help="Download parallelism (default: 4). DOPs are "
                             "much larger than DGMs so 4 is a good balance.")
    parser.add_argument("--download-timeout", type=float, default=600.0,
                        help="GET download timeout in seconds (default: 600).")
    parser.add_argument("--download-retries", type=int, default=4,
                        help="Retries for transient download failures "
                            "(e.g. connection reset). Default: 4.")
    parser.add_argument("--process", action="store_true",
                        help="After downloading, invoke process_lgl_dop.py to "
                             "stream-convert ZIPs into ortho JPEGs.")
    parser.add_argument("--process-arg", action="append", default=[],
                        help="Extra arg forwarded to process_lgl_dop.py "
                             "(e.g. --process-arg=--jobs=8 "
                             "--process-arg=--delete-zips). Repeatable.")
    args = parser.parse_args()

    if args.process and not args.output:
        parser.error("--output is required when --process is given")

    download_dir = Path(args.download_dir).expanduser().resolve()

    if args.cells_file:
        cells_path = Path(args.cells_file).expanduser()
        cells: list[tuple[int, int]] = []
        with cells_path.open() as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 2:
                    continue
                try:
                    cells.append((int(parts[0]), int(parts[1])))
                except ValueError:
                    continue
        cells = sorted(set(cells))
        print(f"[1/3] Loaded {len(cells)} cells from {cells_path}")
    else:
        print(f"[1/3] Enumerating 2 km cells in UTM32 extent "
              f"E [{args.extent[0]}, {args.extent[2]}] × "
              f"N [{args.extent[1]}, {args.extent[3]}]")
        cells = enumerate_candidates(tuple(args.extent))
        print(f"      {len(cells)} candidate cells")

    print(f"[2/3] Probing availability with {args.probe_workers} workers")
    hits = probe_and_index(cells,
                           workers=args.probe_workers,
                           retries=args.probe_retries,
                           timeout=args.probe_timeout)
    if not hits:
        print("No available tiles found for this extent. Exiting.",
              file=sys.stderr)
        return 1
    total_gb = sum(s for _, _, s in hits) / 1e9
    print(f"      {len(hits)} tiles available (~{total_gb:.1f} GB total)")

    print(f"[3/3] Downloading missing ZIPs into {download_dir}")
    download_all(hits,
                 download_dir,
                 workers=args.download_workers,
                 timeout=args.download_timeout,
                 retries=args.download_retries)

    if not args.process:
        print("\nDownload complete. Next step: stream-convert with\n"
              "  python tools/process_lgl_dop.py \\\n"
              f"      --zip-dir {download_dir} \\\n"
              "      --output  /path/to/orthophoto \\\n"
              "      --jobs 8 --delete-zips\n"
              "(--delete-zips removes each ZIP once every LV95 tile it "
              "contributes to has been written; safe at boundaries.)")
        return 0

    output_dir = Path(args.output).expanduser().resolve()
    print(f"Running process_lgl_dop.py → {output_dir}")
    run_streaming_processor(download_dir, output_dir, args.process_arg)

    print("All done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
