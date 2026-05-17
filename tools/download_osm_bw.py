#!/usr/bin/env python3
"""
Download the Baden-Württemberg OpenStreetMap extract from Geofabrik.

Output:
  <outdir>/baden-wuerttemberg-latest.osm.pbf

OSM data © OpenStreetMap contributors, ODbL 1.0
https://www.openstreetmap.org/copyright
"""

import argparse
import sys
import time
import urllib.request
from pathlib import Path

URL = "https://download.geofabrik.de/europe/germany/baden-wuerttemberg-latest.osm.pbf"
MD5_URL = URL + ".md5"
POLY_URL = "https://download.geofabrik.de/europe/germany/baden-wuerttemberg.poly"


def download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    print(f"Downloading {url}", flush=True)
    t0 = time.time()
    last = t0

    def hook(blocks, blksize, total):
        nonlocal last
        now = time.time()
        if now - last < 1.0 and blocks * blksize < total:
            return
        last = now
        done = blocks * blksize
        if total > 0:
            pct = 100.0 * done / total
            mb = done / (1024 * 1024)
            mb_total = total / (1024 * 1024)
            rate = done / max(now - t0, 1e-6) / (1024 * 1024)
            print(f"  {pct:5.1f}%  {mb:7.1f} / {mb_total:7.1f} MiB  {rate:5.1f} MiB/s",
                  flush=True)

    urllib.request.urlretrieve(url, tmp, reporthook=hook)
    tmp.rename(dest)
    print(f"Done in {time.time() - t0:.1f}s -> {dest}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outdir", default="/Volumes/Data1/scenery_in/osm_bw",
                    help="Destination directory.")
    ap.add_argument("--force", action="store_true",
                    help="Re-download even if file already exists.")
    args = ap.parse_args()

    out = Path(args.outdir) / "baden-wuerttemberg-latest.osm.pbf"
    if out.exists() and not args.force:
        print(f"Already present: {out} ({out.stat().st_size / (1024 * 1024):.1f} MiB)")
        print("Use --force to re-download.")
        return 0

    download(URL, out)

    # Best-effort: fetch md5 alongside (informational; not verified here).
    try:
        md5_dest = out.with_suffix(out.suffix + ".md5")
        urllib.request.urlretrieve(MD5_URL, md5_dest)
        print(f"md5: {md5_dest.read_text().strip()}")
    except Exception as e:
        print(f"(md5 fetch failed: {e})")

    # Boundary polygon used by the converter to clip cross-border bleed.
    try:
        poly_dest = out.parent / "baden-wuerttemberg.poly"
        if args.force or not poly_dest.exists():
            urllib.request.urlretrieve(POLY_URL, poly_dest)
        print(f"poly: {poly_dest} ({poly_dest.stat().st_size} bytes)")
    except Exception as e:
        print(f"(poly fetch failed: {e})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
