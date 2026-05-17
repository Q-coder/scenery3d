#!/usr/bin/env bash
# Download + convert all of Baden-Württemberg LoD2 buildings, in 20 km N-strips.
# Each strip is one invocation of download_lgl_lod2.py and produces a
# partial manifest update. Re-running resumes (ZIPs + extracts cached).
#
# Usage:
#   tools/download_lgl_bw_full.sh
# Tweak DOWNLOAD_DIR / OUTPUT below if needed.

set -euo pipefail

DOWNLOAD_DIR="/Volumes/Data1/scenery_in/lgl_lod2"
OUTPUT="/Users/gery/provpilot/scenery/Germany/Baden-Wuertemberg/buildings"
LOG="$DOWNLOAD_DIR/_log"

# Full BW: E 432..612, N 5262..5518 km.
E_MIN=432; E_MAX=612
N_MIN=5262; N_MAX=5518
STRIP=20  # km of N per pass

mkdir -p "$LOG"

n=$N_MIN
while (( n < N_MAX )); do
    n_end=$(( n + STRIP ))
    (( n_end > N_MAX )) && n_end=$N_MAX
    name="strip_${n}_${n_end}"
    log="$LOG/${name}.log"
    echo "=== $(date '+%F %T')  $name  (E $E_MIN..$E_MAX, N $n..$n_end) ==="
    python3 tools/download_lgl_lod2.py \
        --download-dir "$DOWNLOAD_DIR" \
        --output       "$OUTPUT" \
        --extent       "$E_MIN" "$n" "$E_MAX" "$n_end" \
        --probe-workers 8 --download-workers 8 \
        2>&1 | tee "$log"
    n=$n_end
done

echo "=== $(date '+%F %T')  ALL STRIPS DONE ==="
echo "Buildings dir: $OUTPUT"
du -sh "$OUTPUT" "$DOWNLOAD_DIR"
