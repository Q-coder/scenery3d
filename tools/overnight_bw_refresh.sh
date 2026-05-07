#!/bin/zsh
# Overnight orchestrator for the BW re-download.
#
# Runs alongside the two long-running downloaders started earlier:
#   - tools/download_lgl_bw.py  (DGM ZIPs → extract → convert .raw)
#   - tools/download_lgl_dop.py (DOP ZIPs only — must be processed by us)
#
# Responsibilities:
#   1. Periodically drain newly-arrived DOP ZIPs by streaming them into
#      ortho JPEGs with --delete-zips, so DOP storage stays bounded.
#   2. Once the DGM downloader exits, delete the extracted XYZ source
#      directories under scenery_in_bw/ — convert_lgl_bw has already
#      written .raw tiles by then and the XYZ text takes ~5–10× the
#      disk of the source ZIPs.
#   3. Abort the loop if free space on /Volumes/Data1 drops under
#      MIN_FREE_GB (default 80) so the SSD cannot fill.
#
# Safe to run concurrently with the downloaders: ZIPs are written via
# atomic .part rename and the DOP processor only opens files that have
# their final names.

set -u

ZIP_DIR_DOP=/Volumes/Data1/scenery_in_dop
ZIP_DIR_DGM=/Volumes/Data1/scenery_in_bw
ORTHO_OUT=/Volumes/Data1/scenery/Germany/Baden-Wuertemberg/orthophoto
DATA_VOLUME=/Volumes/Data1
LOG=/tmp/overnight_bw.log
SLEEP_SEC=${SLEEP_SEC:-600}     # 10 min
MIN_FREE_GB=${MIN_FREE_GB:-80}
DGM_PID=${DGM_PID:-}
DOP_PID=${DOP_PID:-}

cd "$(dirname "$0")/.."

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*" | tee -a "$LOG"; }

free_gb() {
    df -g "$DATA_VOLUME" | awk 'NR==2 {print $4}'
}

is_running() {
    local pid=$1
    [[ -z "$pid" ]] && return 1
    kill -0 "$pid" 2>/dev/null
}

cleanup_dgm_extracts() {
    # Remove extracted XYZ subdirs (they outlive ZIPs but convert_lgl_bw
    # only needs them once). Keep .zip files so re-runs are cheap.
    if [[ -d "$ZIP_DIR_DGM" ]]; then
        local n=$(find "$ZIP_DIR_DGM" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l | tr -d ' ')
        if (( n > 0 )); then
            log "DGM: removing $n extracted XYZ dirs under $ZIP_DIR_DGM"
            find "$ZIP_DIR_DGM" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
        fi
    fi
}

drain_dop() {
    local count=$(ls "$ZIP_DIR_DOP"/*.zip 2>/dev/null | wc -l | tr -d ' ')
    if (( count == 0 )); then
        log "DOP: nothing to process"
        return 0
    fi
    log "DOP: $count ZIP(s) present — running process_lgl_dop --delete-zips"
    # --delete-zips removes each ZIP as soon as all LV95 tiles it
    # contributes to are written. --jobs 8 stays well below the open
    # file limit and keeps a couple cores free for the downloads.
    python3 tools/process_lgl_dop.py \
        --zip-dir "$ZIP_DIR_DOP" \
        --output  "$ORTHO_OUT" \
        --jobs 8 \
        --delete-zips >> "$LOG" 2>&1
    log "DOP: drain pass done"
}

log "Overnight refresh starting (DGM_PID=${DGM_PID:-?} DOP_PID=${DOP_PID:-?})"
log "Min free GB threshold: $MIN_FREE_GB"

iteration=0
dgm_cleaned=0
while true; do
    iteration=$((iteration + 1))
    fg=$(free_gb)
    log "iter $iteration  free=${fg}GB  dgm_running=$(is_running "$DGM_PID" && echo y || echo n)  dop_running=$(is_running "$DOP_PID" && echo y || echo n)"

    if (( fg < MIN_FREE_GB )); then
        log "ABORT: free space ${fg}GB below threshold ${MIN_FREE_GB}GB — stopping refresh loop"
        log "(downloaders are NOT killed; investigate manually)"
        exit 2
    fi

    drain_dop

    if ! is_running "$DGM_PID" && (( dgm_cleaned == 0 )); then
        log "DGM downloader has exited — cleaning extracted XYZ dirs"
        cleanup_dgm_extracts
        dgm_cleaned=1
    fi

    if ! is_running "$DGM_PID" && ! is_running "$DOP_PID"; then
        log "Both downloaders done. Final DOP drain..."
        drain_dop
        log "Overnight refresh finished."
        exit 0
    fi

    sleep "$SLEEP_SEC"
done
