# root_preflight.sh -- sourced by the HOST-side harness scripts (PATCHES s61).
# Refuse to start if the host root partition (/) has less than ROOT_MIN_FREE_GB
# (default 5) free. / is only 50 GB; a full root broke a build in s60 and can
# break docker, apt and the desktop. Bags and builds belong on /mnt/data.
root_preflight() {
    local min_gb="${ROOT_MIN_FREE_GB:-5}"
    local free_kb; free_kb=$(df --output=avail -k / | tail -1 | tr -d ' ')
    if (( free_kb < min_gb * 1024 * 1024 )); then
        echo "root preflight: REFUSING -- / has $(( free_kb / 1024 / 1024 )) GB free (< ${min_gb} GB)." >&2
        echo "  Free space on the root partition first (bags/builds go to /mnt/data)." >&2
        exit 6
    fi
}
root_preflight
