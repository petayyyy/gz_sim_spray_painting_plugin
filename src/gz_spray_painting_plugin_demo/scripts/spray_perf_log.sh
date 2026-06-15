#!/usr/bin/env bash
# spray_perf_log.sh — live performance dashboard for the spray painting plugin
#
# Usage:
#   ./spray_perf_log.sh [world_name] [output_csv]
#
# Defaults:
#   world_name  = factory
#   output_csv  = spray_perf_YYYYMMDD_HHMMSS.csv
#
# Requires:
#   gz    (Gazebo Harmonic CLI, accessible from this shell)
#   ros2  (for spray state; optional — falls back to 0 if not sourced)

set -euo pipefail

WORLD="${1:-factory}"
OUT="${2:-spray_perf_$(date +%Y%m%d_%H%M%S).csv}"
STATS_TOPIC="/world/${WORLD}/stats"

# ── Find gz sim server PID ────────────────────────────────────────────────────
echo "Searching for 'gz sim server'..."
GZ_PID=""
for i in $(seq 1 15); do
    GZ_PID=$(pgrep -f "gz sim server" 2>/dev/null | head -1 || true)
    [[ -n "$GZ_PID" ]] && break
    sleep 1
done
if [[ -z "$GZ_PID" ]]; then
    echo "ERROR: 'gz sim server' not found. Is the simulation running?" >&2
    exit 1
fi

# ── Spray state: background ros2 echo → temp file ────────────────────────────
SPRAY_FILE=$(mktemp /tmp/spray_state_XXXXXX)
echo "0" > "$SPRAY_FILE"

_CLEANED=0
cleanup() {
    [[ "$_CLEANED" -eq 1 ]] && return; _CLEANED=1
    [[ -n "${SPRAY_BG:-}" ]] && kill "$SPRAY_BG" 2>/dev/null || true
    rm -f "$SPRAY_FILE"
    # Move past the dashboard before printing the summary
    printf "\n\n\n\n\n\n\n\n\n\n"
    echo "Saved → $OUT"
    if [[ -s "$OUT" ]]; then
        awk -F',' 'NR>1 {
            n++
            rs+=$2; if(n==1||$2<rn)rn=$2; if($2>rx)rx=$2
            if($4~/^[0-9]/){cs+=$4; cn++}
        }
        END {
            if(n>0){
                printf "Summary (%d samples)\n", n
                printf "  RTF : min=%.3f  mean=%.3f  max=%.3f\n", rn, rs/n, rx
                if(cn>0) printf "  CPU : mean=%.1f%%\n", cs/cn
            }
        }' "$OUT"
    fi
}
trap cleanup INT TERM EXIT

if command -v ros2 &>/dev/null; then
    (ros2 topic echo --no-arr /spray_paint/trigger std_msgs/msg/Bool 2>/dev/null | \
        grep --line-buffered "data:" | \
        awk '{v=($2=="true")?1:0; print v; fflush()}' > "$SPRAY_FILE") &
    SPRAY_BG=$!
else
    SPRAY_BG=""
fi

# ── CSV header ────────────────────────────────────────────────────────────────
echo "elapsed_s,rtf,sim_time_s,server_cpu_pct,spray_active" > "$OUT"

# Clear screen and hide cursor
printf "\033[2J\033[H\033[?25l"
trap 'printf "\033[?25h"' EXIT   # restore cursor on exit

START_NS=$(date +%s%N)

# ── Parse gz stats → live dashboard ──────────────────────────────────────────
gz topic -e -t "$STATS_TOPIC" 2>/dev/null | \
awk \
    -v pid="$GZ_PID" \
    -v outfile="$OUT" \
    -v start_ns="$START_NS" \
    -v spray_file="$SPRAY_FILE" \
    -v world="$WORLD" \
'
BEGIN {
    in_sim   = 0
    sim_sec  = 0; sim_nsec  = 0
    rtf      = 0
    sample   = 0

    # ANSI colours
    RST  = "\033[0m"
    BOLD = "\033[1m"
    GRN  = "\033[32m"
    YLW  = "\033[33m"
    RED  = "\033[31m"
    CYN  = "\033[36m"
    DIM  = "\033[2m"

    # Dashboard is 10 lines tall; on first draw we are already at top
    first = 1

    BAR_W = 36   # width of the RTF progress bar
}

/^sim_time/          { in_sim = 1;  next }
/^real_time /        { in_sim = 0;  next }
in_sim && /^ *sec:/  { sim_sec  = $2; next }
in_sim && /^ *nsec:/ { sim_nsec = $2; next }

/^real_time_factor:/ {
    rtf   = $2 + 0
    sim_s = sim_sec + sim_nsec / 1e9

    "date +%s%N" | getline now_ns; close("date +%s%N")
    elapsed = (now_ns + 0 - start_ns + 0) / 1e9

    ps_cmd = "ps -p " pid " -o %cpu= 2>/dev/null"
    cpu = "?"
    if ((ps_cmd | getline cpu) > 0) gsub(/[[:space:]]/, "", cpu)
    close(ps_cmd)

    scmd = "tail -1 " spray_file; spray = "0"
    scmd | getline spray; close(scmd)
    gsub(/[[:space:]]/, "", spray)

    sample++

    # ── Write CSV ─────────────────────────────────────────────────────────────
    printf "%.3f,%.4f,%.3f,%s,%s\n", elapsed, rtf, sim_s, cpu, spray >> outfile
    fflush(outfile)

    # ── Build RTF bar ─────────────────────────────────────────────────────────
    filled = int(rtf * BAR_W + 0.5)
    if (filled > BAR_W) filled = BAR_W
    bar = ""
    for (i = 0; i < filled; i++) bar = bar "█"
    for (i = filled; i < BAR_W; i++) bar = bar "░"

    # Colour the bar: green ≥0.80, yellow ≥0.40, red below
    if      (rtf >= 0.80) bar_col = GRN
    else if (rtf >= 0.40) bar_col = YLW
    else                  bar_col = RED

    # Spray indicator
    if (spray == "1") { spray_str = GRN "ON  ●" RST }
    else              { spray_str = DIM "OFF ○" RST }

    # CPU colour
    cpu_val = cpu + 0
    if      (cpu_val > 150) cpu_col = RED
    else if (cpu_val >  80) cpu_col = YLW
    else                    cpu_col = GRN

    # ── Reposition cursor to top of dashboard ─────────────────────────────────
    if (!first) printf "\033[10A"   # move up 10 lines
    first = 0

    # ── Draw dashboard (10 lines) ─────────────────────────────────────────────
    printf "%-60s\n", BOLD "  SprayPaint Plugin  ·  Monitor" RST
    printf "  %-56s\n", DIM "gz sim server PID " pid "   world=" world RST
    printf "  %-56s\n", DIM "─────────────────────────────────────────────" RST
    printf "  Elapsed   %s%.1f s%s\n",             CYN, elapsed, RST
    printf "  Sim time  %s%.1f s%s\n",             CYN, sim_s,   RST
    printf "  RTF       %s%s%s  %s%.1f%%%s\n",     bar_col, bar, RST, BOLD, rtf*100, RST
    printf "  Server CPU  %s%s%%%s\n",             cpu_col, cpu, RST
    printf "  Spray     %s\n",                     spray_str
    printf "  Samples   %s%d%s  → %s%s%s\n",      BOLD, sample, RST, DIM, outfile, RST
    printf "  %s\n",                               DIM "[Ctrl-C to stop]" RST

    fflush()
}
'
