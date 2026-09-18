#!/usr/bin/env bash
# Run replay_slam.sh once per Cartographer config variant and collect the results side by side.
# replay_slam.sh always writes to /tmp/airmouse_sim/replay, so each run is archived here first.
#
#   sweep_slam.sh <bag_dir> [rate] [variant ...]        (default: every .lua in sim/slam_configs)
#   e.g. sweep_slam.sh ~/airmouse_ws/sim/bag_run6 2 base local_lc no_lc
#
# Results: sim/replay_out/<variant>/{eval.txt,eval.csv,map.png,carto.log} + summary.txt
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BAG=${1:?bag dir}
RATE=${2:-2}
if [ $# -gt 2 ]; then shift 2; VARIANTS=("$@"); else
    mapfile -t VARIANTS < <(cd "$HERE/slam_configs" && ls *.lua | sed 's/\.lua$//')
fi

# bag length / rate, plus slack for Cartographer startup, the final optimisation and save_map
BAG_SEC=$(python3 - "$BAG/metadata.yaml" <<'PY'
import sys, yaml
info = yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]
print(int(info["duration"]["nanoseconds"] / 1e9))
PY
)
LIMIT=$(( BAG_SEC / RATE + 180 ))

OUT=$HERE/replay_out
mkdir -p "$OUT"
SUMMARY=$OUT/summary.txt
: > "$SUMMARY"
echo "bag $BAG (${BAG_SEC}s) at rate ${RATE}x -> ${LIMIT}s limit per run"
echo "${#VARIANTS[@]} variants: ${VARIANTS[*]}"
echo

for name in "${VARIANTS[@]}"; do
    lua=$HERE/slam_configs/$name.lua
    if [ ! -f "$lua" ]; then echo "!! no such variant: $name"; continue; fi
    dst=$OUT/$name
    rm -rf "$dst"; mkdir -p "$dst"
    echo "=== [$name] started $(date +%H:%M:%S)"
    start=$(date +%s)
    timeout -k 15 "$LIMIT" "$HERE/replay_slam.sh" "$BAG" "$lua" "$RATE" > "$dst/run.log" 2>&1
    rc=$?
    took=$(( $(date +%s) - start ))
    for f in eval.txt eval.csv map.png map.npz carto.log play.log; do
        if [ -e "/tmp/airmouse_sim/replay/$f" ]; then cp "/tmp/airmouse_sim/replay/$f" "$dst/"; fi
    done
    line=$(grep -m1 '^samples:' "$dst/eval.txt" 2>/dev/null)
    if [ "$rc" -eq 124 ] || [ "$rc" -eq 137 ]; then
        line="TIMED OUT after ${LIMIT}s"
    elif [ -z "$line" ]; then
        line="FAILED (rc=$rc) - see $dst/run.log"
    fi
    printf '%-10s %-5s %s\n' "$name" "${took}s" "$line" | tee -a "$SUMMARY"
    grep -m1 '^worst at' "$dst/eval.txt" 2>/dev/null | sed 's/^/                 /' | tee -a "$SUMMARY"
    grep -m1 '^time jumps' "$dst/run.log" 2>/dev/null | sed 's/^/                 /' | tee -a "$SUMMARY"
    echo
done

echo "================ summary ================"
cat "$SUMMARY"
echo
echo "maps + csv per variant: $OUT/<variant>/"
