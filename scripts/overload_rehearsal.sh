#!/usr/bin/env bash
# Overload rehearsal A/B/C for the 2026-07-07 hardening (hazard 69).
#
# Runs the SAME bag three times under a CPU constraint and compares the
# overload metrics leg by leg — the validation step the hardening series
# left pending (needs a real bag, so this runs on YOUR machine, not CI):
#
#   leg A  baseline        — overload knobs off (stock behaviour)
#   leg B  shedding        — max_latency_ms=200, max_scan_buffer=20
#   leg C  shedding+damping— B plus gap_extrap_decay=0.9
#
# Expected signature if the hardening works on your hardware:
#   A: backlog_ms grows (or saw-tooths high), rt_factor << rate, pose jumps
#   B: backlog_ms saw-tooths <= ~budget, shed_scans climbs, odom continuous
#   C: like B with a smaller max pose step after sheds; NIS never DIVERGED
#
# Usage:
#   ./scripts/overload_rehearsal.sh <bag_path> [options]
#     --config  <yaml>   base RESPLE config     (default: resple/config/config_ouster.yaml)
#     --launch  <file>   resple launch file     (default: resple_ouster.launch.py)
#     --cpus    <list>   taskset CPU list for RESPLE (default: 0,1 — the constraint)
#     --rate    <r>      bag playback rate      (default: 2.0 — overdrive the input)
#     --secs    <s>      seconds per leg        (default: 120)
#     --legs    <A,B,C>  subset of legs         (default: A,B,C)
#
# Requirements: sourced ROS 2 + built workspace (scripts/build_workspace.sh),
# the bag topics matching the config, and `--clock` support in ros2 bag.
# Every leg runs with use_sim_time:=true so the clock-domain audit's ROS-time
# waits behave identically at any --rate.
set -euo pipefail

BAG="${1:?usage: $0 <bag_path> [--config yaml] [--launch file] [--cpus 0,1] [--rate 2.0] [--secs 120] [--legs A,B,C]}"
shift
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${RESPLE_WS:-${HOME}/ros2_ws}"
CONFIG="${ROOT}/resple/config/config_ouster.yaml"
LAUNCH="resple_ouster.launch.py"
CPUS="0,1"
RATE="2.0"
SECS="120"
LEGS="A,B,C"
while [ $# -gt 0 ]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --launch) LAUNCH="$2"; shift 2 ;;
    --cpus)   CPUS="$2"; shift 2 ;;
    --rate)   RATE="$2"; shift 2 ;;
    --secs)   SECS="$2"; shift 2 ;;
    --legs)   LEGS="$2"; shift 2 ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found"; exit 1; }
set +u; source /opt/ros/jazzy/setup.bash; source "${WS}/install/setup.bash"; set -u

# rclpy needs the Python ABI Jazzy was built for (3.12); see e2e_smoke.sh.
PYEXE="$(for p in /usr/bin/python3.12 /usr/bin/python3; do "$p" -c 'import numpy' 2>/dev/null >&2 && { echo "$p"; break; }; done)"

OUT="$(mktemp -d /tmp/resple_rehearsal.XXXX)"
echo "==> results in ${OUT} (bag=${BAG}, rate=${RATE}, cpus=${CPUS}, ${SECS}s/leg)"

leg_overrides() {
  case "$1" in
    A) echo "{}" ;;
    B) echo '{"max_latency_ms": 200, "max_scan_buffer": 20}' ;;
    C) echo '{"max_latency_ms": 200, "max_scan_buffer": 20, "gap_extrap_decay": 0.9, "gap_extrap_free_knots": 3}' ;;
    *) echo "bad leg $1" >&2; exit 2 ;;
  esac
}

run_leg() {
  local leg="$1"
  local cfg="${OUT}/config_${leg}.yaml"
  python3 - "$CONFIG" "$cfg" "$(leg_overrides "${leg}")" <<'EOF'
import sys, yaml, json
base, out, overrides = sys.argv[1], sys.argv[2], json.loads(sys.argv[3])
d = yaml.safe_load(open(base))
p = d['/**']['ros__parameters']
p.update(overrides)
p['use_sim_time'] = True   # bag replay with --clock: ROS-time waits stay honest
yaml.safe_dump(d, open(out, 'w'), sort_keys=False)
EOF
  echo "==> leg ${leg}: overrides $(leg_overrides "${leg}")"
  taskset -c "${CPUS}" ros2 launch resple "${LAUNCH}" \
    config_file:="${cfg}" > "${OUT}/resple_${leg}.log" 2>&1 &
  local lp=$!
  sleep 6   # let the node configure/activate before data flows
  "${PYEXE}" "${ROOT}/scripts/overload_rehearsal_collect.py" \
    --duration "${SECS}" --csv "${OUT}/diag_${leg}.csv" \
    --summary "${OUT}/summary_${leg}.txt" &
  local cp=$!
  ros2 bag play "${BAG}" --clock --rate "${RATE}" > "${OUT}/bag_${leg}.log" 2>&1 &
  local bp=$!
  # WHY not `wait "${cp}" || true`: that discarded the collector's status, so a
  # collector that never started (rclpy ABI mismatch — see the PYEXE note above)
  # or died mid-leg looked exactly like a completed leg. Combined with the
  # `cat ... || echo "(no summary)"` at the bottom, a leg that measured NOTHING
  # printed a friendly note and the script still exited 0 — the A/B verdict was
  # unfalsifiable.
  local crc=0
  wait "${cp}" || crc=$?
  kill -INT "${bp}" "${lp}" 2>/dev/null || true
  sleep 4
  kill -KILL "${bp}" "${lp}" 2>/dev/null || true
  wait 2>/dev/null || true
  sleep 2   # DDS teardown between legs

  # A leg is only usable if the collector exited cleanly AND actually recorded
  # diagnostics samples. The collector always writes its summary file (even an
  # all-zeros one), so file existence alone proves nothing — count CSV data rows
  # beyond the header instead. Zero rows means the node never published
  # resple_diagnostics: it failed to launch/activate, or taskset/the config was
  # wrong. Nothing was measured, so the leg cannot support any conclusion.
  local rows=0
  if [ -f "${OUT}/diag_${leg}.csv" ]; then
    rows="$(( $(wc -l < "${OUT}/diag_${leg}.csv") - 1 ))"
    [ "${rows}" -ge 0 ] || rows=0
  fi
  if [ "${crc}" -ne 0 ] || [ ! -s "${OUT}/summary_${leg}.txt" ] || [ "${rows}" -le 0 ]; then
    echo "ERROR: leg ${leg} produced NO usable data" \
         "(collector exit ${crc}, diag samples ${rows}," \
         "summary $([ -s "${OUT}/summary_${leg}.txt" ] && echo present || echo missing))." >&2
    echo "       Check ${OUT}/resple_${leg}.log and ${OUT}/bag_${leg}.log." >&2
    return 1
  fi
}

IFS=',' read -ra LEG_ARR <<< "${LEGS}"
FAILED_LEGS=""
for leg in "${LEG_ARR[@]}"; do
  # Record instead of aborting: the remaining legs still run and the comparison
  # table below still prints (that is the whole point of the script), but the
  # exit status at the end tells the truth.
  run_leg "${leg}" || FAILED_LEGS="${FAILED_LEGS} ${leg}"
done

echo
echo "==================== overload rehearsal summary ===================="
for leg in "${LEG_ARR[@]}"; do
  echo "--- leg ${leg} ($(leg_overrides "${leg}"))"
  cat "${OUT}/summary_${leg}.txt" 2>/dev/null || echo "  (no summary — check ${OUT}/resple_${leg}.log)"
done
echo "CSVs + logs: ${OUT}"

if [ -n "${FAILED_LEGS}" ]; then
  echo >&2
  echo "ERROR: rehearsal INCOMPLETE — leg(s) with no usable data:${FAILED_LEGS}" >&2
  echo "       The A/B/C comparison above is not valid. Logs: ${OUT}" >&2
  exit 1
fi
