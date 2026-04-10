#!/usr/bin/env bash
# Launch the complete ROVR stack:
#   - ros2 launch rovr_bringup bringup.launch.py  (arm + chassis + camera)
#   - communications/control_project/src/rovr/main.py (WebRTC server + Unity lobby)
#
# Both services are supervised: if one exits, it is restarted automatically.
#
# Usage:
#   bash scripts/run_full_stack.sh [real|mock]
#
# Default backend is "real". Override with:
#   BACKEND=mock bash scripts/run_full_stack.sh
#   bash scripts/run_full_stack.sh mock
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---- log file ----------------------------------------------------------------
LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/rovr_$(date +"%Y%m%d_%H%M%S").log"
exec > >(tee "${LOG_FILE}") 2>&1
# ------------------------------------------------------------------------------

BACKEND="${1:-${BACKEND:-real}}"
RESTART_DELAY_SEC="${RESTART_DELAY_SEC:-1.0}"

CONTROL_PROJECT_DIR="${CONTROL_PROJECT_DIR:-${ROOT_DIR}/communications/control_project}"
if [[ ! -f "${CONTROL_PROJECT_DIR}/src/rovr/main.py" ]]; then
  echo "[rovr] ERROR: control_project main.py not found" >&2
  echo "[rovr]   checked: ${CONTROL_PROJECT_DIR}" >&2
  echo "[rovr]   set CONTROL_PROJECT_DIR=/path/to/control_project to override" >&2
  exit 1
fi

# ---- helpers ----------------------------------------------------------------

STOP_FILE="$(mktemp /tmp/rovr_stop.XXXXXX)"
rm -f "${STOP_FILE}"
PIDS=()

log()          { printf "%(%H:%M:%S)T [rovr] %s\n" -1 "$*"; }
prefix_stream(){ local p="$1"; while IFS= read -r l; do printf "%(%H:%M:%S)T [%-14s] %s\n" -1 "${p}" "${l}"; done; }

service_loop() {
  local name="$1"
  local delay="$2"
  shift 2
  local cmd="$*"
  while [[ ! -f "${STOP_FILE}" ]]; do
    log "starting ${name}"
    set +e
    stdbuf -oL -eL bash -lc "${cmd}" 2>&1 | prefix_stream "${name}"
    local code=${PIPESTATUS[0]}
    set -e
    [[ -f "${STOP_FILE}" ]] && break
    log "${name} exited (code=${code}); restarting in ${delay}s"
    sleep "${delay}"
  done
  log "${name} stopped"
}

cleanup() {
  trap - INT TERM EXIT
  touch "${STOP_FILE}"
  log "shutting down all services..."
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  rm -f "${STOP_FILE}"
  log "all services stopped"
}
trap cleanup INT TERM EXIT

# ---- kill stale processes ---------------------------------------------------

for pattern in \
    "arm_controller_node" \
    "chassis_controller_node" \
    "camera_ptz_node" \
    "rovr_bringup.*bringup.launch.py" \
    "python3 .*src/rovr/main.py"; do
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    log "killing stale process: ${pattern}"
    pkill -f "${pattern}" 2>/dev/null || true
  fi
done
sleep 0.5

# ---- build ROS source prefix ------------------------------------------------

ROS_SOURCE="set +u; source \"${ROOT_DIR}/scripts/dev_env.sh\"; set -u; export ROVR_CONFIG_DIR=\"${ROOT_DIR}/config\""

# ---- service commands -------------------------------------------------------

BRINGUP_CMD="${ROS_SOURCE}; ros2 launch rovr_bringup bringup.launch.py hardware:=\"${BACKEND}\""

if [[ -f "${CONTROL_PROJECT_DIR}/.venv/bin/activate" ]]; then
  COMM_CMD="${ROS_SOURCE}; source \"${CONTROL_PROJECT_DIR}/.venv/bin/activate\"; \
    cd \"${CONTROL_PROJECT_DIR}\" && PYTHONUNBUFFERED=1 python3 -u src/rovr/main.py"
else
  COMM_CMD="${ROS_SOURCE}; \
    cd \"${CONTROL_PROJECT_DIR}\" && PYTHONUNBUFFERED=1 python3 -u src/rovr/main.py"
fi

# ---- start ------------------------------------------------------------------

log "========================================"
log " ROVR full stack starting"
log " backend   : ${BACKEND}"
log " comm      : ${CONTROL_PROJECT_DIR}/src/rovr/main.py"
log " log file  : ${LOG_FILE}"
log "========================================"
log "Press Ctrl+C to stop everything"
log ""

service_loop "bringup" "${RESTART_DELAY_SEC}" "${BRINGUP_CMD}" &
PIDS+=("$!")
service_loop "comm"    "${RESTART_DELAY_SEC}" "${COMM_CMD}"    &
PIDS+=("$!")

wait
