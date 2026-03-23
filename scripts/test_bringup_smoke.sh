#!/usr/bin/env bash
set -eo pipefail

MODE="${1:-sim}"
LAUNCH_LOG="/tmp/antenna_tracker_bringup_${MODE}.log"

cleanup() {
  local exit_code=$?
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  if [[ $exit_code -ne 0 ]]; then
    echo "=== Bringup smoke (${MODE}) launch log tail ==="
    tail -n 200 "${LAUNCH_LOG}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ensure_launch_alive() {
  if [[ -n "${LAUNCH_PID:-}" ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Launch for mode ${MODE} exited unexpectedly"
    return 1
  fi
}

echo "=== Starting bringup smoke (${MODE}) ==="
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u

case "${MODE}" in
  web)
    ros2 launch antenna_tracker_bringup web.launch.py >"${LAUNCH_LOG}" 2>&1 &
    LAUNCH_PID=$!
    sleep 2
    ensure_launch_alive
    python3 /ros2_ws/scripts/test_bringup_smoke.py \
      --mode web \
      --launch-pid "${LAUNCH_PID}" \
      --timeout 30 \
      --node /rosbridge_websocket \
      --node /gcs_web_server \
      --tcp 9090 \
      --http 8080
    ;;
  sim)
    ros2 launch antenna_tracker_bringup sim.launch.py launch_rviz:=false >"${LAUNCH_LOG}" 2>&1 &
    LAUNCH_PID=$!
    sleep 2
    ensure_launch_alive
    python3 /ros2_ws/scripts/test_bringup_smoke.py \
      --mode sim \
      --launch-pid "${LAUNCH_PID}" \
      --timeout 60 \
      --service /antenna/set_mode \
      --service /antenna/get_status \
      --node /controller_node \
      --node /state_machine_node \
      --node /rosbridge_websocket \
      --node /gcs_web_server \
      --tcp 9090 \
      --http 8080
    ;;
  hardware)
    ros2 launch antenna_tracker_bringup hardware.launch.py can_interface:=vcan0 >"${LAUNCH_LOG}" 2>&1 &
    LAUNCH_PID=$!
    sleep 2
    ensure_launch_alive
    python3 /ros2_ws/scripts/test_bringup_smoke.py \
      --mode hardware \
      --launch-pid "${LAUNCH_PID}" \
      --timeout 30 \
      --node /can_bridge_node \
      --node /controller_node \
      --service /antenna/set_mode \
      --service /antenna/get_status
    ;;
  *)
    echo "Unsupported smoke mode: ${MODE}"
    exit 2
    ;;
esac

echo "=== Bringup smoke (${MODE}) passed ==="
