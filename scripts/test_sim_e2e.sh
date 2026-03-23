#!/usr/bin/env bash
set -eo pipefail

LAUNCH_LOG=/tmp/antenna_tracker_sim_e2e.log
STATE_SNAPSHOT=/tmp/antenna_tracker_state.txt

cleanup() {
  local exit_code=$?
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  if [[ $exit_code -ne 0 ]]; then
    echo "=== End-to-end simulation launch log tail ==="
    tail -n 200 "${LAUNCH_LOG}" 2>/dev/null || true
    echo "=== End-to-end simulation state snapshot ==="
    cat "${STATE_SNAPSHOT}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ensure_launch_alive() {
  if [[ -n "${LAUNCH_PID:-}" ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Simulation launch exited unexpectedly"
    return 1
  fi
}

wait_for_service() {
  local service_name="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))

  while (( SECONDS < deadline )); do
    ensure_launch_alive
    if ros2 service type "${service_name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out waiting for service ${service_name}"
  return 1
}

wait_for_message() {
  local topic_name="$1"
  local timeout_s="$2"
  local output_file="$3"

  ensure_launch_alive
  if ! timeout "${timeout_s}" ros2 topic echo "${topic_name}" --once >"${output_file}" 2>/dev/null; then
    echo "Timed out waiting for topic ${topic_name}"
    return 1
  fi
}

extract_field() {
  local field_name="$1"
  local file_path="$2"
  grep -m1 -oP "^[[:space:]]*${field_name}:[[:space:]]*\\K.*" "${file_path}" | tr -d ' '
}

echo "=== Starting end-to-end simulation ==="
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u

ros2 launch antenna_tracker_simulation sim.launch.py launch_rviz:=false >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 2

wait_for_service /antenna/set_mode 60
wait_for_service /antenna/get_status 60
wait_for_service /antenna/set_manual_target 60
wait_for_message /imu/raw 60 /tmp/imu_raw.txt
wait_for_message /antenna/encoder_feedback 60 /tmp/encoder_feedback.txt

echo "=== Switching to MANUAL mode ==="
ros2 service call /antenna/set_mode antenna_tracker_msgs/srv/SetMode "{mode: 1}" > /tmp/set_mode.txt
grep -Eq 'success[:=][[:space:]]*(true|True)' /tmp/set_mode.txt

echo "=== Sending manual target ==="
ros2 service call /antenna/set_manual_target antenna_tracker_msgs/srv/SetManualTarget \
  "{azimuth_deg: 55.0, elevation_deg: 45.0}" > /tmp/manual_target.txt
grep -Eq 'success[:=][[:space:]]*(true|True)' /tmp/manual_target.txt

echo "=== Waiting for closed-loop tracking to converge ==="
deadline=$((SECONDS + 45))
while (( SECONDS < deadline )); do
  ensure_launch_alive
  if timeout 5 ros2 topic echo /antenna/state --once >"${STATE_SNAPSHOT}" 2>/dev/null; then
    tracking_active=$(extract_field tracking_active "${STATE_SNAPSHOT}" || true)
    current_azimuth=$(extract_field current_azimuth "${STATE_SNAPSHOT}" || true)
    current_elevation=$(extract_field current_elevation "${STATE_SNAPSHOT}" || true)
    target_azimuth=$(extract_field target_azimuth "${STATE_SNAPSHOT}" || true)
    target_elevation=$(extract_field target_elevation "${STATE_SNAPSHOT}" || true)

    if [[ -n "${tracking_active}" && -n "${current_azimuth}" && -n "${current_elevation}" && \
          -n "${target_azimuth}" && -n "${target_elevation}" ]]; then
      az_error=$(awk -v c="${current_azimuth}" -v t="${target_azimuth}" 'BEGIN {
        d=t-c;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        if (d < 0.0) d = -d;
        printf "%.3f", d;
      }')
      el_error=$(awk -v c="${current_elevation}" -v t="${target_elevation}" 'BEGIN {
        d=t-c;
        if (d < 0.0) d = -d;
        printf "%.3f", d;
      }')

      if [[ "${tracking_active}" == "true" ]] && \
         awk -v az="${az_error}" -v el="${el_error}" -v taz="${target_azimuth}" -v tel="${target_elevation}" 'BEGIN {
           if (taz < 5.0 || tel < 5.0) exit 1;
           if (az > 12.0 || el > 12.0) exit 1;
           exit 0;
         }'; then
        echo "=== End-to-end tracking converged ==="
        cat "${STATE_SNAPSHOT}"
        exit 0
      fi
    fi
  fi
  sleep 1
done

echo "End-to-end simulation did not converge within timeout"
exit 1
