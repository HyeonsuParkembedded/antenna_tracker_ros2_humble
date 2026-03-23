#!/usr/bin/env bash
set -eo pipefail

LAUNCH_LOG=/tmp/antenna_tracker_sim_regression.log
STATE_SNAPSHOT=/tmp/antenna_tracker_regression_state.txt
STATUS_SNAPSHOT=/tmp/antenna_tracker_regression_status.txt

cleanup() {
  local exit_code=$?
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  if [[ $exit_code -ne 0 ]]; then
    echo "=== Simulation regression launch log tail ==="
    tail -n 200 "${LAUNCH_LOG}" 2>/dev/null || true
    echo "=== Simulation regression state snapshot ==="
    cat "${STATE_SNAPSHOT}" 2>/dev/null || true
    echo "=== Simulation regression status snapshot ==="
    cat "${STATUS_SNAPSHOT}" 2>/dev/null || true
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

capture_status_snapshot() {
  ros2 service call /antenna/get_status antenna_tracker_msgs/srv/GetStatus "{}" \
    >"${STATUS_SNAPSHOT}" 2>/dev/null || true
}

await_manual_mode() {
  echo "=== Switching to MANUAL mode ==="
  ros2 service call /antenna/set_mode antenna_tracker_msgs/srv/SetMode "{mode: 1}" > /tmp/set_mode_regression.txt
  grep -Eq 'success[:=][[:space:]]*(true|True)' /tmp/set_mode_regression.txt
}

await_convergence() {
  local case_name="$1"
  local target_az="$2"
  local target_el="$3"
  local timeout_s="$4"
  local az_tol_deg="$5"
  local el_tol_deg="$6"
  local deadline=$((SECONDS + timeout_s))

  echo "=== Tracking case: ${case_name} -> az=${target_az}, el=${target_el} ==="
  ros2 service call /antenna/set_manual_target antenna_tracker_msgs/srv/SetManualTarget \
    "{azimuth_deg: ${target_az}, elevation_deg: ${target_el}}" > /tmp/manual_target_regression.txt
  grep -Eq 'success[:=][[:space:]]*(true|True)' /tmp/manual_target_regression.txt

  while (( SECONDS < deadline )); do
    ensure_launch_alive
    if timeout 5 ros2 topic echo /antenna/state --once >"${STATE_SNAPSHOT}" 2>/dev/null; then
      local tracking_active
      local current_azimuth
      local current_elevation
      local published_target_azimuth
      local published_target_elevation
      tracking_active=$(extract_field tracking_active "${STATE_SNAPSHOT}" || true)
      current_azimuth=$(extract_field current_azimuth "${STATE_SNAPSHOT}" || true)
      current_elevation=$(extract_field current_elevation "${STATE_SNAPSHOT}" || true)
      published_target_azimuth=$(extract_field target_azimuth "${STATE_SNAPSHOT}" || true)
      published_target_elevation=$(extract_field target_elevation "${STATE_SNAPSHOT}" || true)

      if [[ -n "${tracking_active}" && -n "${current_azimuth}" && -n "${current_elevation}" && \
            -n "${published_target_azimuth}" && -n "${published_target_elevation}" ]]; then
        local az_error
        local el_error
        az_error=$(awk -v c="${current_azimuth}" -v t="${published_target_azimuth}" 'BEGIN {
          d=t-c;
          while (d > 180.0) d -= 360.0;
          while (d < -180.0) d += 360.0;
          if (d < 0.0) d = -d;
          printf "%.3f", d;
        }')
        el_error=$(awk -v c="${current_elevation}" -v t="${published_target_elevation}" 'BEGIN {
          d=t-c;
          if (d < 0.0) d = -d;
          printf "%.3f", d;
        }')

        if [[ "${tracking_active}" == "true" ]] && \
           awk -v az="${az_error}" -v el="${el_error}" -v az_tol="${az_tol_deg}" -v el_tol="${el_tol_deg}" \
             'BEGIN { if (az > az_tol || el > el_tol) exit 1; exit 0; }'; then
          echo "=== Converged: ${case_name} ==="
          cat "${STATE_SNAPSHOT}"
          return 0
        fi
      fi
    fi
    sleep 1
  done

  echo "Case ${case_name} did not converge within timeout"
  capture_status_snapshot
  return 1
}

monitor_soak() {
  local duration_s="$1"
  local az_tol_deg="$2"
  local el_tol_deg="$3"
  local deadline=$((SECONDS + duration_s))

  echo "=== Starting soak monitor for ${duration_s}s ==="
  while (( SECONDS < deadline )); do
    ensure_launch_alive
    if ! timeout 5 ros2 topic echo /antenna/state --once >"${STATE_SNAPSHOT}" 2>/dev/null; then
      echo "Failed to read /antenna/state during soak"
      capture_status_snapshot
      return 1
    fi

    local current_azimuth
    local current_elevation
    local published_target_azimuth
    local published_target_elevation
    current_azimuth=$(extract_field current_azimuth "${STATE_SNAPSHOT}" || true)
    current_elevation=$(extract_field current_elevation "${STATE_SNAPSHOT}" || true)
    published_target_azimuth=$(extract_field target_azimuth "${STATE_SNAPSHOT}" || true)
    published_target_elevation=$(extract_field target_elevation "${STATE_SNAPSHOT}" || true)

    if [[ -z "${current_azimuth}" || -z "${current_elevation}" || -z "${published_target_azimuth}" || \
          -z "${published_target_elevation}" ]]; then
      echo "Incomplete state during soak"
      capture_status_snapshot
      return 1
    fi

    if ! awk -v caz="${current_azimuth}" -v cel="${current_elevation}" \
         -v taz="${published_target_azimuth}" -v tel="${published_target_elevation}" \
         -v az_tol="${az_tol_deg}" -v el_tol="${el_tol_deg}" 'BEGIN {
           if (caz != caz || cel != cel || taz != taz || tel != tel) exit 1;
           daz=taz-caz;
           while (daz > 180.0) daz -= 360.0;
           while (daz < -180.0) daz += 360.0;
           if (daz < 0.0) daz = -daz;
           del=tel-cel;
           if (del < 0.0) del = -del;
           if (daz > az_tol || del > el_tol) exit 1;
           exit 0;
         }'; then
      echo "Tracking drift exceeded soak thresholds"
      capture_status_snapshot
      return 1
    fi

    sleep 2
  done

  echo "=== Soak monitor passed ==="
}

echo "=== Starting simulation regression suite ==="
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
wait_for_message /imu/raw 60 /tmp/imu_raw_regression.txt
wait_for_message /antenna/encoder_feedback 60 /tmp/encoder_feedback_regression.txt

await_manual_mode
await_convergence "az-wrap-negative" 359.0 15.0 45 12.0 12.0
await_convergence "az-wrap-positive" 1.0 15.0 45 12.0 12.0
await_convergence "elevation-upper-limit" 45.0 90.0 50 12.0 14.0
await_convergence "elevation-lower-limit" 45.0 0.0 45 12.0 10.0
await_convergence "midrange-hold" 55.0 45.0 45 12.0 12.0
monitor_soak 30 15.0 15.0

echo "=== Simulation regression suite passed ==="
