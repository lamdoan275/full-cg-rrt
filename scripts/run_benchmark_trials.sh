#!/usr/bin/env bash

# Run the map/planner currently selected in user_config.yaml repeatedly.
# This script never edits user_config.yaml, goal_config.yaml, or planner YAMLs.

set -u

# Force a stable numeric locale so reports always use a dot as the decimal
# separator. Floating-point metrics are rounded to exactly four decimals.
export LC_ALL=C

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scripts_dir="${workspace_dir}/scripts"
user_config="${workspace_dir}/src/user_config/user_config.yaml"
trial_count="${1:-20}"
startup_timeout="${STARTUP_TIMEOUT_S:-90}"
navigation_timeout="${NAVIGATION_TIMEOUT_S:-180}"

if ! [[ "${trial_count}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [positive_trial_count]" >&2
  exit 2
fi

map_name="$(awk -F'"' '/^map:[[:space:]]*"/ { print $2; exit }' "${user_config}")"
planner_name="$(awk -F'"' '/robot1_global_planner:[[:space:]]*"/ { print $2; exit }' "${user_config}")"

if [[ -z "${map_name}" || -z "${planner_name}" ]]; then
  echo "Cannot read map or robot1_global_planner from ${user_config}" >&2
  exit 2
fi

case "${planner_name}" in
  rrt|rrt_star|rrt_astar|rrt_cut|informed_rrt|informed_rrt_star|quick_informed_rrt|rrt_connect)
    plan_topic="/move_base/SamplePlanner/plan"
    ;;
  a_star|jps|gbfs|dijkstra|d_star|lpa_star|voronoi|d_star_lite|theta_star|lazy_theta_star|s_theta_star|hybrid_a_star)
    plan_topic="/move_base/GraphPlanner/plan"
    ;;
  aco|pso|ga)
    plan_topic="/move_base/EvolutionaryPlanner/plan"
    ;;
  *)
    echo "Unsupported global planner for automatic plan-topic selection: ${planner_name}" >&2
    exit 2
    ;;
esac

run_tag="$(date +%Y%m%d_%H%M%S)"
result_dir="${workspace_dir}/benchmark_results/${map_name}_${planner_name}_${run_tag}"
mkdir -p "${result_dir}"

source /opt/ros/noetic/setup.bash
source "${workspace_dir}/devel/setup.bash"

if timeout 2 rosnode list >/dev/null 2>&1; then
  echo "A ROS master is already running. Stop it before starting this benchmark." >&2
  exit 3
fi

results_file="${result_dir}/results.txt"
planner_title="$(printf '%s' "${planner_name}" | tr '[:lower:]' '[:upper:]')"
printf '%s\n\n' "${planner_title}" > "${results_file}"

main_pid=""
trace_pid=""

stop_group() {
  local leader="${1:-}"
  [[ -n "${leader}" ]] || return 0

  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -INT -- "-${leader}" 2>/dev/null || true
    sleep 3
  fi
  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -TERM -- "-${leader}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -KILL -- "-${leader}" 2>/dev/null || true
  fi
  wait "${leader}" 2>/dev/null || true
}

tagged_pids() {
  # Every process started by roslaunch receives an __log argument below this
  # run-specific directory. This also catches ROS children that create their
  # own session/process group and therefore survive a signal to main.sh.
  pgrep -f "__log:=/tmp/ros_motion_planning_${run_tag}_" 2>/dev/null || true
}

signal_tagged_processes() {
  local signal="$1"
  local pids=()
  mapfile -t pids < <(tagged_pids)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    kill "-${signal}" "${pids[@]}" 2>/dev/null || true
  fi
}

wait_for_clean_shutdown() {
  local attempt
  for attempt in $(seq 1 20); do
    if [[ -z "$(tagged_pids)" ]] && ! timeout 1 rosnode list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

cleanup_run() {
  stop_group "${trace_pid}"
  stop_group "${main_pid}"

  signal_tagged_processes INT
  sleep 3
  signal_tagged_processes TERM
  sleep 2
  signal_tagged_processes KILL

  trace_pid=""
  main_pid=""

  if ! wait_for_clean_shutdown; then
    echo "Cleanup failed: ROS/Gazebo processes from run ${run_tag} are still alive." >&2
    return 1
  fi
}

on_exit() {
  local code=$?
  trap - EXIT INT TERM
  cleanup_run
  exit "${code}"
}
trap on_exit EXIT INT TERM

record_result() {
  local id="$1"
  local file="$2"

  awk -F': ' -v trial="${id#0}" '
    function val(key) { return data[key] }
    /^[^ -][^:]*: / { data[$1] = $2 }
    END {
      printf "%d,  TONG QUAN\n", trial
      printf "   time                 %.4f s\n", val("time(s)")
      printf "   cost                 %.4f m\n", val("cost(m)")
      printf "   nodes                %s\n", val("nodes")
      printf "   plan_updates         %s\n", val("plan_updates")
      printf "   replan_count         %s\n\n", val("replan_count")
      printf " PLAN (resample %.4f m)\n", val("evaluation_spacing_m")
      printf "   detour ratio         %.4f\n", val("plan_mean_detour_ratio")
      printf "   turn density         %.4f rad/m\n", val("plan_mean_turn_density_rad_per_m")
      printf "   curvature mean/max   %.4f / %.4f 1/m\n", val("plan_mean_curvature_1_per_m"), val("plan_max_curvature_1_per_m")
      printf "   robot mean/max       %.4f / %.4f\n\n", val("traj_smoothness_mean_curvature"), val("traj_smoothness_max_curvature")
      printf " BAM PLAN ACTIVE (m)\n"
      printf "   rmse                 %.4f\n", val("active_tracking_rmse_m")
      printf "   mean / max           %.4f / %.4f\n", val("active_tracking_mean_error_m"), val("active_tracking_max_error_m")
      printf "   backtrack            %.4f\n\n", val("active_tracking_backtrack_m")
      printf "   (episodes %s, active tracking %s, traj %s diem, tf bo %s, frame %s)\n\n", val("plan_evaluated"), val("active_tracking_episodes"), val("traj_points"), val("tf_dropped_points"), val("eval_frame")
    }
  ' "${file}" >> "${results_file}"
}

echo "Map: ${map_name}"
echo "Planner: ${planner_name}"
echo "Trials: ${trial_count}"
echo "Plan topic: ${plan_topic}"
echo "Results: ${result_dir}"

for trial in $(seq 1 "${trial_count}"); do
  trial_id="$(printf '%02d' "${trial}")"
  export ROS_LOG_DIR="/tmp/ros_motion_planning_${run_tag}_${trial_id}"
  mkdir -p "${ROS_LOG_DIR}"
  main_log="${ROS_LOG_DIR}/main.log"
  trace_log="${ROS_LOG_DIR}/path_trace.log"
  yaml_file="${ROS_LOG_DIR}/result.yaml"

  echo "[trial ${trial_id}/${trial_count}] starting"
  setsid stdbuf -oL -eL bash -c "cd '${scripts_dir}' && exec ./main.sh" >"${main_log}" 2>&1 &
  main_pid=$!

  ready=0
  for _ in $(seq 1 "${startup_timeout}"); do
    if grep -q "SpawnModel: Successfully spawned entity" "${main_log}" 2>/dev/null &&
       rosparam get /move_base/base_global_planner >/dev/null 2>&1; then
      ready=1
      break
    fi
    if grep -q "SpawnModel: Failure" "${main_log}" 2>/dev/null || ! kill -0 "${main_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if [[ "${ready}" -ne 1 ]]; then
    echo "[trial ${trial_id}/${trial_count}] startup_failed"
    printf '%d,  FAILED: startup_failed\n\n' "${trial}" >> "${results_file}"
    cp "${main_log}" "${result_dir}/failed_trial_${trial_id}_main.log" 2>/dev/null || true
    if ! cleanup_run; then
      echo "Benchmark stopped to prevent the next trial from using stale ROS/TF state." >&2
      exit 4
    fi
    sleep 3
    continue
  fi

  # Let AMCL, TF and move_base subscriptions settle before the node publishes
  # its one-shot goal from goal_config.yaml.
  sleep 5

  setsid stdbuf -oL -eL rosrun path_trace_node path_trace_node \
    _plan_topic:="${plan_topic}" _output_file:="${yaml_file}" >"${trace_log}" 2>&1 &
  trace_pid=$!

  status="timeout"
  for _ in $(seq 1 "${navigation_timeout}"); do
    if [[ -s "${yaml_file}" ]] && grep -q "Goal reached!" "${trace_log}" 2>/dev/null; then
      status="success"
      break
    fi
    # move_base aborts with either "valid control" (controller gave up) or
    # "valid plan" (planner gave up). Matching only the first one mislabels a
    # planner abort as a timeout and wastes the full navigation budget.
    if grep -qE "Aborting because a valid (control|plan) could not be found" "${main_log}" 2>/dev/null; then
      status="navigation_aborted"
      break
    fi
    if ! kill -0 "${trace_pid}" 2>/dev/null; then
      status="trace_exited"
      break
    fi
    sleep 1
  done

  if [[ "${status}" == "success" ]]; then
    record_result "${trial_id}" "${yaml_file}"
  else
    printf '%d,  FAILED: %s\n\n' "${trial}" "${status}" >> "${results_file}"
    cp "${main_log}" "${result_dir}/failed_trial_${trial_id}_main.log" 2>/dev/null || true
    cp "${trace_log}" "${result_dir}/failed_trial_${trial_id}_path_trace.log" 2>/dev/null || true
  fi

  echo "[trial ${trial_id}/${trial_count}] ${status}"
  if ! cleanup_run; then
    echo "Benchmark stopped to prevent the next trial from using stale ROS/TF state." >&2
    exit 4
  fi
  sleep 3
done

success_count="$(grep -c 'TONG QUAN' "${results_file}" || true)"
echo "Completed: ${success_count}/${trial_count} successful trials"
echo "Results: ${results_file}"

trap - EXIT INT TERM
