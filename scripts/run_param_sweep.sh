#!/usr/bin/env bash

# Sweep planner parameters over the map currently selected in user_config.yaml.
#
# Every sweep point is one call to run_benchmark_trials.sh, so the measurement
# itself is unchanged. This script only sets the parameters before each call and
# records which values produced which result directory -- run_benchmark_trials.sh
# names its output <map>_<planner>_<timestamp> and does not encode parameters,
# so without that record the sweep points are indistinguishable afterwards.
#
# user_config.yaml and sample_planner_params.yaml are restored on exit, including
# on Ctrl-C.
#
# Usage:
#   ./run_param_sweep.sh stepsize [trials]          sample_max_d, both planners
#   ./run_param_sweep.sh k <sample_max_d> [trials]  number_node_k, rrt_cut only
#
# sample_max_d is SHARED by every sample planner, so sweeping it for only one of
# them and comparing against the other's default is not a fair comparison. That
# is why the stepsize mode always runs both.

set -u
export LC_ALL=C

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scripts_dir="${workspace_dir}/scripts"
user_config="${workspace_dir}/src/user_config/user_config.yaml"
planner_params="${workspace_dir}/src/sim_env/config/planner/sample_planner_params.yaml"
results_root="${workspace_dir}/benchmark_results"

# sweep grids -- edit these.
# maze_20 corridors measured from the .pgm: p5 0.90 m, median 1.50 m wide. A
# 30-cell step is 1.5 m, as long as the median corridor is wide, so most edges
# would be rejected by the collision check. The grid stops at 20 cells = 1.0 m.
STEP_SIZES=(5 8 10 15 20)        # sample_max_d, in costmap cells (0.05 m each)
STEP_PLANNERS=(rrt_cut informed_rrt_star)
NODE_K=(3 10)                    # number_node_k; sweep below and above the default 5

mode="${1:-}"
case "${mode}" in
  stepsize) trials="${2:-6}" ;;
  k)        k_step="${2:-}" ; trials="${3:-6}" ;;
  *) sed -n '/^# Usage:/,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2 ; exit 2 ;;
esac

if ! [[ "${trials}" =~ ^[1-9][0-9]*$ ]]; then
  echo "trials must be a positive integer" >&2
  exit 2
fi
if [[ "${mode}" == "k" ]] && ! [[ "${k_step}" =~ ^[0-9]+$ ]]; then
  echo "k mode needs the sample_max_d to hold fixed, e.g. ./run_param_sweep.sh k 20 6" >&2
  exit 2
fi

# A benchmark already in flight relaunches main.sh once per trial, so it would
# pick up the parameters this script writes and silently mix configurations into
# one result directory. run_benchmark_trials.sh has its own master check, but it
# only fires after the YAML has been edited, which is too late.
source /opt/ros/noetic/setup.bash 2>/dev/null || true
if timeout 2 rosnode list >/dev/null 2>&1; then
  echo "A ROS master is running. Finish or stop the current benchmark first -- editing" >&2
  echo "the parameter files now would change the configuration mid-run." >&2
  exit 3
fi

backup_dir="$(mktemp -d)"
cp "${user_config}" "${backup_dir}/user_config.yaml"
cp "${planner_params}" "${backup_dir}/sample_planner_params.yaml"

restore() {
  cp "${backup_dir}/user_config.yaml" "${user_config}"
  cp "${backup_dir}/sample_planner_params.yaml" "${planner_params}"
  rm -rf "${backup_dir}"
  echo "Config files restored."
}
trap restore EXIT INT TERM

set_planner() {
  sed -i "s|^\([[:space:]]*robot1_global_planner:[[:space:]]*\)\"[^\"]*\"|\1\"$1\"|" "${user_config}"
}
set_param() {  # set_param <key> <value>
  sed -i "s|^\([[:space:]]*$1:[[:space:]]*\)[0-9.]\+.*|\1$2|" "${planner_params}"
}
read_param() {
  awk -v k="$1" '$1 == k":" { print $2; exit }' "${planner_params}"
}

map_name="$(awk -F'"' '/^map:[[:space:]]*"/ { print $2; exit }' "${user_config}")"

run_point() {  # run_point <planner> <sample_max_d> <number_node_k>
  local planner="$1" smd="$2" k="$3"
  set_planner "${planner}"
  set_param sample_max_d "${smd}"
  set_param number_node_k "${k}"

  echo
  echo "=============================================================="
  echo " ${map_name} | ${planner} | sample_max_d=${smd} | number_node_k=${k} | ${trials} trials"
  echo "=============================================================="

  ( cd "${scripts_dir}" && ./run_benchmark_trials.sh "${trials}" )
  local rc=$?

  # run_benchmark_trials.sh created a fresh directory; label it so the sweep
  # point can still be identified once several runs share a planner name
  local newest
  newest="$(ls -dt "${results_root}"/*/ 2>/dev/null | head -1)"
  if [[ -n "${newest}" ]]; then
    {
      echo "map=${map_name}"
      echo "planner=${planner}"
      echo "sample_max_d=${smd}"
      echo "number_node_k=${k}"
      echo "sample_points=$(read_param sample_points)"
      echo "obstacle_factor=$(read_param obstacle_factor)"
      echo "trials=${trials}"
    } > "${newest}/params.txt"
    echo "labelled ${newest}params.txt"
  fi
  return "${rc}"
}

case "${mode}" in
  stepsize)
    total=$(( ${#STEP_SIZES[@]} * ${#STEP_PLANNERS[@]} ))
    echo "Sweeping sample_max_d ${STEP_SIZES[*]} over ${STEP_PLANNERS[*]}"
    echo "${total} runs x ${trials} trials"
    for smd in "${STEP_SIZES[@]}"; do
      for planner in "${STEP_PLANNERS[@]}"; do
        run_point "${planner}" "${smd}" "$(read_param number_node_k)" || true
      done
    done
    ;;
  k)
    echo "Sweeping number_node_k ${NODE_K[*]} for rrt_cut at sample_max_d=${k_step}"
    for k in "${NODE_K[@]}"; do
      run_point rrt_cut "${k_step}" "${k}" || true
    done
    ;;
esac

echo
echo "Sweep finished. Summarise with:"
echo "  python3 ${scripts_dir}/summarize_benchmarks.py"
