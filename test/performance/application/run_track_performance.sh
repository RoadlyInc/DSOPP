#!/usr/bin/sh

check_file_existence() {
  if [ ! -f "$1" ]
  then
    echo "$1 does not exist. Please check that there is valid path to $2"
    exit 1
  fi
}

run_command() {
  eval "$1"
  code=$?
  if [ "$code" -ne "0" ]
  then
    echo "Failed on $1"
    exit 1
  fi
}

if [ "$#" -ne "10" ] && [ "$#" -ne "13" ]
then
  echo "Usage : $0 dsopp_main_path track_bin_path track2trajectory_path output_track_positions_tum_path output_ecef_poses_enu_path evaluate_ate_py_path gt_tum_path gt_enu_path results_odometry_txt_path results_ecef_poses_txt_path"
  echo "        $0 dsopp_main_path track_bin_path track2trajectory_path output_track_positions_tum_path output_ecef_poses_enu_path evaluate_ate_py_path gt_tum_path gt_enu_path results_odometry_txt_path results_ecef_poses_txt_path config_file_path trans_error_odometry_txt_path trans_error_ecef_poses_txt_path"
  exit 1
fi

dsopp_main_path=$1
check_file_existence "$dsopp_main_path" "dsopp_main"

track_bin_path=$2
track2trajectory_path=$3
check_file_existence "$track2trajectory_path" "track2trajectory"

output_track_positions_tum_path=$4
output_ecef_poses_enu_path=$5
evaluate_ate_py_path=$6
check_file_existence "$evaluate_ate_py_path" "evaluate_ate.py"

gt_tum_path=$7
check_file_existence "$gt_tum_path" "gt.tum"

gt_enu_path=$8
check_file_existence "$gt_enu_path" "gt.enu"

results_odometry_txt_path=$9
results_ecef_poses_txt_path=${10}

config_file_path=${11}
trans_error_odometry_txt_path=${12}
trans_error_ecef_poses_txt_path=${13}

if [ -z "$config_file_path" ]
then
  run_dsopp_main="$dsopp_main_path --output_file_path=$track_bin_path --novisualization"
else
  check_file_existence "$config_file_path" "config file"
  run_dsopp_main="$dsopp_main_path --config_file_path=$config_file_path --output_file_path=$track_bin_path --novisualization"
fi

time_start=$(date +%s)
run_command "$run_dsopp_main"
time_stop=$(date +%s)
echo "Elapsed time : $((time_stop - time_start)) s"

run_track2tum="$track2trajectory_path odometry $track_bin_path $output_track_positions_tum_path"
run_command "$run_track2tum"

run_track2enu="$track2trajectory_path ecef_poses $track_bin_path $output_ecef_poses_enu_path"
run_command "$run_track2enu"

if [ -z "$trans_error_odometry_txt_path" ]
then
  run_evaluate_ate_py_odometry="python3 $evaluate_ate_py_path $gt_tum_path $output_track_positions_tum_path --verbose > $results_odometry_txt_path"
else
  run_evaluate_ate_py_odometry="python3 $evaluate_ate_py_path $gt_tum_path $output_track_positions_tum_path --save_translational_error $trans_error_odometry_txt_path --verbose > $results_odometry_txt_path"
fi

run_command "$run_evaluate_ate_py_odometry"

if [ -z "$trans_error_ecef_poses_txt_path" ]
then
  run_evaluate_ate_py_ecef_poses="python3 $evaluate_ate_py_path $gt_enu_path $output_ecef_poses_enu_path --files_format=enu --verbose > $results_ecef_poses_txt_path"
else
  run_evaluate_ate_py_ecef_poses="python3 $evaluate_ate_py_path $gt_enu_path $output_ecef_poses_enu_path --files_format=enu --save_translational_error $trans_error_ecef_poses_txt_path --verbose > $results_ecef_poses_txt_path"
fi

run_command "$run_evaluate_ate_py_ecef_poses"
