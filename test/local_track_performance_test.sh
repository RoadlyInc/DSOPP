dsopp_main=${DSOPP_SOURCE_PATH}/dsopp_main
track2trajectory=${DSOPP_SOURCE_PATH}/track2trajectory
evaluator_path=${PYTHON_EVALUATOR_PATH}
dataset_mono=${DATASET_MONO}
dataset_gt_tum=${DATASET_GT_TUM}
dataset_gt_enu=${DATASET_GT_ENU}

rm -f track.bin result_odometry.tum result_ecef_poses.enu

GLOG_logtostderr=1 GLOG_v=2 $dsopp_main -visualization=false -config_file_path=$dataset_mono
$track2trajectory odometry track.bin result_odometry.tum
$track2trajectory ecef_poses track.bin result_ecef_poses.enu
python3 $evaluator_path $dataset_gt_tum result_odometry.tum --verbose
python3 $evaluator_path $dataset_gt_enu result_ecef_poses.enu --files_format=enu --verbose

rm -f track.bin result_odometry.tum result_ecef_poses.enu
