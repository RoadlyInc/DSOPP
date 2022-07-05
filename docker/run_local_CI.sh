#!/usr/bin/sh

if [ "$#" -ne "1" ] && [ "$#" -ne "2" ]
then
  echo "Usage : $0 image_OS(ubuntu,..) clear_build(default false)"
  exit 1
fi

clear_build=false

if [ "$#" -eq "2" ]
then
  clear_build_arg=$2
  if [ "$clear_build_arg" = "true" ]
  then
    clear_build=true
  elif [ "$clear_build_arg" = "false" ]
  then
    clear_build=false
  else
    echo "Unsupported $2, expected true/false"
    exit 1
  fi
fi

script_directory="$(dirname "$(readlink -f "$0")")"
docker_OS=$1
Dockerfile="$script_directory/$docker_OS/Dockerfile"

DSOPP_ROOT=$(readlink -f "$script_directory/../")


if [ ! -f $Dockerfile ]
then
  echo "$Dockerfile does not exist. Please check that there is valid os image"
  exit 1
fi

cd "$script_directory/$docker_OS/"

gcc_build="docker_gcc_build"
clang_build="docker_clang_build"
gcc_float_build="docker_gcc_float_build"

if [ "$clear_build" = "true" ]
then
  if [ -d $gcc_build ]
  then
    echo "Removing $gcc_build"
    rm "$gcc_build" -rf
  fi
  if [ -d "$clang_build" ]
  then
    echo "Removing $clang_build"
    rm "$clang_build" -rf
  fi
  if [ -d "$gcc_float_build" ]
  then
    echo "Removing $gcc_float_build"
    rm "$gcc_float_build" -rf
  fi
fi

# Cleaning DOCKER

docker system prune -f

# BUILDING DOCKER

docker image build -t dsopp:test --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) $script_directory/$docker_OS

# RUNNING CI PIPELINE

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test bash -c "cd DSOPP/docker/$docker_OS/ && \
    cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCOMPILE_HIDDEN_CODE=OFF -DLTO_OPTIMIZATION=OFF -B$gcc_build && cmake --build $gcc_build -- -j4"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"Build gcc\" stage"
  exit 1
fi

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test bash -c "cd DSOPP/docker/$docker_OS/$gcc_build && ctest -VV --schedule-random"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"Test gcc\" stage"
  exit 1
fi

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test bash -c "cd DSOPP/docker/$docker_OS/ && \
    cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang++ -DCOMPILE_HIDDEN_CODE=ON -DLTO_OPTIMIZATION=OFF -B$clang_build && cmake --build $clang_build -- -j4"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"Build clang hidden code\" stage"
  exit 1
fi

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test bash -c "cd DSOPP/docker/$docker_OS/ && \
    cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCOMPILE_HIDDEN_CODE=OFF -DUSE_FLOAT=ON -DLTO_OPTIMIZATION=ON -B$gcc_float_build && cmake --build $gcc_float_build -- -j4"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"Build gcc float\" stage"
  exit 1
fi

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test bash -c "cd DSOPP/docker/$docker_OS/ && \
    GLOG_logtostderr=1 bash ./../../test/performance/application/run_track_performance.sh ./$gcc_float_build/src/application/dsopp_main track.bin ./$gcc_float_build/src/application/track2trajectory output_track_positions.tum output_ecef_poses.enu ../../test/tools/evaluation/evaluate_ate.py ../../test/test_data/track30seconds/gt.tum ../../test/test_data/track30seconds/gt.enu results_odometry.txt results_ecef_poses.txt && \
    cat results_odometry.txt && \
    cat results_ecef_poses.txt && \
    maxrmse=$(cat results_odometry.txt | awk '/^absolute_translational_error.max/ { print $2 < 0.5 }'); if [ \$maxrmse  -eq 0 ] ; then exit 1; fi;"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"performance\" stage"
  exit 1
fi
