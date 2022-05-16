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

coverage_build="docker_coverage_build"

if [ "$clear_build" = "true" ]
then
  if [ -d $coverage_build ]
  then
    echo "Removing $coverage_build"
    rm "$coverage_build" -rf
  fi
fi

# Cleaning DOCKER

docker system prune -f

# BUILDING DOCKER

docker image build -t dsopp:test_coverage $script_directory/$docker_OS

# RUNNING CI PIPELINE

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test_coverage bash -c "cd DSOPP/docker/$docker_OS/ && \
    cmake ../../ -DCMAKE_BUILD_TYPE=Debug -DTEST_COVERAGE=ON -DCOMPILE_HIDDEN_CODE=OFF -DLTO_OPTIMIZATION=OFF -B$coverage_build && cmake --build $coverage_build -- -j4"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"Build coverage\" stage"
  exit 1
fi

docker run -v $DSOPP_ROOT/:/app/DSOPP/ -it dsopp:test_coverage bash -c "cd DSOPP/docker/$docker_OS/$coverage_build && find . -name \"*.gcda\" -delete && ctest --label-regex \"Coverage\" && lcov --directory . --capture --output-file coverage.info && lcov --remove coverage.info \"/usr/*\" \"*/3rd_party/*\" \"*/_deps/*\" \"*/test/*\" --output-file coverage.info && lcov --list coverage.info"

code=$?
if [ "$code" -ne "0" ]
then
  echo "Build failed on \"Test coverage\" stage"
  exit 1
fi
