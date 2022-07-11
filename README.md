# DSO++

Reimplementation of **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, In arXiv:1607.02565, 2016 (https://github.com/JakobEngel/dso) 

if used in a publication please consider citing: **Upcoming publication**

for commercial use and additional functionality contact Roadly (en@road.ly)

## Building DSOPP

First of all, you need to clone this repository

```
git clone git@github.com:RoadlyInc/DSOPP.git
cd DSOPP
```

### Dependencies installation

<details>
<summary>Without Docker [Not recommended]</summary>

Note that this is not a recommended way to build this code.
Even if you want to extend it and take part in a development process, many modern IDEs give you an opportunity to [develp inside a docker contatiner](https://code.visualstudio.com/docs/remote/containers).
This section is more of an outline of the build process. If you encounter any problems please open an issue.

Be sure that `python3` is installed on your machine.

#### Using python virtual evironment [recommended]

It is recommended to exploit python virtual environments for each external project not to mess up your system's python dependecies.
To install `virtualenv` package:

```
pip3 install virtualenv 
```

To create a virtual environment run the following command

```
python3 -m venv dsopp_venv
```

And to activate:

```
source dsopp_venv/bin/activate
```

Now depending on your shell settings, you might see `(e) dsopp_venv` in your terminal line.

#### Installing python dependencies

From the root of this repository run:

```
pip3 -r requirements.txt
```

#### system dependencies

If you are on ubuntu or debian system:

```
apt install cmake libboost-all-dev graphviz doxygen libgl1-mesa-dev libglew-dev 
apt install git-lfs libprotobuf-dev protobuf-compiler libhdf5-dev libfreetype6-dev
apt install libtbb-dev openexr libavcodec-dev libavformat-dev libswscale-dev 
apt install libavutil-dev clang sed gawk clang-10 zlib1g clang-format-10 ninja-build
apt install wget ffmpeg libeigen3-dev libsuitesparse-dev lcov python2
```

Note: This package names are valid for ubuntu 18.04 and 20.04.

To use `clang-format-10` as `clang-format` add following to your `.bashrc` file:

```
alias clang-format='clang-fromat-10'
```

For installation on other systems, please find the alternative name of each package above in your package manager.

#### g++-10 compiler

You need to install at least `g++-10` compiler. For ubuntu it could be done via:

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update
sudo apt -y install g++-10
```

You would need to pass `-DCMAKE_CXX_COMPILER=g++-10` to cmake

#### Recent cmake version

It is recommended ot use one of recent [cmake versions](https://cmake.org/download/). It could be installed via:

```
wget https://cmake.org/files/v3.17/cmake-3.17.0-Linux-x86_64.tar.gz
tar xvf cmake-3.17.0-Linux-x86_64.tar.gz
cd cmake-3.17.0-Linux-x86_64 
sudo cp -r bin /usr/
sudo cp -r share /usr/
sudo cp -r doc /usr/share/
sudo cp -r man /usr/share/
cd ..
rm -rf cmake*
```

Note that this would overwrite you system's cmake. As an alternative you can us it directly from untared directory.

</details>

<details>
<summary>Docker [recommended]</summary>

Docker is a great way to run a code within specific environment on any machine.
To install docker on ubuntu or debian:

```
sudo apt install docker
```

After installation [docker group should be created and added to user](https://docs.docker.com/engine/install/linux-postinstall/).

```
sudo groupadd docker
sudo usermod -aG docker $USER
```

Do not forget to log out after running the above commands.
Now `cd` to `docker/ubuntu/` and run the following command to build an image

```
docker image build -t dsopp:main --build-arg USER_ID=(id−u) −−build−arg GROUPID=(id -g) .
```

To connect to `dsopp:main` image and use gui docker should be added to xhost

```
xhost local:docker
```

And following command to open shell inside `dsopp:main` image in the current directory:

```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e HOME=HOME−eDISPLAY=DISPLAY -w (pwd) −v HOME:$HOME --device=/dev/dri:/dev/dri -it dsopp:main bash
```

</details>

#### Build

Using `virtualenv` or form `docker image` in root directory of this repo:

```
mkdir build;
cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make -j;
```

##### Download test data
To run tests test data should be downloaded


```
navigate to ./test
sh download_test_data.sh
```

#### Tests

```
cd build;
ctest .;
```

## Running on test sequence

To run on synthetic sequence (`./test/test_data/track30seconds/`)

```
./build/src/application/dsopp_main
```

To run on TUMMONO dataset:

```
naviagate to ./test/test_data/tummono/
sh download_data.sh (sequence id)

navigate to .
./build/src/application/dsopp_main --config_file_path ./test/test_data/tummono/mono.yaml
```

`dsopp_main` outputs `track.bin` file. This is the whole track output with points and all frame poses.
To view it use `viewer_main` application

```
./build/src/application/viewer_main track.bin
```

### [Running on own dataset](docs/running_on_own_dataset.md)

### [Extending DSOPP](docs/extending_dsopp.md)

<!---
### Python library only build

the library would be built in `build/pydsopp`

```
git clone --depth 1 https://github.com/RoadAR/DSOPP
mkdir build && cd build
cmake ../ -DBUILD_DOC=ON -DCHECK_FORMAT=ON
make pydsopp_lib
```

## Python library utils:
from `build` directory:

* `track2json` -- export `track.bin` to json

```
python3 -m pydsopp.utils.track2json --track input_track.bin --output output_track.json
```

* `extract_images` -- extract images from `track.bin`

```
python3 -m pydsopp.utils.extract_images --track input_track.bin --images output_image_folder
```

* `point_cloud_exporter` -- export point cloud in `xyz` or `las` format, in addition poses could be exported via `poses_file` path

Poses are exported in `frame_id t_x t_y t_z q_x q_y q_z q_w` format. Each pose is from camera to `ecef` world.
```
python3 -m pydsopp.utils.point_cloud_exporter --track track.bin --output output.xyz --coord_system ecef --color_scheme image_colors --file_format xyz --poses_file poses.txt
```
-->
