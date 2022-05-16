# DSO++

Reimplementation of **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, In arXiv:1607.02565, 2016 (https://github.com/JakobEngel/dso) 

if used in a publication please consider citing: **Upcomming publication**

for commercial use and additional functionality contact Roadly (en@road.ly)

## Installation

### dependencies

to install dependecies follow a corresponding docker file:
  [Ubuntu](docker/ubuntu/Dockerfile)


### Build

```
mkdir build;
cd build;
cmake .. -DCMAKE_CXX_COMPILER=g++-10 -DCMAKE_BUILD_TYPE=Release;
make -j;
```

### Python library only build

library would be build in `build/pydsopp`

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

## Download test data

```
navigate to ./test
sh download_test_data.sh
```

## Tests

```
cd build;
ctest .;
```
