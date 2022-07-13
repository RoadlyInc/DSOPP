`pydsopp` offers an easy way to process `track.bin` from `dsopp_main`

### `nerf_exporter.py`

Export `track.bin` format to [Nvidia instant-ngp](https://github.com/NVlabs/instant-ngp) nerf format

From the build folder:

```
python3 -m pydsopp.utils.nerf_exporter --track track_bin_path --output output_directory
```

### `track2colmap.py`

Export `track.bin` to colmap `txt` track format

From build folder:

```
python3 -m pydsopp.utils.track2colmap --track track_bin_path --output output_directory
```

Note: `save_images` should be set to `on` in `mono.yaml` to save undistorted images too

### `point_cloud_exporter.py`

Sparse point cloud exporter

From the build folder:

```
python3 -m pydsopp.utils.point_cloud_exporter --track track_bin_path --coord_system odometry --color_scheme image_colors --file_format xyz --output output_cloud_path.xyz
```

Note: that `--color_schame image_colors` would work only if `save_images` was set to `on` in `mono.yaml`, otherwise use `--color_scheme semantic`
