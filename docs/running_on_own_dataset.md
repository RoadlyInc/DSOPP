# Running on own dataset

## Calibration

To run `dsopp_main` on your sequence calibration should be obtained

* photometric calibration
  * gamma correction `pcalib.txt`
  * vignetting `vignetting.png`
* geometric calibration `calib.txt`

For photometric calibration please refer to [ONLINE PHOTOMETRIC CALIBRATION](https://vision.in.tum.de/research/vslam/photometric-calibration) and [mono_dataset_code](https://github.com/tum-vision/mono_dataset_code)

For geometric calibration for example to [OpenCV calibration tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

### Currently supported camera models:

####  Pinhole Camera Model

`calib.txt` file for pinhole camera model:

```
pinhole
width height
focal_x focal_y
center_x center_y
```

An example could be found in `test/test_data/track30seconds/calib.txt`

#### Simple Radial Camera Model

This model is the same as the radial model in colmap or pinhole from OpenCV but only `k1` `k2` is non-zero.

```
simple_radial
width height
focal
center_x center_y
k1 k2
```

## Other files

`times.csv` contains timestamps and frames ids. For example

```
0 5.84253863990307
1 5.89253864064813
2 5.94253864139318
```

If you do not plan to use any sensors, then create `timest.csv` with `number of frames` lines with the following content

```
0 0
1 1
2 2
3 3
...
```

`Video file` or `Image folder` with your images, and videos. If you decided to use images, please name them `%06d.png`.

`[Optional] Mask` uses black to exclude region, and white to include.

Then the  minimal config `mono.yaml` would be:

<details>
<summary>Minimal config</summary>

```
sensors:
  - id: camera_1
    type: camera
    provider:
      type: video
      video_file: "./images.mkv"
      timestamps: "./times.csv"
      timestamps_start_id: 0
    model:
      calibration: "./calib.txt"
      shutter_time_seconds: 0
      photometric_calibration: "./pcalib.txt"
      vignetting: "./vignetting.png"
    position: world

time:
  type: no_synchronization

tracker:
  type: monocular
  sensor_id: camera_1
  number_of_desired_points: 2000
  keyframe_strategy:
    strategy: mean_square_optical_flow
    factor: 1.0 #the larger the more keyframes are created (e.g., 2 = double number of keyframes)
  marginalization_strategy:
    strategy: sparse
    minimum_size: 5
    maximum_size: 7
    maximum_percentage_of_marginalized_points_in_frame: 0.95

initializer:
  type: monocular
  initializer_type: calibrated
  sensor_id: camera_1
  essential_matrix_ransac_threshold: 0.2 # in pixels at 1280x720 image
  pnp_ransac_threshold: 0.2 # in pixels at 1280x720 image
  se3_inlier_ratio: 0.6
  reprojection_threshold: 1 # in pixels at 1280x720 image
  pnp_inlier_ratio: 0.5
  features_extractor:
    type: ORB
    number_of_features: 500
  keyframe_strategy:
    strategy: wait_for_movement
    sliding_window_length: 4

gnss_alignmenter:
  mode: off
```
</details>

