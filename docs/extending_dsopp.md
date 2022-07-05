# Extending DSOPP

DSOPP was designed as a modular system. Almost every component could be replaced at runtime, providing all needed classes and creation from `yaml` config in factory files.

### Motion

This helps to use this code for research purposes. 
For example, if you came up with a new Rolling Shutter formulation, then all you need is to create another `Motion` class in `src/energy/motion`. And create `dsopp<Motion>` class in `src/application/dsopp_main.cpp` with your own brand new motion model. Then try to compile, and implement new methods like derivatives in `reprojector`/`projector` files. Then try to compile, implement some more methods, and it is done.

### New Camera Model

To implement new camera model in dsopp you need to create model class in `src/energy/camera_model`. Construct of your model in `src/sensors/camera_calibration/src/fabric.cpp` from `calib.txt` file.

### New Frontend Solver

Let us imagine that you have trained NN to predict transformation from one image into another and you want to use it instead of a frontend solver. 
You'll need to create new `pose_aligner` in `src/energy/problems/include/energy/problems/pose_alignment/` and construct it from `mono.yaml` in `src/tracker/tracker/src/fabric.cpp` inside `createPoseAlignment` function.

### N channel images in Frontend Solver

Want to use some image embedding to spice the backend solver up? No problem! Implement embedder code somewhere. Add desired instantations on the bootom of `src/energy/problems/src/eigen_pose_alignment.cpp`, pass your new images to solver in `src/tracker/tracker/src/monocular_tracker.cpp` in `estimatePose` function.
