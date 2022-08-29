The core of the code and the enterance point for familirazaing yourself with a code is the interface [Tracker](../src/tracker/tracker/include/tracker/tracker.hpp) and its' only (for now) implementation [MonocularTracker](../src/tracker/tracker/include/tracker/monocular/monocular_tracker.hpp). MonocularTracker fetches the data from the sensor and pass it to 5 main stages of a direct odometry:

1. Pose estimation
2. Feature selection
3. Depth estimation
4. Backend optimization 

All four stages have identical behaviour to the original DSO implementation. 

## Pose estimation

On the pose estimation stage we align a new frame to the built point cloud.The interface responsible for pose estimation ([PoseAlignment](../src/energy/problems/include/energy/problems/pose_alignment/pose_alignment.hpp)) has 2 implemenations: [CeresPhotometricBundleAdjustment](../src/energy/problems/include/energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp) and [EigenPoseAlignment](../src/energy/problems/include/energy/problems/pose_alignment/eigen_pose_alignment.hpp). Those implementations have identical behaviour. CeresPhotometricBundleAdjustment is implemented using the ceres library and mostly used in test to prove validiy of a custom pose aligner(EigenPoseAligment). 

## Feature selection

The interface responsible for feature selection is [TrackingFeaturesExtractor](../src/features/include/features/camera/tracking_features_extractor.hpp) and also has two implentation [Sobel](../src/features/include/features/camera/sobel_tracking_features_extractor.hpp) and [Eigen](../src/features/include/features/camera/eigen_tracking_features_extractor.hpp). The second is identical to the original DSO feature selector. 

## Depth estimation

The class [DepthEstimation](../src/tracker/depth_estimators/include/tracker/depth_estimators/depth_estimation.hpp) is responsible for gradualy refine the depth of the immature point. [LandmarksActivator](../src/tracker/landmarks_activator/include/tracker/landmarks_activator/landmarks_activator.hpp) selects point to be activated from the immature points pool. 

## Backend optimization

Same to pose estimation backend optimization has 2 implementations based on [Ceres](../src/energy/problems/include/energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp) and [Eigen](../src/energy/problems/include/energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp). 