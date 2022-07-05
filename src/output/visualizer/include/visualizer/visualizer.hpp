#ifndef DSOPP_VISUALIZER_HPP
#define DSOPP_VISUALIZER_HPP

#include <deque>

#include <GL/glew.h>
#include <pangolin/pangolin.h>

#include "energy/motion/motion.hpp"
#include "output_interfaces/camera_output_interfaces.hpp"
#include "visualizer/output_parameters.hpp"

namespace dsopp {
namespace sensors {
class Camera;
}
namespace relocalization {
class CameraCorrespondence;
}
namespace output {
class Renderable;
class LocalTrack;
class LocalCorrespondence;
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class TrackOutputInterface;
class VisualizerOutputInterface;
class ImageOutputInterface;
class TextOutputInterface;

/**
 * \brief Main visualizer with multiple interface.
 *
 * 3D display of tracks and point clouds.
 */
class Visualizer {
 public:
  /** type for mutable objects */
  using MutableObjects =
      decltype(std::tuple_cat(std::array<pangolin::Var<bool> *, 15>(), std::array<pangolin::Var<double> *, 3>()));
  /**
   * @param width, height sizes of the window with the visualization
   * @param output_parameters parameters for visualization
   */
  Visualizer(int width, int height, OutputParameters output_parameters = {});
  /**
   * create the window with the visualization
   */
  void init();
  /**
   * render frame of the visualization
   */
  void render();
  /**
   * running state of the visualization window
   * @return true if the visualization window is running and false otherwise
   */
  bool running() const;
  /**
   * close the window with the visualization
   */
  void close();
  /**
   * create new visualizer and return pointer to it
   * @return pointer to new visualizer
   */
  template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
  TrackOutputInterface<OdometryTrackType, Motion> *createTrackOutputInterface();
  /**
   * create new image output interface and return pointer to it
   * @param cameras CameraOutputInterfaces will be created for every camera
   * @param width,height size of the image
   * @return CameraOutputInterfaces corresponds to the camera id
   */
  std::map<size_t, CameraOutputInterfaces> createCameraOutputInterfaces(
      const std::deque<const sensors::Camera *> &cameras, int width, int height);

  /**
   * create status text output interface
   * @param name field name in ui
   * @return pointer to brand new text interface
   */
  TextOutputInterface *createTextOutputInterface(const std::string &name);

  ~Visualizer();

 private:
  /** width of the window with the visualization */
  int width_;
  /** height of the window with the visualization */
  int height_;
  /** visualizer control variable */
  bool running_ = false;
  /** 3d scene of the visualizer */
  pangolin::View display3D_;
  /** camera view of the visualizer */
  pangolin::OpenGlRenderState camera_;
  /** contain call renderable object of the scene */
  std::deque<Renderable *> renderable_objects_;
  /** all tracks of the scene */
  std::vector<std::unique_ptr<LocalTrack>> tracks_;
  /** all output interfaces of the visualizer
   * stored here to manage their lifetime
   */
  std::vector<std::unique_ptr<VisualizerOutputInterface>> output_interfaces_;

  /** button to show/hide the panel for setting parameters of landmarks visualization */
  pangolin::Var<bool> landmarks_view_;
  /** button to show/hide the immature landmarks in the 3d */
  pangolin::Var<bool> show_immature_landmarks_;
  /** button to show/hide the active landmarks in the 3d */
  pangolin::Var<bool> show_active_landmarks_;
  /** button to show/hide the marginalized landmarks in the 3d */
  pangolin::Var<bool> show_marginalized_landmarks_;
  /** button to show/hide the outliers in the 3d */
  pangolin::Var<bool> show_outliers_;
  /** display the color of the landmark according to the type or white */
  pangolin::Var<bool> landmarks_colored_view_;
  /** dsiplay points in each track with different colors */
  pangolin::Var<bool> track_based_landmark_colored_view_;
  /** display the semantics color (this is less priority */
  pangolin::Var<bool> landmarks_semantics_colored_view_;
  /** button for turn on/off of filtering by idepth variance and relative baseline */
  pangolin::Var<bool> apply_filters_;
  /** slider for filtering points by inverse depth variance */
  pangolin::Var<double> idepth_variance_threshold_;
  /** slider for filtering points by relative baseline */
  pangolin::Var<double> relative_baseline_threshold_;

  /** button to show/hide the panel for setting parameters of GNSS visualization */
  pangolin::Var<bool> gnss_view_;
  /** button to show the odometry track instead of the ECEF poses */
  pangolin::Var<bool> show_odometry_track_;
  /** button to show/hide the GNSS track */
  pangolin::Var<bool> show_gnss_track_;

  /** button to show/hide the panel for settings parameters of Camera visualization */
  pangolin::Var<bool> camera_view_;
  /** button to show/hide camera */
  pangolin::Var<bool> show_camera_;
  /** slider for camera size */
  pangolin::Var<double> camera_size_;

  /** button to show/hide the panel for settings parameters of correspondences visualization */
  pangolin::Var<bool> correspondences_view_;
  /** button to show/hide candidates*/
  pangolin::Var<bool> show_candidates_;
  /** button to show/hide failed correspondences*/
  pangolin::Var<bool> show_failed_;
  /** button to show/hide relocalized correspondences*/
  pangolin::Var<bool> show_relocalized_;
  /** button to show skipped correspondences */
  pangolin::Var<bool> show_skipped_;

  /** tuple of pointers to all mutable objects */
  MutableObjects mutable_objects_;
};

}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_VISUALIZER_HPP
