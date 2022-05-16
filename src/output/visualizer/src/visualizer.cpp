#include "visualizer/visualizer.hpp"

#include "sensors/camera/camera.hpp"
#include "track/active_track.hpp"
#include "track/odometry_track.hpp"
#include "visualizer/local_track.hpp"
#include "visualizer/renderable.hpp"
#include "visualizer/visualizer_image_output_interface.hpp"
#include "visualizer/visualizer_text_output_interface.hpp"
#include "visualizer/visualizer_track_output_interface.hpp"

#include <glog/logging.h>
#include <cstdint>

namespace dsopp {
namespace output {

namespace {

void addViewPanel(pangolin::View &main_view, const std::string &name, int left, int right) {
  pangolin::View &view =
      pangolin::CreatePanel(name).SetBounds(0.0, 1.0, pangolin::Attach::Pix(left), pangolin::Attach::Pix(right));
  view.ToggleShow();
  main_view.AddDisplay(view);
}

void handleButtonPush(const std::string &name, pangolin::View &display) {
  auto &view = pangolin::Display(name);
  display.SetBounds(
      0.0, 1.0, pangolin::Attach::Pix(pangolin::Display("ui").GetBounds().w * (std::abs(-2 + view.IsShown()))), 1.0);
  view.ToggleShow();
}

void closeView(const std::string &name) {
  auto &view = pangolin::Display(name);
  if (view.IsShown()) {
    view.ToggleShow();
  }
}
const int kUiWidth = 170;
/**
 * set up settings of the visualizer
 * @param width, height sizes of the window with the visualization
 * @param[out] display3D 3d scene of the visualizer
 * @param[out] camera camera view of the visualizer
 * @return main view
 */
pangolin::View &setUpVisualizer(int width, int height, pangolin::View &display3D, pangolin::OpenGlRenderState &camera) {
  pangolin::CreateWindowAndBind("VisualizerOutputInterface", width, height);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_POINT_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  pangolin::View &main_view = pangolin::Display("main").SetBounds(0.0, 1.0, 0.0, 1.0);

  camera = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 1, 10000),
                                       pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY));
  display3D = pangolin::Display("scene")
                  .SetBounds(0.0, 1.0, pangolin::Attach::Pix(2 * kUiWidth), 1.0)
                  .SetAspect(-640 / 480.0)
                  .SetHandler(new pangolin::Handler3D(camera));
  main_view.AddDisplay(display3D);

  pangolin::View &ui = pangolin::CreatePanel("ui")
                           .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(kUiWidth))
                           .SetHandler(new pangolin::Handler());

  main_view.AddDisplay(ui);

  addViewPanel(main_view, "landmarks_view", kUiWidth, 2 * kUiWidth);
  addViewPanel(main_view, "gnss_view", kUiWidth, 2 * kUiWidth);
  addViewPanel(main_view, "camera_view", kUiWidth, 2 * kUiWidth);
  addViewPanel(main_view, "correspondences_view", kUiWidth, 2 * kUiWidth);
  // This allows you to make the windows hidden.
  display3D.SetBounds(0.0, 1.0, pangolin::Attach::Pix(kUiWidth), 1.0);

  glClearColor(0.13f, 0.18f, 0.21f, 1.0f);
  {
    pangolin::View &status = pangolin::CreatePanel("status").SetBounds(
        pangolin::Attach::Pix(0), pangolin::Attach::Pix(50), pangolin::Attach::Pix(kUiWidth), 1.0);

    main_view.AddDisplay(status);
  }
  return main_view;
}

OutputParameters createOutputParameters(const Visualizer::MutableObjects &mutable_objects) {
  auto unpack = []<typename... Ts>(Ts... ts) { return OutputParameters({*ts...}); };
  return std::apply(unpack, mutable_objects);
}

VisualizerImageOutputInterface *addImageOutputInterface(
    std::deque<Renderable *> &renderable_objects,
    std::vector<std::unique_ptr<VisualizerOutputInterface>> &output_interfaces, int width, int height, int location_x,
    int location_y) {
  auto new_interface = std::make_unique<VisualizerImageOutputInterface>(width, height, location_x, location_y);
  auto *return_interface = new_interface.get();
  renderable_objects.push_back(return_interface);
  output_interfaces.push_back(std::move(new_interface));
  return return_interface;
}

}  // namespace

Visualizer::Visualizer(int width, int height, OutputParameters output_parameters)
    : width_(width),
      height_(height),
      landmarks_view_("ui.Landmarks view", false, false),
      show_immature_landmarks_("landmarks_view.Immature", output_parameters.show_immature_landmarks, true),
      show_active_landmarks_("landmarks_view.Active", output_parameters.show_active_landmarks, true),
      show_marginalized_landmarks_("landmarks_view.Marginalized", output_parameters.show_marginalized_landmarks, true),
      show_outliers_("landmarks_view.Outliers", output_parameters.show_outliers, true),
      landmarks_colored_view_("landmarks_view.Colored view", output_parameters.colored_view, true),
      track_based_landmark_colored_view_("landmarks_view.Track_colors", output_parameters.track_based_colors, true),
      landmarks_semantics_colored_view_("landmarks_view.Semantic color", output_parameters.show_semantics_colors, true),
      apply_filters_("landmarks_view.Filters", output_parameters.apply_filters, true),
      idepth_variance_threshold_("landmarks_view.Idepth variance", output_parameters.idepth_variance_threshold, 1e-16,
                                 1e-5, true),
      relative_baseline_threshold_("landmarks_view.Rel baseline", output_parameters.relative_baseline_threshold, 0.1, 5,
                                   true),
      gnss_view_("ui.GNSS", false, false),
      show_odometry_track_("gnss_view.Odometry <-> ECEF", output_parameters.show_odometry_track, true),
      show_gnss_track_("gnss_view.Show GNSS track", output_parameters.show_gnss_track, true),
      camera_view_("ui.Camera view", false, false),
      show_camera_("camera_view.Show camera", output_parameters.show_camera, true),
      camera_size_("camera_view.Camera size", output_parameters.camera_size, 0.05, 20, true),
      correspondences_view_("ui.Correspondences", false, false),
      show_candidates_("correspondences_view.Candidates", output_parameters.show_candidates, true),
      show_failed_("correspondences_view.Failed", output_parameters.show_failed, true),
      show_relocalized_("correspondences_view.Relocalized", output_parameters.show_relocalized, true),
      show_skipped_("correspondences_view.Skipped", output_parameters.show_skipped, true) {
  /**
   * Order of variables should match order of variables in ``OutputParameters`` struct
   */

  mutable_objects_ = {&show_immature_landmarks_,
                      &show_active_landmarks_,
                      &show_marginalized_landmarks_,
                      &show_outliers_,
                      &landmarks_colored_view_,
                      &track_based_landmark_colored_view_,
                      &landmarks_semantics_colored_view_,
                      &show_odometry_track_,
                      &show_gnss_track_,
                      &show_camera_,
                      &apply_filters_,
                      &show_candidates_,
                      &show_failed_,
                      &show_relocalized_,
                      &show_skipped_,
                      &camera_size_,
                      &idepth_variance_threshold_,
                      &relative_baseline_threshold_};
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
TrackOutputInterface<OdometryTrackType, Motion> *Visualizer::createTrackOutputInterface() {
  output::OutputParameters parameters = createOutputParameters(mutable_objects_);
  tracks_.emplace_back(std::make_unique<LocalTrack>(parameters));
  renderable_objects_.push_back(tracks_.back().get());
  auto new_interface = std::make_unique<VisualizerTrackOutputInterface<OdometryTrackType, Motion>>(*tracks_.back());
  auto *return_interface = new_interface.get();
  output_interfaces_.push_back(std::move(new_interface));
  return return_interface;
}

std::map<size_t, CameraOutputInterfaces> Visualizer::createCameraOutputInterfaces(
    const std::deque<const sensors::Camera *> &cameras, int width, int height) {
  const int kXMargin = 60;
  const int kYMargin = 10;

  std::map<size_t, CameraOutputInterfaces> result;
  int next_image_location_y = 0;
  int next_image_location_x = 0;
  for (const auto &camera : cameras) {
    // current keyframe
    result[camera->id()].current_keyframe =
        addImageOutputInterface(renderable_objects_, output_interfaces_, width, height,
                                kXMargin + next_image_location_x, kUiWidth + kYMargin + next_image_location_y);
    next_image_location_y += kYMargin + width;
    // current frame
    result[camera->id()].current_frame =
        addImageOutputInterface(renderable_objects_, output_interfaces_, width, height,
                                kXMargin + next_image_location_x, kUiWidth + kYMargin + next_image_location_y);
    next_image_location_y += kYMargin + width;
    next_image_location_x += kXMargin + height;
    next_image_location_y = 0;
  }

  return result;
}

TextOutputInterface *Visualizer::createTextOutputInterface(const std::string &name) {
  auto new_interface = std::make_unique<VisualizerTextOutputInterface>(name);
  auto *return_interface = new_interface.get();
  renderable_objects_.push_back(return_interface);
  output_interfaces_.push_back(std::move(new_interface));
  return return_interface;
}

bool Visualizer::running() const { return running_ && !pangolin::ShouldQuit(); }

void Visualizer::close() { running_ = false; }

void Visualizer::init() {
  static std::array<Eigen::Vector3<uint8_t>, 20> colors = {
      Eigen::Vector3<uint8_t>{230, 25, 75},  Eigen::Vector3<uint8_t>{60, 180, 75},
      Eigen::Vector3<uint8_t>{255, 225, 25}, Eigen::Vector3<uint8_t>{0, 130, 200},
      Eigen::Vector3<uint8_t>{245, 130, 48}, Eigen::Vector3<uint8_t>{145, 30, 180},
      Eigen::Vector3<uint8_t>{70, 240, 240}, Eigen::Vector3<uint8_t>{240, 50, 230},
      Eigen::Vector3<uint8_t>{210, 245, 60}, Eigen::Vector3<uint8_t>{250, 190, 190},
      Eigen::Vector3<uint8_t>{0, 128, 128},  Eigen::Vector3<uint8_t>{230, 190, 255},
      Eigen::Vector3<uint8_t>{170, 110, 40}, Eigen::Vector3<uint8_t>{255, 250, 200},
      Eigen::Vector3<uint8_t>{128, 0, 0},    Eigen::Vector3<uint8_t>{170, 255, 195},
      Eigen::Vector3<uint8_t>{128, 128, 0},  Eigen::Vector3<uint8_t>{255, 215, 180},
      Eigen::Vector3<uint8_t>{0, 0, 128},    Eigen::Vector3<uint8_t>{128, 128, 128}};

  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (i < colors.size()) {
      tracks_[i]->setUniqueColor(colors[i]);
    } else {
      tracks_[i]->setUniqueColor(Eigen::Vector3<uint8_t>::Random());
    }
  }

  running_ = true;
  auto &main_view = setUpVisualizer(width_, height_, this->display3D_, this->camera_);

  for (auto &renderer : renderable_objects_) renderer->init(main_view);
}

void Visualizer::render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  std::vector<std::pair<std::string, pangolin::Var<bool>>> views = {{"landmarks_view", landmarks_view_},
                                                                    {"gnss_view", gnss_view_},
                                                                    {"camera_view", camera_view_},
                                                                    {"correspondences_view", correspondences_view_}};

  auto close_views = [views](const std::string &except_name) {
    for (auto &[name, view] : views)
      if (name != except_name) closeView(name);
  };

  for (auto &[name, view] : views)
    if (pangolin::Pushed(view)) {
      close_views(name);
      handleButtonPush(name, display3D_);
    }

  auto has_gui_changed = []<typename... Ts>(Ts... ts) { return (ts->GuiChanged() || ...); };
  auto gui_changed = std::apply(has_gui_changed, mutable_objects_);

  if (gui_changed) {
    for (auto &renderer : renderable_objects_) {
      renderer->setParameters(createOutputParameters(mutable_objects_));
    }
  }

  display3D_.Activate(camera_);
  pangolin::glDrawAxis(2.0f);

  for (auto &renderer : renderable_objects_) {
    renderer->render();
  }

  pangolin::FinishFrame();
}

Visualizer::~Visualizer() = default;

template TrackOutputInterface<track::ActiveOdometryTrack, energy::motion::SE3<Precision>>
    *Visualizer::createTrackOutputInterface<track::ActiveOdometryTrack, energy::motion::SE3<Precision>>();
template TrackOutputInterface<track::OdometryTrack, energy::motion::SE3<Precision>>
    *Visualizer::createTrackOutputInterface<track::OdometryTrack, energy::motion::SE3<Precision>>();

}  // namespace output
}  // namespace dsopp
