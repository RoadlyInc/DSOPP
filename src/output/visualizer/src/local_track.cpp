#include "visualizer/local_track.hpp"

#include "energy/motion/se3_motion.hpp"

#include "track/active_odometry_track.hpp"
#include "track/odometry_track.hpp"
#include "track/track_base.hpp"

namespace dsopp {
namespace output {
namespace {
/**
 * draw frame camera in the 3d space
 * @param transformation agent position matrix
 * @param color color of the agent
 */
void drawCamera(const Eigen::Matrix4d &transformation, const Eigen::Vector4d &color, const double size_factor) {
  glPushMatrix();

  glMultMatrixd(transformation.data());

  glColor4dv(color.data());

  const double width = 640, height = 480, fx = 500, fy = 500, cx = 320, cy = 240;

  glLineWidth(2);
  glBegin(GL_LINES);
  glVertex3d(0, 0, 0);
  glVertex3d(size_factor * (0 - cx) / fx, size_factor * (0 - cy) / fy, size_factor);
  glVertex3d(0, 0, 0);
  glVertex3d(size_factor * (0 - cx) / fx, size_factor * (height - 1 - cy) / fy, size_factor);
  glVertex3d(0, 0, 0);
  glVertex3d(size_factor * (width - 1 - cx) / fx, size_factor * (height - 1 - cy) / fy, size_factor);
  glVertex3d(0, 0, 0);
  glVertex3d(size_factor * (width - 1 - cx) / fx, size_factor * (0 - cy) / fy, size_factor);

  glVertex3d(size_factor * (width - 1 - cx) / fx, size_factor * (0 - cy) / fy, size_factor);
  glVertex3d(size_factor * (width - 1 - cx) / fx, size_factor * (height - 1 - cy) / fy, size_factor);

  glVertex3d(size_factor * (width - 1 - cx) / fx, size_factor * (height - 1 - cy) / fy, size_factor);
  glVertex3d(size_factor * (0 - cx) / fx, size_factor * (height - 1 - cy) / fy, size_factor);

  glVertex3d(size_factor * (0 - cx) / fx, size_factor * (height - 1 - cy) / fy, size_factor);
  glVertex3d(size_factor * (0 - cx) / fx, size_factor * (0 - cy) / fy, size_factor);

  glVertex3d(size_factor * (0 - cx) / fx, size_factor * (0 - cy) / fy, size_factor);
  glVertex3d(size_factor * (width - 1 - cx) / fx, size_factor * (0 - cy) / fy, size_factor);

  glEnd();

  glPopMatrix();
}

void initialize(const PointsStorage &points_storage, BufferStorage &buffer_storage,
                const Eigen::Vector3<uint8_t> &track_unique_color) {
  for (const auto &[sensor, points] : points_storage.points) {
    unsigned int size = static_cast<unsigned int>(points.size());

    auto &vertex_buffer = buffer_storage.points[sensor];
    vertex_buffer.Reinitialise(pangolin::GlArrayBuffer, size, std::is_same_v<Precision, float> ? GL_FLOAT : GL_DOUBLE,
                               3, GL_DYNAMIC_DRAW);

    std::vector<Eigen::Vector3<uint8_t>> track_unique_colors(points_storage.colors.at(sensor).size(),
                                                             track_unique_color);

    using BufferColor = std::pair<pangolin::GlBuffer *, const std::vector<Eigen::Vector3<uint8_t>> *>;
    std::vector<BufferColor> buffer_color = {
        {&buffer_storage.colors[sensor], &points_storage.colors.at(sensor)},
        {&buffer_storage.default_colors[sensor], &points_storage.default_colors.at(sensor)},
        {&buffer_storage.semantic_colors[sensor], &points_storage.semantics_colors.at(sensor)},
        {&buffer_storage.track_specific_color[sensor], &track_unique_colors}};

    for (auto &b_c : buffer_color) {
      b_c.first->Reinitialise(pangolin::GlArrayBuffer, size, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
    }
  }
}

void upload(const PointsStorage &points_storage, BufferStorage &buffer_storage,
            const Eigen::Vector3<uint8_t> &track_unique_color) {
  for (const auto &[sensor, points] : points_storage.points) {
    unsigned int size = static_cast<unsigned int>(points.size());
    auto &vertex_buffer = buffer_storage.points[sensor];
    vertex_buffer.Upload(points.data(), static_cast<GLsizeiptr>(size * sizeof(Precision) * 3), 0);

    std::vector<Eigen::Vector3<uint8_t>> track_unique_colors(points_storage.colors.at(sensor).size(),
                                                             track_unique_color);

    using BufferColor = std::pair<pangolin::GlBuffer *, const std::vector<Eigen::Vector3<uint8_t>> *>;
    std::vector<BufferColor> buffer_color = {
        {&buffer_storage.colors[sensor], &points_storage.colors.at(sensor)},
        {&buffer_storage.default_colors[sensor], &points_storage.default_colors.at(sensor)},
        {&buffer_storage.semantic_colors[sensor], &points_storage.semantics_colors.at(sensor)},
        {&buffer_storage.track_specific_color[sensor], &track_unique_colors}};

    for (auto &b_c : buffer_color) {
      b_c.first->Upload(b_c.second->data(), static_cast<GLsizeiptr>(size * sizeof(unsigned char) * 3), 0);
    }
  }
}

void show(const PointsStorage &points_storage, BufferStorage &buffer_storage, const OutputParameters &output_parameters,
          float point_size) {
  for (const auto &[sensor, points] : points_storage.points) {
    pangolin::GlBuffer *color_buffer;
    if (output_parameters.colored_view) {
      if (output_parameters.show_semantics_colors)
        color_buffer = &buffer_storage.semantic_colors[sensor];
      else
        color_buffer = &buffer_storage.colors[sensor];
    } else {
      if (output_parameters.track_based_colors) {
        color_buffer = &buffer_storage.track_specific_color[sensor];
      } else {
        color_buffer = &buffer_storage.default_colors[sensor];
      }
    }

    auto &vertex_buffer = buffer_storage.points[sensor];

    glPointSize(point_size);
    color_buffer->Bind();
    glColorPointer(static_cast<int>(color_buffer->count_per_element), color_buffer->datatype, 0, nullptr);
    glEnableClientState(GL_COLOR_ARRAY);

    vertex_buffer.Bind();
    glVertexPointer(static_cast<int>(vertex_buffer.count_per_element), vertex_buffer.datatype, 0, nullptr);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, static_cast<int>(points.size()));

    glDisableClientState(GL_VERTEX_ARRAY);
    vertex_buffer.Unbind();

    glDisableClientState(GL_COLOR_ARRAY);
    color_buffer->Unbind();
  }
}

void initializeVisiblePointsStorage(const PointsStorage &points_storage, PointsStorage &visible_points_storage,
                                    Precision idepth_variance_threshold, Precision relative_baseline_threshold) {
  for (const auto &[sensor, points] : points_storage.points) {
    visible_points_storage.points.insert({sensor, std::vector<Eigen::Vector<Precision, 3>>()});
    visible_points_storage.colors.insert({sensor, std::vector<Eigen::Vector3<uint8_t>>()});
    visible_points_storage.default_colors.insert({sensor, std::vector<Eigen::Vector3<uint8_t>>()});
    visible_points_storage.semantics_colors.insert({sensor, std::vector<Eigen::Vector3<uint8_t>>()});

    for (size_t i = 0; i < points.size(); ++i) {
      Precision idepth_variance = points_storage.idepth_variances.at(sensor)[i];
      Precision relative_baseline = points_storage.relative_baselines.at(sensor)[i];
      if (idepth_variance < idepth_variance_threshold && relative_baseline > relative_baseline_threshold) {
        visible_points_storage.points.at(sensor).push_back(points[i]);
        visible_points_storage.colors.at(sensor).push_back(points_storage.colors.at(sensor)[i]);
        visible_points_storage.default_colors.at(sensor).push_back(points_storage.default_colors.at(sensor)[i]);
        visible_points_storage.semantics_colors.at(sensor).push_back(points_storage.semantics_colors.at(sensor)[i]);
      }
    }
  }
}

/**
 * draw point cloud of the frame in the 3d space
 * @param[out] frame
 * @param transformation agent position matrix
 * @param output_parameters parameters for the visualization
 * @param track_unique_color unique (for each track) color
 */
void drawPointCloud(LocalOdometryFrame &frame, const Eigen::Matrix4d &transformation,
                    const OutputParameters &output_parameters, const Eigen::Vector3<uint8_t> &track_unique_color) {
  const float kPointSize = 1;
  const int kOutlierSize = 10;

  glPushMatrix();
  glMultMatrixd(transformation.data());
  if (not frame.are_buffers_initialized) {
    for (const auto &[type, points_storage] : frame.frame_points_storage) {
      initialize(points_storage, frame.frame_buffer_storage[type], track_unique_color);
    }
    frame.are_buffers_initialized = true;
  }

  upload(frame.frame_points_storage["immature"], frame.frame_buffer_storage["immature"], track_unique_color);
  std::map<std::string, PointsStorage> visible_frame_points_storage;
  for (const auto &[type, points_storage] : frame.frame_points_storage) {
    if (type == "immature") continue;
    if (output_parameters.apply_filters) {
      visible_frame_points_storage.insert({type, PointsStorage()});
      initializeVisiblePointsStorage(points_storage, visible_frame_points_storage[type],
                                     static_cast<Precision>(output_parameters.idepth_variance_threshold),
                                     static_cast<Precision>(output_parameters.relative_baseline_threshold));
    } else {
      visible_frame_points_storage.insert({type, frame.frame_points_storage[type]});
    }
    upload(visible_frame_points_storage[type], frame.frame_buffer_storage[type], track_unique_color);
  }

  if (output_parameters.show_immature_landmarks) {
    show(frame.frame_points_storage["immature"], frame.frame_buffer_storage["immature"], output_parameters, kPointSize);
  }

  if (output_parameters.show_active_landmarks) {
    show(visible_frame_points_storage["active"], frame.frame_buffer_storage["active"], output_parameters, kPointSize);
  }

  if (output_parameters.show_marginalized_landmarks) {
    show(visible_frame_points_storage["marginalized"], frame.frame_buffer_storage["marginalized"], output_parameters,
         kPointSize);
  }

  if (output_parameters.show_outliers) {
    show(visible_frame_points_storage["outlier"], frame.frame_buffer_storage["outlier"], output_parameters,
         kOutlierSize);
  }

  glPopMatrix();
}

void removeFrames(LocalOdometryTrack &track) {
  auto frame_it = track.frames.begin();
  while (frame_it != track.frames.end()) {
    if (frame_it->should_be_removed) {
      frame_it = track.frames.erase(frame_it);
    } else {
      ++frame_it;
    }
  }
}

void drawFrame(const LocalFrame &frame, const Eigen::Vector4d &base_color, const double size_factor) {
  drawCamera(frame.t_w_agent.matrix().cast<double>(), base_color, size_factor);
  if (frame.debug_camera) {
    const Eigen::Vector4d rs_camera_color(0.5, 0, 1, 0.5);
    drawCamera(frame.debug_camera->matrix().cast<double>(), rs_camera_color, size_factor);
  }
}

void drawOdometryTrack(LocalOdometryTrack &track, const OutputParameters &output_parameters,
                       const Eigen::Vector3<uint8_t> &track_unique_color) {
  for (auto &frame : track.frames) {
    if (output_parameters.show_camera) {
      for (const auto &attached_frame : frame.attached_frames) {
        drawFrame(attached_frame, attached_frame.color, output_parameters.camera_size);
      }
      drawFrame(frame, frame.color, output_parameters.camera_size);
    }
    drawPointCloud(frame, frame.t_w_agent.matrix().template cast<double>(), output_parameters, track_unique_color);
  }
}

void drawTrack(LocalOdometryTrack &odometry_track, const OutputParameters &output_parameters,
               const Eigen::Vector3<uint8_t> &track_unique_color) {
  if (output_parameters.show_odometry_track) {
    drawOdometryTrack(odometry_track, output_parameters, track_unique_color);
  }
}

}  // namespace

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
void LocalTrack::fromTrack(const track::TrackBase<OdometryTrackType, Motion> &track) {
  const std::lock_guard lock(mutex_);
  odometry_track_ = track.odometryTrack();
  track_id_ = &track;
}

LocalTrack::LocalTrack(const OutputParameters &output_parameters)
    : output_parameters_(output_parameters),
      track_id_(nullptr),
      unique_point_color_(Eigen::Vector3<uint8_t>::Constant(200)) {}

const void *LocalTrack::trackId() const {
  const std::lock_guard lock(mutex_);
  return track_id_;
}
const Eigen::Vector3d LocalTrack::getCameraPosition(size_t frame_id) const {
  const std::lock_guard lock(mutex_);
  auto it = odometry_track_.frames.begin();
  std::advance(it, frame_id);
  return it->t_w_agent.translation();
}
void LocalTrack::render() {
  const std::lock_guard lock(mutex_);
  if (track_id_ == nullptr) {
    return;
  }
  removeFrames(odometry_track_);

  drawTrack(odometry_track_, output_parameters_, unique_point_color_);
}

void LocalTrack::setParameters(const OutputParameters &output_parameters) { output_parameters_ = output_parameters; }

void LocalTrack::setUniqueColor(const Eigen::Vector3<uint8_t> &unique_color) { unique_point_color_ = unique_color; }

template <typename OdometryTrack>
void LocalTrack::updateActiveOdometryFrames(const OdometryTrack &odometry_track,
                                            const std::vector<size_t> &active_frames_indicies,
                                            const std::vector<size_t> &current_active_frames_indicies) {
  const std::lock_guard lock(mutex_);
  auto local_frames_it = odometry_track_.frames.begin();
  size_t frame_index = 0;
  size_t i;
  for (i = 0; i < active_frames_indicies.size(); ++i) {
    while (local_frames_it != odometry_track_.frames.end() and
           (local_frames_it->should_be_removed or frame_index != active_frames_indicies[i])) {
      if (not local_frames_it->should_be_removed) {
        ++frame_index;
      }
      ++local_frames_it;
    }

    if (local_frames_it == odometry_track_.frames.end()) {
      break;
    }

    local_frames_it->should_be_removed = true;
    ++local_frames_it;
    local_frames_it =
        odometry_track_.frames.emplace(local_frames_it, *odometry_track.keyframes()[active_frames_indicies[i]]);
  }

  for (size_t j = i; j < current_active_frames_indicies.size(); ++j) {
    odometry_track_.frames.push_back(*odometry_track.keyframes()[current_active_frames_indicies[j]]);
  }
}
#ifndef BUILDING_DOCS
template void LocalTrack::fromTrack(
    const track::TrackBase<track::ActiveOdometryTrack, energy::motion::SE3<Precision>> &);
template void LocalTrack::fromTrack(const track::TrackBase<track::OdometryTrack, energy::motion::SE3<Precision>> &);

template void LocalTrack::updateActiveOdometryFrames(const track::ActiveOdometryTrack<energy::motion::SE3<Precision>> &,
                                                     const std::vector<size_t> &, const std::vector<size_t> &);
template void LocalTrack::updateActiveOdometryFrames(const track::OdometryTrack<energy::motion::SE3<Precision>> &,
                                                     const std::vector<size_t> &, const std::vector<size_t> &);

#endif /* BUILDING_DOCS */
}  // namespace output
}  // namespace dsopp
