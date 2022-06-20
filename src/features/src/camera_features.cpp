#include "features/camera/camera_features.hpp"

#include <glog/logging.h>
#include <tbb/parallel_for.h>

#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_data_frame_extractor.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_features_extractor.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "semantics/semantic_filter.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp {
namespace features {
CameraFeatures::CameraFeatures(const size_t frame_id, cv::Mat&& raw_image, cv::Mat&& photocorrected_undistorted_image,
                               const time time,
                               const std::vector<sensors::calibration::CameraMask>& pyramid_of_static_masks,
                               features::TrackingFeaturesExtractor& tracking_features_extractor,
                               const features::PixelDataFrameExtractor& pixel_data_frame_extractor,
                               std::unique_ptr<cv::Mat>&& semantics_data,
                               const semantics::SemanticFilter* semantic_filter)
    : frame_id_(frame_id),
      raw_image_(std::move(raw_image)),
      frame_data_(std::move(photocorrected_undistorted_image)),
      time_(time),
      pyramid_of_static_masks_(pyramid_of_static_masks),
      tracking_features_extractor_(tracking_features_extractor),
      pixel_data_frame_extractor_(pixel_data_frame_extractor),
      semantics_data_(std::move(semantics_data)),
      semantic_filter_(semantic_filter) {
  cv::cvtColor(raw_image_, raw_frame_data_, cv::COLOR_BGR2GRAY);
}

size_t CameraFeatures::id() const { return frame_id_; }
const features::TrackingFeaturesFrame& CameraFeatures::tracking() {
  if (tracking_features_ == nullptr) {
    tracking_features_ = tracking_features_extractor_.extract(raw_frame_data_, pyramidOfMasks()[0]);
  }
  return *tracking_features_;
}
const features::PixelDataFrame& CameraFeatures::pixelData() {
  if (pixel_data_frame_ == nullptr) {
    pixel_data_frame_ = pixel_data_frame_extractor_.extract(frame_data_);
  }
  return *pixel_data_frame_;
}

const cv::Mat& CameraFeatures::image() const { return raw_image_; }

const cv::Mat& CameraFeatures::frameData() const { return raw_frame_data_; }

std::unique_ptr<cv::Mat> CameraFeatures::moveSemanticsData() { return std::move(semantics_data_); }

std::unique_ptr<features::TrackingFeaturesFrame> CameraFeatures::moveTracking() {
  tracking();
  return std::move(tracking_features_);
}
std::unique_ptr<features::PixelDataFrame> CameraFeatures::movePixelData() {
  pixelData();
  return std::move(pixel_data_frame_);
}

CameraFeatures::PyramidOfMasks CameraFeatures::movePyramidOfMasks() {
  pyramidOfMasks();
  return std::move(pyramid_of_masks_);
}

const std::vector<sensors::calibration::CameraMask>& CameraFeatures::pyramidOfMasks() {
  if (pyramid_of_masks_ == nullptr) {
    pyramid_of_masks_ = std::make_unique<std::vector<sensors::calibration::CameraMask>>(pyramid_of_static_masks_);
    if (semantics_data_ && semantic_filter_ && semantic_filter_->filterBySemantic()) {
      auto finest_mask = (*pyramid_of_masks_)[0].filterSemanticObjects(*semantics_data_, *semantic_filter_);
      tbb::parallel_for(tbb::blocked_range<size_t>(0, pyramid_of_masks_->size()), [&](auto& r) {
        for (size_t lvl = r.begin(); lvl != r.end(); ++lvl) {
          (*pyramid_of_masks_)[lvl] = finest_mask.resize(1._p / static_cast<Precision>(1 << lvl));
        }
      });
    }
  }
  return *pyramid_of_masks_;
}

time CameraFeatures::timestamp() const { return time_; }
CameraFeatures::~CameraFeatures() = default;
}  // namespace features
}  // namespace dsopp
