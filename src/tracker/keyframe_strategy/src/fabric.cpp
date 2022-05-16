#include "tracker/keyframe_strategy/fabric.hpp"
#include <glog/logging.h>
#include "common/settings.hpp"

#include "tracker/keyframe_strategy/frequency_keyframe_strategy.hpp"
#include "tracker/keyframe_strategy/mean_square_optical_flow_and_rmse_keyframe_strategy.hpp"
#include "tracker/keyframe_strategy/tracker_keyframe_strategy.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {

template <energy::motion::Motion Motion>
std::unique_ptr<TrackerKeyframeStrategy<Motion>> createTrackerKeyframeStrategy(
    const std::map<std::string, std::any> &parameters) {
  if (parameters.count("strategy") == 0) {
    LOG(WARNING)
        << "Missing field \"strategy\" in the keyframe strategy parameters. Keyframe strategy was not created.";
    return nullptr;
  }
  const auto &keyframe_strategy_type = std::any_cast<std::string>(parameters.at("strategy"));
  if (keyframe_strategy_type == "frequency") {
    if (parameters.count("frequency") == 0) {
      LOG(WARNING)
          << "Missing field \"frequency\" in the keyframe strategy parameters. Keyframe strategy was not created.";
      return nullptr;
    }
    double frequency = std::stod(std::any_cast<std::string>(parameters.at("frequency")));
    if (frequency <= 0) {
      LOG(WARNING) << "Frequency of keyframes must be above zero. Keyframe strategy was not created.";
      return nullptr;
    }
    return std::make_unique<FrequencyKeyframeStrategy<Motion>>(frequency);
  } else if (keyframe_strategy_type == "mean_square_optical_flow") {
    if (parameters.count("factor") == 0) {
      LOG(WARNING)
          << "Missing field \"factor\" in the keyframe strategy parameters. Keyframe strategy was not created.";
      return nullptr;
    }
    double factor = std::stod(std::any_cast<std::string>(parameters.at("factor")));
    if (factor <= 0) {
      LOG(WARNING) << "Factor must be above zero. Keyframe strategy was not created.";
      return nullptr;
    }
    return std::make_unique<MeanSquareOpticalFlowAndRmseKeyframeStrategy<Motion>>(factor);
  } else {
    LOG(WARNING) << "Undefined keyframe strategy";
    return nullptr;
  }
  return nullptr;
}

template std::unique_ptr<TrackerKeyframeStrategy<energy::motion::SE3<Precision>>> createTrackerKeyframeStrategy(
    const std::map<std::string, std::any> &parameters);
}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp
