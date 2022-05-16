#include "marginalization/fabric.hpp"

#include "marginalization/maximum_size_frame_marginalization_strategy.hpp"
#include "marginalization/sparse_frame_marginalization_strategy.hpp"

#include <glog/logging.h>
namespace dsopp {
namespace marginalization {
namespace {
bool checkField(const std::map<std::string, std::any> &parameters, const std::string &field) {
  if (!parameters.contains(field)) {
    LOG(WARNING) << "Missing field \"" + field + "\" in parameters. Marginalization strategy was not created.";
    return false;
  }
  return true;
}
}  // namespace

template <energy::motion::Motion Motion>
std::unique_ptr<FrameMarginalizationStrategy<Motion>> create(const std::map<std::string, std::any> &parameters) {
  if (!checkField(parameters, "strategy")) {
    return nullptr;
  }
  auto marginalization_strategy_type = std::any_cast<std::string>(parameters.at("strategy"));
  if (marginalization_strategy_type == "maximum_size") {
    if (!checkField(parameters, "maximum_size")) {
      return nullptr;
    }
    size_t maximum_size = std::stoul(std::any_cast<std::string>(parameters.at("maximum_size")));
    return std::make_unique<MaximumSizeFrameMarginalizationStrategy<Motion>>(maximum_size);
  } else if (marginalization_strategy_type == "sparse") {
    if (!checkField(parameters, "minimum_size")) {
      return nullptr;
    }
    if (!checkField(parameters, "maximum_size")) {
      return nullptr;
    }
    if (!checkField(parameters, "maximum_percentage_of_marginalized_points_in_frame")) {
      return nullptr;
    }
    size_t minimum_size = std::stoul(std::any_cast<std::string>(parameters.at("minimum_size")));
    size_t maximum_size = std::stoul(std::any_cast<std::string>(parameters.at("maximum_size")));
    double maximum_number_of_marginalized =
        std::stod(std::any_cast<std::string>(parameters.at("maximum_percentage_of_marginalized_points_in_frame")));
    return std::make_unique<SparseFrameMarginalizationStrategy<Motion>>(minimum_size, maximum_size,
                                                                        maximum_number_of_marginalized);
  }
  LOG(ERROR) << "Undefined marginalization type";
  return nullptr;
}

template std::unique_ptr<FrameMarginalizationStrategy<energy::motion::SE3<Precision>>> create(
    const std::map<std::string, std::any> &parameters);

}  // namespace marginalization
}  // namespace dsopp
