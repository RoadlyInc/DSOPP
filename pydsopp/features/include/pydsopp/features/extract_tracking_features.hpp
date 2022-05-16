#ifndef DSOPP_PYDSOPP_FEATURES_EXTRACT_TRACKING_FEATURES_HPP
#define DSOPP_PYDSOPP_FEATURES_EXTRACT_TRACKING_FEATURES_HPP

#include <pybind11/numpy.h>

#include "common/settings.hpp"

namespace dsopp::pydsopp::features {
std::vector<std::pair<dsopp::Precision, dsopp::Precision>> extract_tracking_features(
    const pybind11::array_t<uint8_t> &image_array);
}

#endif  // DSOPP_PYDSOPP_FEATURES_EXTRACT_TRACKING_FEATURES_HPP
