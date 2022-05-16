#ifndef DSOPP_MARGINALIZATION_STRATEGY_FABRIC_HPP
#define DSOPP_MARGINALIZATION_STRATEGY_FABRIC_HPP

#include "marginalization/frame_marginalization_strategy.hpp"

#include <any>
#include <cstddef>
#include <map>
#include <memory>
#include <string>

namespace dsopp {
namespace marginalization {

template <energy::motion::Motion Motion>
std::unique_ptr<FrameMarginalizationStrategy<Motion>> create(const std::map<std::string, std::any> &parameters);

}  // namespace marginalization
}  // namespace dsopp

#endif  // DSOPP_MARGINALIZATION_STRATEGY_FABRIC_HPP
