
#ifndef DSOPP_TUM_GT_HPP
#define DSOPP_TUM_GT_HPP

#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "common/timestamp_storage/timestamp_storage.hpp"

namespace dsopp {
namespace test_tools {
/** Class for reading and accesing TUM gt files */
class TumGt final : public common::timestamp_storage::TimestampStorage<Sophus::SE3<Precision>> {
 public:
  /**
   * creates TumGt object from the .tum file
   * @param file file to read
   */
  explicit TumGt(std::string file);
  /**
   * @param ts queried timestamp
   * @return pose (camera to world) with a closest timestamp to the queried timestamp
   */
  const Sophus::SE3<Precision> getPose(time ts) const;
};
}  // namespace test_tools
}  // namespace dsopp

#endif  // DSOPP_TUM_GT_HPP
