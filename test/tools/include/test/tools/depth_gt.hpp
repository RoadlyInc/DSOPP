#ifndef DSOPP_TEST_TOOLS_DEPTH_GT_HPP_
#define DSOPP_TEST_TOOLS_DEPTH_GT_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "common/timestamp_storage/timestamp_storage.hpp"

namespace dsopp {
namespace test_tools {
/** Class for reading and accessing depth gt files */
class DepthGt final : public common::timestamp_storage::TimestampStorage<size_t> {
 public:
  /** shortcut for frame*/
  using Frame = std::vector<std::vector<Precision>>;
  /**
   * creates DepthGt object from the .npy files
   * @param path path to depth files
   * @param timestamps_file timestamp file path
   *
   */
  explicit DepthGt(const std::string& path, const std::string& timestamps_file);
  /**
   * @param ts queried timestamp
   * @return depth frame with a closest timestamp to the queried timestamp
   */
  std::shared_ptr<Frame> getFrame(time ts);

 private:
  std::string depth_path_;
  std::map<size_t, std::weak_ptr<Frame>> depths_;
};
}  // namespace test_tools
}  // namespace dsopp

#endif  // DSOPP_TEST_TOOLS_DEPTH_GT_HPP_
