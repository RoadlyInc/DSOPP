#ifndef DSOPP_COMMON_TIMESTAMP_STORAGE_HPP_
#define DSOPP_COMMON_TIMESTAMP_STORAGE_HPP_

#include <cstdint>
#include <map>
#include <optional>

#include "common/time/time.hpp"

namespace dsopp::common::timestamp_storage {

/** \brief an interface for storing and accessing data through nearest timestamp
 *
 * @tparam T type of the data
 */
template <typename T>
class TimestampStorage {
 public:
  TimestampStorage() = default;
  /**
   * @param data data
   */
  TimestampStorage(std::map<time, T> &&data);

  /**
   * @param ts time
   * @return data with timestamp
   */
  std::optional<T> getData(time ts) const;

  /**
   * @param ts timestamp
   * @param d data
   */
  void pushData(time ts, const T &d);

 protected:
  /** some data with timestamps */
  std::map<time, T> data_;
};

}  // namespace dsopp::common::timestamp_storage

#endif  // DSOPP_COMMON_TIMESTAMP_STORAGE_HPP_
