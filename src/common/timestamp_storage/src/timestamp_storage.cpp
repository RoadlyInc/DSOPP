#include "common/timestamp_storage/timestamp_storage.hpp"

#include <optional>
#include <vector>

#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp::common::timestamp_storage {
template <class T>
TimestampStorage<T>::TimestampStorage(std::map<time, T> &&data) : data_(std::move(data)) {}

template <class T>
std::optional<T> TimestampStorage<T>::getData(time ts) const {
  auto found = data_.find(ts);
  if (found == data_.end()) {
    return std::nullopt;
  }
  return found->second;
}

template <class T>
void TimestampStorage<T>::pushData(time ts, const T &d) {
  data_.emplace(ts, d);
}

template class TimestampStorage<Sophus::SE3<Precision>>;
template class TimestampStorage<energy::motion::SE3<Precision>>;
template class TimestampStorage<size_t>;

}  // namespace dsopp::common::timestamp_storage
