
#include "features/camera/pixel_map.hpp"

#include <glog/logging.h>

namespace dsopp {
namespace features {

template <int C>
PixelMap<C>::PixelMap(std::vector<Precision> &&data, long width, long height) : plain_data_(std::move(data)) {
  CHECK_EQ(plain_data_.size(), width * height * C) << "Incorrect size of vector of data";

  using Channel = Eigen::Map<Eigen::Matrix<Precision, -1, -1, Eigen::RowMajor>>;

  map_.resize(height, width);

  for (int i = 0; i < C; ++i) {
    Channel channel(&plain_data_[static_cast<size_t>(i * width * height)], height, width);

    for (long y = 0; y < height; ++y)
      for (long x = 0; x < width; ++x) {
        Precision c = channel(y, x);
        map_(y, x).data_(i, 0) = c;

        Precision dx;
        if (x == 0) {
          dx = channel(y, x + 1) - channel(y, x);
        } else if (x == width - 1) {
          dx = channel(y, x) - channel(y, x - 1);
        } else {
          dx = 0.5_p * (channel(y, x + 1) - channel(y, x - 1));
        }
        map_(y, x).data_(i, 1) = dx;

        Precision dy;
        if (y == 0) {
          dy = channel(y + 1, x) - channel(y, x);
        } else if (y == height - 1) {
          dy = channel(y, x) - channel(y - 1, x);
        } else {
          dy = 0.5_p * (channel(y + 1, x) - channel(y - 1, x));
        }
        map_(y, x).data_(i, 2) = dy;
      }
  }
}

template <int C>
PixelMap<C>::PixelMap() = default;

template <int C>
PixelMap<C>::PixelMap(PixelMap &&) = default;

template <int C>
PixelMap<C> &PixelMap<C>::operator=(PixelMap &&) = default;

template <int C>
PixelMap<C>::PixelMap(const PixelMap &) = default;

template <int C>
PixelMap<C> PixelMap<C>::clone() const {
  return *this;
}

template <int C>
long PixelMap<C>::width() const {
  return map_.cols();
}
template <int C>
long PixelMap<C>::height() const {
  return map_.rows();
}
template <int C>
int PixelMap<C>::channels() const {
  return C;
}
template <int C>
size_t PixelMap<C>::size() const {
  return plain_data_.size();
}

template <int C>
const std::vector<Precision> &PixelMap<C>::data() const {
  return plain_data_;
}
template <int C>
const PixelInfo<C> &PixelMap<C>::operator()(int x, int y) const {
  return map_(y, x);
}

template <int C>
PixelInfo<C> &PixelMap<C>::operator()(size_t index) {
  return map_.data()[index];
}

template <int C>
const PixelInfo<C> &PixelMap<C>::operator()(size_t index) const {
  return map_.data()[index];
}

template class PixelMap<1>;
template class PixelMap<3>;
template class PixelMap<8>;
template class PixelMap<16>;
template class PixelMap<32>;
template class PixelMap<128>;

}  // namespace features
}  // namespace dsopp
