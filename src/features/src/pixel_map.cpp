
#include "features/camera/pixel_map.hpp"
#include "features/camera/calculate_pixelinfo.hpp"

#include <mutex>
#include <stack>

#include <glog/logging.h>

namespace dsopp {
namespace features {

/**
 * object pool that caches the underlying data storage for computed `PixelMap`s the pool caches storage containers
 * based on their size to avoid performing resizes on them
 */
template <int C>
class PixelInfoPool {
  /** type of the container used to store PixelInfo data */
  using PixelInfoContainer = std::vector<PixelInfo<C>, typename PixelInfo<C>::PixelInfoAllocator>;

 private:
  /** custom deleter for shared pointers, returns the data storage container back into its original pool */
  struct ExternalDeleter {
    /**
     * creates a custom deleter object
     * @param pool reference to the pool that the deleted container should be placed into
     */
    explicit ExternalDeleter(std::weak_ptr<PixelInfoPool *> pool) : pool_(pool) {}

    /**
     * called after the last reference to a shared_ptr dies
     * @param ptr pointer to the container instance that the shared_ptr managed
     */
    void operator()(PixelInfoContainer *ptr) {
      if (auto pool_ptr = pool_.lock()) {
        try {
          (*pool_ptr.get())->add(std::unique_ptr<PixelInfoContainer>{ptr}, ptr->capacity());
          return;
        } catch (...) {
        }
      }
      std::default_delete<PixelInfoContainer>{}(ptr);
    }

   private:
    /** reference to the pool that created this deleter */
    std::weak_ptr<PixelInfoPool *> pool_;
  };

 public:
  PixelInfoPool() : this_ptr_(new PixelInfoPool *(this)) {}
  virtual ~PixelInfoPool() {}

  /**
   * add the given storage container into the pool with a given size
   * @param t container to be placed into the pool
   * @param container_size number of allocated elements in the given container
   */
  void add(std::unique_ptr<PixelInfoContainer> t, const size_t container_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    pools_[container_size].push(std::move(t));
  }

  /**
   * get a storage container of a desired size from the pool
   * @param desired_size number of elements the container should have allocated
   * @return a shared pointer to a container of requested size
   */
  std::shared_ptr<PixelInfoContainer> acquire(const size_t desired_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::stack<std::unique_ptr<PixelInfoContainer>> &pool = pools_[desired_size];
    if (pool.empty()) {
      // if no unused containers are available, create and store a new one
      std::unique_ptr<PixelInfoContainer> t(new PixelInfoContainer());
      t->resize(desired_size);
      pool.push(std::move(t));
    }

    // create a shared_ptr with a custom deleter, that will automatically return the container back to this pool
    std::shared_ptr<PixelInfoContainer> tmp(pool.top().release(),
                                            ExternalDeleter{std::weak_ptr<PixelInfoPool *>{this_ptr_}});
    pool.pop();
    return tmp;
  }

 private:
  /** mutex to share the pool between threads */
  std::mutex mutex_;
  /** pointer to this pool, used to create shared_ptr deleters */
  std::shared_ptr<PixelInfoPool *> this_ptr_;
  /** this map stores multiple object pools according to the capacity of the containers for each of them */
  std::unordered_map<size_t, std::stack<std::unique_ptr<PixelInfoContainer>>> pools_;
};

template <int C>
PixelMap<C>::PixelMap(std::vector<Precision, PrecisionAllocator> &&data, long width, long height)
    : plain_data_(std::move(data)) {
  CHECK_EQ(plain_data_.size(), width * height * C) << "Incorrect size of vector of data";

  // note: initialization of a static local variable is supposed to be thread-safe according to
  // https://en.cppreference.com/w/cpp/language/storage_duration#Static_local_variables
  static PixelInfoPool<C> pool_;

  pixelinfo_data_ = pool_.acquire(static_cast<size_t>(height * width));
  new (&map_) PixelInfoStorage(pixelinfo_data_->data(), height, width);

  calculate_pixelinfo<C>(plain_data_.data(), static_cast<Precision *>(static_cast<void *>(map_.data())),
                         static_cast<int>(width), static_cast<int>(height));
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
const std::vector<Precision, PrecisionAllocator> &PixelMap<C>::data() const {
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
