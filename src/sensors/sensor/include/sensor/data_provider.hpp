
#ifndef DSOPP_DATA_PROVIDER_HPP
#define DSOPP_DATA_PROVIDER_HPP

#include <memory>
namespace dsopp {
namespace sensors {
namespace providers {

/**
 * \brief DataProvider interface.
 *
 * DataProvider is an interface that supply information to a sensor. For example, Camera can receive data both from
 * image folder or live camera. That could be achieved by implementing corresponding DataProviders for a Camera. Every
 * data provider should implement this interface.
 */
class DataProvider {
 public:
  virtual ~DataProvider() = default;
};
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_DATA_PROVIDER_HPP
