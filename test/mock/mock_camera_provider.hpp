
#ifndef DSOPP_MOCK_CAMERA_PROVIDER_HPP
#define DSOPP_MOCK_CAMERA_PROVIDER_HPP

#include <gmock/gmock.h>

#include "sensors/camera_providers/image_folder_provider.hpp"

namespace dsopp {
namespace sensors {
namespace providers {
class MockCameraProvider : public CameraProvider {
 public:
  /*
   * workaround to return unique ptr with google mock
   * see https://stackoverflow.com/questions/7616475/can-google-mock-a-method-with-a-smart-pointer-return-type
   * for detatails
   */
  virtual std::unique_ptr<CameraDataFrame> nextFrame() { return std::unique_ptr<CameraDataFrame>(nextFrameProxy()); }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
  MOCK_METHOD(CameraDataFrame *, nextFrameProxy, ());
  MOCK_METHOD(size_t, queueSize, ());
#pragma GCC diagnostic pop
};
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_MOCK_CAMERA_PROVIDER_HPP
