#include "test/tools/depth_gt.hpp"

#include <cnpy.h>
#include <glog/logging.h>
#include <fstream>
#include <list>

#include "common/file_tools/parsing.hpp"

namespace dsopp {
namespace test_tools {
namespace {
/**
 * function to new .npy file if it necessary
 *
 * @param path path to the directory with depth files
 * @param n number of the frame which has to be in the .npy file
 * @param depths[out] containers with depth frames
 * @return frame with a closest timestamp to the queried timestamp
 */
std::shared_ptr<DepthGt::Frame> readFrame(const std::string &path, const size_t n) {
  const size_t kDepthImagesInFile = 20;
  const size_t kMaxDistanceInCarla = 1000;
  size_t file_num = n / kDepthImagesInFile;
  auto file_name = path + "/" + std::to_string(file_num) + ".npy";
  try {
    cnpy::NpyArray arr = cnpy::npy_load(file_name);
    auto data = arr.as_vec<float>();
    size_t height = arr.shape[1];
    size_t width = arr.shape[2];
    size_t rel_pos = n - (n / kDepthImagesInFile) * kDepthImagesInFile;
    auto frame = std::make_shared<DepthGt::Frame>();
    frame->resize(width);
    for (size_t xi = 0; xi < width; xi++) {
      (*frame)[xi].resize(height);
      for (size_t yi = 0; yi < height; yi++) {
        (*frame)[xi][yi] = data[rel_pos * width * height + yi * width + xi] * kMaxDistanceInCarla;
      }
    }
    return frame;
  } catch (const std::exception &e) {
    LOG(ERROR) << "Can't load " << file_name;
    return nullptr;
  }
}
}  // namespace
DepthGt::DepthGt(const std::string &path, const std::string &timestamps_file) : depth_path_(path) {
  auto stream = std::ifstream(timestamps_file);
  if (!stream.is_open()) {
    LOG(WARNING) << "File " << timestamps_file << " does not exist!";
    return;
  }
  uint32_t iterator = 0, frame_id;
  std::string ts;

  while (stream >> frame_id >> ts) {
    time timestamp = common::file_tools::stringToTime(ts);
    data_[timestamp] = iterator++;
  }
  stream.close();
}
std::shared_ptr<DepthGt::Frame> DepthGt::getFrame(time ts) {
  auto n = getData(ts);
  if (!n) {
    LOG(ERROR) << "Depth for " << ts << " was not found";
    return nullptr;
  }
  if (depths_.count(*n) == 0) {
    auto frame = readFrame(depth_path_, *n);
    depths_[*n] = frame;
    return frame;
  }
  if (auto frame = depths_.at(*n).lock()) {
    return frame;
  } else {
    frame = readFrame(depth_path_, *n);
    depths_[*n] = frame;
    return frame;
  }
}
}  // namespace test_tools
}  // namespace dsopp
