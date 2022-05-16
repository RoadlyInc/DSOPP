#include "track/export/track2tum_exporter.hpp"

#include <fstream>
#include <iomanip>

#include <glog/logging.h>

#include "common/settings.hpp"

#include "track/export/timestamp_output.hpp"
#include "track/export/transformations_output.hpp"
#include "track/frames/tracking_frame.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

namespace dsopp {
namespace test_tools {
template <energy::motion::Motion Motion>
void track2TumExporter(const track::OdometryTrack<Motion> &track, const std::string &file) {
  std::ofstream stream = std::ofstream(file);
  if (!stream.is_open()) {
    LOG(ERROR) << "Can't create TUM file " << file;
    return;
  }

  stream << std::setprecision(20);
  for (const auto &frame : track.keyframes()) {
    auto ts_string = timestampToString(frame->timestamp());
    stream << ts_string << " " << frame->tWorldAgent().se3() << std::endl;
    for (const auto &attached_frame : frame->attachedFrames()) {
      auto attached_ts_string = timestampToString(attached_frame->timestamp());
      stream << attached_ts_string << " " << attached_frame->tWorldAgent().se3() << std::endl;
    }
  }
}
template void track2TumExporter<energy::motion::SE3<Precision>>(
    const track::OdometryTrack<energy::motion::SE3<Precision>> &, const std::string &);

}  // namespace test_tools
}  // namespace dsopp
