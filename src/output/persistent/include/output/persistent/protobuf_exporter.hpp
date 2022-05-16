#ifndef DSOPP_PROTOBUF_EXPORTER_HPP
#define DSOPP_PROTOBUF_EXPORTER_HPP
#include "output_interfaces/track_output_interface.hpp"
#include "storage/track_storage.hpp"

#include <string>

namespace dsopp {
namespace output {
/**
 * saves results to a binary file
 * @tparam OdometryTrackType type of a odometry track
 */
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class ProtobufExporter : public TrackOutputInterface<OdometryTrackType, Motion> {
 public:
  /**
   * @param file path to a file to be saved
   * @param save_stride number of keyframes between saving. When 0 saves only in the end.
   */
  ProtobufExporter(const std::string &file, size_t save_stride = 0);

  void notify(const track::TrackBase<OdometryTrackType, Motion> &track) override;
  void finish(const track::TrackBase<OdometryTrackType, Motion> &track) override;

  ~ProtobufExporter() override;

 private:
  /**  path to a file to be saved */
  const std::string file_;
  /**  number of tick between savings */
  const size_t save_stride_;
  /** caching lates proto message */
  track::storage::Track track_storage_;
  /** keyframe count on the last save */
  size_t current_keyframes_;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_PROTOBUF_EXPORTER_HPP
