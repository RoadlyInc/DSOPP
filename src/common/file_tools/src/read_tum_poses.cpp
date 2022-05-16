#include "common/file_tools/read_tum_poses.hpp"

#include "common/file_tools/parsing.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp::common::file_tools {

template <energy::motion::Motion Motion>
std::map<time, Motion> readTumPoses(std::ifstream &stream) {
  std::map<time, Motion> out;
  std::string ts;
  Precision x, y, z;
  Precision rot_w, rot_x, rot_y, rot_z;
  while (stream >> ts >> x >> y >> z >> rot_x >> rot_y >> rot_z >> rot_w) {
    Sophus::SE3<Precision> T_w_c;
    T_w_c.translation() = Eigen::Vector3<Precision>(x, y, z);
    T_w_c.setQuaternion(Eigen::Quaternion<Precision>(rot_w, rot_x, rot_y, rot_z).normalized());
    time timestamp = stringToTime(ts);
    out.insert({timestamp, Motion(T_w_c)});
  }
  return out;
}

template std::map<time, energy::motion::SE3<Precision>> readTumPoses(std::ifstream &stream);

}  // namespace dsopp::common::file_tools
