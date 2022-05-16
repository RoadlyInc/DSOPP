
#ifndef DSOPP_CONFIG_LOADER_HPP
#define DSOPP_CONFIG_LOADER_HPP

#include <map>
#include <memory>
#include <string>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
template <energy::motion::Motion Motion>
class DSOPP;
/**
 * \brief ConfigLoader reads application configuration from the provided configuration file.
 *
 * ConfigLoader parses yaml file and creates SLAM system with a required configuration.
 * */
class ConfigLoader {
 public:
  /**
   * creates loader from the specified yaml file
   * @param file path to yaml config file
   */
  explicit ConfigLoader(const std::string &file);

  /**
   * method to access loaded agent.
   * @param config_args argument to replace values from the config file
   * @return loaded application or nullptr on failure
   */
  template <energy::motion::Motion Motion, bool TRANSFORM_TO_PINHOLE>
  std::unique_ptr<DSOPP<Motion>> application(const std::map<std::string, std::string> &config_args = {});

 private:
  /** path to yaml configuration file*/
  const std::string &file_;
};
}  // namespace dsopp

#endif  // DSOPP_CONFIG_LOADER_HPP
