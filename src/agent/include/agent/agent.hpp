
#ifndef DSOPP_AGENT_HPP
#define DSOPP_AGENT_HPP
#include <memory>
#include <vector>
namespace dsopp {
namespace sensors {
class Sensors;
}
/**
 * \brief class to represent the main agent of the system.
 *
 * Agent stores all sensors and constraints on their positions. Agent is the object that we track
 */
class Agent {
 public:
  /**
   * creates an agent from the array of sensors
   *
   */
  explicit Agent();
  /**
   * get the sensors agent have
   *
   * @return  sensors agent have
   */
  sensors::Sensors& sensors() const;

 private:
  /** Sensors that are processed by the application*/
  std::unique_ptr<sensors::Sensors> sensors_;
};
}  // namespace dsopp

#endif  // DSOPP_AGENT_HPP
