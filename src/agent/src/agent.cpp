
#include "agent/agent.hpp"

#include "sensors/camera/camera.hpp"
#include "sensors/sensors.hpp"

namespace dsopp {
Agent::Agent() : sensors_(std::make_unique<sensors::Sensors>()) {}

sensors::Sensors& Agent::sensors() const { return *sensors_; }
}  // namespace dsopp
