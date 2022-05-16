#include "energy/motion/motion.hpp"

#include "energy/motion/se3_motion.hpp"

namespace dsopp::energy::motion {
static_assert(Motion<SE3<double>>);
// static_assert(Motion<UniformRollingShutter<double>>);

static_assert(MotionProduct<SE3<double>::Product>);
// static_assert(MotionProduct<UniformRollingShutter<double>::Product>);

}  // namespace dsopp::energy::motion
