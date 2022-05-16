#ifndef DSOPP_SANITY_CHECK_STATUS_HPP
#define DSOPP_SANITY_CHECK_STATUS_HPP

namespace dsopp::sanity_checker {
/** sanity check status */
enum struct SanityCheckStatus {
  kExceededGravityAngle,            /**< exceeded gravity angle. */
  kExceededGravityAngularVelocity,  /**< exceeded gravity angular velocity. */
  kExceededRotationAngle,           /**< exceeded rotation angle. */
  kExceededRotationAngularVelocity, /**< exceeded rotation angular velocity. */
  kExceededTranslationError,        /**< exceeded translation error. */
};
}  // namespace dsopp::sanity_checker

#endif  // DSOPP_SANITY_CHECK_STATUS_HPP
