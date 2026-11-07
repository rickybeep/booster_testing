#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace booster_policy {

// Reject a clearly failed terminal pose, not motion needed for balancing.
// At the model's terminal state WALK must take over immediately: waiting for
// motionless ankles on the SIT policy prevented balancing on EARL and TEAR.
// The push-up itself is intentionally non-upright and is not assessed here.
class StandGuard {
 public:
  void Reset() { stable_s_ = 0.0F; fault_.clear(); }
  static bool CanBalance(float upright, float gyro, float leg_speed, float leg_error) {
    return std::isfinite(upright) && std::isfinite(gyro) && std::isfinite(leg_speed)
           && std::isfinite(leg_error) && upright >= 0.5F;
  }
  bool Update(bool requested_sit, bool terminal, float dt, float upright,
              float gyro, float leg_speed, float leg_error, bool fresh = true) {
    if (!fault_.empty()) return false;  // never retry a failed stand automatically
    if (requested_sit || !terminal) {
      stable_s_ = 0.0F;
      return false;
    }
    const bool finite = std::isfinite(upright) && std::isfinite(gyro)
                        && std::isfinite(leg_speed) && std::isfinite(leg_error);
    if (!finite || upright < 0.5F) {
      fault_ = "Stand trajectory ended without measured upright posture";
      stable_s_ = 0.0F;
      return false;
    }
    // Retain the trace field for compatibility; it is no longer a dwell gate.
    stable_s_ = fresh ? stable_s_ + dt : 0.0F;
    return fresh;
  }
  float stable_s() const { return stable_s_; }
  const std::string& fault() const { return fault_; }
 private:
  float stable_s_ = 0.0F;
  std::string fault_;
};
}  // namespace booster_policy
