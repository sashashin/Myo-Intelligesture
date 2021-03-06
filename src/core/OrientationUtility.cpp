#include "OrientationUtility.h"

namespace core {
namespace OrientationUtility {
float QuaternionToRoll(const myo::Quaternion<float>& quat) {
  return std::atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                    1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
}

float QuaternionToPitch(const myo::Quaternion<float>& quat) {
  return std::asin(std::max(
      -1.0f,
      std::min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
}

float QuaternionToYaw(const myo::Quaternion<float>& quat) {
  return std::atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
}

float RelativeOrientation(float start, float end) {
  float diff = end - start;
  if (diff > M_PI) {
    diff -= (2 * M_PI);
  }
  if (diff < -M_PI) {
    diff += (2 * M_PI);
  }
  return diff;
}

float RelativeOrientation(
    const myo::Quaternion<float>& start, const myo::Quaternion<float>& end,
    const std::function<float(const myo::Quaternion<float>&)>&
        QuaternionConversion) {
  float orientation_start = QuaternionConversion(start);
  float orientation_end = QuaternionConversion(end);
  return RelativeOrientation(orientation_start, orientation_end);
}
}
}
