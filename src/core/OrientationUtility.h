/* Various utility functions to use with orientation data. The math for
 * quaternion to roll/pitch/yaw was taken from the Myo SDK sample, but all of
 * the math in this namespace should probably be double checked.
 */

#pragma once

/* for MSVC++ */
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <functional>

#include <myo/myo.hpp>

namespace core {
namespace OrientationUtility {
float QuaternionToRoll(const myo::Quaternion<float>& quat);

float QuaternionToPitch(const myo::Quaternion<float>& quat);

float QuaternionToYaw(const myo::Quaternion<float>& quat);

float RelativeOrientation(float start, float end);

float RelativeOrientation(
    const myo::Quaternion<float>& start, const myo::Quaternion<float>& end,
    const std::function<float(const myo::Quaternion<float>&)>&
        QuaternionConversion);
}
}
