/* DeviceListenerWrapper wraps the myo::DeviceListener class in order to add the
 * virtual function onPeriodic(myo::Myo*) in order to allow the derived classes
 * to call Base::onPeriodic.
 */

#pragma once

#include <set>
#include <memory>
#include <myo/myo.hpp>

#include "Pose.h"
#include "Gesture.h"

namespace core {
class DeviceListenerWrapper {
 protected:
  typedef DeviceListenerWrapper* child_feature_t;
  std::set<child_feature_t> child_features_;

 public:
  void addChildFeature(child_feature_t feature);
  void removeChildFeature(child_feature_t feature);

  virtual void onPair(myo::Myo* myo, uint64_t timestamp,
                      myo::FirmwareVersion firmware_version);

  virtual void onUnpair(myo::Myo* myo, uint64_t timestamp);

  virtual void onConnect(myo::Myo* myo, uint64_t timestamp,
                         myo::FirmwareVersion firmware_version);

  virtual void onDisconnect(myo::Myo* myo, uint64_t timestamp);

  virtual void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                         myo::XDirection x_direction,
                         float rotation, myo::WarmupState warmupState);

  virtual void onArmUnsync(myo::Myo* myo, uint64_t timestamp);

  virtual void onUnlock(myo::Myo* myo, uint64_t timestamp);

  virtual void onLock(myo::Myo* myo, uint64_t timestamp);

  virtual void onPose(myo::Myo* myo, uint64_t timestamp,
                      const std::shared_ptr<Pose>& pose);

  virtual void onGesture(myo::Myo* myo, uint64_t timestamp,
                         const std::shared_ptr<Gesture>& gesture);

  virtual void onOrientationData(myo::Myo* myo, uint64_t timestamp,
                                 const myo::Quaternion<float>& rotation);

  virtual void onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                   const myo::Vector3<float>& acceleration);

  virtual void onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                               const myo::Vector3<float>& gyro);

  virtual void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi);

  virtual void onEmgData(myo::Myo* myo, uint64_t timestamp,
                         const int8_t* emg);

  virtual void onPeriodic(myo::Myo* myo);
};
}
