#include "DeviceListenerWrapper.h"

namespace core {
void DeviceListenerWrapper::addChildFeature(child_feature_t feature) {
  child_features_.insert(feature);
}

void DeviceListenerWrapper::removeChildFeature(child_feature_t feature) {
  child_features_.erase(feature);
}

void DeviceListenerWrapper::onPair(myo::Myo* myo, uint64_t timestamp,
                      myo::FirmwareVersion firmware_version) {
  for (auto feature : child_features_) {
    feature->onPair(myo, timestamp, firmware_version);
  }
}

void DeviceListenerWrapper::onUnpair(myo::Myo* myo, uint64_t timestamp) {
  for (auto feature : child_features_) {
    feature->onUnpair(myo, timestamp);
  }
}

void DeviceListenerWrapper::onConnect(myo::Myo* myo, uint64_t timestamp,
                       myo::FirmwareVersion firmware_version) {
  for (auto feature : child_features_) {
    feature->onConnect(myo, timestamp, firmware_version);
  }
}

void DeviceListenerWrapper::onDisconnect(myo::Myo* myo, uint64_t timestamp) {
  for (auto feature : child_features_) {
    feature->onDisconnect(myo, timestamp);
  }
}

void DeviceListenerWrapper::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm,
                       myo::XDirection x_direction) {
  for (auto feature : child_features_) {
    feature->onArmSync(myo, timestamp, arm, x_direction);
  }
}

void DeviceListenerWrapper::onArmUnsync(myo::Myo* myo, uint64_t timestamp) {
  for (auto feature : child_features_) {
    feature->onArmUnsync(myo, timestamp);
  }
}

void DeviceListenerWrapper::onUnlock(myo::Myo* myo, uint64_t timestamp) {
  for (auto feature : child_features_) {
    feature->onUnlock(myo, timestamp);
  }
}

void DeviceListenerWrapper::onLock(myo::Myo* myo, uint64_t timestamp) {
  for (auto feature : child_features_) {
    feature->onLock(myo, timestamp);
  }
}

void DeviceListenerWrapper::onPose(myo::Myo* myo, uint64_t timestamp,
                    const std::shared_ptr<Pose>& pose) {
  for (auto feature : child_features_) {
    feature->onPose(myo, timestamp, pose);
  }
}

void DeviceListenerWrapper::onGesture(myo::Myo* myo, uint64_t timestamp,
                       const std::shared_ptr<Gesture>& gesture) {
  for (auto feature : child_features_) {
    feature->onGesture(myo, timestamp, gesture);
  }
}

void DeviceListenerWrapper::onOrientationData(myo::Myo* myo, uint64_t timestamp,
                               const myo::Quaternion<float>& rotation) {
  for (auto feature : child_features_) {
    feature->onOrientationData(myo, timestamp, rotation);
  }
}

void DeviceListenerWrapper::onAccelerometerData(myo::Myo* myo, uint64_t timestamp,
                                 const myo::Vector3<float>& acceleration) {
  for (auto feature : child_features_) {
    feature->onAccelerometerData(myo, timestamp, acceleration);
  }
}

void DeviceListenerWrapper::onGyroscopeData(myo::Myo* myo, uint64_t timestamp,
                             const myo::Vector3<float>& gyro) {
  for (auto feature : child_features_) {
    feature->onGyroscopeData(myo, timestamp, gyro);
  }
}

void DeviceListenerWrapper::onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
  for (auto feature : child_features_) {
    feature->onRssi(myo, timestamp, rssi);
  }
}

void DeviceListenerWrapper::onEmgData(myo::Myo* myo, uint64_t timestamp,
                       const int8_t* emg) {
  for (auto feature : child_features_) {
    feature->onEmgData(myo, timestamp, emg);
  }
}

void DeviceListenerWrapper::onPeriodic(myo::Myo* myo) {
  for (auto feature : child_features_) {
    feature->onPeriodic(myo);
  }
}
}
