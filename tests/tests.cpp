#include <myo/myo.hpp>
#include <string>
#include <array>
#include <memory>
#include <map>

#include "../src/core/DeviceListenerWrapper.h"
#include "../src/features/RootFeature.h"
#include "../src/features/filters/Debounce.h"
#include "../src/features/filters/ExponentialMovingAverage.h"
#include "../src/features/filters/MovingAverage.h"

#include "hub.h"
#include "event_types.h"
#include "event_player_hub.h"
#include "event_recorder.h"

#include "PrintEventsFeature.h"

// Test without MyoSimulator.
void testRootFeature();
void testDebounce();
void testExponentialMovingAverage();
void testMovingAverage();

// Test using MyoSimulator.
void myoSimTestRootFeature(myosim::Hub &hub);
void myoSimTestDebounce(myosim::Hub& hub);
void myoSimTestExponentialMovingAverage(myosim::Hub& hub);
void myoSimTestMovingAverage(myosim::Hub& hub);

int main() {
  // Test without MyoSimulator.
  testRootFeature();
  testDebounce();
  testExponentialMovingAverage();
  testMovingAverage();

  // Test using MyoSimulator.
  myosim::Hub hub("com.voidingwarranties.myo-intelligesture-tests");
  myoSimTestRootFeature(hub);
#if 0
  myoSimTestDebounce(hub);
  myoSimTestExponentialMovingAverage(hub);
  myoSimTestMovingAverage(hub);
#endif

  return 0;
}

////////////////////////////////
// Tests without MyoSimulator //
////////////////////////////////

void testRootFeature() {
  features::RootFeature root_feature;
  std::string str;
  PrintEvents print_events(root_feature, str);

  uint64_t timestamp = 0;
  root_feature.onPair(nullptr, timestamp++, myo::FirmwareVersion{0, 1, 2, 3});
  root_feature.onUnpair(nullptr, timestamp++);
  root_feature.onConnect(nullptr, timestamp++, myo::FirmwareVersion{0, 1, 2, 3});
  root_feature.onDisconnect(nullptr, timestamp++);
  root_feature.onArmSync(nullptr, timestamp++, myo::armLeft, myo::xDirectionTowardWrist);
  root_feature.onArmUnsync(nullptr, timestamp++);
  root_feature.onUnlock(nullptr, timestamp++);
  root_feature.onLock(nullptr, timestamp++);
  root_feature.onPose(nullptr, timestamp++, myo::Pose::rest);
  root_feature.onGesture(nullptr, timestamp++, std::make_shared<core::Gesture>(core::Gesture::unknown));
  root_feature.onOrientationData(nullptr, timestamp++, myo::Quaternion<float>(0.f, 1.f, 2.f, 3.f));
  root_feature.onAccelerometerData(nullptr, timestamp++, myo::Vector3<float>(0.f, 1.f, 2.f));
  root_feature.onGyroscopeData(nullptr, timestamp++, myo::Vector3<float>(0.f, 1.f, 2.f));
  root_feature.onRssi(nullptr, timestamp++, 123);
  std::array<int8_t, 8> emg_data = {0, 1, 2, 3, 4, 5, 6, 7};
  root_feature.onEmgData(nullptr, timestamp++, emg_data.data());
  root_feature.onPeriodic(nullptr);

  assert(str ==
         "onPair - myo: 00000000 timestamp: 0 firmwareVersion: (0, 1, 2, 3)\n"
         "onUnpair - myo: 00000000 timestamp: 1\n"
         "onConnect - myo: 00000000 timestamp: 2 firmwareVersion: (0, 1, 2, 3)\n"
         "onDisconnect - myo: 00000000 timestamp: 3\n"
         "onArmSync - myo: 00000000 timestamp: 4 arm: armLeft xDirection: xDirectionTowardWrist\n"
         "onArmUnsync - myo: 00000000 timestamp: 5\n"
         "onUnlock - myo: 00000000 timestamp: 6\n"
         "onLock - myo: 00000000 timestamp: 7\n"
         "onPose - myo: 00000000 timestamp: 8 pose->toString(): rest\n"
         "onGesture - myo: 00000000 timestamp: 9 gesture->toString(): unknown\n"
         "onOrientationData - myo: 00000000 timestamp: 10 rotation: (0, 1, 2, 3)\n"
         "onAccelerometerData - myo: 00000000 timestamp: 11 accel: (0, 1, 2)\n"
         "onGyroscopeData - myo: 00000000 timestamp: 12 gyro: (0, 1, 2)\n"
         "onRssi - myo: 00000000 timestamp: 13 rssi: 123\n"
         "onEmgData - myo: 00000000 timestamp: 14 emg: (0, 1, 2, 3, 4, 5, 6, 7)\n"
         "onPeriodic - myo: 00000000\n");
}

void testDebounce() {
  for (int debounce_ms : {5, 10, 100}) {
    auto test_debounce = [debounce_ms](int timestamp_offset) {
      features::RootFeature root_feature;
      features::filters::Debounce debounce(root_feature, debounce_ms);
      std::string str;
      PrintEvents print_events(debounce, str);

      uint64_t timestamp = 0;
      debounce.onPose(nullptr, timestamp++,
                      std::make_shared<core::Pose>(myo::Pose::rest));
      debounce.onPose(nullptr, timestamp++,
                      std::make_shared<core::Pose>(myo::Pose::fist));
      debounce.onPose(nullptr, timestamp++ - 1 + timestamp_offset,
                      std::make_shared<core::Pose>(myo::Pose::rest));

      return str;
    };

    std::string result;
    result = test_debounce(debounce_ms * 1000 + 1);
    assert(result == "onPose - myo: 00000000 timestamp: 1 pose->toString(): fist\n");

    result = test_debounce(debounce_ms * 1000 - 1);
    assert(result == "");
  }
}

void testExponentialMovingAverage() {
  using features::filters::ExponentialMovingAverage;
  std::map<float, std::string> expected_results;
  expected_results[1.f] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (1, 1, 1, 1)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (1, 1, 1)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (1, 1, 1)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (2, 2, 2, 2)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (2, 2, 2)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (2, 2, 2)\n";
  expected_results[0.5f] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.5, 0.5, 0.5, 0.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.5, 0.5, 0.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.5, 0.5, 0.5)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (1.25, 1.25, 1.25, 1.25)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (1.25, 1.25, 1.25)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (1.25, 1.25, 1.25)\n";
  expected_results[0.25f] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.25, 0.25, 0.25, 0.25)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.25, 0.25, 0.25)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.25, 0.25, 0.25)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (0.6875, 0.6875, 0.6875, 0.6875)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (0.6875, 0.6875, 0.6875)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (0.6875, 0.6875, 0.6875)\n";

  for (const auto& alpha : expected_results) {
    features::RootFeature root_feature;
    ExponentialMovingAverage avg(
        root_feature, ExponentialMovingAverage::OrientationData |
                      ExponentialMovingAverage::AccelerometerData |
                      ExponentialMovingAverage::GyroscopeData,
        alpha.first);
    std::string str;
    PrintEvents print_events(avg, str);

    uint64_t timestamp = 0;
    for (std::size_t i = 0; i < 3; ++i) {
      float j = (float) i;
      avg.onOrientationData(nullptr, timestamp++,
                            myo::Quaternion<float>(j, j, j, j));
      avg.onAccelerometerData(nullptr, timestamp++,
                              myo::Vector3<float>(j, j, j));
      avg.onGyroscopeData(nullptr, timestamp++, myo::Vector3<float>(j, j, j));
    }

    assert (str == alpha.second);
  }
}

void testMovingAverage() {
  using features::filters::MovingAverage;
  std::map<int, std::string> expected_results;
  expected_results[1] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (1, 1, 1, 1)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (1, 1, 1)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (1, 1, 1)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (2, 2, 2, 2)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (2, 2, 2)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (2, 2, 2)\n";
  expected_results[2] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.5, 0.5, 0.5, 0.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.5, 0.5, 0.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.5, 0.5, 0.5)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (1.5, 1.5, 1.5, 1.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (1.5, 1.5, 1.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (1.5, 1.5, 1.5)\n";
  expected_results[3] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.5, 0.5, 0.5, 0.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.5, 0.5, 0.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.5, 0.5, 0.5)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (1, 1, 1, 1)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (1, 1, 1)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (1, 1, 1)\n";

  for (const auto& window_size : expected_results) {
    features::RootFeature root_feature;
    MovingAverage avg(root_feature, MovingAverage::OrientationData |
                                    MovingAverage::AccelerometerData |
                                    MovingAverage::GyroscopeData,
                      window_size.first);
    std::string str;
    PrintEvents print_events(avg, str);

    uint64_t timestamp = 0;
    for (std::size_t i = 0; i < 3; ++i) {
      float j = (float) i;
      avg.onOrientationData(nullptr, timestamp++,
                            myo::Quaternion<float>(j, j, j, j));
      avg.onAccelerometerData(nullptr, timestamp++,
                              myo::Vector3<float>(j, j, j));
      avg.onGyroscopeData(nullptr, timestamp++, myo::Vector3<float>(j, j, j));
    }

    assert (str == window_size.second);
  }
}

//////////////////////////////
// Tests using MyoSimulator //
//////////////////////////////

void myoSimTestRootFeature(myosim::Hub &hub) {
  features::RootFeature root_feature;
  std::string str;
  PrintEvents print_events(root_feature, str);
  hub.addListener(&root_feature);

  uint64_t timestamp = 0;
  myosim::EventRecorder er(myosim::EventRecorder::ALL_EVENTS);
  hub.addListener(&er);
  er.onPair(nullptr, timestamp++, myo::FirmwareVersion{0, 1, 2, 3});
  er.onUnpair(nullptr, timestamp++);
  er.onConnect(nullptr, timestamp++, myo::FirmwareVersion{0, 1, 2, 3});
  er.onDisconnect(nullptr, timestamp++);
  er.onArmSync(nullptr, timestamp++, myo::armLeft, myo::xDirectionTowardWrist, 0, myo::warmupStateCold );
  er.onArmUnsync(nullptr, timestamp++);
  er.onUnlock(nullptr, timestamp++);
  er.onLock(nullptr, timestamp++);
  er.onPose(nullptr, timestamp++, myo::Pose::rest);
  er.onOrientationData(nullptr, timestamp++, myo::Quaternion<float>(0.f, 1.f, 2.f, 3.f));
  er.onAccelerometerData(nullptr, timestamp++, myo::Vector3<float>(0.f, 1.f, 2.f));
  er.onGyroscopeData(nullptr, timestamp++, myo::Vector3<float>(0.f, 1.f, 2.f));
  er.onRssi(nullptr, timestamp++, 123);
  std::array<int8_t, 8> emg_data = {0, 1, 2, 3, 4, 5, 6, 7};
  er.onEmgData(nullptr, timestamp++, emg_data.data());

  myosim::EventPlayerHub eph(er.getEventQueue());
  eph.runAll();
  hub.removeListener(&root_feature);

  std::cout << str << std::endl;
  assert(str ==
         "onPair - myo: 00000000 timestamp: 0 firmwareVersion: (0, 1, 2, 3)\n"
         "onUnpair - myo: 00000000 timestamp: 1\n"
         "onConnect - myo: 00000000 timestamp: 2 firmwareVersion: (0, 1, 2, 3)\n"
         "onDisconnect - myo: 00000000 timestamp: 3\n"
         "onArmSync - myo: 00000000 timestamp: 4 arm: armLeft xDirection: xDirectionTowardWrist\n"
         "onArmUnsync - myo: 00000000 timestamp: 5\n"
         "onUnlock - myo: 00000000 timestamp: 6\n"
         "onLock - myo: 00000000 timestamp: 7\n"
         "onPose - myo: 00000000 timestamp: 8 pose->toString(): rest\n"
         "onOrientationData - myo: 00000000 timestamp: 9 rotation: (0, 1, 2, 3)\n"
         "onAccelerometerData - myo: 00000000 timestamp: 10 accel: (0, 1, 2)\n"
         "onGyroscopeData - myo: 00000000 timestamp: 11 gyro: (0, 1, 2)\n"
         "onRssi - myo: 00000000 timestamp: 12 rssi: 123\n"
         "onEmgData - myo: 00000000 timestamp: 13 emg: (0, 1, 2, 3, 4, 5, 6, 7)\n");
}

#if 0
void myoSimTestDebounce(myosim::Hub& hub) {
  for (int debounce_ms : {5, 10, 100}) {
    auto test_debounce = [&hub, debounce_ms](int timestamp_offset) {
      features::RootFeature root_feature;
      features::filters::Debounce debounce(root_feature, debounce_ms);
      std::string str;
      PrintEvents print_events(debounce, str);
      hub.addListener(&root_feature);

      uint64_t timestamp = 0;
      myosim::EventLoopGroup elg;
      elg.group.push_back(std::make_shared<myosim::PoseEvent>(
          0, timestamp++, myo::Pose::rest));
      elg.group.push_back(std::make_shared<myosim::PoseEvent>(
          0, timestamp++, myo::Pose::fist));
      elg.group.push_back(std::make_shared<myosim::PoseEvent>(
          0, timestamp++ - 1 + timestamp_offset, myo::Pose::rest));
      myosim::EventSession event_session;
      event_session.events.push_back(elg);

      myosim::EventPlayer event_player(hub);
      event_player.play(event_session);
      hub.removeListener(&root_feature);
      return str;
    };

    std::string result;
    result = test_debounce(debounce_ms * 1000 + 1);
    assert(result == "onPose - myo: 00000000 timestamp: 1 pose->toString(): fist\n");

    result = test_debounce(debounce_ms * 1000 - 1);
    assert(result == "");
  }
}

void myoSimTestExponentialMovingAverage(myosim::Hub& hub) {
  using features::filters::ExponentialMovingAverage;
  std::map<float, std::string> expected_results;
  expected_results[1.f] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (1, 1, 1, 1)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (1, 1, 1)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (1, 1, 1)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (2, 2, 2, 2)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (2, 2, 2)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (2, 2, 2)\n";
  expected_results[0.5f] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.5, 0.5, 0.5, 0.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.5, 0.5, 0.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.5, 0.5, 0.5)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (1.25, 1.25, 1.25, 1.25)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (1.25, 1.25, 1.25)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (1.25, 1.25, 1.25)\n";
  expected_results[0.25f] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.25, 0.25, 0.25, 0.25)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.25, 0.25, 0.25)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.25, 0.25, 0.25)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (0.6875, 0.6875, 0.6875, 0.6875)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (0.6875, 0.6875, 0.6875)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (0.6875, 0.6875, 0.6875)\n";

  for (const auto& alpha : expected_results) {
    features::RootFeature root_feature;
    ExponentialMovingAverage avg(
        root_feature, ExponentialMovingAverage::OrientationData |
                      ExponentialMovingAverage::AccelerometerData |
                      ExponentialMovingAverage::GyroscopeData,
        alpha.first);
    std::string str;
    PrintEvents print_events(avg, str);
    hub.addListener(&root_feature);

    uint64_t timestamp = 0;
    myosim::EventLoopGroup elg;
    for (std::size_t i = 0; i < 3; ++i) {
      elg.group.push_back(std::make_shared<myosim::OrientationDataEvent>(
        0, timestamp++, myo::Quaternion<float>(i, i, i, i)));
    elg.group.push_back(std::make_shared<myosim::AccelerometerDataEvent>(
        0, timestamp++, myo::Vector3<float>(i, i, i)));
    elg.group.push_back(std::make_shared<myosim::GyroscopeDataEvent>(
        0, timestamp++, myo::Vector3<float>(i, i, i)));
    }
    myosim::EventSession event_session;
    event_session.events.push_back(elg);

    myosim::EventPlayer event_player(hub);
    event_player.play(event_session);
    hub.removeListener(&root_feature);

    assert (str == alpha.second);
  }
}

void myoSimTestMovingAverage(myosim::Hub& hub) {
  using features::filters::MovingAverage;
  std::map<int, std::string> expected_results;
  expected_results[1] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (1, 1, 1, 1)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (1, 1, 1)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (1, 1, 1)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (2, 2, 2, 2)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (2, 2, 2)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (2, 2, 2)\n";
  expected_results[2] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.5, 0.5, 0.5, 0.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.5, 0.5, 0.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.5, 0.5, 0.5)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (1.5, 1.5, 1.5, 1.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (1.5, 1.5, 1.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (1.5, 1.5, 1.5)\n";
  expected_results[3] =
      "onOrientationData - myo: 00000000 timestamp: 0 rotation: (0, 0, 0, 0)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 1 accel: (0, 0, 0)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 2 gyro: (0, 0, 0)\n"
      "onOrientationData - myo: 00000000 timestamp: 3 rotation: (0.5, 0.5, 0.5, 0.5)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 4 accel: (0.5, 0.5, 0.5)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 5 gyro: (0.5, 0.5, 0.5)\n"
      "onOrientationData - myo: 00000000 timestamp: 6 rotation: (1, 1, 1, 1)\n"
      "onAccelerometerData - myo: 00000000 timestamp: 7 accel: (1, 1, 1)\n"
      "onGyroscopeData - myo: 00000000 timestamp: 8 gyro: (1, 1, 1)\n";

  for (const auto& window_size : expected_results) {
    features::RootFeature root_feature;
    MovingAverage avg(root_feature, MovingAverage::OrientationData |
                                    MovingAverage::AccelerometerData |
                                    MovingAverage::GyroscopeData,
                      window_size.first);
    std::string str;
    PrintEvents print_events(avg, str);
    hub.addListener(&root_feature);

    uint64_t timestamp = 0;
    myosim::EventLoopGroup elg;
    for (std::size_t i = 0; i < 3; ++i) {
      elg.group.push_back(std::make_shared<myosim::OrientationDataEvent>(
        0, timestamp++, myo::Quaternion<float>(i, i, i, i)));
    elg.group.push_back(std::make_shared<myosim::AccelerometerDataEvent>(
        0, timestamp++, myo::Vector3<float>(i, i, i)));
    elg.group.push_back(std::make_shared<myosim::GyroscopeDataEvent>(
        0, timestamp++, myo::Vector3<float>(i, i, i)));
    }
    myosim::EventSession event_session;
    event_session.events.push_back(elg);

    myosim::EventPlayer event_player(hub);
    event_player.play(event_session);
    hub.removeListener(&root_feature);

    assert (str == window_size.second);
  }
}
#endif