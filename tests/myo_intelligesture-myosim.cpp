#define BOOST_TEST_MODULE myo_intelligesture_myosim_tests
#include <boost/test/included/unit_test.hpp>

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

struct MyoSimFixture {
  MyoSimFixture() : hub("com.voidingwarranties.myo-intelligesture-tests") { }
  ~MyoSimFixture() { }
  myosim::Hub hub;
};

//////////////////////////////
// Tests using MyoSimulator //
//////////////////////////////

BOOST_FIXTURE_TEST_SUITE(myosim, MyoSimFixture)

BOOST_AUTO_TEST_CASE(myoSimTestRootFeature) {
  features::RootFeature root_feature;
  std::string str;
  PrintEvents print_events(root_feature, str);
  hub.addListener(&root_feature);

  uint64_t timestamp = 0;
  myosim::EventRecorder er(myosim::EventRecorder::ALL_EVENTS);
  hub.addListener(&er);
  hub.simulatePair(nullptr, timestamp++, myo::FirmwareVersion{0, 1, 2, 3});
  hub.simulateUnpair(nullptr, timestamp++);
  hub.simulateConnect(nullptr, timestamp++, myo::FirmwareVersion{0, 1, 2, 3});
  hub.simulateDisconnect(nullptr, timestamp++);
  hub.simulateArmSync(nullptr, timestamp++, myo::armLeft, myo::xDirectionTowardWrist, 0, myo::warmupStateCold );
  hub.simulateArmUnsync(nullptr, timestamp++);
  hub.simulateUnlock(nullptr, timestamp++);
  hub.simulateLock(nullptr, timestamp++);
  hub.simulatePose(nullptr, timestamp++, myo::Pose::rest);
  hub.simulateOrientationData(nullptr, timestamp++, myo::Quaternion<float>(0.f, 1.f, 2.f, 3.f));
  hub.simulateAccelerometerData(nullptr, timestamp++, myo::Vector3<float>(0.f, 1.f, 2.f));
  hub.simulateGyroscopeData(nullptr, timestamp++, myo::Vector3<float>(0.f, 1.f, 2.f));
  hub.simulateRssi(nullptr, timestamp++, 123);
  std::array<int8_t, 8> emg_data = {0, 1, 2, 3, 4, 5, 6, 7};
  hub.simulateEmgData(nullptr, timestamp++, emg_data.data());

  myosim::EventPlayerHub eph(er.getEventQueue());
  eph.runAll();
  hub.removeListener(&root_feature);

  BOOST_CHECK_EQUAL(str,
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

BOOST_AUTO_TEST_CASE(myoSimTestDebounce) {
  for (int debounce_ms : {5, 10, 100}) {
    auto test_debounce = [this, debounce_ms](int timestamp_offset) {
      features::RootFeature root_feature;
      features::filters::Debounce debounce(root_feature, debounce_ms);
      std::string str;
      PrintEvents print_events(debounce, str);
      hub.addListener(&root_feature);

      uint64_t timestamp = 0;
      myosim::EventRecorder er(myosim::EventRecorder::ALL_EVENTS);
      hub.addListener(&er);
      hub.simulatePose(0, timestamp++, myo::Pose::rest);
      hub.simulatePose(0, timestamp++, myo::Pose::fist);
      hub.simulatePose(0, timestamp++ - 1 + timestamp_offset, myo::Pose::rest);

      myosim::EventPlayerHub eph(er.getEventQueue());
      eph.runAll();
      hub.removeListener(&root_feature);
      return str;
    };

    std::string result;
    result = test_debounce(debounce_ms * 1000 + 1);
    BOOST_CHECK_EQUAL(result, "onPose - myo: 00000000 timestamp: 1 pose->toString(): fist\n");

    result = test_debounce(debounce_ms * 1000 - 1);
    BOOST_CHECK_EQUAL(result, "");
  }
}

BOOST_AUTO_TEST_CASE(myoSimTestExponentialMovingAverage) {
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
    myosim::EventRecorder er(myosim::EventRecorder::ALL_EVENTS);
    for (std::size_t i = 0; i < 3; ++i) {
      float j = (float) i;
      hub.simulateOrientationData(0, timestamp++, myo::Quaternion<float>(j, j, j, j));
      hub.simulateAccelerometerData(0, timestamp++, myo::Vector3<float>(j, j, j));
      hub.simulateGyroscopeData(0, timestamp++, myo::Vector3<float>(j, j, j));
    }
    myosim::EventPlayerHub eph(er.getEventQueue());
    eph.runAll();
    hub.removeListener(&root_feature);

    BOOST_CHECK_EQUAL(str, alpha.second);
  }
}

BOOST_AUTO_TEST_CASE(myoSimTestMovingAverage) {
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
    myosim::EventRecorder er(myosim::EventRecorder::ALL_EVENTS);
    for (std::size_t i = 0; i < 3; ++i) {
      float j = (float) i;
      hub.simulateOrientationData(0, timestamp++, myo::Quaternion<float>(j, j, j, j));
      hub.simulateAccelerometerData(0, timestamp++, myo::Vector3<float>(j, j, j));
      hub.simulateGyroscopeData(0, timestamp++, myo::Vector3<float>(j, j, j));
    }
    myosim::EventPlayerHub eph(er.getEventQueue());
    eph.runAll();
    hub.removeListener(&root_feature);

    BOOST_CHECK_EQUAL(str, window_size.second);
  }
}

BOOST_AUTO_TEST_SUITE_END()