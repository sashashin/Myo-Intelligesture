cmake_minimum_required(VERSION 3.1)

project(MYO_INTELLIGESTURE)

# TODO: support WithSerialization for MyoSim...
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(Myo REQUIRED)

set(SOURCES
	src/core/DeviceListenerWrapper.cpp
	src/core/Gesture.cpp
	src/core/OrientationUtility.cpp
	src/core/Pose.cpp)

set(HEADERS
	src/core/DeviceListenerWrapper.h
	src/core/Gesture.h
	src/core/OrientationUtility.h
	src/core/Pose.h
	src/features/Blocker.h
	src/features/CorrectForOrientation.h
	src/features/Orientation.h
	src/features/OrientationPoses.h
	src/features/RootFeature.h
	src/features/gestures/PoseGestures.h
	src/features/filters/Debounce.h
	src/features/filters/ExponentialMovingAverage.h
	src/features/filters/FiniteImpulseResponse.h
	src/features/filters/MovingAverage.h)

add_library(myo_intelligesture "${SOURCES}" "${HEADERS}")
include_directories(${Myo_INCLUDE_DIRS})
target_link_libraries(myo_intelligesture ${Myo_LIBRARIES})
target_compile_features(myo_intelligesture PRIVATE cxx_auto_type)
