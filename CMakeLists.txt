cmake_minimum_required(VERSION 3.1)
project(MYO_INTELLIGESTURE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(Myo REQUIRED)
# TODO: don't hardcode these, windows sucks :/
# we only need the header files, no libraries to link against
# still, it's required for a feature, and also in MyoSim
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED unit_test_framework)

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
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(myo_intelligesture ${Myo_LIBRARIES})
target_compile_features(myo_intelligesture PRIVATE cxx_auto_type)

add_subdirectory(tests)
