enable_testing()
include(ExternalProject)

# START ExternalProject_Add MyoSimulator
# git clones MyoSimulator into /lib, and builds it with boost
ExternalProject_Add(project_MyoSimulator
  GIT_REPOSITORY https://github.com/toshipiazza/myosim
  PREFIX ${CMAKE_SOURCE_DIR}/lib/MyoSimulator
  # does the INSTALL_DIR not work?
  INSTALL_DIR ${CMAKE_SOURCE_DIR}/lib/MyoSimulator
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/lib/MyoSimulator
             -DMyo_INCLUDE_DIR=${Myo_INCLUDE_DIR}
             -DMyo_LIBRARY=${Myo_LIBRARY}
             -DWithSerialization=On  # we know we have Boost...
             -DBoost_USE_STATIC_LIBS=On
             -DBoost_USE_MULTITHREADED=On)
set(MyoSim_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/MyoSimulator/include)
set(MyoSim_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/MyoSimulator/lib)
add_library(myosim STATIC IMPORTED)
if (UNIX OR APPLE)
  # TODO: test this
  set_property(TARGET myosim PROPERTY IMPORTED_LOCATION
               ${MyoSim_LIB_DIR}/libmyosim.a)
else() # windows
  set_property(TARGET myosim PROPERTY IMPORTED_LOCATION
               ${MyoSim_LIB_DIR}/myosim.lib)
endif()
add_dependencies(myosim project_MyoSimulator)
# END ExternalProject_Add MyoSimulator

# BEGIN tests
# set up include and library dirs
include_directories(${Myo_INCLUDE_DIRS})
include_directories(${MyoSim_INCLUDE_DIR})
link_directories(${MyoSim_LIB_DIR})

# TODO: link gtest for more robust testing than assert
add_executable(myo_intelligesture_test myo_intelligesture.cpp)
target_link_libraries(myo_intelligesture_test myosim)
target_link_libraries(myo_intelligesture_test ${Myo_LIBRARIES})
target_link_libraries(myo_intelligesture_test myo_intelligesture)
target_link_libraries(myo_intelligesture_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_features(myo_intelligesture_test PRIVATE cxx_auto_type)
add_test(NAME myo_intelligesture_test COMMAND myo_intelligesture_test)

add_executable(myo_intelligesture_myosim_test myo_intelligesture-myosim.cpp)
target_link_libraries(myo_intelligesture_myosim_test myosim)
target_link_libraries(myo_intelligesture_myosim_test ${Myo_LIBRARIES})
target_link_libraries(myo_intelligesture_myosim_test myo_intelligesture)
target_link_libraries(myo_intelligesture_myosim_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_compile_features(myo_intelligesture_myosim_test PRIVATE cxx_auto_type)
add_test(NAME myo_intelligesture_myosim_test COMMAND myo_intelligesture_myosim_test)
# END tests
