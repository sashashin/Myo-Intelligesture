enable_testing()
include(ExternalProject)

# START ExternalProject_Add MyoSimulator
# git clones MyoSimulator into /lib, and builds it with boost
ExternalProject_Add(MyoSimulator
  GIT_REPOSITORY https://github.com/toshipiazza/myosim
  PREFIX ${CMAKE_SOURCE_DIR}/lib/MyoSimulator
  INSTALL_DIR ${CMAKE_SOURCE_DIR}/lib/MyoSimulator
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_SOURCE_DIR}/lib/MyoSimulator
             -DMyo_INCLUDE_DIR=${Myo_INCLUDE_DIR}
             -DMyo_LIBRARY=${Myo_LIBRARY}
             -DWithSerialization=On  # we know we have Boost...
             -DBuildShared=On)       # for simplicity
set(MyoSim_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/MyoSimulator/include)
set(MyoSim_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/MyoSimulator/lib)
add_library(myosim SHARED IMPORTED)
if (UNIX OR APPLE)
  # TODO: test this
  set_target_properties(myosim PROPERTIES IMPORTED_LOCATION
                        ${MYOSIM_LIB_DIR}/libmyosim.so)
else() # windows
  set_target_properties(myosim PROPERTIES IMPORTED_LOCATION
                        ${MYOSIM_LIB_DIR}/myosim.lib)
endif()
# END ExternalProject_Add MyoSimulator

# BEGIN tests
# set up include and library dirs
include_directories(${Myo_INCLUDE_DIRS})
include_directories(${MyoSim_INCLUDE_DIR})
link_directories(${MyoSim_LIB_DIR})

# TODO: link gtest for more robust testing than assert
# TODO: split this test up (it's huge)
add_executable(tests tests.cpp)
target_link_libraries(tests ${Myo_LIBRARIES})
target_link_libraries(tests myosim)
target_compile_features(tests PRIVATE cxx_auto_type)
add_test(NAME test COMMAND tests)
# END tests
