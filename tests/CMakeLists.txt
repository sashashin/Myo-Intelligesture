enable_testing()
include_directories(${Myo_INCLUDE_DIRS})

# single test file... TODO: split this up!
add_executable(tests tests.cpp)
target_link_libraries(tests ${Myo_LIBRARIES})
target_compile_features(tests PRIVATE cxx_auto_type)
add_test(tests)