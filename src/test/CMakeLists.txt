enable_testing()

# Simple test
add_executable(test_simple
    test_simple.cpp
)
target_link_libraries(test_simple
    GTest::gtest_main
)

# ORBSLAM3 tests
add_executable(test_mono
    test_mono.cpp
)
target_link_libraries(test_mono
    GTest::gtest_main
    ORB_SLAM3
)

include(GoogleTest)
gtest_discover_tests(test_simple test_mono)