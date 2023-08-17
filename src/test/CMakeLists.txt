enable_testing()

# Simple test
add_executable(test_simple
    test_simple.cpp
)
target_link_libraries(test_simple
    GTest::gtest_main
)

# ORBSLAM3 integration tests
add_executable(test_mono
    test_mono.cpp
)
target_link_libraries(test_mono
    GTest::gtest_main
    ORB_SLAM3
)

# ORBSLAM3 unittests
add_executable(test_map
    test_map.cpp
)
target_link_libraries(test_map
    GTest::gtest_main
    ORB_SLAM3
)
add_executable(test_key_frame
    test_key_frame.cpp
)
target_link_libraries(test_key_frame
    GTest::gtest_main
    ORB_SLAM3
)
add_executable(test_atlas
    test_atlas.cpp
)
target_link_libraries(test_atlas
    GTest::gtest_main
    ORB_SLAM3
)

include(GoogleTest)
gtest_discover_tests(test_simple test_mono test_map)
