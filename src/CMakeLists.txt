cmake_minimum_required(VERSION 3.15)
project(main)

# Debugging purposes
set(CMAKE_DEBUG TRUE)

if(NOT CMAKE_DEBUG)
    # Release has CPU/Memory optimizations, but won't provide debugging info
    set(CMAKE_BUILD_TYPE Release)
else()
    set(CMAKE_BUILD_TYPE Debug)
    message(STATUS "Running CMake build in DEBUG mode!")
endif()

# Handle possible Anaconda environment
# if(DEFINED ENV{CONDA_PREFIX})
#     set(CMAKE_PREFIX_PATH CONDA_PREFIX)
# endif()

# Disable Warnings
add_compile_options(
    -Wno-deprecated-declarations 
    -Wreorder 
    -Wsign-compare 
    -Wunused-variable 
    -Wunused-but-set-variable 
    -Wmaybe-uninitialized 
    -Waggressive-loop-optimizations
)

# Adding testing
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(DBoW2)
add_subdirectory(g2o)
add_subdirectory(Sophus) # Target named sophus
add_subdirectory(ORB_SLAM3)
add_subdirectory(python) # ORB_SLAM3_PYTHON_BINDING
add_subdirectory(test) # Testing the library

# For debugging purposes
if(CMAKE_DEBUG)
    target_compile_options(DBoW2 PRIVATE -g)
    target_compile_options(g2o PRIVATE -g)
    # target_compile_options(sophus PRIVATE -g) # INTERFACE, cannot do it
    target_compile_options(ORB_SLAM3 PRIVATE -g)
    target_compile_options(orbslam3 PRIVATE -g)
endif()

# This must be true to make g2o work
# set_target_properties(g2o PROPERTIES COMPILE_DEFINITIONS Debug)

# Make target's visible to each other
set_target_properties(DBoW2 PROPERTIES
    INSTALL_RPATH "$ORIGIN")
set_target_properties(g2o PROPERTIES
    INSTALL_RPATH "$ORIGIN")
set_target_properties(sophus PROPERTIES
    INSTALL_RPATH "$ORIGIN")
set_target_properties(ORB_SLAM3 PROPERTIES
    INSTALL_RPATH "$ORIGIN")
set_target_properties(orbslam3 PROPERTIES
    INSTALL_RPATH "$ORIGIN")

# Install the executable
install(TARGETS DBoW2
        EXCLUDE_FROM_ALL
        COMPONENT python_modules
        DESTINATION ${PY_BUILD_CMAKE_MODULE_NAME})

install(TARGETS g2o
        EXCLUDE_FROM_ALL
        COMPONENT python_modules
        DESTINATION ${PY_BUILD_CMAKE_MODULE_NAME})

install(TARGETS sophus
        EXCLUDE_FROM_ALL
        COMPONENT python_modules
        DESTINATION ${PY_BUILD_CMAKE_MODULE_NAME})

install(TARGETS ORB_SLAM3
        EXCLUDE_FROM_ALL
        COMPONENT python_modules
        DESTINATION ${PY_BUILD_CMAKE_MODULE_NAME})

install(TARGETS orbslam3
        EXCLUDE_FROM_ALL
        COMPONENT python_modules
        DESTINATION ${PY_BUILD_CMAKE_MODULE_NAME})
