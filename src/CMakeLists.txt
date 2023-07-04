cmake_minimum_required(VERSION 3.15)
project(main)

# Release has CPU/Memory optimizations, but won't provide debugging info
# Disable this if you want to debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Handle possible Anaconda environment
if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_PREFIX_PATH CONDA_PREFIX)
endif()

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

add_subdirectory(DBoW2)
add_subdirectory(g2o)
add_subdirectory(Sophus) # Target named sophus
add_subdirectory(ORB_SLAM3)
add_subdirectory(python) # ORB_SLAM3_PYTHON_BINDING

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

install(TARGETS _orbslam3
        EXCLUDE_FROM_ALL
        COMPONENT python_modules
        DESTINATION ${PY_BUILD_CMAKE_MODULE_NAME})
