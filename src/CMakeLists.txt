cmake_minimum_required(VERSION 3.15)
project(orbslam3)
set(PY_VERSION_SUFFIX "")
set(PY_FULL_VERSION ${PROJECT_VERSION}${PY_VERSION_SUFFIX})

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
add_subdirectory(Sophus)
add_subdirectory(ORB_SLAM3)
add_subdirectory(python) # ORB_SLAM3_PYTHON_BINDING

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE DBoW2 g2o sophus ORB_SLAM3 ORB_SLAM3_PYTHON_BINDING)

# Install the executable
install(TARGETS ${PROJECT_NAME} DBoW2 g2o sophus ORB_SLAM3 ORB_SLAM3_PYTHON_BINDING
        EXCLUDE_FROM_ALL
        COMPONENT python_binaries
        RUNTIME DESTINATION ${PY_BUILD_CMAKE_PACKAGE_NAME}-${PY_BUILD_CMAKE_PACKAGE_VERSION}.data/scripts
        LIBRARY DESTINATION ${PY_BUILD_CMAKE_PACKAGE_NAME}-${PY_BUILD_CMAKE_PACKAGE_VERSION}.data/platlib)
