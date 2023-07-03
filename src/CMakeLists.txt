cmake_minimum_required(VERSION 3.15)
project(orbslam3-dev)

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
add_subdirectory(Sophus) # Project named sophus
add_subdirectory(ORB_SLAM3)
add_subdirectory(python) # ORB_SLAM3_PYTHON_BINDING
