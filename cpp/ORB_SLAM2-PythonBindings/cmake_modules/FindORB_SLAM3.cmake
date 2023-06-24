# - Try to find ORB_SLAM3
# Set alternative paths to search for using ORB_SLAM3_DIR
# Once done this will define
#  ORB_SLAM3_FOUND - System has ORB_SLAM3
#  ORB_SLAM3_INCLUDE_DIRS - The ORB_SLAM3 include directories
#  ORB_SLAM3_LIBRARIES - The libraries needed to use ORB_SLAM3
#  ORB_SLAM3_DEFINITIONS - Compiler switches required for using ORB_SLAM3

# TODO: This need to find dependencies properly, I can't find an example of how to do that
#find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)

set(_ORB_SLAM3_SEARCHES /usr/local)
if (ORB_SLAM3_DIR)
    set(_ORB_SLAM3_SEARCHES ${ORB_SLAM3_DIR} ${_ORB_SLAM3_SEARCHES})
endif()
find_path(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3/System.h
          PATHS ${_ORB_SLAM3_SEARCHES} PATH_SUFFIXES include)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${_ORB_SLAM3_SEARCHES} PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM3  DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY )

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR})

