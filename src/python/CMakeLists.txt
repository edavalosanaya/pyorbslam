cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3_PYTHON_BINDING)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(NumPy REQUIRED)
find_package(Boost REQUIRED COMPONENTS python numpy3)
find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)

# RPATH fiddling for the library, so that the installed version can find the linked libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
${PROJECT_SOURCE_DIR}/src
${PYTHON_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${PYTHON_NUMPY_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
src/ORBSlamPython.cpp
src/pyboostcvconverter/pyboost_cv4_converter.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_link_libraries(${PROJECT_NAME}
ORB_SLAM3
${OpenCV_LIBS}
${Boost_LIBRARIES}
${PYTHON_LIBRARIES}
)
