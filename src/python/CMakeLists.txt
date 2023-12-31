cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3_PYTHON_BINDING)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3")

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

# find_package(Python 3 COMPONENTS Interpreter Development Numpy)
find_package(PythonInterp 3)
find_package(PythonLibs)

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Get the NumPy include directory from Python
set(PYTHON_NUMPY_INCLUDE_DIR "${PYTHON_SITE_PACKAGES}/numpy/core/include")

message(STATUS "PYTHON_SITE_PACKAGES = ${PYTHON_SITE_PACKAGES}")
message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
message(STATUS "PYTHON_NUMPY_INCLUDE_DIR = ${PYTHON_NUMPY_INCLUDE_DIR}")

find_package(Boost REQUIRED COMPONENTS python numpy3)
message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)

# RPATH fiddling for the library, so that the installed version can find the linked libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
${PROJECT_SOURCE_DIR}/src
${PYTHON_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${PYTHON_NUMPY_INCLUDE_DIR}
)

set(TARGET_MODULE_NAME orbslam3)

add_library(${TARGET_MODULE_NAME} SHARED
    src/ORBSlamPython.cpp 
    src/pyboostcvconverter/pyboost_cv4_converter.cpp)
set_target_properties(${TARGET_MODULE_NAME} PROPERTIES PREFIX "") # Necessary to remove liborbslam3 -> orbslam3

target_compile_definitions(${TARGET_MODULE_NAME} PRIVATE
    MODULE_NAME=$<TARGET_FILE_BASE_NAME:${TARGET_MODULE_NAME}>
)

# Hide all symbols by default (including external libraries on Linux)
set_target_properties(${TARGET_MODULE_NAME} PROPERTIES
    CXX_VISIBILITY_PRESET "hidden"
    VISIBILITY_INLINES_HIDDEN true)
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    target_link_options(${TARGET_MODULE_NAME} PRIVATE "LINKER:--exclude-libs,ALL")
endif()

target_link_libraries(${TARGET_MODULE_NAME}
ORB_SLAM3
${OpenCV_LIBS}
${Boost_LIBRARIES}
${PYTHON_LIBRARIES}
)
