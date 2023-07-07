cmake_minimum_required(VERSION 2.8)
project(DBoW2)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3 -march=native")

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/DBow2
    ${CMAKE_CURRENT_SOURCE_DIR}/DUtils
    ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED 
    ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/src/BowVector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/src/FeatureVector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/src/FORB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/src/ScoringObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/src/Random.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/src/Timestamp.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/DBoW2>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/DUtils>)

target_include_directories(${PROJECT_NAME} INTERFACE PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
