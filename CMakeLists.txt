cmake_minimum_required( VERSION 2.8 )

project(Pangolin_test)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Pangolin REQUIRED)
find_package( Sophus REQUIRED )

include_directories( "/usr/include/eigen3" )

include_directories(
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
)

add_executable(Pangolin_test Pangolin_test.cpp)

target_link_libraries(Pangolin_test
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES}
${OpenCV_LIBS}
)
