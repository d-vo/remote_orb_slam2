cmake_minimum_required(VERSION 2.4.6)

project(compression)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_genmsg()


IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")



# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)


find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)



set(LIBS 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES} 
${Boost_LIBRARIES}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

include_directories(
${OpenCV_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../Thirdparty/featureCompression/install/include
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)





## TUM
# SENDER
rosbuild_add_executable(DemoSenderTUM
${PROJECT_SOURCE_DIR}/src/compression/Sender_TUM.cpp
)

target_link_libraries(DemoSenderTUM
${LIBS}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/featureCompression/install/lib/libfeatureCompression.so
)


# RECEIVER
rosbuild_add_executable(DemoReceiverTUM
${PROJECT_SOURCE_DIR}/src/compression/Receiver_TUM.cpp
)

target_link_libraries(DemoReceiverTUM
${LIBS}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/featureCompression/install/lib/libfeatureCompression.so
)

