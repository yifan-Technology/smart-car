cmake_minimum_required(VERSION 3.5)
project(webvideoserver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release )
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug )
    message(" * Debug Mode")
endif()

###############################################################################
#Dependencies
###############################################################################
# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(async_web_server_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig REQUIRED)


find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

#pkg_check_modules(avcodec libavcodec REQUIRED)
#pkg_check_modules(avformat libavformat REQUIRED)
#pkg_check_modules(avutil libavutil REQUIRED)
#pkg_check_modules(swscale libswscale REQUIRED)
###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
###############################################################################
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${AVCODEC_INCLUDE_DIR}
    ${AVFORMAT_INCLUDE_DIR} 
    ${AVUTIL_INCLUDE_DIR} 
    ${AVDEVICE_INCLUDE_DIR}
    ${SWSCALE_INCLUDE_DIR}
    ./include  
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${AVCODEC_LIBRARY} 
  ${AVFORMAT_LIBRARY} 
  ${AVUTIL_LIBRARY} 
  ${AVDEVICE_LIBRARY}
  ${SWSCALE_LIBRARY}
)


set(DEPENDENCIES        
  async_web_server_cpp 
  cv_bridge 
  image_transport 
  rclcpp 
  sensor_msgs
  OpenCV
)

set(MY_CPP
  src/image_streamer.cpp
  src/libav_streamer.cpp
  src/vp8_streamer.cpp
  src/h264_streamer.cpp
  src/vp9_streamer.cpp
  src/multipart_stream.cpp
  src/ros_compressed_streamer.cpp
  src/jpeg_streamers.cpp
  src/png_streamers.cpp
)
set(MY_H
  include/web_video_server/web_video_server.h
  include/web_video_server/image_streamer.h
  include/web_video_server/libav_streamer.h
  include/web_video_server/h264_streamer.h
  include/web_video_server/vp9_streamer.h
  include/web_video_server/multipart_stream.h
  include/web_video_server/ros_compressed_streamer.h
  include/web_video_server/jpeg_streamers.h
  include/web_video_server/png_streamers.h
)

set(MY_TARGET_LINK_LIBRARIES        
  ${OpenCV_LIBRARY_DIRS}
  ${AVCODEC_LIBRARY} 
  ${AVFORMAT_LIBRARY} 
  ${AVUTIL_LIBRARY} 
  ${AVDEVICE_LIBRARY}
  ${SWSCALE_LIBRARY} 
)

###############################################################################
# executable
###############################################################################

add_executable(${PROJECT_NAME} src/web_video_server.cpp ${MY_CPP} ${MY_H})
target_link_libraries(${PROJECT_NAME} ${MY_TARGET_LINK_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${MY_TARGET_LINK_LIBRARIES})

add_executable(pubSimpleImage src/pubSimpleImage.cpp)
target_link_libraries(pubSimpleImage ${MY_TARGET_LINK_LIBRARIES})
ament_target_dependencies(pubSimpleImage ${DEPENDENCIES})

target_include_directories(pubSimpleImage
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${MY_TARGET_LINK_LIBRARIES})



install(TARGETS
  ${PROJECT_NAME}
  pubSimpleImage
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
