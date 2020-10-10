cmake_minimum_required(VERSION 3.5)
project(dwa)

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
#############################
#Dependencies
#############################
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
#find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)

###############################################################################
# INCLUDES and LIBS

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${jsoncpp_INCLUDE_DIRS}
    ./include  
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  jsoncpp
)
add_library(${PROJECT_NAME}_lib
"src/DWA_Planner.cpp"
)

set(DEPENDENCIES        
    rclcpp
    rclcpp_components
    image_transport
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    stereo_msgs
    OpenCV
    cv_bridge
    jsoncpp
)

set(MY_TARGET_LINK_LIBRARIES        
    ${OpenCV_LIBRARY_DIRS} 
    jsoncpp
)

###############################################################################
# executable

#add_executable(hallo src/publisher_member_function.cpp)
#target_link_libraries(hallo ${MY_TARGET_LINK_LIBRARIES} ${PROJECT_NAME}_lib)
#ament_target_dependencies(hallo ${DEPENDENCIES})

#add_executable(dwa src/dynamic_window_approach.cpp)
#target_link_libraries(dwa ${MY_TARGET_LINK_LIBRARIES} ${PROJECT_NAME}_lib)
#ament_target_dependencies(dwa ${DEPENDENCIES})

#add_executable(talker src/dwa_publisher.cpp)
#ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/dwa_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(planner src/yf_dwa.cpp
			src/DWA_Planner.cpp
			include/DWA_Planner.h)
ament_target_dependencies(planner Eigen3 OpenCV rclcpp std_msgs)

target_include_directories(planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
   listener
  planner
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