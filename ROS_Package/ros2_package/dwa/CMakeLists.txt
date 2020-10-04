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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(yaml-cpp REQUIRED)

###############################################################################
# INCLUDES and LIBS

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
    ./include  
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARIES}
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
    yaml-cpp
)

set(MY_TARGET_LINK_LIBRARIES        
    ${OpenCV_LIBRARY_DIRS} 
    ${YAML_CPP_LIBRARIES}  
    ${PROJECT_NAME}_lib
)

###############################################################################
# executable

add_executable(hallo src/publisher_member_function.cpp)
target_link_libraries(hallo ${MY_TARGET_LINK_LIBRARIES})
ament_target_dependencies(hallo ${DEPENDENCIES})

add_executable(dwa src/dynamic_window_approach.cpp)
target_link_libraries(dwa ${MY_TARGET_LINK_LIBRARIES})
ament_target_dependencies(dwa ${DEPENDENCIES})


install(TARGETS
  hallo
  dwa
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
