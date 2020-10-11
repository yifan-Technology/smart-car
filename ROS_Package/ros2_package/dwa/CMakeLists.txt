cmake_minimum_required(VERSION 3.5)
project(dwa)

# Default to C99
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()
#
# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()
#
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
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

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

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

###############################################################################
# INCLUDES and LIBS
###############################################################################

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
set(MY_TARGET_LINK_LIBRARIES        
    ${OpenCV_LIBRARY_DIRS} 
    jsoncpp
)
include_directories("${JSONCPP_INCLUDE_DIRS}")


###############################################################################
# executable
###############################################################################

add_executable(setgoal src/dwa_publisher.cpp)
ament_target_dependencies(setgoal rclcpp std_msgs geometry_msgs)

add_executable(listener src/dwa_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(planner src/yf_dwa.cpp
			src/DWA_Planner.cpp
			include/DWA_Planner.h)

ament_target_dependencies(planner
			  Eigen3 
			  OpenCV
			  rclcpp
			  std_msgs
			  sensor_msgs
			  geometry_msgs
			  jsoncpp
			  )

target_include_directories(planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(planner ${MY_TARGET_LINK_LIBRARIES})

#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)

install(TARGETS
  listener
  planner
  setgoal
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
