cmake_minimum_required(VERSION 2.8.9)
project(serial_pc)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5SerialPort REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS system-lib
)

qt5_add_resources(QT_RESOURCES_CPP qt_resources/serial_pc.qrc)
qt5_wrap_cpp(QT_MOC_HPP src/mainwindow.h src/settingsdialog.h ui/ui_mainwindow.h ui/ui_settingsdialog.h src/RobotThread.h)

include_directories(src ui ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5SerialPort_INCLUDE_DIRS})

add_executable(serial_pc src/mainwindow.cpp src/settingsdialog.cpp src/main.cpp src/RobotThread.cpp ${QT_RESOURCES_CPP} ${QT_MOC_HPP})

target_link_libraries(serial_pc ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::Widgets Qt5::SerialPort)

install(TARGETS serial_pc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
