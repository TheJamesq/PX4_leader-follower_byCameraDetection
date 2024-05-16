# Required CMake Version
cmake_minimum_required(VERSION 3.8)

# Project Name
project(drone_control VERSION 0.1 LANGUAGES CXX)

# Common compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS packages
find_package(ament_cmake REQUIRED)
find_package(dronecontrol_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# find dependencies

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED) 
#find_package(Qt53DCore REQUIRED)
find_package(Qt5DataVisualization REQUIRED)
find_package(Qt5Charts REQUIRED)



#find_package(Qt5DataVisualization REQUIRED)

# Find Qt packages
#find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets LinguistTools REQUIRED)
#find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets LinguistTools REQUIRED)
#find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Widgets REQUIRED)


# Manually specify ROS2 dependencies for gui target



# Qt related settingssrc/UIinit.cpp
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(TS_FILES drone_GUI_en_GB.ts)

# Add move_forward_node executable
add_executable(control src/drone_control.cpp)
add_executable(gui src/main.cpp src/mainwindow.cpp src/UIInitializer.cpp)


ament_target_dependencies(control
  rclcpp 
  px4_msgs 
  mavros_msgs
  geometry_msgs
  image_transport 
  cv_bridge
  dronecontrol_msg
)

target_link_libraries(control
${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
   ${OpenCV_LIBS}
)

ament_target_dependencies(gui
  rclcpp 
  px4_msgs 
  mavros_msgs
  geometry_msgs
  image_transport 
  cv_bridge
)

target_include_directories(gui PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)


# Link ROS2 libraries


# Link Qt libraries
target_link_libraries(gui
${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::Gui
   Qt${QT_VERSION_MAJOR}::DataVisualization
   Qt${QT_VERSION_MAJOR}::Charts
   ${OpenCV_LIBS}
)


# Add drone_GUI executable

#if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#  qt_add_executable(gui MANUAL_FINALIZATION ${PROJECT_SOURCES})
#else()
#  add_executable(gui ${PROJECT_SOURCES})
#endif()
#target_link_libraries(gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# Install Targets
install(TARGETS
  move_forward_node
  gui
  DESTINATION lib/${PROJECT_NAME}
)

# Testing and Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize
if(QT_VERSION_MAJOR EQUAL 6)
  qt_finalize_executable(gui)
endif()

# Package the project
ament_package()
