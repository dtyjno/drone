cmake_minimum_required(VERSION 3.5)
project(px4_ros_com VERSION 1.0)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
#sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/
#find_package(GeographicLib REQUIRED)
#
find_package(vision_msgs REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "~/ardupilot_ws/install/ros2_interfaces")
find_package(ros2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros2_yolo_msgs REQUIRED)


#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# examples/listeners/pose_subscriber
add_executable(pose_subscriber src/examples/listeners/pose_subscriber.cpp)
ament_target_dependencies(pose_subscriber rclcpp geometry_msgs nav_msgs)
install(TARGETS pose_subscriber DESTINATION lib/${PROJECT_NAME})

# examlles/listeners/yolo_subscriber
add_executable(yolo_subscriber src/examples/listeners/yolo_subscriber.cpp)
ament_target_dependencies(yolo_subscriber rclcpp sensor_msgs nav_msgs ros2_interfaces)
install(TARGETS yolo_subscriber DESTINATION lib/${PROJECT_NAME})

# examlles/listeners/camera_image_subscriber
#add_executable(camera_image_subscriber src/examples/listeners/camera_image_subscriber.cpp)
#ament_target_dependencies(camera_image_subscriber rclcpp mavros_msgs OpenCV)
#install(TARGETS camera_image_subscriber DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv
#
# add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
# ament_target_dependencies(offboard_control_srv rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs Eigen3 GeographicLib vision_msgs ros2_interfaces sensor_msgs OpenCV cv_bridge)
# install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

# # examples/offboard/offboard_control_gps
# add_executable(offboard_control_gps src/examples/offboard/offboard_control_gps.cpp)
# ament_target_dependencies(offboard_control_gps rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs Eigen3 GeographicLib vision_msgs)
# install(TARGETS offboard_control_gps DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
include_directories(src/examples/offboard_control)
file(GLOB SOURCES "src/examples/offboard_control/*.cpp")
#list(REMOVE_ITEM SOURCES "src/examples/drone1.0/offboard_control_srv.cpp")
add_executable(offboard_control ${SOURCES}
    src/examples/offboard_control/FuzzyPID.cpp)
ament_target_dependencies(offboard_control rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs ros2_interfaces sensor_msgs nav_msgs OpenCV cv_bridge ament_index_cpp)
target_link_libraries(offboard_control yaml-cpp)  
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# examples/offboard/_offboard
include_directories(src/examples/_offboard)
file(GLOB SOURCES "src/examples/_offboard/*.cpp")
add_executable(_offboard ${SOURCES}
    src/examples/_offboard/FuzzyPID.cpp)
ament_target_dependencies(_offboard rclcpp mavros_msgs geometry_msgs nav_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs ros2_interfaces sensor_msgs OpenCV cv_bridge ament_index_cpp)

ament_target_dependencies(_offboard rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs ros2_yolo_msgs sensor_msgs OpenCV cv_bridge ament_index_cpp)

target_link_libraries(_offboard yaml-cpp)  
install(TARGETS _offboard DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_land

 include_directories(src/examples/offboard_land)
 file(GLOB SOURCES "src/examples/offboard_land/*.cpp")
# #list(REMOVE_ITEM SOURCES "src/examples/drone1.0/offboard_land_srv.cpp")
 add_executable(offboard_land ${SOURCES})
 target_link_libraries(offboard_land ${CURSES_LIBRARIES} yaml-cpp)
 ament_target_dependencies(offboard_land rclcpp mavros_msgs geometry_msgs nav_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs ros2_interfaces sensor_msgs OpenCV cv_bridge ament_index_cpp)
 install(TARGETS offboard_land DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_sub

include_directories(src/examples/offboard_sub)
file(GLOB SOURCES "src/examples/offboard_sub/*.cpp")
# #list(REMOVE_ITEM SOURCES "src/examples/drone1.0/offboard_sub_srv.cpp")
add_executable(offboard_sub ${SOURCES})
target_link_libraries(offboard_sub ${CURSES_LIBRARIES} yaml-cpp)
ament_target_dependencies(offboard_sub rclcpp mavros_msgs geometry_msgs nav_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs ros2_interfaces sensor_msgs OpenCV cv_bridge ament_index_cpp)
install(TARGETS offboard_sub DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 ardupilot_msgs geometry_msgs sensor_msgs nav_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

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

# ###########
# # Python ##
# ###########

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/examples/offboard_py/offboard_control.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
