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
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
#sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/
find_package(GeographicLib REQUIRED)
find_package(OpenCV REQUIRED)

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
ament_target_dependencies(pose_subscriber rclcpp geometry_msgs )
install(TARGETS pose_subscriber DESTINATION lib/${PROJECT_NAME})

# examlles/camera/camera_image_subscriber
add_executable(camera_image_subscriber src/examples/listeners/camera_image_subscriber.cpp)
ament_target_dependencies(camera_image_subscriber rclcpp mavros_msgs OpenCV)
install(TARGETS camera_image_subscriber DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv

add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs Eigen3 GeographicLib)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})


# examples/offboard/offboard_control_drone
add_executable(offboard_control_drone src/examples/offboard/offboard_control_drone.cpp)
ament_target_dependencies(offboard_control_drone rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs)
install(TARGETS offboard_control_drone DESTINATION lib/${PROJECT_NAME})


# examples/offboard/offboard_control_srv_simple
add_executable(offboard_control_srv_simple src/examples/offboard/offboard_control_srv_simple.cpp)
ament_target_dependencies(offboard_control_srv_simple rclcpp mavros_msgs geometry_msgs geographic_msgs)
install(TARGETS offboard_control_srv_simple DESTINATION lib/${PROJECT_NAME})

#examples/offboard/test_offboard
add_executable(test_offboard src/examples/offboard/test_offboard.cpp)
ament_target_dependencies(test_offboard rclcpp mavros_msgs geometry_msgs geographic_msgs)
install(TARGETS test_offboard DESTINATION lib/${PROJECT_NAME})

#examples/offboard/base_local
add_executable(base_local src/examples/offboard/base_local.cpp)
ament_target_dependencies(base_local rclcpp mavros_msgs geometry_msgs geographic_msgs trajectory_msgs Eigen3 GeographicLib)
install(TARGETS base_local DESTINATION lib/${PROJECT_NAME})

# examples/drone/offboard_control
include_directories(src/examples/drone)
file(GLOB SOURCES "src/examples/drone/*.cpp")
add_executable(offboard_control ${SOURCES})
ament_target_dependencies(offboard_control rclcpp mavros_msgs geometry_msgs geographic_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 ardupilot_msgs geometry_msgs sensor_msgs)

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
