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
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pal_statistics_msgs QUIET)
if(pal_statistics_msgs_FOUND)
    message(STATUS "PAL Statistics support enabled")
    add_definitions(-DPAL_STATISTIC_VISIBILITY)
    set(PAL_STATISTICS_DEPS pal_statistics_msgs)
else()
    message(WARNING "PAL Statistics requested but pal_statistics_msgs not found")
    set(PAL_STATISTICS_DEPS "")
endif()
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


# listeners/pose_subscriber
add_executable(pose_subscriber src/listeners/pose_subscriber.cpp)
ament_target_dependencies(pose_subscriber rclcpp geometry_msgs nav_msgs)
install(TARGETS pose_subscriber DESTINATION lib/${PROJECT_NAME})

# # listeners/yolo_subscriber
# add_executable(yolo_subscriber src/listeners/yolo_subscriber.cpp)
# ament_target_dependencies(yolo_subscriber rclcpp sensor_msgs nav_msgs ros2_interfaces)
# install(TARGETS yolo_subscriber DESTINATION lib/${PROJECT_NAME})

# offboard/offboard_control
include_directories(
    src/offboard_control
  ${Eigen3_INCLUDE_DIRS}  # Add Eigen include directories
)
file(GLOB SOURCES
    "src/offboard_control/*.cpp"
    "src/offboard_control/*/*.cpp"
    "src/offboard_control/*/*/*.cpp"
)
# Remove stateMachine sources from SOURCES
file(GLOB STATE_MACHINE_SOURCES
    "src/offboard_control/stateMachine/*.cpp"
    "src/offboard_control/stateMachine/*/*.cpp"
)
list(REMOVE_ITEM SOURCES ${STATE_MACHINE_SOURCES})

add_executable(offboard_control ${SOURCES})
ament_target_dependencies(offboard_control rclcpp mavros_msgs geometry_msgs nav_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs visualization_msgs sensor_msgs OpenCV cv_bridge ament_index_cpp ${PAL_STATISTICS_DEPS})
target_link_libraries(offboard_control yaml-cpp)  
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(
    ament_cmake
    rclcpp
    rosidl_default_runtime
    eigen3_cmake_module
    Eigen3
    ardupilot_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs)

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

# if(BUILD_TESTING)
#     find_package(ament_lint_auto REQUIRED)
#     # the following line skips the linter which checks for copyrights
#     # uncomment the line when a copyright and license is not present in all source files
#     #set(ament_cmake_copyright_FOUND TRUE)
#     # the following line skips cpplint (only works in a git repo)
#     # uncomment the line when this package is not in a git repo
#     #set(ament_cmake_cpplint_FOUND TRUE)
#     ament_lint_auto_find_test_dependencies()
# endif()
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
  target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
#   target_link_libraries(${PROJECT_NAME}_tutorial_test name_of_local_library)
endif()

# ###########
# # Python ##
# ###########

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/offboard_py/offboard_control.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
