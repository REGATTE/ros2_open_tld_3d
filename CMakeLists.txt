cmake_minimum_required(VERSION 3.8)
project(ros2_open_tld_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# for defining custom services and messages
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/AddTwoInts.srv"
  #"msg/StdMsgsTime.msg"
#)

include_directories(include ${cv_bridge_INCLUDE_DIRS})

add_library(pcl_object_detection SHARED
  src/pcl_object_detection.cpp)
target_compile_definitions(pcl_object_detection
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pcl_object_detection
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
    "cv_bridge"
    "OpenCV"
    "pcl_conversions"
    "PCL"
  geometry_msgs
  pcl_ros
  shape_msgs
  tf2_ros
)

rclcpp_components_register_nodes(pcl_object_detection "pcl_object_detection::PclObjectDetection")
set(node_plugins "${node_plugins}pcl_object_detection::PclObjectDetection;$<TARGET_FILE:pcl_object_detection>\n")

add_executable(pcl_object_detection_node
    src/pcl_object_detection_node.cpp)
target_link_libraries(pcl_object_detection_node
    pcl_object_detection
)
    
ament_target_dependencies(pcl_object_detection_node
    "rclcpp"
)


install(TARGETS
    pcl_object_detection
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(TARGETS
    pcl_object_detection_node
    DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
