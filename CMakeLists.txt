cmake_minimum_required(VERSION 3.8)
project(cone_detection)

# Enable C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# Additional system dependencies
find_package(PCL REQUIRED COMPONENTS segmentation filters kdtree)

# Include directories
include_directories(
  include
)

# Specify source files
set(SOURCES
  src/cone_detection_node.cpp
  src/utils/cone_classification.cpp
  src/utils/filtering_utils.cpp
  src/utils/ground_removal.cpp
  src/utils/cone_clustering.cpp
)

# Add the executable
add_executable(${PROJECT_NAME}_node ${SOURCES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_msgs
)

# Link PCL libraries manually if required
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Ament package
ament_package()
