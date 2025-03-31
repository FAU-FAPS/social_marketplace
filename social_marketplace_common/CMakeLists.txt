cmake_minimum_required(VERSION 3.8)
project(social_marketplace_common)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Add any dependencies needed by table_functions

# Include the header files
include_directories(include)

# Create the shared library
add_library(social_marketplace_common SHARED src/table_functions.cpp)

# Include directories for the library
target_include_directories(social_marketplace_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link any required libraries (e.g., rclcpp if needed)
ament_target_dependencies(social_marketplace_common
  rclcpp
)

# Install the library
install(TARGETS social_marketplace_common
  EXPORT export_social_marketplace_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export targets and dependencies
ament_export_targets(export_social_marketplace_common HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp)  # Include all dependencies
ament_export_include_directories(include)
ament_export_libraries(social_marketplace_common)

ament_package()