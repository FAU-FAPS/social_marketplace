cmake_minimum_required(VERSION 3.8)
project(social_marketplace_scene_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(social_marketplace_interfaces REQUIRED)
find_package(social_marketplace_common REQUIRED)

include_directories(include)

# Define the executable
add_executable(mock_scene_publisher src/mock_scene_publisher.cpp)

# Link the common library properly
ament_target_dependencies(mock_scene_publisher 
  rclcpp 
  rclcpp_action
  ament_index_cpp 
  social_marketplace_interfaces 
  social_marketplace_common
)

# Install the executable
install(TARGETS 
  mock_scene_publisher
  DESTINATION lib/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)  

ament_package()