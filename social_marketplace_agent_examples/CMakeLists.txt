cmake_minimum_required(VERSION 3.8)
project(social_marketplace_agent_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(social_marketplace_interfaces REQUIRED)
find_package(social_marketplace_common REQUIRED)
find_package(std_msgs REQUIRED)

# Create the shared library
add_library(agent_1 SHARED src/agent_1.cpp)
target_include_directories(agent_1 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(agent_1
  PRIVATE "ACTION_SOCIAL_MARKETPLACE_AGENT_BUILDING_DLL")
ament_target_dependencies(agent_1
  "social_marketplace_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "ament_index_cpp"
  "social_marketplace_common"
  "std_msgs"
)
rclcpp_components_register_node(agent_1 PLUGIN "action_social_marketplace_agent_cpp::Agent_1" EXECUTABLE agent_action_server)

install(TARGETS agent_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config
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