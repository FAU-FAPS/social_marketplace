cmake_minimum_required(VERSION 3.8)
project(social_marketplace_feedback_server)

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

add_library(feedback_server SHARED
  src/feedback_server.cpp)
target_include_directories(feedback_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(feedback_server
  PRIVATE "ACTION_SOCIAL_MARKETPLACE_FEEDBACK_BUILDING_DLL")
ament_target_dependencies(feedback_server
  "social_marketplace_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "ament_index_cpp"
  "social_marketplace_common")

rclcpp_components_register_node(feedback_server PLUGIN "action_social_marketplace_feedback_cpp::FeedbackCollectionActionServer" EXECUTABLE feedback_server_action_server)
install(TARGETS
  feedback_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

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