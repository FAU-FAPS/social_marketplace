cmake_minimum_required(VERSION 3.8)
project(social_marketplace_bidding_platform)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(social_marketplace_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(social_marketplace_common REQUIRED)


add_executable(bidding_platform src/bidding_platform.cpp)


target_include_directories(bidding_platform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  bidding_platform
  "rclcpp"
  "rclcpp_action"
  "social_marketplace_interfaces"
  "std_msgs"
  "social_marketplace_common"
)

install(TARGETS bidding_platform
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()