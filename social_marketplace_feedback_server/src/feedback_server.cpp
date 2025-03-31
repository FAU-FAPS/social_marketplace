#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cstdio>
#include <functional>
#include <memory>
#include <thread>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "social_marketplace_interfaces/action/feedback_collection.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_marketplace_common/table_functions.hpp"

namespace fs = std::filesystem;

namespace action_social_marketplace_feedback_cpp
{

// This node acts as a **Feedback Collection Action Server** in the social marketplace system.
// It listens for agent feedback requests, loads expected agent parameters, and processes feedback.

class FeedbackCollectionActionServer : public rclcpp::Node
{
public:
  using FeedbackCollection = social_marketplace_interfaces::action::FeedbackCollection;
  using GoalHandleFeedbackCollection = rclcpp_action::ServerGoalHandle<FeedbackCollection>;

  // Constructor: Initializes the action server and loads agent parameters from CSV files.
  explicit FeedbackCollectionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("feedback_collection_action_server", options)
  {
    using namespace std::placeholders;

    // Create the action server and bind the goal, cancel, and accept handlers.
    this->action_server_ = rclcpp_action::create_server<FeedbackCollection>(
      this,
      "feedback_collection",
      std::bind(&FeedbackCollectionActionServer::handle_goal, this, _1, _2),
      std::bind(&FeedbackCollectionActionServer::handle_cancel, this, _1),
      std::bind(&FeedbackCollectionActionServer::handle_accepted, this, _1));

    // Load agent parameters from CSV files located in the feedback server package.
    loadAgentParameters(ament_index_cpp::get_package_share_directory("social_marketplace_feedback_server") + "/config");
  }

private:
  social_marketplace_common::Table actual_agent_call_table;
  std::unordered_map<std::string, social_marketplace_common::Table> agent_parameters;
  std::unordered_map<std::string, size_t> current_row_index;

  // Function to load agent parameters from CSV files in the specified directory
  void loadAgentParameters(const std::string& directory) {
      RCLCPP_INFO(this->get_logger(), "Loading agent parameters from directory: %s", directory.c_str());

      if (!fs::exists(directory)) {
          RCLCPP_ERROR(this->get_logger(), "Directory does not exist: %s", directory.c_str());
          return;
      }

      if (!fs::is_directory(directory)) {
          RCLCPP_ERROR(this->get_logger(), "Path is not a directory: %s", directory.c_str());
          return;
      }

      for (const auto& entry : fs::directory_iterator(directory)) {
          if (entry.path().extension() == ".csv") {
              RCLCPP_INFO(this->get_logger(), "Reading file: %s", entry.path().string().c_str());
              social_marketplace_common::Table table = social_marketplace_common::readCSV(entry.path().string());
              std::string agent_name = entry.path().stem().string();
              RCLCPP_INFO(this->get_logger(), "Loaded parameters for agent: %s", agent_name.c_str());
              agent_parameters[agent_name] = table;
          } else {
              RCLCPP_WARN(this->get_logger(), "Ignoring non-CSV file: %s", entry.path().string().c_str());
          }
      }
  }

  // Handles incoming goal requests for feedback collection.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FeedbackCollection::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received Feedback Goal Request From Agent ID: %ld with the Name: %s", goal->agent_id, goal->agent_name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handles requests to cancel an active goal.
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFeedbackCollection> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handles accepted goals by launching a separate thread to execute them.
  void handle_accepted(const std::shared_ptr<GoalHandleFeedbackCollection> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&FeedbackCollectionActionServer::execute, this, _1), goal_handle}.detach();
  }

  // Processes the feedback request, compares agent parameters, and returns adjusted values.
  void execute(const std::shared_ptr<GoalHandleFeedbackCollection> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FeedbackCollection::Feedback>();
    auto result = std::make_shared<FeedbackCollection::Result>();

    // Store agent's current feedback parameters in a table.
    social_marketplace_common::resetTableWithNewHeaders(actual_agent_call_table, goal->parameter_names);
    social_marketplace_common::addRowFromDoubleVector(actual_agent_call_table, goal->parameter_values);
    auto current_agent_name = goal->agent_name;
    social_marketplace_common::Table temp_table;

    // Load the agent's target parameters from a CSV file.
    std::string csv_file_path = ament_index_cpp::get_package_share_directory("social_marketplace_feedback_server") 
                            + "/config/" + current_agent_name + "_target_parameters.csv";

    try {
        social_marketplace_common::Table agent_feedback_table = social_marketplace_common::readCSV(csv_file_path);
        int last_row_index = social_marketplace_common::getLastRowIndex(current_row_index, current_agent_name);
        int next_row_index = (last_row_index + 1) % social_marketplace_common::getRowCount(agent_feedback_table);

        social_marketplace_common::resetTableWithNewHeaders(temp_table, social_marketplace_common::getHeadersAsStringArray(agent_feedback_table));
        social_marketplace_common::addRowFromDoubleVector(temp_table, social_marketplace_common::getRowAsDoubleArray(agent_feedback_table, next_row_index));

        social_marketplace_common::updateLastRowIndex(current_row_index, current_agent_name, next_row_index);

    } catch (const std::exception& e) {
        std::cerr << "Error reading CSV for agent " << current_agent_name << ": " << e.what() << std::endl;
    }

    // Adjust the agent's parameters based on feedback, using a small learning factor.
    social_marketplace_common::Table new_agent_parameter_table = social_marketplace_common::compareAndTransformTables(temp_table, actual_agent_call_table, 0.1);
    result->feedback_values = social_marketplace_common::getRowAsDoubleArray(new_agent_parameter_table, 0);

    // If the goal is canceled mid-execution, return the canceled state.
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Publish feedback updates and mark the goal as successful.
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Published feedback");

    if (rclcpp::ok()) {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}


  rclcpp_action::Server<FeedbackCollection>::SharedPtr action_server_;
};

}  // namespace action_social_marketplace_feedback_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(action_social_marketplace_feedback_cpp::FeedbackCollectionActionServer)