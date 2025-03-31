// Copyright (c) 2025 Sebastian Reitelshöfer, FAPS, FAU Erlangen-Nuremberg
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cstdio>
#include <chrono>

#include "social_marketplace_interfaces/msg/agent_parameters.hpp"
#include "social_marketplace_interfaces/srv/agent_registration.hpp"
#include "social_marketplace_interfaces/action/feedback_collection.hpp"
#include "social_marketplace_interfaces/action/agent_interaction.hpp"

#include "std_msgs/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "social_marketplace_common/table_functions.hpp"

namespace action_social_marketplace_agent_cpp {

// This node represents an agent in a social marketplace system.
// It registers with the marketplace, listens for bid requests, 
// sends bid offers, and participates in interactions.

class Agent_1 : public rclcpp::Node {
public:
    using FeedbackCollection = social_marketplace_interfaces::action::FeedbackCollection;
    using GoalHandleFeedbackCollection = rclcpp_action::ClientGoalHandle<FeedbackCollection>;

    using AgentInteraction = social_marketplace_interfaces::action::AgentInteraction;
    using GoalHandleAgentInteraction = rclcpp_action::ServerGoalHandle<AgentInteraction>;

    // Constructor: Initializes the agent, registers it with the marketplace, and sets up communication.
    explicit Agent_1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("agent", options), agent_id(0) { // Initialize agent ID to 0
        using namespace std::placeholders;
        using namespace std::chrono_literals;

        // Locate the package directory to find configuration files
        std::string node_name = this->get_name();
        std::string package_name = "social_marketplace_agent_examples";
        std::string package_share_directory;

        try {
            package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        } catch (std::runtime_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Package '%s' could not be found: %s", package_name.c_str(), e.what());
            return; // Exit early if the package is not found
        }

        RCLCPP_INFO(this->get_logger(), "Package Path is: %s", package_share_directory.c_str());

        // Define the path to the CSV file for loading agent parameters
        std::string config_directory = package_share_directory + "/config/";
        std::string file_path = config_directory + node_name + "_initial_parameters.csv";

        // Load agent parameters from the CSV file or set default values if not found
        try {
            actual_agent_parameter_table = social_marketplace_common::readCSV(file_path);
            RCLCPP_INFO(this->get_logger(), "Successfully loaded CSV file: %s", file_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "File '%s' was not found. Using standard agent parameters: %s", file_path.c_str(), e.what());
            social_marketplace_common::resetTableWithNewHeaders(actual_agent_parameter_table, {"age", "profession_level", "emotional_state"});
            social_marketplace_common::addRowFromDoubleVector(actual_agent_parameter_table, {2.5, 2.5, 2.5});
        }

        // Display the loaded or default agent parameters
        social_marketplace_common::printTable(actual_agent_parameter_table, 0);



        // Create a client to register the agent with the marketplace
        agent_registration_client_ = this->create_client<social_marketplace_interfaces::srv::AgentRegistration>("agent_registration");

        // Wait for the agent registration service to be available
        while (!agent_registration_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the Agent Registration Service");
        }

        // Prepare and send the agent registration request
        auto request = std::make_shared<social_marketplace_interfaces::srv::AgentRegistration::Request>();
        request->agent_name = this->get_name();

        auto result = agent_registration_client_->async_send_request(request);

        // Block until registration completes and retrieve the assigned agent ID
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            agent_id = result.get()->agent_id;
            RCLCPP_INFO(this->get_logger(), "%s Received Agent ID: %d", this->get_name(), agent_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }

        // Subscribe to bid requests and publish bid offers
        bid_subscriber_ = this->create_subscription<std_msgs::msg::Int64>("bids", 1, std::bind(&Agent_1::bid_callback, this, _1));
        bid_offer_publisher_ = this->create_publisher<social_marketplace_interfaces::msg::AgentParameters>("bid_offers", 1);

        // Create an action server for handling agent interactions
        std::string agent_name_for_action = this->get_name();
        agent_name_for_action += "_interaction";
        this->agent_interaction_action_server_ = rclcpp_action::create_server<AgentInteraction>(
            this, agent_name_for_action,
            std::bind(&Agent_1::handle_goal_agent_interaction, this, _1, _2),
            std::bind(&Agent_1::handle_cancel_agent_interaction, this, _1),
            std::bind(&Agent_1::handle_accepted_agent_interaction, this, _1)
        );
    }

private:
    // Handles incoming bid requests and responds with the agent's parameters.
    void bid_callback(const std_msgs::msg::Int64 & msg) {
        RCLCPP_INFO(this->get_logger(), "Received Bid with the ID'%ld' Publishing actual parameters...", msg.data);
        // Prepare and publish the bid offer message
        auto message = social_marketplace_interfaces::msg::AgentParameters();
        message.agent_name = this->get_name();
        message.agent_id = agent_id;
        message.parameter_names = social_marketplace_common::getHeadersAsStringArray(actual_agent_parameter_table);
        message.parameter_values = social_marketplace_common::getRowAsDoubleArray(actual_agent_parameter_table, 0);
        bid_offer_publisher_->publish(message);
    }



    rclcpp::Publisher<social_marketplace_interfaces::msg::AgentParameters>::SharedPtr bid_offer_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr bid_subscriber_;
    rclcpp::Client<social_marketplace_interfaces::srv::AgentRegistration>::SharedPtr agent_registration_client_;
    rclcpp_action::Client<FeedbackCollection>::SharedPtr feedback_collection_action_client_;
    rclcpp_action::Server<AgentInteraction>::SharedPtr agent_interaction_action_server_;

    // Action server callback for handling goal requests
    rclcpp_action::GoalResponse handle_goal_agent_interaction(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AgentInteraction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request for agent ID: %ld", goal->agent_id);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Action server callback for handling cancel requests
    rclcpp_action::CancelResponse handle_cancel_agent_interaction(const std::shared_ptr<GoalHandleAgentInteraction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Action server callback for handling accepted goals
    void handle_accepted_agent_interaction(const std::shared_ptr<GoalHandleAgentInteraction> goal_handle) {
        std::thread{std::bind(&Agent_1::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Function to execute the goal
    void execute(const std::shared_ptr<GoalHandleAgentInteraction> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<AgentInteraction::Result>();

        // Simulates positive feedback, assuming all values are between 2.5 and 5.
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Agent Interaction of Agent %s Succeeded. NOW Collecting Feedback", this->get_name());
        this->feedback_collection_action_client_ = rclcpp_action::create_client<FeedbackCollection>(this, "feedback_collection");

        if (!feedback_collection_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Prepare goal message for feedback collection
        auto goal_msg = FeedbackCollection::Goal();
        goal_msg.agent_id = agent_id;
        goal_msg.agent_name = this->get_name();
        goal_msg.parameter_names = social_marketplace_common::getHeadersAsStringArray(actual_agent_parameter_table);
        goal_msg.parameter_values = social_marketplace_common::getRowAsDoubleArray(actual_agent_parameter_table, 0);

        RCLCPP_INFO(this->get_logger(), "Sending goal to FeedbackCollectionServer");

        auto send_goal_options = rclcpp_action::Client<FeedbackCollection>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&Agent_1::goal_response_callback_feedback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&Agent_1::feedback_callback_feedback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&Agent_1::result_callback_feedback, this, std::placeholders::_1);

        feedback_collection_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // Feedback collection goal response callback
    void goal_response_callback_feedback(const GoalHandleFeedbackCollection::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // Feedback collection feedback callback
    void feedback_callback_feedback(GoalHandleFeedbackCollection::SharedPtr, const std::shared_ptr<const FeedbackCollection::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback received: %f", feedback->progress);
    }

    // Feedback collection result callback
    void result_callback_feedback(const GoalHandleFeedbackCollection::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                RCLCPP_INFO(this->get_logger(), "Feedback Collection succeeded");
                auto temp_headers = social_marketplace_common::getHeadersAsStringArray(actual_agent_parameter_table);
                social_marketplace_common::resetTableWithNewHeaders(actual_agent_parameter_table, temp_headers);
                social_marketplace_common::addRowFromDoubleVector(actual_agent_parameter_table, result.result->feedback_values);
                social_marketplace_common::printTable(actual_agent_parameter_table);
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Feedback Collection aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Feedback Collection canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    social_marketplace_common::Table actual_agent_parameter_table;
    int agent_id;
    std::string agent_name;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_social_marketplace_agent_cpp::Agent_1)