#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <string>
#include <unordered_map>
#include <cstdio>
#include <cmath>
#include <chrono>

#include "social_marketplace_interfaces/msg/scene_parameters.hpp"
#include "social_marketplace_interfaces/msg/agent_parameters.hpp"
#include "social_marketplace_interfaces/srv/agent_registration.hpp"
#include "social_marketplace_interfaces/action/agent_interaction.hpp"
#include "std_msgs/msg/int64.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "social_marketplace_common/table_functions.hpp"

using AgentInteraction = social_marketplace_interfaces::action::AgentInteraction;
using GoalHandleAgentInteraction = rclcpp_action::ClientGoalHandle<AgentInteraction>;

using namespace social_marketplace_common;

// This node manages the bidding process in a social marketplace system.
// It listens for scene parameters, receives bid offers from agents, 
// and determines the winning bid based on the Manhattan distance metric.

class BiddingPlatform : public rclcpp::Node
{
public:
  BiddingPlatform() : Node("bidding_platform"), agent_ids(0)
  {
    using namespace std::placeholders;

        // Initialize known agents table with predefined headers
        resetTableWithNewHeaders(known_agents, {"agent_name", "agent_id"});

        // Publisher to announce new bids
        bid_publisher_ = this->create_publisher<std_msgs::msg::Int64>("bids", 1);

        // Subscribe to scene parameters to understand the current context
        scene_parameter_subscriber_ = this->create_subscription<social_marketplace_interfaces::msg::SceneParameters>(
            "scene_parameters", 1, std::bind(&BiddingPlatform::scene_parameter_callback, this, _1));

        // Subscribe to bid offers from agents
        bid_offer_subscriber_ = this->create_subscription<social_marketplace_interfaces::msg::AgentParameters>(
            "bid_offers", 20, std::bind(&BiddingPlatform::bid_offer_callback, this, _1));

        // Service to handle agent registration requests
        agent_registration_service_ = this->create_service<social_marketplace_interfaces::srv::AgentRegistration>(
            "agent_registration", std::bind(&BiddingPlatform::agent_registration, this, _1, _2));
  }

private:
    // Handles incoming scene parameters from the scene analysis node.
    // Stores the received parameters in a table and publishes the scene ID.
  void scene_parameter_callback(const social_marketplace_interfaces::msg::SceneParameters &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received Scene Parameters for the scene_id '%ld'", msg.scene_id);
        
        resetTableWithNewHeaders(actual_scene_parameter_table, msg.parameter_names);
        addRowFromDoubleVector(actual_scene_parameter_table, msg.parameter_values);

        // Print the scene parameters to verify data reception
        printTable(actual_scene_parameter_table, 0);

        auto message = std_msgs::msg::Int64();
        message.data = msg.scene_id;
        
        // Prepare bid storage table
        std::vector<std::string> additionalHeaders = {"agent_id", "agent_name"};
        resetTableWithNewHeaders(actual_offers, additionalHeaders, msg.parameter_names);

        // Notify all agents that bidding for the scene is open
        bid_publisher_->publish(message);
  }

    // Handles incoming bid offers from registered agents.
    // Stores bid data and determines when all agents have submitted their bids.
  void bid_offer_callback(const social_marketplace_interfaces::msg::AgentParameters &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received Scene Parameters from the Agent ID '%ld'", msg.agent_id);
    
    std::vector<std::string> additionalElements_1 = {std::to_string(msg.agent_id), msg.agent_name};
    std::vector<double> additionalElements_2 = msg.parameter_values;

    addRowFromMixedStringDoubleVectors(actual_offers, additionalElements_1, additionalElements_2);

    // Check if all known agents have submitted their bids
    if (areFirstColumnElementsContained(known_agents, actual_offers))
    {
        RCLCPP_INFO(this->get_logger(), "Received Bids from All Registered Agents");

        int winning_id = calculate_winning_agent_id_offer(actual_scene_parameter_table, actual_offers);
        if (winning_id > -1)
        {
            // Retrieve parameters for the winning agent
            int temp_agent_row = findRowByValueInColumn(actual_offers, std::to_string(winning_id), 0);
            auto temp_agent_parameters = getRowAsStringArray(actual_offers, temp_agent_row);

            // Create goal message for agent interaction
            auto goal_msg = AgentInteraction::Goal();
            goal_msg.agent_id = winning_id;
            goal_msg.agent_name = temp_agent_parameters[1];

            RCLCPP_INFO(this->get_logger(), "Sending goal to AgentInteractionServer");

            auto send_goal_options = rclcpp_action::Client<AgentInteraction>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&BiddingPlatform::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&BiddingPlatform::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(&BiddingPlatform::result_callback, this, std::placeholders::_1);

            for (const auto &element : agent_interaction_action_clients)
            {
                if (std::get<0>(element) == winning_id)
                {
                    const rclcpp_action::Client<AgentInteraction>::SharedPtr &temp_action_client = std::get<1>(element);
                    while (!temp_action_client->wait_for_action_server(std::chrono::seconds(10)))
                    {
                        RCLCPP_WARN(this->get_logger(), "Waiting for the Agent Interaction Service");
                    }
                    temp_action_client->async_send_goal(goal_msg, send_goal_options);
                    break;
                }
            }
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Still Did Not Receive Bids from All Agents");
    }
  }

  // Registers new agents and assigns a unique agent ID.
  void agent_registration(const std::shared_ptr<social_marketplace_interfaces::srv::AgentRegistration::Request> request,
                          std::shared_ptr<social_marketplace_interfaces::srv::AgentRegistration::Response> response)
  {
    if (findRowByValue(known_agents, request->agent_name.c_str()) < 0)
    {
        RCLCPP_INFO(this->get_logger(), "Agent Registration Started for the Agent with the Name: %s", request->agent_name.c_str());
        
        addRowFromStringVector(known_agents, {std::to_string(agent_ids), request->agent_name.c_str()});

        std::string temp_action_name = request->agent_name.c_str();
        temp_action_name += "_interaction";

        agent_interaction_action_clients.emplace_back(agent_ids, rclcpp_action::create_client<AgentInteraction>(this, temp_action_name));

        response->agent_id = agent_ids;
        agent_ids++;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Agent already known");
        response->agent_id = -1;
    }
  }

    // Determines the winning bid based on Manhattan distance calculations.
    int calculate_winning_agent_id_offer(Table& table_scene_parameters, Table& table_agent_offers) {
        int winning_agent_id = -1;
        double winning_manhattan_distance = -1;
        double temp_distance = -1;
        std::vector<double> current_scene_parameters = getRowAsDoubleArray(table_scene_parameters, 0);
        for (int i = 0; i < getRowCount(table_agent_offers); i++) {
            std::vector<double> temp_agent_offer = getRowValuesFromStartColumn(table_agent_offers, i, 2);
            temp_distance = calculate_manhattan_distance(current_scene_parameters, temp_agent_offer);
            if (temp_distance < winning_manhattan_distance || winning_manhattan_distance == -1) {
                winning_manhattan_distance = temp_distance;
                temp_agent_offer = getRowValuesFromStartColumn(table_agent_offers, i, 0);
                winning_agent_id = temp_agent_offer[0];
            }
        }
        RCLCPP_INFO(this->get_logger(), "Calculated Winning Agent with the ID: %d and the Distance: %f", winning_agent_id, winning_manhattan_distance);
        return winning_agent_id;
    }

    double calculate_manhattan_distance(const std::vector<double>& vec1, const std::vector<double>& vec2) {
        if (vec1.size() != vec2.size()) {
            throw std::invalid_argument("Vectors must have the same size");
        }
        double distance = 0.0;
        for (size_t i = 0; i < vec1.size(); ++i) {
            distance += std::abs(vec1[i] - vec2[i]);
        }
        return distance;
    }

    // Callback function for the goal response of agent interaction
void goal_response_callback(const GoalHandleAgentInteraction::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

// Callback function for the feedback of agent interaction
void feedback_callback(
    GoalHandleAgentInteraction::SharedPtr,
    const std::shared_ptr<const AgentInteraction::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Feedback received: %s", feedback->interaction_feedback.c_str());
}

// Callback function for the result of agent interaction
void result_callback(const GoalHandleAgentInteraction::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Agent Interaction succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Agent Interaction aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Agent Interaction canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}


  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr bid_publisher_;
  rclcpp::Subscription<social_marketplace_interfaces::msg::SceneParameters>::SharedPtr scene_parameter_subscriber_;
  rclcpp::Subscription<social_marketplace_interfaces::msg::AgentParameters>::SharedPtr bid_offer_subscriber_;
  rclcpp::Service<social_marketplace_interfaces::srv::AgentRegistration>::SharedPtr agent_registration_service_;

  std::vector<std::tuple<int, rclcpp_action::Client<AgentInteraction>::SharedPtr>> agent_interaction_action_clients;

// Member variables for the class
  Table actual_scene_parameter_table;
  Table known_agents;
  Table actual_offers;
  int agent_ids;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BiddingPlatform>());
  rclcpp::shutdown();
  return 0;
}
