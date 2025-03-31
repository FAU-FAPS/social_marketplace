#ifndef PARAMETER_ADJUSTMENT_CLIENT_HPP
#define PARAMETER_ADJUSTMENT_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "social_marketplace_interfaces/action/parameter_adjustment.hpp"


using ParameterAdjustment = social_marketplace_interfaces::action::ParameterAdjustment;
using GoalHandleParameterAdjustment = rclcpp_action::ClientGoalHandle<social_marketplace_interfaces::action::ParameterAdjustment>;

class ParameterAdjustmentClient : public rclcpp::Node
{
public:
    ParameterAdjustmentClient();

    bool is_goal_done() const;

    void send_goal(int64_t agent_id, 
                 const std::vector<std::string>& parameter_names, 
                 const std::vector<double>& initial_parameter_values,
                 const std::vector<std::string>& feedback_values);

private:
    rclcpp_action::Client<social_marketplace_interfaces::action::ParameterAdjustment>::SharedPtr client_ptr_;
    bool goal_done_;
    rclcpp_action::ClientGoalHandle<social_marketplace_interfaces::action::ParameterAdjustment>::SharedPtr goal_handle_;
    std::vector<std::string> current_parameter_names_; // Add a member variable to store the parameter name
    
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<social_marketplace_interfaces::action::ParameterAdjustment>::SharedPtr & goal_handle);

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<social_marketplace_interfaces::action::ParameterAdjustment>::SharedPtr,
        const std::shared_ptr<const social_marketplace_interfaces::action::ParameterAdjustment::Feedback> feedback);

    void result_callback(const rclcpp_action::ClientGoalHandle<social_marketplace_interfaces::action::ParameterAdjustment>::WrappedResult & result);
};

#endif // PARAMETER_ADJUSTMENT_CLIENT_HPP
