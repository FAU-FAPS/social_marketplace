#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "social_marketplace_interfaces/msg/scene_parameters.hpp"
#include "social_marketplace_interfaces/srv/trigger_scene_analysis.hpp"
#include "social_marketplace_common/table_functions.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>

using namespace social_marketplace_common;

//This node simulates a scene analysis system by reading a predefined CSV file containing scene parameters and publishing them upon request.
//It provides a service (`/trigger_scene_analysis`) that, when triggered, retrieves scene parameters from the dataset and publishes them to the `/scene_parameters` topic.
//If the CSV file is not found, a default dataset is initialized to ensure functionality.
//The scene data is published sequentially, cycling back to the beginning when all entries have been processed.

class MockScenePublisher : public rclcpp::Node
{
public:
  // Constructor
  MockScenePublisher()
  : Node("mock_scene_publisher") // Initialize the node with the name "mock_scene_publisher"
  {
    using namespace std::placeholders;

    // Create a publisher to publish scene parameters on the "scene_parameters" topic.
    publisher_ = this->create_publisher<social_marketplace_interfaces::msg::SceneParameters>("scene_parameters", 1);

    // Create a service "trigger_scene_analysis" that listens for scene analysis triggers.
    service_ = this->create_service<social_marketplace_interfaces::srv::TriggerSceneAnalysis>(
        "trigger_scene_analysis", std::bind(&MockScenePublisher::analyse, this, _1, _2));

    // Initialize counters to keep track of scene updates.
    scene_count = 0;
    table_position_count = 0;

    // Locate and load scene data from a CSV file.
    std::string package_name = "social_marketplace_scene_analysis"; 
    std::string package_share_directory;

    try {
        package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    } catch (std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Package '%s' could not be found: %s", package_name.c_str(), e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Package Path is: %s", package_share_directory.c_str());

    std::string config_directory = package_share_directory + "/config/";
    std::string file_path = config_directory + "scene_data.csv";

    try {
        // Attempt to read the CSV file containing scene parameters.
        scene_table = social_marketplace_common::readCSV(file_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "File '%s' was not found or could not be read: %s", file_path.c_str(), e.what());
    
        // Set default scene parameters to prevent failure.
        social_marketplace_common::resetTableWithNewHeaders(scene_table, {"age", "profession_level", "emotional_state"});
        social_marketplace_common::addRowFromDoubleVector(scene_table, {2.5, 2.5, 2.5});
    }

    // Display the first row of the table for debugging and verification.
    social_marketplace_common::printTable(scene_table, table_position_count);
  }


private:

    // Handles incoming service requests to trigger scene analysis.
    // - If `trigger` is `true`, retrieves the next scene parameters from the dataset and publishes them.
    // - If the end of the dataset is reached, it cycles back to the first row.

    void analyse(const std::shared_ptr<social_marketplace_interfaces::srv::TriggerSceneAnalysis::Request> request,
                 std::shared_ptr<social_marketplace_interfaces::srv::TriggerSceneAnalysis::Response> response) {
        if (request->trigger) {
            auto message = social_marketplace_interfaces::msg::SceneParameters();
            message.scene_id = ++scene_count; // Increment scene ID for tracking.

            // Reset the table position counter if all rows have been processed.
            if (static_cast<size_t>(table_position_count) == scene_table.rows.size()) {
                table_position_count = 0;
            }

            // Extract scene parameters and populate the message.
            message.parameter_names = scene_table.headers;
            for (const auto& header : scene_table.headers) {
                message.parameter_values.push_back(std::stod(scene_table.rows[table_position_count][header]));
            }

            // Publish the scene parameters to the topic.
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published scene parameters");

            // Update the service response with the scene ID.
            response->scene_id = message.scene_id;

            // Move to the next row for the next scene publication.
            table_position_count++;
        } else {
            // If `trigger` is `false`, do not publish any data.
            RCLCPP_INFO(this->get_logger(), "Trigger false");
            response->scene_id = -1;
        }
    }

  //rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<social_marketplace_interfaces::srv::TriggerSceneAnalysis>::SharedPtr service_;
  rclcpp::Publisher<social_marketplace_interfaces::msg::SceneParameters>::SharedPtr publisher_;
   
  // Scene count tracker to assign unique scene IDs.
  int scene_count;

  // Keeps track of the current row in the dataset to ensure sequential publication.
  int table_position_count;

  // Table structure holding scene parameter data from the CSV file.
  Table scene_table;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockScenePublisher>());
  rclcpp::shutdown();
  return 0;
}