#include "chrono_ros/ChSimulationManager.h"

namespace chrono {
namespace ros {

ChSimulationManager::ChSimulationManager() : Node("ChSimulationManager") {
    m_started.store(false);

    m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // m_subscription = this->create_subscription<std_msgs::msg::String>(
    // "topic", 10, std::bind(&ChSimulationManager::topic_callback, this, _1));

    m_service = this->create_service<chrono_ros_interfaces::srv::ChStartSimulation>(
        "start", std::bind(&ChSimulationManager::start_callback, this, _1, _2));
}

void ChSimulationManager::Publish() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(m_count++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    m_publisher->publish(message);
}

void ChSimulationManager::WaitForStartCommand() {
    std::cout << "Waiting for ChStartSimulation command..." << std::endl;

    // Use a Rate object to control the frequency of checking for requests
    rclcpp::Rate rate(10);  // 10 Hz

    while (rclcpp::ok()) {
        // Process any callbacks for the node (including service requests)
        rclcpp::spin_some(this->get_node_base_interface());

        // Break if the start command has been received
        if (m_started.load())
            break;

        // Sleep according to the rate to avoid busy-waiting
        rate.sleep();
    }
}

void ChSimulationManager::topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void ChSimulationManager::start_callback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response) {
    std::cout << "Received ChStartSimulation command..." << std::endl;
    m_started.store(true);
    response->success = true;
}

}  // namespace ros
}  // namespace chrono