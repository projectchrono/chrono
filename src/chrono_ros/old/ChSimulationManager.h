#ifndef CH_SIMULATION_MANAGER_H
#define CH_SIMULATION_MANAGER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "chrono_ros_interfaces/srv/ch_start_simulation.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace chrono {
namespace ros {

/// Managers the ROS converters.
class ChSimulationManager {
  public:
    ChSimulationManager();
};

class ChSimulationManager : public rclcpp::Node {
  public:
    ChSimulationManager();

    void Publish();

    void WaitForStartCommand();

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

    void start_callback(const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
                        std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
    rclcpp::Service<chrono_ros_interfaces::srv::ChStartSimulation>::SharedPtr m_service;

    size_t m_count;

    std::atomic<bool> m_started;
};
}  // namespace ros
}  // namespace chrono

#endif
