#ifndef CH_ROS_INTERFACE_H
#define CH_ROS_INTERFACE_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

namespace chrono {
namespace ros {

/// brief This class handles the API interface between Chrono and ROS. It contains a
/// rclcpp::Node which is accessible through GetNode().
class ChROSInterface {
  public:
    ChROSInterface(const std::string node_name = "chrono_ros_node");

    /// Initialize the underlying ROS 2 node.
    void Initialize();

    /// Tick once. Will basically just call rclcpp::spin_some()
    void SpinSome();

    /// Retrieve the ROS node. Use this API to create a publisher or subscriber or any
    /// other ROS component.
    rclcpp::Node::SharedPtr GetNode() { return m_node; }

    /// Get the namespace to append to topic/node names
    const std::string& GetNamespace() { return m_node_name; }

  private:
    const std::string m_node_name;

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Executor::SharedPtr m_executor;
};

}  // namespace ros
}  // namespace chrono

#endif