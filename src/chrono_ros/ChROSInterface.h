// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// The API interface between ROS/rclcpp and Chrono
//
// =============================================================================

#ifndef CH_ROS_INTERFACE_H
#define CH_ROS_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

#include "chrono_ros/ChApiROS.h"

#include <memory>
#include <chrono>

namespace chrono {
namespace ros {

/// @addtogroup ros_core
/// @{

/// This class handles the API interface between Chrono and ROS. It contains a
/// rclcpp::Node which is accessible through GetNode(). Multiple ChROSInterfaces can be created to support multiple
/// nodes.
class CH_ROS_API ChROSInterface {
  public:
    /// Constructor for the ChROSInterface class.
    /// @param node_name the name to set to the created node. ROS will throw an error if the node name is identical to
    /// previously created nodes. 
    ChROSInterface(const std::string node_name);
    
    /// Virtual destructor for polymorphism
    virtual ~ChROSInterface() = default;

    /// Initialize the underlying ROS 2 node.
    /// A SingleThreadedExecutor will be created and the node will be added to it.
    virtual void Initialize(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    /// Tick once. Will basically just call rclcpp::spin_some()
    /// NOTE: This is non-blocking. Available work will be executed, but it won't wait until it's completed if
    /// max_duration is 0 or the time since the last call to SpinSome is less than max_duration.
    virtual void SpinSome(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

    /// Retrieve the ROS node. Use this API to create a publisher or subscriber or any
    /// other ROS component.
    virtual rclcpp::Node::SharedPtr GetNode() { return m_node; }
    
    /// Get the node name (for IPC interface access)
    const std::string& GetNodeName() const { return m_node_name; }

  private:
    const std::string m_node_name;

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Executor::UniquePtr m_executor;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif