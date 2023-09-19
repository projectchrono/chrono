// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// The API interface between ROS/rclcpp and Chrono
//
// =============================================================================

#ifndef CH_ROS_INTERFACE_H
#define CH_ROS_INTERFACE_H

#include "chrono_ros/ChApiROS.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

#include <memory>

namespace chrono {
namespace ros {

/// @addtogroup ros_core
/// @{

/// This class handles the API interface between Chrono and ROS. It contains a
/// rclcpp::Node which is accessible through GetNode(). Multiple ChROSInterfaces can be created to support multiple nodes.
class CH_ROS_API ChROSInterface {
  public:
    /// Constructor for the ChROSInterface class. 
    /// @param node_name the name to set to the created node. ROS will throw an error if the node name is identical to previously created nodes. Defaults to "chrono_ros_node".
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

    static bool m_has_initialized = false

    rclcpp::Node::SharedPtr m_node;
    static rclcpp::Executor::SharedPtr m_executor;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif