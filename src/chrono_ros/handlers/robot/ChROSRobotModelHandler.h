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
// Handler responsible for publishing a Robot Model to be visualized in RViz
//
// =============================================================================

#ifndef CH_ROS_ROBOT_MODEL_HANDLER_H
#define CH_ROS_ROBOT_MODEL_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_parsers/ChParserURDF.h"

#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// This handler is responsible for publishing a robot model to be visualized in RViz
/// RViz expects a string containing the robot model. We don't really need to publish this at a high rate, so it's
/// recommended to set updated_rate to a very large value. We'll be setting the QoS to be local transient, meaning late
/// joiners will still receive the message even if it's already been published.
///
/// In addition to the aforementioned logic, we'll parse the robot model string and update all filename paths to be
/// absolute. This then ensures that RViz will be able to find the meshes and textures.
class ChROSRobotModelHandler : public ChROSHandler {
  public:
    /// Constructor.
    /// The robot model will be loaded in the constructor.
    /// The topic name defaults to "/robot_description".
    ChROSRobotModelHandler(double update_rate,
                           const std::string& robot_model_filename,
                           const std::string& topic_name = "/robot_description");

    /// Initializes the handler. This creates the publisher for the robot model topic.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Publishes the robot model string. Should be called infrequently (i.e. set update_rate to some really high
    /// value).
    virtual void Tick(double time) override;

  private:
    /// Loads the robot model from the given filename and returns the string. The robot model filenames will be updated
    /// to be absolute paths.
    std::string LoadRobotModel(const std::string& filename);

  private:
    const std::string m_topic_name;   ///< name of the topic to publish to
    const std::string m_robot_model;  ///< the robot model string to publish

    std_msgs::msg::String m_msg;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif
