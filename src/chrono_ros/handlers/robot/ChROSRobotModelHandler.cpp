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

#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include <filesystem>

namespace chrono {
namespace ros {

ChROSRobotModelHandler::ChROSRobotModelHandler(double update_rate,
                                               const std::string& robot_model_filename,
                                               const std::string& topic_name)
    : ChROSHandler(update_rate), m_robot_model(LoadRobotModel(robot_model_filename)), m_topic_name(topic_name) {}

bool ChROSRobotModelHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    m_publisher = node->create_publisher<std_msgs::msg::String>(m_topic_name, qos);

    m_msg.data = m_robot_model;

    return true;
}

void ChROSRobotModelHandler::Tick(double time) {
    m_publisher->publish(m_msg);
}

std::string ChROSRobotModelHandler::LoadRobotModel(const std::string& filename) {
    // The robot model files used in chrono are relative to the chrono data directory
    // ROS expects the file to be absolute, so we'll convert that here
    auto robot_model_path = std::filesystem::canonical(filename).parent_path().string();

    const std::string to_replace = "filename=\"./";
    const std::string replacement = "filename=\"file://" + robot_model_path + "/";

    // Read input file into XML string
    std::string line;
    std::string robot_model;
    std::fstream file(filename, std::fstream::in);
    while (std::getline(file, line)) {
        size_t pos = line.find(to_replace);
        if (pos != std::string::npos)
            line.replace(pos, to_replace.size(), replacement);

        robot_model += (line + "\n");
    }
    file.close();

    return robot_model;
}

}  // namespace ros
}  // namespace chrono
