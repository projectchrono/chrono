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
// Handler responsible for publishing information about a ChBody
//
// =============================================================================

#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

namespace chrono {
namespace ros {

ChROSBodyHandler::ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body, const std::string& topic)
    : ChROSHandler(update_rate), m_body(body), m_topic(topic) {}

bool ChROSBodyHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();
    if (!node) {
        return true;  // IPC mode - no node in main process
    }

    auto pose_topic = m_topic + "/pose";
    auto twist_topic = m_topic + "/twist";
    auto accel_topic = m_topic + "/accel";
    
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, pose_topic) ||
        !ChROSHandlerUtilities::CheckROSTopicName(interface, twist_topic) ||
        !ChROSHandlerUtilities::CheckROSTopicName(interface, accel_topic)) {
        return false;
    }

    m_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1);
    m_twist_publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 1);
    m_accel_publisher = node->create_publisher<geometry_msgs::msg::AccelStamped>(accel_topic, 1);

    m_pose_msg.header.frame_id = m_body->GetName();
    m_twist_msg.header.frame_id = m_body->GetName();
    m_accel_msg.header.frame_id = m_body->GetName();

    return true;
}

std::vector<uint8_t> ChROSBodyHandler::GetSerializedData(double time) {
    // Extract body state from Chrono into plain data struct (no ROS symbols)
    ChROSBodyData data;
    
    // Copy body name and topic safely
    const std::string& body_name = m_body->GetName();
    strncpy(data.body_name, body_name.c_str(), sizeof(data.body_name) - 1);
    data.body_name[sizeof(data.body_name) - 1] = '\0';
    
    strncpy(data.topic_prefix, m_topic.c_str(), sizeof(data.topic_prefix) - 1);
    data.topic_prefix[sizeof(data.topic_prefix) - 1] = '\0';
    
    // Extract pose
    auto pos = m_body->GetPos();
    auto rot = m_body->GetRot();
    data.pos_x = pos.x();
    data.pos_y = pos.y();
    data.pos_z = pos.z();
    data.rot_w = rot.e0();
    data.rot_x = rot.e1();
    data.rot_y = rot.e2();
    data.rot_z = rot.e3();
    
    // Extract velocity
    auto lin_vel = m_body->GetPosDt();
    auto ang_vel = m_body->GetAngVelLocal();
    data.lin_vel_x = lin_vel.x();
    data.lin_vel_y = lin_vel.y();
    data.lin_vel_z = lin_vel.z();
    data.ang_vel_x = ang_vel.x();
    data.ang_vel_y = ang_vel.y();
    data.ang_vel_z = ang_vel.z();
    
    // Extract acceleration
    auto lin_acc = m_body->GetPosDt2();
    auto ang_acc = m_body->GetAngAccLocal();
    data.lin_acc_x = lin_acc.x();
    data.lin_acc_y = lin_acc.y();
    data.lin_acc_z = lin_acc.z();
    data.ang_acc_x = ang_acc.x();
    data.ang_acc_y = ang_acc.y();
    data.ang_acc_z = ang_acc.z();
    
    // Serialize to byte vector
    std::vector<uint8_t> bytes(sizeof(ChROSBodyData));
    std::memcpy(bytes.data(), &data, sizeof(ChROSBodyData));
    return bytes;
}

void ChROSBodyHandler::PublishFromSerialized(const std::vector<uint8_t>& data, 
                                             std::shared_ptr<ChROSInterface> interface) {
    if (data.size() != sizeof(ChROSBodyData)) return;
    
    const ChROSBodyData* body_data = reinterpret_cast<const ChROSBodyData*>(data.data());
    
    // Update timestamps
    rclcpp::Time ros_time = interface->GetNode()->get_clock()->now();
    m_pose_msg.header.stamp = ros_time;
    m_twist_msg.header.stamp = ros_time;
    m_accel_msg.header.stamp = ros_time;
    
    // Update Pose
    m_pose_msg.pose.position.x = body_data->pos_x;
    m_pose_msg.pose.position.y = body_data->pos_y;
    m_pose_msg.pose.position.z = body_data->pos_z;
    m_pose_msg.pose.orientation.w = body_data->rot_w;
    m_pose_msg.pose.orientation.x = body_data->rot_x;
    m_pose_msg.pose.orientation.y = body_data->rot_y;
    m_pose_msg.pose.orientation.z = body_data->rot_z;
    m_pose_publisher->publish(m_pose_msg);
    
    // Update Twist
    m_twist_msg.twist.linear.x = body_data->lin_vel_x;
    m_twist_msg.twist.linear.y = body_data->lin_vel_y;
    m_twist_msg.twist.linear.z = body_data->lin_vel_z;
    m_twist_msg.twist.angular.x = body_data->ang_vel_x;
    m_twist_msg.twist.angular.y = body_data->ang_vel_y;
    m_twist_msg.twist.angular.z = body_data->ang_vel_z;
    m_twist_publisher->publish(m_twist_msg);
    
    // Update Accel
    m_accel_msg.accel.linear.x = body_data->lin_acc_x;
    m_accel_msg.accel.linear.y = body_data->lin_acc_y;
    m_accel_msg.accel.linear.z = body_data->lin_acc_z;
    m_accel_msg.accel.angular.x = body_data->ang_acc_x;
    m_accel_msg.accel.angular.y = body_data->ang_acc_y;
    m_accel_msg.accel.angular.z = body_data->ang_acc_z;
    m_accel_publisher->publish(m_accel_msg);
}

}  // namespace ros
}  // namespace chrono
