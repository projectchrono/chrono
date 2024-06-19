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

void ChROSBodyHandler::PublishPose(double time) {
    auto pos = m_body->GetPos();
    auto rot = m_body->GetRot();

    m_pose_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

    m_pose_msg.pose.position.x = pos[0];
    m_pose_msg.pose.position.y = pos[1];
    m_pose_msg.pose.position.z = pos[2];

    m_pose_msg.pose.orientation.x = rot[0];
    m_pose_msg.pose.orientation.y = rot[1];
    m_pose_msg.pose.orientation.z = rot[2];
    m_pose_msg.pose.orientation.w = rot[3];

    m_pose_publisher->publish(m_pose_msg);
}

void ChROSBodyHandler::PublishTwist(double time) {
    auto lin_vel = m_body->GetPosDt();
    auto ang_vel = m_body->GetAngVelLocal();

    m_twist_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

    m_twist_msg.twist.linear.x = lin_vel[0];
    m_twist_msg.twist.linear.y = lin_vel[1];
    m_twist_msg.twist.linear.z = lin_vel[2];

    m_twist_msg.twist.angular.x = ang_vel[0];
    m_twist_msg.twist.angular.y = ang_vel[1];
    m_twist_msg.twist.angular.z = ang_vel[2];

    m_twist_publisher->publish(m_twist_msg);
}

void ChROSBodyHandler::PublishAccel(double time) {
    auto lin_acc = m_body->GetPosDt2();
    auto ang_acc = m_body->GetAngAccLocal();

    m_accel_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

    m_accel_msg.accel.linear.x = lin_acc[0];
    m_accel_msg.accel.linear.y = lin_acc[1];
    m_accel_msg.accel.linear.z = lin_acc[2];

    m_accel_msg.accel.angular.x = ang_acc[0];
    m_accel_msg.accel.angular.y = ang_acc[1];
    m_accel_msg.accel.angular.z = ang_acc[2];

    m_accel_publisher->publish(m_accel_msg);
}

void ChROSBodyHandler::Tick(double time) {
    PublishPose(time);
    PublishTwist(time);
    PublishAccel(time);
}

}  // namespace ros
}  // namespace chrono
