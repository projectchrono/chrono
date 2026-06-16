// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Handler that publishes the state of a ChBody.
//
// =============================================================================

#include "chrono_ros/handlers/ChROSBodyHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"

namespace chrono {
namespace ros {

ChROSBodyHandler::ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body, const std::string& topic)
    : ChROSHandler(update_rate), m_body(body), m_topic(topic) {}

bool ChROSBodyHandler::Initialize(ChROSBridge& bridge) {
    m_pose_publisher = bridge.CreatePublisher(m_topic + "/pose", "geometry_msgs/msg/PoseStamped");
    m_twist_publisher = bridge.CreatePublisher(m_topic + "/twist", "geometry_msgs/msg/TwistStamped");
    m_accel_publisher = bridge.CreatePublisher(m_topic + "/accel", "geometry_msgs/msg/AccelStamped");
    return true;
}

void ChROSBodyHandler::PublishPose(double time) {
    const auto& pos = m_body->GetPos();
    const auto& rot = m_body->GetRot();

    auto msg = m_pose_publisher->NewMessage();
    msg.SetString("header.frame_id", m_body->GetName());
    msg.SetTime("header.stamp", time);

    msg.SetDouble("pose.position.x", pos.x());
    msg.SetDouble("pose.position.y", pos.y());
    msg.SetDouble("pose.position.z", pos.z());

    // ROS geometry_msgs/Quaternion is (x, y, z, w); Chrono is (e0=w, e1=x, e2=y,
    // e3=z). NOTE: the Chrono 9.0 ChROSBodyHandler mapped these off-by-one
    // (orientation.x = e0, ... orientation.w = e3), publishing a wrong rotation;
    // corrected here to match the ROS convention and the TF handler.
    msg.SetDouble("pose.orientation.x", rot.e1());
    msg.SetDouble("pose.orientation.y", rot.e2());
    msg.SetDouble("pose.orientation.z", rot.e3());
    msg.SetDouble("pose.orientation.w", rot.e0());

    m_pose_publisher->Publish(msg);
}

void ChROSBodyHandler::PublishTwist(double time) {
    const auto& lin_vel = m_body->GetPosDt();
    auto ang_vel = m_body->GetAngVelLocal();

    auto msg = m_twist_publisher->NewMessage();
    msg.SetString("header.frame_id", m_body->GetName());
    msg.SetTime("header.stamp", time);

    msg.SetDouble("twist.linear.x", lin_vel.x());
    msg.SetDouble("twist.linear.y", lin_vel.y());
    msg.SetDouble("twist.linear.z", lin_vel.z());

    msg.SetDouble("twist.angular.x", ang_vel.x());
    msg.SetDouble("twist.angular.y", ang_vel.y());
    msg.SetDouble("twist.angular.z", ang_vel.z());

    m_twist_publisher->Publish(msg);
}

void ChROSBodyHandler::PublishAccel(double time) {
    const auto& lin_acc = m_body->GetPosDt2();
    auto ang_acc = m_body->GetAngAccLocal();

    auto msg = m_accel_publisher->NewMessage();
    msg.SetString("header.frame_id", m_body->GetName());
    msg.SetTime("header.stamp", time);

    msg.SetDouble("accel.linear.x", lin_acc.x());
    msg.SetDouble("accel.linear.y", lin_acc.y());
    msg.SetDouble("accel.linear.z", lin_acc.z());

    msg.SetDouble("accel.angular.x", ang_acc.x());
    msg.SetDouble("accel.angular.y", ang_acc.y());
    msg.SetDouble("accel.angular.z", ang_acc.z());

    m_accel_publisher->Publish(msg);
}

void ChROSBodyHandler::Tick(double time) {
    PublishPose(time);
    PublishTwist(time);
    PublishAccel(time);
}

}  // namespace ros
}  // namespace chrono
