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
// Handler responsible for running the InterativeMarkerServer
//
// =============================================================================

#include "chrono_ros/handlers/ChROSInteractiveMarkerHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

using namespace chrono;

namespace chrono {
namespace ros {

ChROSInteractiveMarkerHandler::ChROSInteractiveMarkerHandler(double update_rate) : ChROSHandler(update_rate) {}

bool ChROSInteractiveMarkerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    m_marker_server =
        std::make_unique<interactive_markers::InteractiveMarkerServer>("interactive_markers", interface->GetNode(), 1);

    for (auto& [name, marker_link] : m_marker_links)
        m_marker_server->insert(marker_link.marker, marker_link.callback);
    m_marker_server->applyChanges();

    return true;
}

void ChROSInteractiveMarkerHandler::AddMotor(std::shared_ptr<ChLinkMotor> motor,
                                             Axis axis,
                                             const std::string& frame_id) {
    auto marker = visualization_msgs::msg::InteractiveMarker();
    marker.header.frame_id = frame_id;
    marker.name = motor->GetName();
    marker.scale = 0.3;

    auto control = visualization_msgs::msg::InteractiveMarkerControl();
    auto motor_orientation = motor->GetLinkAbsoluteCoords().rot;
    control.orientation.w = motor_orientation.e0();
    control.orientation.x = motor_orientation.e1();
    control.orientation.y = motor_orientation.e2();
    control.orientation.z = motor_orientation.e3();
    control.always_visible = true;
    control.name = motor->GetNameString() + "_control";

    if (std::dynamic_pointer_cast<ChLinkMotorRotation>(motor)) {
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    } else if (std::dynamic_pointer_cast<ChLinkMotorLinear>(motor)) {
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    } else {
        throw ChException("Unsupported motor type");
    }

    marker.controls.push_back(control);

    auto feedback_callback = std::bind(&ChROSInteractiveMarkerHandler::ProcessFeedback, this, std::placeholders::_1);

    m_marker_links[motor->GetName()] = {axis, 0.0, motor, marker, feedback_callback};
}

#ifdef CHRONO_PARSERS_URDF
void ChROSInteractiveMarkerHandler::AddURDF(chrono::parsers::ChParserURDF& parser) {
    auto model = parser.GetModelTree();

    std::vector<urdf::LinkSharedPtr> links;
    model->getLinks(links);

    for (const auto& link : links) {
        auto joint = link->parent_joint;
        if (!joint)
            continue;

        auto motor = parser.GetChMotor(joint->name);
        if (!motor)
            continue;

        // AddMotor(motor, joint->parent_link_name);
    }
}
#endif

void ChROSInteractiveMarkerHandler::ProcessFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
    if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
        return;

    auto pose = feedback->pose;
    auto pos = ChVector<>(pose.position.x, pose.position.y, pose.position.z);
    auto rot = ChQuaternion<>(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    auto& marker_link = m_marker_links[feedback->marker_name];
    if (std::dynamic_pointer_cast<ChLinkMotorRotation>(marker_link.motor)) {
        // auto angle = frame.GetRotAngle();
        // auto delta_angle = angle - func->GetSetpoint();
        // func->SetSetpoint(delta_angle, ChROSHandlerUtilities::GetChronoTime(timestamp));
    }
    if (auto linear_motor = std::dynamic_pointer_cast<ChLinkMotorLinear>(marker_link.motor)) {
        double x;
        switch (marker_link.axis) {
            case X:
                x = VECT_X.Dot(pos);
                break;
            case Y:
                x = VECT_Y.Dot(pos);
                break;
            case Z:
                x = VECT_Z.Dot(pos);
                break;
            default:
                throw ChException("Unsupported axis");
        }
        marker_link.setpoint = x;
    }
}

void ChROSInteractiveMarkerHandler::Tick(double time) {
    for (auto& [name, marker_link] : m_marker_links) {
        auto motor = marker_link.motor;
        if (auto setpoint_function = std::dynamic_pointer_cast<ChFunction_Setpoint>(motor->GetMotorFunction())) {
            std::cout << "Setting setpoint to " << marker_link.setpoint << " for " << name << std::endl;
            setpoint_function->SetSetpoint(marker_link.setpoint, time);
        }
    }
}

}  // namespace ros
}  // namespace chrono
