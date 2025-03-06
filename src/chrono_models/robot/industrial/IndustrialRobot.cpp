// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Base class for industrial robotics models.
//
// =============================================================================

#include "IndustrialRobot.h"

namespace chrono {
namespace industrial {

void IndustrialRobot::SetSetpoints(const ChVectorDynamic<>& setpoints, double t) {
    size_t num_setpoints = setpoints.size();
    if (num_setpoints != m_motfunlist.size()) {
        std::cerr << "Invalid setpoints size" << std::endl;
        throw std::invalid_argument("Invalid setpoints size");
    }

    for (auto i = 0; i < num_setpoints; ++i) {
        m_motfunlist[i]->SetSetpoint(setpoints[i], t);
    }
}

void IndustrialRobot::SetSetpoints(double setpoint, double t) {
    for (int i = 0; i < m_motfunlist.size(); ++i)
        m_motfunlist[i]->SetSetpoint(setpoint, t);
}

void IndustrialRobot::SetMotorsDisabled(bool disable) {
    m_motors_disabled = disable;
    for (const auto& motor : m_motorlist) {
        motor->SetDisabled(m_motors_disabled);
    }
}

void IndustrialRobot::SetBaseFrame(const ChFramed& base_frame) {
    for (auto& body : m_bodylist)
        body->ConcatenatePreTransformation(ChFrameMoving<>(base_frame));
    m_sys->Update(true);
}

AssemblyAnalysis::ExitFlag IndustrialRobot::SetPoseTCP(const ChFramed& target_frame, unsigned int max_iters) {
    bool original_motors_disabled = m_motors_disabled;

    // Temporary disable motors to guide end-effector
    if (!original_motors_disabled)
        SetMotorsDisabled(true);

    // Add temporary guide link
    auto link_teach = chrono_types::make_shared<ChLinkMateFix>();
    link_teach->Initialize(m_end_effector, m_base, false, m_marker_TCP->GetAbsFrame(), target_frame);
    m_sys->Add(link_teach);
    auto exit_flag = m_sys->DoAssembly(AssemblyAnalysis::Level::POSITION, max_iters);

    // Remove temporary link
    m_sys->Remove(link_teach);

    // Update motor setpoints and restore original activation state
    SetSetpoints(GetMotorsPos(), m_sys->GetChTime());
    SetMotorsDisabled(original_motors_disabled);

    return exit_flag;
}

void IndustrialRobot::SetColor(const ChColor& col) {
    for (unsigned int i = 0; i < m_bodylist.size(); ++i)
        if (m_bodylist[i]->GetVisualModel())
            for (unsigned int j = 0; j < m_bodylist[i]->GetVisualModel()->GetNumShapes(); ++j)
                m_bodylist[i]->GetVisualShape(j)->SetColor(col);
}

void IndustrialRobot::AttachBody(std::shared_ptr<ChBody> body_attach,
                                 std::shared_ptr<ChBody> robot_body,
                                 const ChFrame<>& frame) {
    m_body_attached = true;
    m_link_attach->Initialize(body_attach, robot_body, frame);
    body_attach->SetFixed(false);
    m_sys->AddLink(m_link_attach);
    m_sys->Update(true);
}

void IndustrialRobot::DetachBody(std::shared_ptr<ChBody> body_attach, bool setfix) {
    if (m_body_attached) {
        m_body_attached = false;
        m_sys->RemoveLink(m_link_attach);
        body_attach->SetFixed(setfix);
        m_sys->Update(true);
    }
}

void IndustrialRobot::UpdateEncoder() {
    m_encoder += (GetMotorsPos() - m_encoder_prev).cwiseAbs();
    m_encoder_prev = GetMotorsPos();
}

double IndustrialRobot::GetMass() const {
    double mass = 0;
    for (auto& body : m_bodylist)
        mass += body->GetMass();
    return mass;
}

void IndustrialRobot::ClearShapes() {
    for (auto& body : m_bodylist)
        body->GetVisualModel()->Clear();
}

std::shared_ptr<ChLinkMotorRotationAngle> IndustrialRobot::CreateMotorRotationAngle(
    ChSystem* sys,
    std::shared_ptr<ChBody> body1,
    std::shared_ptr<ChBody> body2,
    const ChFramed& frame,
    std::shared_ptr<ChFunction> motfun) {
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body1, body2, frame);
    if (motfun) {
        motor->SetMotorFunction(motfun);
    }
    sys->Add(motor);
    return motor;
}

std::shared_ptr<ChLinkMotorLinearPosition> IndustrialRobot::CreateMotorLinearPosition(
    ChSystem* sys,
    std::shared_ptr<ChBody> body1,
    std::shared_ptr<ChBody> body2,
    const ChFramed& frame,
    std::shared_ptr<ChFunction> motfun) {
    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->Initialize(body1, body2, frame);
    if (motfun) {
        motor->SetMotorFunction(motfun);
    }
    sys->Add(motor);
    return motor;
}

void IndustrialRobot::CreatePassiveLinks() {
    for (const auto& motor : m_motorlist) {
        std::shared_ptr<ChBodyFrame> body1(motor->GetBody1(), [](ChBodyFrame*) {});  // provide empy deleter function
        std::shared_ptr<ChBodyFrame> body2(motor->GetBody2(), [](ChBodyFrame*) {});  // provide empy deleter function
        ChFramed absframe(motor->GetFrame2Abs());

        if (std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(motor)) {
            std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(motor)->SetGuideConstraint(
                ChLinkMotorLinear::GuideConstraint::FREE);
            auto passive_link = chrono_types::make_shared<ChLinkMatePrismatic>();
            passive_link->SetName(motor->GetName() + "_passive");
            passive_link->Initialize(body1, body2, absframe);
            m_sys->Add(passive_link);
        } else if (std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(motor)) {
            std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(motor)->SetSpindleConstraint(
                ChLinkMotorRotation::SpindleConstraint::FREE);
            auto passive_link = chrono_types::make_shared<ChLinkMateRevolute>();
            passive_link->SetName(motor->GetName() + "_passive");
            passive_link->Initialize(body1, body2, absframe);
            m_sys->Add(passive_link);
        }
    }
}

}  // end namespace industrial
}  // end namespace chrono