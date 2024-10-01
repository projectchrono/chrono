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

#include "ChRobot.h"

namespace chrono {
namespace industrial {

void ChRobot::SetSetpoints(const ChVectorDynamic<>& setpoints, double t) {
    for (int i = 0; i < m_motfunlist.size(); ++i)
        m_motfunlist[i]->SetSetpoint(setpoints[i], t);
}

void ChRobot::SetSetpoints(double setpoint, double t) {
    for (int i = 0; i < m_motfunlist.size(); ++i)
        m_motfunlist[i]->SetSetpoint(setpoint, t);
}

void ChRobot::DisableMotors(bool disable) {
    for (const auto& motor : m_motorlist) {
        motor->SetDisabled(disable);
    }
}

void ChRobot::SetBaseFrame(const ChFramed& base_frame) {
    for (auto& body : m_bodylist)
        body->ConcatenatePreTransformation(ChFrameMoving<>(base_frame));
    m_sys->Update();
}

void ChRobot::SetColor(const ChColor& col) {
    for (unsigned int i = 0; i < m_bodylist.size(); ++i)
        if (m_bodylist[i]->GetVisualModel())
            for (unsigned int j = 0; j < m_bodylist[i]->GetVisualModel()->GetNumShapes(); ++j)
                m_bodylist[i]->GetVisualShape(j)->SetColor(col);
}

void ChRobot::AttachBody(std::shared_ptr<ChBody> slave, std::shared_ptr<ChBody> master, const ChFrame<>& frame) {
    m_body_attached = true;
    m_link_attach->Initialize(slave, m_bodylist.back(), frame);
    slave->SetFixed(false);
    m_sys->AddLink(m_link_attach);
    m_sys->Update();
}

void ChRobot::DetachBody(std::shared_ptr<ChBody> slave, bool setfix) {
    if (m_body_attached) {
        m_body_attached = false;
        m_sys->RemoveLink(m_link_attach);
        slave->SetFixed(setfix);
        m_sys->Update();
    }
}

void ChRobot::UpdateEncoder() {
    m_encoder += (GetMotorsPos() - m_encoder_prev).cwiseAbs();
    m_encoder_prev = GetMotorsPos();
}

double ChRobot::GetMass() const {
    double mass = 0;
    for (auto& body : m_bodylist)
        mass += body->GetMass();
    return mass;
}

void ChRobot::ClearShapes() {
    for (auto& body : m_bodylist)
        body->GetVisualModel()->Clear();
}

std::shared_ptr<ChLinkMotorRotationAngle> ChRobot::CreateMotorRotationAngle(ChSystem* sys,
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

std::shared_ptr<ChLinkMotorLinearPosition> ChRobot::CreateMotorLinearPosition(ChSystem* sys,
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

void ChRobot::CreatePassiveLinks() {
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