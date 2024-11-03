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
// Model of industrial SCARA robot imported from CAD.
//
// =============================================================================

#include "IndustrialRobotSCARA_CAD.h"

namespace chrono {
namespace industrial {

IndustrialRobotSCARA_CAD::IndustrialRobotSCARA_CAD(ChSystem* sys,
                                                   const ChFramed& base_frame,
                                                   unsigned int id,
                                                   std::vector<std::string> bodynames)
    : m_id(id), m_bodynames(bodynames) {
    m_sys = sys;
    m_base_frame = base_frame;
    m_link_attach = chrono_types::make_shared<ChLinkMateFix>();

    m_encoder.resize(4);
    m_encoder << 0.0, 0.0, 0.0, 0.0;
    m_encoder_prev = m_encoder;

    SetupBodies();
    SetupMarkers();
    SetupLinks();

    SetBaseFrame(m_base_frame);
}

void IndustrialRobotSCARA_CAD::SetupBodies() {
    // Ground (virtual)
    auto ground = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody("SLDW_GROUND"));
    ground->SetName(("robot" + std::to_string(m_id) + "_ground").c_str());

    // Base
    m_base = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[0] + "-1").c_str()));
    m_base->SetName(("robot" + std::to_string(m_id) + "_base").c_str());

    // Biceps
    m_biceps = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[1] + "-1").c_str()));
    m_biceps->SetName(("robot" + std::to_string(m_id) + "_biceps").c_str());

    // Forearm
    m_forearm = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[2] + "-1").c_str()));
    m_forearm->SetName(("robot" + std::to_string(m_id) + "_forearm").c_str());

    // Wrist
    m_screw = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[3] + "-1").c_str()));
    m_screw->SetName(("robot" + std::to_string(m_id) + "_screw").c_str());

    // End effector
    m_end_effector = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[4] + "-1").c_str()));
    m_end_effector->SetName(("robot" + std::to_string(m_id) + "_end_effector").c_str());

    m_bodylist = {m_base, m_biceps, m_forearm, m_screw, m_end_effector};
}

void IndustrialRobotSCARA_CAD::SetupMarkers() {
    // Marker ground-base
    m_marker_ground_base = m_sys->SearchMarker("MARKER_0");
    m_marker_ground_base->SetName(("robot" + std::to_string(m_id) + "_MARKER_0").c_str());
    m_base->AddMarker(m_marker_ground_base);
    m_marker_ground_base->ImposeAbsoluteTransform(m_marker_ground_base->GetAbsFrame());

    // Marker base-biceps
    m_marker_base_biceps = m_sys->SearchMarker("MARKER_1");
    m_marker_base_biceps->SetName(("robot" + std::to_string(m_id) + "_MARKER_1").c_str());
    m_base->AddMarker(m_marker_base_biceps);
    m_marker_base_biceps->ImposeAbsoluteTransform(m_marker_base_biceps->GetAbsFrame());

    // Marker biceps-forearm
    m_marker_biceps_forearm = m_sys->SearchMarker("MARKER_2");
    m_marker_biceps_forearm->SetName(("robot" + std::to_string(m_id) + "_MARKER_2").c_str());
    m_biceps->AddMarker(m_marker_biceps_forearm);
    m_marker_biceps_forearm->ImposeAbsoluteTransform(m_marker_biceps_forearm->GetAbsFrame());

    // Marker forearm-screw
    m_marker_forearm_screw = m_sys->SearchMarker("MARKER_3");
    m_marker_forearm_screw->SetName(("robot" + std::to_string(m_id) + "_MARKER_3").c_str());
    m_forearm->AddMarker(m_marker_forearm_screw);
    m_marker_forearm_screw->ImposeAbsoluteTransform(m_marker_forearm_screw->GetAbsFrame());

    // Marker TCP
    m_marker_TCP = m_sys->SearchMarker("MARKER_TCP");
    m_marker_TCP->SetName(("robot" + std::to_string(m_id) + "_MARKER_TCP").c_str());
    m_end_effector->AddMarker(m_marker_TCP);
    m_marker_TCP->ImposeAbsoluteTransform(m_marker_TCP->GetAbsFrame());

    m_markerlist = {m_marker_ground_base, m_marker_base_biceps, m_marker_biceps_forearm, m_marker_forearm_screw,
                    m_marker_TCP};
    m_joint_frames = {};  // no use

    // Lenghts
    double H = (m_marker_base_biceps->GetAbsFrame().GetPos() - m_marker_ground_base->GetAbsFrame().GetPos())
                   .z();  // base height
    double L1 = (m_marker_biceps_forearm->GetAbsFrame().GetPos() - m_marker_base_biceps->GetAbsFrame().GetPos())
                    .x();  // biceps length
    double L2 = (m_marker_forearm_screw->GetAbsFrame().GetPos() - m_marker_biceps_forearm->GetAbsFrame().GetPos())
                    .x();  // forearm length
    double D = (m_marker_TCP->GetAbsFrame().GetPos() - m_marker_forearm_screw->GetAbsFrame().GetPos())
                   .z();  // TCP vertical offset length (wrt marker forearm-screw)
    double L3 = (m_marker_TCP->GetAbsFrame().GetPos() - m_marker_forearm_screw->GetAbsFrame().GetPos())
                    .x();  // TCP horizontal offset length (wrt to screw axis)
    m_lengths = {H, L1, L2, D, L3};
}

void IndustrialRobotSCARA_CAD::SetupLinks() {
    m_motfunlist = {chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>(),
                    chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>()};

    // Link base-biceps
    m_link_base_biceps = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_base_biceps->Initialize(m_biceps, m_base, m_marker_base_biceps->GetAbsFrame());
    m_link_base_biceps->SetMotorFunction(m_motfunlist[0]);
    // std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_link_base_biceps)->SetSpindleConstraint(ChLinkMotorRotationAngle::SpindleConstraint::FREE);
    m_sys->Add(m_link_base_biceps);

    // Link biceps-forearm
    m_link_biceps_forearm = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_biceps_forearm->Initialize(m_forearm, m_biceps, m_marker_biceps_forearm->GetAbsFrame());
    m_link_biceps_forearm->SetMotorFunction(m_motfunlist[1]);
    // std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_link_biceps_forearm)->SetSpindleConstraint(ChLinkMotorRotationAngle::SpindleConstraint::FREE);
    m_sys->Add(m_link_biceps_forearm);

    // Link forearm-screw (rotational)
    m_link_forearm_screw_rot = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_forearm_screw_rot->Initialize(m_screw, m_forearm, m_marker_forearm_screw->GetAbsFrame());
    m_link_forearm_screw_rot->SetMotorFunction(m_motfunlist[2]);
    // std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_link_forearm_screw_rot)->SetSpindleConstraint(ChLinkMotorRotationAngle::SpindleConstraint::FREE);
    m_sys->Add(m_link_forearm_screw_rot);

    // Link forearm-screw (translational)
    m_link_forearm_screw_transl = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    m_link_forearm_screw_transl->Initialize(
        m_screw, m_forearm,
        ChFrame<>(m_marker_forearm_screw->GetAbsFrame().GetPos(),
                  m_marker_forearm_screw->GetAbsFrame().GetRot() * Q_ROTATE_X_TO_Z));
    m_link_forearm_screw_transl->SetMotorFunction(m_motfunlist[3]);
    // std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(m_link_forearm_screw_transl)->SetGuideConstraint(ChLinkMotorLinearPosition::GuideConstraint::FREE);
    m_sys->Add(m_link_forearm_screw_transl);

    // Fixture screw-end effector
    auto fix_screw_end_effector = chrono_types::make_shared<ChLinkMateFix>();
    fix_screw_end_effector->Initialize(m_end_effector, m_screw, m_end_effector->GetFrameCOMToAbs());
    m_sys->Add(fix_screw_end_effector);

    m_motorlist = {m_link_base_biceps, m_link_biceps_forearm, m_link_forearm_screw_rot, m_link_forearm_screw_transl};

    CreatePassiveLinks();
}

}  // end namespace industrial
}  // end namespace chrono