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
// Model of industrial 6-DOF articulated robot imported from CAD.
//
// =============================================================================

#include "IndustrialRobot6dofCAD.h"

namespace chrono {
namespace industrial {

IndustrialRobot6dofCAD::IndustrialRobot6dofCAD(ChSystem* sys,
                                               const ChFramed& base_frame,
                                               unsigned int id,
                                               std::vector<std::string> bodynames)
    : m_id(id), m_bodynames(bodynames) {
    m_sys = sys;
    m_base_frame = base_frame;
    m_link_attach = chrono_types::make_shared<ChLinkMateFix>();

    m_encoder.resize(6);
    m_encoder << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_encoder_prev = m_encoder;

    SetupBodies();
    SetupMarkers();
    SetupLinks();

    SetBaseFrame(m_base_frame);
}

void IndustrialRobot6dofCAD::SetupBodies() {
    // Check if 6+1 robot bodies are provided (i.e. including a robot basement).
    // If not, fallback to use internal 'ground' as robot base.
    bool has_base = true;
    int offset = 0;
    if (m_bodynames.size() == 6) {
        has_base = false;
        offset = -1;
    }

    // Ground (virtual)
    m_ground = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody("SLDW_GROUND"));
    m_ground->SetName(("robot" + std::to_string(m_id) + "_ground").c_str());
    m_ground->SetFixed(true);
    m_ground->SetMass(1e-10);  // set it to negligible value to not interfere with overall robot mass

    // Shoulder
    m_shoulder = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[1 + offset] + "-1").c_str()));
    m_shoulder->SetName(("robot" + std::to_string(m_id) + "_shoulder").c_str());

    // Biceps
    m_biceps = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[2 + offset] + "-1").c_str()));
    m_biceps->SetName(("robot" + std::to_string(m_id) + "_biceps").c_str());

    // Elbow
    m_elbow = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[3 + offset] + "-1").c_str()));
    m_elbow->SetName(("robot" + std::to_string(m_id) + "_elbow").c_str());

    // Forearm
    m_forearm = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[4 + offset] + "-1").c_str()));
    m_forearm->SetName(("robot" + std::to_string(m_id) + "_forearm").c_str());

    // Wrist
    m_wrist = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[5 + offset] + "-1").c_str()));
    m_wrist->SetName(("robot" + std::to_string(m_id) + "_wrist").c_str());

    // End effector
    m_end_effector =
        std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[6 + offset] + "-1").c_str()));
    m_end_effector->SetName(("robot" + std::to_string(m_id) + "_end_effector").c_str());

    if (has_base) {
        // Base (potential robot basement)
        m_base = std::dynamic_pointer_cast<ChBodyAuxRef>(m_sys->SearchBody((m_bodynames[0] + "-1").c_str()));
        m_base->SetName(("robot" + std::to_string(m_id) + "_base").c_str());

        m_bodylist = {m_ground, m_base, m_shoulder, m_biceps, m_elbow, m_forearm, m_wrist, m_end_effector};
    } else {
        m_bodylist = {m_ground, m_shoulder, m_biceps, m_elbow, m_forearm, m_wrist, m_end_effector};
    }

    // If some body has not been properly initialized, throw error
    for (const auto& body : m_bodylist) {
        if (!body) {
            throw std::runtime_error("ERROR: CAD body not found");
        }
    }
}

void IndustrialRobot6dofCAD::SetupMarkers() {
    m_marker_base_shoulder = PreprocessMarker("MARKER_1", m_base ? m_base : m_ground);
    m_marker_shoulder_biceps = PreprocessMarker("MARKER_2", m_shoulder);
    m_marker_biceps_elbow = PreprocessMarker("MARKER_3", m_biceps);
    m_marker_elbow_forearm = PreprocessMarker("MARKER_4", m_elbow);
    m_marker_forearm_wrist = PreprocessMarker("MARKER_5", m_forearm);
    m_marker_wrist_end_effector = PreprocessMarker("MARKER_6", m_wrist);
    m_marker_TCP = PreprocessMarker("MARKER_TCP", m_end_effector);

    m_markerlist = {m_marker_base_shoulder, m_marker_shoulder_biceps,    m_marker_biceps_elbow, m_marker_elbow_forearm,
                    m_marker_forearm_wrist, m_marker_wrist_end_effector, m_marker_TCP};

    // If some marker has not been properly initialized, throw error
    for (const auto& marker : m_markerlist) {
        if (!marker) {
            throw std::runtime_error("ERROR: CAD marker not found");
        }
    }

    m_lengths = {
        (m_marker_shoulder_biceps->GetAbsCoordsys().pos - m_marker_base_shoulder->GetAbsCoordsys().pos).Length(),
        (m_marker_biceps_elbow->GetAbsCoordsys().pos - m_marker_shoulder_biceps->GetAbsCoordsys().pos).Length(),
        (m_marker_forearm_wrist->GetAbsCoordsys().pos - m_marker_biceps_elbow->GetAbsCoordsys().pos).Length(),
        (m_marker_TCP->GetAbsCoordsys().pos - m_marker_forearm_wrist->GetAbsCoordsys().pos).Length()};

    m_joint_frames = {};  // no use
}

void IndustrialRobot6dofCAD::SetupLinks() {
    m_motfunlist = {chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>(),
                    chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>(),
                    chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>()};

    m_link_base_shoulder = CreateMotorRotationAngle(m_sys, m_shoulder, m_base ? m_base : m_ground,
                                                    m_marker_base_shoulder->GetAbsFrame(), m_motfunlist[0]);
    m_link_shoulder_biceps =
        CreateMotorRotationAngle(m_sys, m_biceps, m_shoulder, m_marker_shoulder_biceps->GetAbsFrame(), m_motfunlist[1]);
    m_link_biceps_elbow =
        CreateMotorRotationAngle(m_sys, m_elbow, m_biceps, m_marker_biceps_elbow->GetAbsFrame(), m_motfunlist[2]);
    m_link_elbow_forearm =
        CreateMotorRotationAngle(m_sys, m_forearm, m_elbow, m_marker_elbow_forearm->GetAbsFrame(), m_motfunlist[3]);
    m_link_forearm_wrist =
        CreateMotorRotationAngle(m_sys, m_wrist, m_forearm, m_marker_forearm_wrist->GetAbsFrame(), m_motfunlist[4]);
    m_link_wrist_end_effector = CreateMotorRotationAngle(m_sys, m_end_effector, m_wrist,
                                                         m_marker_wrist_end_effector->GetAbsFrame(), m_motfunlist[5]);

    m_motorlist = {m_link_base_shoulder, m_link_shoulder_biceps, m_link_biceps_elbow,
                   m_link_elbow_forearm, m_link_forearm_wrist,   m_link_wrist_end_effector};

    CreatePassiveLinks();
}

std::shared_ptr<ChMarker> IndustrialRobot6dofCAD::PreprocessMarker(const std::string& name,
                                                                   std::shared_ptr<ChBody> body) {
    auto marker = m_sys->SearchMarker(name);
    marker->SetName("robot" + std::to_string(m_id) + "_" + name);
    body->AddMarker(marker);
    marker->ImposeAbsoluteTransform(marker->GetAbsFrame());
    return marker;
}

}  // end namespace industrial
}  // end namespace chrono