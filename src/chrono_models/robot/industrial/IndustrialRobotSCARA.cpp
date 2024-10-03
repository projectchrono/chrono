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
// Model of industrial SCARA robot.
//
// =============================================================================

#include "IndustrialRobotSCARA.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeBox.h"

namespace chrono {
namespace industrial {

IndustrialRobotSCARA::IndustrialRobotSCARA(ChSystem* sys,
                                           const std::array<double, 5>& lengths,
                                           const ChFramed& base_frame) {
    m_sys = sys;
    m_lengths = lengths;  // H, L1, L2, D, L3
    m_base_frame = Q_ROTATE_Z_TO_Y * base_frame;
    m_link_attach = chrono_types::make_shared<ChLinkMateFix>();

    // Z-based build
    m_joint_frames = {ChFramed(ChVector3d(0, 0, m_lengths[0]), QUNIT),                            // (0) base-biceps
                      ChFramed(ChVector3d(m_lengths[1], 0, m_lengths[0]), QUNIT),                 // (1) biceps-forearm
                      ChFramed(ChVector3d(m_lengths[1] + m_lengths[2], 0, m_lengths[0]), QUNIT),  // (2) forearm-screw
                      ChFramed(ChVector3d(m_lengths[1] + m_lengths[2] + m_lengths[4], 0, m_lengths[0] - m_lengths[3]),
                               QUNIT)};  // (3) TCP

    //// Y-based build
    // m_joint_frames = {
    //     ChCoordsysd(ChVector3d(0, m_lengths[0], 0), Q_ROTATE_Z_TO_Y), // (0) base-biceps
    //     ChCoordsysd(ChVector3d(m_lengths[1], m_lengths[0], 0), Q_ROTATE_Z_TO_Y), // (1) biceps-forearm
    //     ChCoordsysd(ChVector3d(m_lengths[1] + m_lengths[2], m_lengths[0], 0), Q_ROTATE_Z_TO_Y), // (2) forearm-screw
    //     ChCoordsysd(ChVector3d(m_lengths[1] + m_lengths[2], m_lengths[0] - 0.5 * m_lengths[3], 0), Q_ROTATE_Z_TO_Y)
    //     }; // (3) TCP

    for (auto& jcood : m_joint_frames)
        jcood.ConcatenatePreTransformation(m_base_frame);

    m_encoder.resize(4);
    m_encoder << 0.0, 0.0, 0.0, 0.0;
    m_encoder_prev = m_encoder;

    SetupBodies();
    SetupMarkers();
    SetupLinks();
}

void IndustrialRobotSCARA::SetupBodies() {
    // Base
    m_base = chrono_types::make_shared<ChBodyAuxRef>();
    m_base->SetFixed(true);
    m_base->SetPos((m_base_frame.GetPos() + m_joint_frames[0].GetPos()) * 0.5);
    m_sys->Add(m_base);

    // Biceps
    m_biceps = chrono_types::make_shared<ChBodyAuxRef>();
    m_biceps->SetPos((m_joint_frames[0].GetPos() + m_joint_frames[1].GetPos()) * 0.5);
    m_sys->Add(m_biceps);

    // Forearm
    m_forearm = chrono_types::make_shared<ChBodyAuxRef>();
    m_forearm->SetPos((m_joint_frames[1].GetPos() + m_joint_frames[2].GetPos()) * 0.5);
    m_sys->Add(m_forearm);

    // Screw
    m_screw = chrono_types::make_shared<ChBodyAuxRef>();
    m_screw->SetPos(m_joint_frames[2].GetPos());
    m_sys->Add(m_screw);

    // End effector
    m_end_effector = chrono_types::make_shared<ChBodyAuxRef>();
    m_end_effector->SetPos(ChVector3d((m_joint_frames[2].GetPos().x() + m_joint_frames[3].GetPos().x()) * 0.5,
                                      m_joint_frames[3].GetPos().y(), 0));
    m_sys->Add(m_end_effector);

    m_bodylist = {m_base, m_biceps, m_forearm, m_screw, m_end_effector};
}

void IndustrialRobotSCARA::SetupMarkers() {
    // Marker ground-base
    m_marker_ground_base = chrono_types::make_shared<ChMarker>();
    m_base->AddMarker(m_marker_ground_base);
    m_marker_ground_base->ImposeAbsoluteTransform(m_base_frame);

    // Marker base-biceps
    m_marker_base_biceps = chrono_types::make_shared<ChMarker>();
    m_base->AddMarker(m_marker_base_biceps);
    m_marker_base_biceps->ImposeAbsoluteTransform(m_joint_frames[0]);

    // Marker biceps-forearm
    m_marker_biceps_forearm = chrono_types::make_shared<ChMarker>();
    m_biceps->AddMarker(m_marker_biceps_forearm);
    m_marker_biceps_forearm->ImposeAbsoluteTransform(m_joint_frames[1]);

    // Marker forearm-screw
    m_marker_forearm_screw = chrono_types::make_shared<ChMarker>();
    m_forearm->AddMarker(m_marker_forearm_screw);
    m_marker_forearm_screw->ImposeAbsoluteTransform(m_joint_frames[2]);

    // Marker TCP
    m_marker_TCP = chrono_types::make_shared<ChMarker>();
    m_end_effector->AddMarker(m_marker_TCP);
    m_marker_TCP->ImposeAbsoluteTransform(m_joint_frames[3]);

    m_markerlist = {m_marker_ground_base, m_marker_base_biceps, m_marker_biceps_forearm, m_marker_forearm_screw,
                    m_marker_TCP};
}

void IndustrialRobotSCARA::SetupLinks() {
    m_motfunlist = {chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>(),
                    chrono_types::make_shared<ChFunctionSetpoint>(), chrono_types::make_shared<ChFunctionSetpoint>()};

    // Link base-biceps
    m_link_base_biceps = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_base_biceps->Initialize(m_biceps, m_base, ChFrame<>(m_joint_frames[0]));
    m_link_base_biceps->SetMotorFunction(m_motfunlist[0]);
    m_sys->Add(m_link_base_biceps);

    // Link biceps-forearm
    m_link_biceps_forearm = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_biceps_forearm->Initialize(m_forearm, m_biceps, ChFrame<>(m_joint_frames[1]));
    m_link_biceps_forearm->SetMotorFunction(m_motfunlist[1]);
    m_sys->Add(m_link_biceps_forearm);

    // Link forearm-screw (rotational)
    m_link_forearm_screw_rot = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_forearm_screw_rot->Initialize(m_screw, m_forearm, ChFrame<>(m_joint_frames[2]));
    m_link_forearm_screw_rot->SetMotorFunction(m_motfunlist[2]);
    m_sys->Add(m_link_forearm_screw_rot);

    // Link forearm-screw (translational)
    m_link_forearm_screw_transl = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    m_link_forearm_screw_transl->Initialize(
        m_screw, m_forearm, ChFrame<>(m_joint_frames[2].GetPos(), m_joint_frames[2].GetRot() * Q_ROTATE_X_TO_Z));
    m_link_forearm_screw_transl->SetMotorFunction(m_motfunlist[3]);
    m_sys->Add(m_link_forearm_screw_transl);

    // Link screw-end_effector
    auto fix = chrono_types::make_shared<ChLinkMateFix>();
    fix->Initialize(m_end_effector, m_screw, ChFrame<>(m_joint_frames[3]));
    m_sys->Add(fix);

    m_motorlist = {m_link_base_biceps, m_link_biceps_forearm, m_link_forearm_screw_rot, m_link_forearm_screw_transl};

    CreatePassiveLinks();
}

ChVectorDynamic<> IndustrialRobotSCARA::GetMotorsPos(bool wrap_angles) const {
    ChVectorDynamic<> anglepos(4);
    if (wrap_angles) {
        anglepos << std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[0])->GetMotorAngleWrapped(),  // R
            std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[1])->GetMotorAngleWrapped(),          // R
            std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[2])->GetMotorAngleWrapped(),          // R
            std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(m_motorlist[3])->GetMotorPos();                  // P
    } else {
        anglepos << std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[0])->GetMotorAngle(),  // R
            std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[1])->GetMotorAngle(),          // R
            std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[2])->GetMotorAngle(),          // R
            std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(m_motorlist[3])->GetMotorPos();           // P
    }
    return anglepos;
}

ChVectorDynamic<> IndustrialRobotSCARA::GetMotorsPosDt() const {
    ChVectorDynamic<> vels(4);
    vels << std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[0])->GetMotorAngleDt(),  // R
        std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[1])->GetMotorAngleDt(),      // R
        std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[2])->GetMotorAngleDt(),      // R
        std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(m_motorlist[3])->GetMotorPosDt();       // P
    return vels;
}

ChVectorDynamic<> IndustrialRobotSCARA::GetMotorsPosDt2() const {
    ChVectorDynamic<> accels(4);
    accels << std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[0])->GetMotorAngleDt2(),  // R
        std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[1])->GetMotorAngleDt2(),        // R
        std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[2])->GetMotorAngleDt2(),        // R
        std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(m_motorlist[3])->GetMotorPosDt2();         // P
    return accels;
}

ChVectorDynamic<> IndustrialRobotSCARA::GetMotorsForce() const {
    ChVectorDynamic<> torques(4);
    torques << std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[0])->GetMotorTorque(),  // R
        std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[1])->GetMotorTorque(),         // R
        std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(m_motorlist[2])->GetMotorTorque(),         // R
        std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(m_motorlist[3])->GetMotorForce();         // P
    return torques;
}

void IndustrialRobotSCARA::Add1dShapes(const ChColor& col) {
    // Link 1 (base)
    auto shape1 = chrono_types::make_shared<ChVisualShapeLine>();
    auto line1 = chrono_types::make_shared<ChLineSegment>();
    line1->pA = m_base->TransformPointParentToLocal(m_base_frame.GetPos());
    line1->pB = m_base->TransformPointParentToLocal(m_joint_frames[0].GetPos());
    shape1->SetLineGeometry(line1);
    shape1->SetColor(col);
    m_base->AddVisualShape(shape1);

    // Link 2 (biceps)
    auto shape2 = chrono_types::make_shared<ChVisualShapeLine>();
    auto line2 = chrono_types::make_shared<ChLineSegment>();
    line2->pA = m_biceps->TransformPointParentToLocal(m_joint_frames[0].GetPos());
    line2->pB = m_biceps->TransformPointParentToLocal(m_joint_frames[1].GetPos());
    shape2->SetLineGeometry(line2);
    shape2->SetColor(col);
    m_biceps->AddVisualShape(shape2);

    // Link 3 (forearm)
    auto shape3 = chrono_types::make_shared<ChVisualShapeLine>();
    auto line3 = chrono_types::make_shared<ChLineSegment>();
    line3->pA = m_forearm->TransformPointParentToLocal(m_joint_frames[1].GetPos());
    line3->pB = m_forearm->TransformPointParentToLocal(m_joint_frames[2].GetPos());
    shape3->SetLineGeometry(line3);
    shape3->SetColor(col);
    m_forearm->AddVisualShape(shape3);

    // Link 4 (screw)
    auto shape4 = chrono_types::make_shared<ChVisualShapeLine>();
    auto line4 = chrono_types::make_shared<ChLineSegment>();
    line4->pA = m_screw->TransformPointParentToLocal(m_joint_frames[2].GetPos());
    line4->pB = m_screw->TransformPointParentToLocal(m_joint_frames[2].GetPos() - VECT_Y * m_lengths[3]);
    shape4->SetLineGeometry(line4);
    shape4->SetColor(col);
    m_screw->AddVisualShape(shape4);

    // Link 5 (end-effector)
    auto shape5 = chrono_types::make_shared<ChVisualShapeLine>();
    auto line5 = chrono_types::make_shared<ChLineSegment>();
    line5->pA = m_end_effector->TransformPointParentToLocal(m_joint_frames[2].GetPos() - VECT_Y * m_lengths[3]);
    line5->pB = m_end_effector->TransformPointParentToLocal(m_joint_frames[3].GetPos());
    shape5->SetLineGeometry(line5);
    shape5->SetColor(col);
    m_end_effector->AddVisualShape(shape5);
}

void IndustrialRobotSCARA::Add3dShapes(double rad, const ChColor& col) {
    ChLineSegment help_segm;

    // Link 1 (base)
    help_segm.pA = m_base->TransformPointParentToLocal(m_base_frame.GetPos()),
    help_segm.pB = m_base->TransformPointParentToLocal(m_joint_frames[0].GetPos());
    auto shape1a = chrono_types::make_shared<ChVisualShapeCylinder>(rad, help_segm.GetLength());
    shape1a->SetColor(col);
    m_base->AddVisualShape(shape1a, help_segm.GetFrame());

    // Link 2 (biceps)
    auto shape2 = chrono_types::make_shared<ChVisualShapeBox>(m_lengths[1], 0.9 * rad, 0.9 * rad);
    shape2->SetColor(col);
    m_biceps->AddVisualShape(shape2);

    help_segm.pA = m_biceps->TransformPointParentToLocal(m_joint_frames[0].GetPos() + ChVector3d(0, 0.9 * rad, 0));
    help_segm.pB = m_biceps->TransformPointParentToLocal(m_joint_frames[0].GetPos() + ChVector3d(0, -0.9 * rad, 0));
    auto shape2b = chrono_types::make_shared<ChVisualShapeCylinder>(0.9 * rad, help_segm.GetLength());
    shape2b->SetColor(col);
    m_biceps->AddVisualShape(shape2b, help_segm.GetFrame());

    help_segm.pA = m_biceps->TransformPointParentToLocal(m_joint_frames[1].GetPos() + ChVector3d(0, 0.9 * rad, 0));
    help_segm.pB = m_biceps->TransformPointParentToLocal(m_joint_frames[1].GetPos() + ChVector3d(0, -0.9 * rad, 0));
    auto shape2c = chrono_types::make_shared<ChVisualShapeCylinder>(0.9 * rad, help_segm.GetLength());
    shape2c->SetColor(col);
    m_biceps->AddVisualShape(shape2c, help_segm.GetFrame());

    // Link 3 (forearm)
    auto shape3 = chrono_types::make_shared<ChVisualShapeBox>(m_lengths[2], 0.8 * rad, 0.8 * rad);
    shape3->SetColor(col);
    m_forearm->AddVisualShape(shape3);

    help_segm.pA = m_forearm->TransformPointParentToLocal(m_joint_frames[2].GetPos() + ChVector3d(0, 0.8 * rad, 0));
    help_segm.pB = m_forearm->TransformPointParentToLocal(m_joint_frames[2].GetPos() + ChVector3d(0, -0.8 * rad, 0));
    auto shape3b = chrono_types::make_shared<ChVisualShapeCylinder>(0.8 * rad, help_segm.GetLength());
    shape3b->SetColor(col);
    m_forearm->AddVisualShape(shape3b, help_segm.GetFrame());

    // Link 4 (screw)
    help_segm.pA = m_screw->TransformPointParentToLocal(m_joint_frames[2].GetPos());
    help_segm.pB = m_screw->TransformPointParentToLocal(m_joint_frames[2].GetPos() - VECT_Y * m_lengths[3]);
    auto shape4 = chrono_types::make_shared<ChVisualShapeCylinder>(0.3 * rad, 2 * help_segm.GetLength());
    shape4->SetColor(col);
    m_screw->AddVisualShape(shape4, ChFramed(VNULL, Q_ROTATE_Z_TO_Y));

    // Link 5 (end-effector)
    auto shape5 = chrono_types::make_shared<ChVisualShapeBox>(m_lengths[4], 0.1 * rad, 0.5 * rad);
    shape5->SetColor(col);
    m_end_effector->AddVisualShape(shape5);
}

}  // end namespace industrial
}  // end namespace chrono