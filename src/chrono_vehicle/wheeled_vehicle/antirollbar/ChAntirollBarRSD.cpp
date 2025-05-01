// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for an anti-roll bar template modeled two arms connected with a
// revolute spring-damper.
// Derived from ChAntirollBar, but still and abstract base class.
//
// The anti-roll bar subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The subsystem reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChAntirollBarRSD::ChAntirollBarRSD(const std::string& name) : ChAntirollBar(name) {}

ChAntirollBarRSD::~ChAntirollBarRSD() {
    if (!m_initialized)
        return;

    auto sys = m_arm_left->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_arm_left);
    sys->Remove(m_arm_right);
    sys->Remove(m_revolute_ch);
    sys->Remove(m_revolute);
    sys->Remove(m_link_left);
    sys->Remove(m_link_right);
}

// -----------------------------------------------------------------------------
void ChAntirollBarRSD::Construct(std::shared_ptr<ChChassis> chassis,
                                 std::shared_ptr<ChSuspension> suspension,
                                 const ChVector3d& location) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> subsystem_to_abs(location);
    subsystem_to_abs.ConcatenatePreTransformation(chassisBody->GetFrameRefToAbs());

    // Chassis orientation (expressed in absolute frame)
    // Recall that the subsystem reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassisBody->GetFrameRefToAbs().GetRot();

    // Convenience names
    double L = getArmLength();
    double W = getArmWidth();
    double H = getDroplinkHeight();

    // Express the local coordinates into the absolute coordinate system
    ChVector3d P_center = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(0, 0, 0));
    ChVector3d P_arm_left = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(0, L / 2, 0));
    ChVector3d P_drop_arm_left = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(W, L, 0));
    ChVector3d P_drop_susp_left = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(W, L, H));
    ChVector3d P_arm_right = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(0, -L / 2, 0));
    ChVector3d P_drop_arm_right = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(W, -L, 0));
    ChVector3d P_drop_susp_right = subsystem_to_abs.TransformPointLocalToParent(ChVector3d(W, -L, H));

    // Create an initialize the arm_left body
    m_arm_left = chrono_types::make_shared<ChBody>();
    m_arm_left->SetName(m_name + "_arm_left");
    m_arm_left->SetPos(P_arm_left);
    m_arm_left->SetRot(subsystem_to_abs.GetRot());
    m_arm_left->SetMass(getArmMass());
    m_arm_left->SetInertiaXX(getArmInertia());
    AddVisualizationArm(m_arm_left, ChVector3d(0, -L / 2, 0), ChVector3d(0, L / 2, 0), ChVector3d(W, L / 2, 0),
                        getArmRadius(), ChColor(0.7f, 0.2f, 0.2f));
    sys->AddBody(m_arm_left);

    // Create an initialize the arm_right body
    m_arm_right = chrono_types::make_shared<ChBody>();
    m_arm_right->SetName(m_name + "_arm_right");
    m_arm_right->SetPos(P_arm_right);
    m_arm_right->SetRot(subsystem_to_abs.GetRot());
    m_arm_right->SetMass(getArmMass());
    m_arm_right->SetInertiaXX(getArmInertia());
    AddVisualizationArm(m_arm_right, ChVector3d(0, L / 2, 0), ChVector3d(0, -L / 2, 0), ChVector3d(W, -L / 2, 0),
                        getArmRadius(), ChColor(0.2f, 0.7f, 0.2f));
    sys->AddBody(m_arm_right);

    // Create and initialize the revolute joint between left arm and chassis.
    ChFrame<> rev_ch_frame(P_arm_left, chassisRot * QuatFromAngleX(CH_PI_2));
    m_revolute_ch = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute_ch->SetName(m_name + "_revolute_ch");
    m_revolute_ch->Initialize(m_arm_left, chassisBody, rev_ch_frame);
    sys->AddLink(m_revolute_ch);

    // Create and initialize the revolute joint between left and right arms.
    ChFrame<> rev_frame(P_center, chassisRot * QuatFromAngleX(CH_PI_2));
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetName(m_name + "_revolute");
    m_revolute->Initialize(m_arm_left, m_arm_right, rev_frame);
    sys->AddLink(m_revolute);

    m_revolute->ForceRz().SetActive(1);
    m_revolute->ForceRz().SetSpringCoefficient(getSpringCoefficient());
    m_revolute->ForceRz().SetDampingCoefficient(getDampingCoefficient());

    // Create distance constraint to model left droplink.
    m_link_left = chrono_types::make_shared<ChLinkDistance>();
    m_link_left->SetName(m_name + "_droplink_left");
    m_link_left->Initialize(m_arm_left, suspension->GetAntirollBody(LEFT), false, P_drop_arm_left, P_drop_susp_left);
    sys->AddLink(m_link_left);

    // Create distance constraint to model right droplink.
    m_link_right = chrono_types::make_shared<ChLinkDistance>();
    m_link_right->SetName(m_name + "_droplink_right");
    m_link_right->Initialize(m_arm_right, suspension->GetAntirollBody(RIGHT), false, P_drop_arm_right,
                             P_drop_susp_right);
    sys->AddLink(m_link_right);
}

void ChAntirollBarRSD::InitializeInertiaProperties() {
    m_mass = 2 * getArmMass();
}

void ChAntirollBarRSD::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_arm_left->GetFrameCOMToAbs(), m_arm_left->GetMass(), m_arm_left->GetInertia());
    composite.AddComponent(m_arm_right->GetFrameCOMToAbs(), m_arm_right->GetMass(), m_arm_right->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
void ChAntirollBarRSD::LogConstraintViolations() {
    // Chassis revolute joint
    {
        ChVectorDynamic<> C = m_revolute_ch->GetConstraintViolation();
        std::cout << "Chassis revolute          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    // Central revolute joint
    {
        ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
        std::cout << "Central revolute          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    // Distance constraints (droplinks)
    std::cout << "Droplink distance (left)  ";
    std::cout << "  " << m_link_left->GetCurrentDistance() - m_link_left->GetImposedDistance() << "\n";
    std::cout << "Droplink distance (right) ";
    std::cout << "  " << m_link_right->GetCurrentDistance() - m_link_right->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
void ChAntirollBarRSD::AddVisualizationArm(std::shared_ptr<ChBody> arm,
                                           const ChVector3d& pt_1,
                                           const ChVector3d& pt_2,
                                           const ChVector3d& pt_3,
                                           double radius,
                                           const ChColor& color) {
    utils::ChBodyGeometry::AddVisualizationCylinder(arm, pt_1, pt_2, radius);

    utils::ChBodyGeometry::AddVisualizationCylinder(arm, pt_2, pt_3, radius);

    utils::ChBodyGeometry::AddVisualizationCylinder(arm,                                  //
                                                    pt_1 + ChVector3d(0, 0, 3 * radius),  //
                                                    pt_1 - ChVector3d(0, 0, 3 * radius),  //
                                                    radius / 2);
}

// -----------------------------------------------------------------------------
void ChAntirollBarRSD::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm_left);
    bodies.push_back(m_arm_right);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_ch);
    joints.push_back(m_revolute);
    joints.push_back(m_link_left);
    joints.push_back(m_link_right);
    ExportJointList(jsonDocument, joints);
}

void ChAntirollBarRSD::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm_left);
    bodies.push_back(m_arm_right);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_ch);
    joints.push_back(m_revolute);
    joints.push_back(m_link_left);
    joints.push_back(m_link_right);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
