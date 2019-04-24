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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChAntirollBarRSD::ChAntirollBarRSD(const std::string& name) : ChAntirollBar(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                  const ChVector<>& location,
                                  std::shared_ptr<ChBody> susp_body_left,
                                  std::shared_ptr<ChBody> susp_body_right) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> subsystem_to_abs(location);
    subsystem_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Chassis orientation (expressed in absolute frame)
    // Recall that the subsystem reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Convenience names
    double L = getArmLength();
    double W = getArmWidth();
    double H = getDroplinkHeight();

    // Express the local coordinates into the absolute coordinate system
    ChVector<> P_center = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(0, 0, 0));
    ChVector<> P_arm_left = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(0, L / 2, 0));
    ChVector<> P_drop_arm_left = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, L, 0));
    ChVector<> P_drop_susp_left = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, L, H));
    ChVector<> P_arm_right = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(0, -L / 2, 0));
    ChVector<> P_drop_arm_right = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, -L, 0));
    ChVector<> P_drop_susp_right = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, -L, H));

    // Create an initialize the arm_left body
    m_arm_left = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm_left->SetNameString(m_name + "_arm_left");
    m_arm_left->SetPos(P_arm_left);
    m_arm_left->SetRot(subsystem_to_abs.GetRot());
    m_arm_left->SetMass(getArmMass());
    m_arm_left->SetInertiaXX(getArmInertia());
    AddVisualizationArm(m_arm_left, ChVector<>(0, -L / 2, 0), ChVector<>(0, L / 2, 0), ChVector<>(W, L / 2, 0),
                        getArmRadius(), ChColor(0.7f, 0.2f, 0.2f));
    chassis->GetSystem()->AddBody(m_arm_left);

    // Create an initialize the arm_right body
    m_arm_right = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm_right->SetNameString(m_name + "_arm_right");
    m_arm_right->SetPos(P_arm_right);
    m_arm_right->SetRot(subsystem_to_abs.GetRot());
    m_arm_right->SetMass(getArmMass());
    m_arm_right->SetInertiaXX(getArmInertia());
    AddVisualizationArm(m_arm_right, ChVector<>(0, L / 2, 0), ChVector<>(0, -L / 2, 0), ChVector<>(W, -L / 2, 0),
                        getArmRadius(), ChColor(0.2f, 0.7f, 0.2f));
    chassis->GetSystem()->AddBody(m_arm_right);

    // Create and initialize the revolute joint between left arm and chassis.
    ChCoordsys<> rev_ch_csys(P_arm_left, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute_ch = std::make_shared<ChLinkLockRevolute>();
    m_revolute_ch->SetNameString(m_name + "_revolute_ch");
    m_revolute_ch->Initialize(m_arm_left, chassis, rev_ch_csys);
    chassis->GetSystem()->AddLink(m_revolute_ch);

    // Create and initialize the revolute joint between left and right arms.
    ChCoordsys<> rev_csys(P_center, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(m_arm_left, m_arm_right, rev_csys);
    chassis->GetSystem()->AddLink(m_revolute);

    m_revolute->GetForce_Rz().SetActive(1);
    m_revolute->GetForce_Rz().SetK(getSpringCoefficient());
    m_revolute->GetForce_Rz().SetR(getDampingCoefficient());

    // Create distance constraint to model left droplink.
    m_link_left = std::make_shared<ChLinkDistance>();
    m_link_left->SetNameString(m_name + "_droplink_left");
    m_link_left->Initialize(m_arm_left, susp_body_left, false, P_drop_arm_left, P_drop_susp_left);
    chassis->GetSystem()->AddLink(m_link_left);

    // Create distance constraint to model right droplink.
    m_link_right = std::make_shared<ChLinkDistance>();
    m_link_right->SetNameString(m_name + "_droplink_right");
    m_link_right->Initialize(m_arm_right, susp_body_right, false, P_drop_arm_right, P_drop_susp_right);
    chassis->GetSystem()->AddLink(m_link_right);
}

// -----------------------------------------------------------------------------
// Get the total mass of the anti-roll bar subsystem
// -----------------------------------------------------------------------------
double ChAntirollBarRSD::GetMass() const {
    return 2 * getArmMass();
}

// -----------------------------------------------------------------------------
// Get the current COM location of the anti-roll bar subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChAntirollBarRSD::GetCOMPos() const {
    ChVector<> com = getArmMass() * m_arm_left->GetPos() + getArmMass() * m_arm_right->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::LogConstraintViolations() {
    // Chassis revolute joint
    {
        ChMatrix<>* C = m_revolute_ch->GetC();
        GetLog() << "Chassis revolute          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }

    // Central revolute joint
    {
        ChMatrix<>* C = m_revolute->GetC();
        GetLog() << "Central revolute          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }

    // Distance constraints (droplinks)
    GetLog() << "Droplink distance (left)  ";
    GetLog() << "  " << m_link_left->GetCurrentDistance() - m_link_left->GetImposedDistance() << "\n";
    GetLog() << "Droplink distance (right) ";
    GetLog() << "  " << m_link_right->GetCurrentDistance() - m_link_right->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::AddVisualizationArm(std::shared_ptr<ChBody> arm,
                                           const ChVector<>& pt_1,
                                           const ChVector<>& pt_2,
                                           const ChVector<>& pt_3,
                                           double radius,
                                           const ChColor& color) {
    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = pt_1;
    cyl_1->GetCylinderGeometry().p2 = pt_2;
    cyl_1->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_1);

    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = pt_2;
    cyl_2->GetCylinderGeometry().p2 = pt_3;
    cyl_2->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_2);

    auto cyl_3 = std::make_shared<ChCylinderShape>();
    cyl_3->GetCylinderGeometry().p1 = pt_1 + ChVector<>(0, 0, 3 * radius);
    cyl_3->GetCylinderGeometry().p2 = pt_1 - ChVector<>(0, 0, 3 * radius);
    cyl_3->GetCylinderGeometry().rad = radius / 2;
    arm->AddAsset(cyl_3);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    arm->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm_left);
    bodies.push_back(m_arm_right);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_ch);
    joints.push_back(m_revolute);
    joints.push_back(m_link_left);
    joints.push_back(m_link_right);
    ChPart::ExportJointList(jsonDocument, joints);
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
