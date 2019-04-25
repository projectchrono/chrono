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
// Base class for a Pitman Arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// =============================================================================

#include <vector>

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChPitmanArm::ChPitmanArm(const std::string& name, bool vehicle_frame_inertia)
    : ChSteering(name), m_vehicle_frame_inertia(vehicle_frame_inertia) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPitmanArm::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                             const ChVector<>& location,
                             const ChQuaternion<>& rotation) {
    m_position = ChCoordsys<>(location, rotation);

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector<>> points(NUM_POINTS);
    std::vector<ChVector<>> dirs(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        points[i] = steering_to_abs.TransformPointLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
        dirs[i] = steering_to_abs.TransformDirectionLocalToParent(rel_dir);
    }

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize the steering link body
    m_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_link->SetNameString(m_name + "_link");
    m_link->SetPos(points[STEERINGLINK]);
    m_link->SetRot(steering_to_abs.GetRot());
    m_link->SetMass(getSteeringLinkMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getSteeringLinkInertiaMoments(), getSteeringLinkInertiaProducts(),
                                                      chassisRot, steering_to_abs.GetRot());
        m_link->SetInertia(inertia);
    } else {
        m_link->SetInertiaXX(getSteeringLinkInertiaMoments());
        m_link->SetInertiaXY(getSteeringLinkInertiaProducts());
    }
    chassis->GetSystem()->AddBody(m_link);

    m_pP = m_link->TransformPointParentToLocal(points[UNIV]);
    m_pI = m_link->TransformPointParentToLocal(points[REVSPH_S]);
    m_pTP = m_link->TransformPointParentToLocal(points[TIEROD_PA]);
    m_pTI = m_link->TransformPointParentToLocal(points[TIEROD_IA]);

    // Create and initialize the Pitman arm body
    m_arm = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm->SetNameString(m_name + "_arm");
    m_arm->SetPos(points[PITMANARM]);
    m_arm->SetRot(steering_to_abs.GetRot());
    m_arm->SetMass(getPitmanArmMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getPitmanArmInertiaMoments(), getPitmanArmInertiaProducts(),
                                                      chassisRot, steering_to_abs.GetRot());
        m_arm->SetInertia(inertia);
    } else {
        m_arm->SetInertiaXX(getPitmanArmInertiaMoments());
        m_arm->SetInertiaXY(getPitmanArmInertiaProducts());
    }
    chassis->GetSystem()->AddBody(m_arm);

    // Cache points for arm visualization (expressed in the arm frame)
    m_pC = m_arm->TransformPointParentToLocal(points[REV]);
    m_pL = m_arm->TransformPointParentToLocal(points[UNIV]);

    // Create and initialize the revolute joint between chassis and Pitman arm.
    // Note that this is modeled as a rotational motor to allow driving it with
    // imposed rotation (steering input).
    // The z direction of the joint orientation matrix is dirs[REV_AXIS], assumed
    // to be a unit vector.
    u = points[PITMANARM] - points[REV];
    v = Vcross(dirs[REV_AXIS], u);
    v.Normalize();
    u = Vcross(v, dirs[REV_AXIS]);
    rot.Set_A_axis(u, v, dirs[REV_AXIS]);

    m_revolute = std::make_shared<ChLinkMotorRotationAngle>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_arm, ChFrame<>(points[REV], rot.Get_A_quaternion()));
    auto motor_fun = std::make_shared<ChFunction_Setpoint>();
    m_revolute->SetAngleFunction(motor_fun);
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the universal joint between the Pitman arm and steering link.
    // The x and y directions of the joint orientation matrix are given by
    // dirs[UNIV_AXIS_ARM] and dirs[UNIV_AXIS_LINK], assumed to be unit vectors
    // and orthogonal.
    w = Vcross(dirs[UNIV_AXIS_ARM], dirs[UNIV_AXIS_LINK]);
    rot.Set_A_axis(dirs[UNIV_AXIS_ARM], dirs[UNIV_AXIS_LINK], w);

    m_universal = std::make_shared<ChLinkUniversal>();
    m_universal->SetNameString(m_name + "_universal");
    m_universal->Initialize(m_arm, m_link, ChFrame<>(points[UNIV], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universal);

    // Create and initialize the revolute-spherical joint (massless idler arm).
    // The length of the idler arm is the distance between the two hardpoints.
    // The z direction of the revolute joint orientation matrix is
    // dirs[REVSPH_AXIS], assumed to be a unit vector.
    double distance = (points[REVSPH_S] - points[REVSPH_R]).Length();

    u = points[REVSPH_S] - points[REVSPH_R];
    v = Vcross(dirs[REVSPH_AXIS], u);
    v.Normalize();
    u = Vcross(v, dirs[REVSPH_AXIS]);
    rot.Set_A_axis(u, v, dirs[REVSPH_AXIS]);

    m_revsph = std::make_shared<ChLinkRevoluteSpherical>();
    m_revsph->SetNameString(m_name + "_revsph");
    m_revsph->Initialize(chassis, m_link, ChCoordsys<>(points[REVSPH_R], rot.Get_A_quaternion()), distance);
    chassis->GetSystem()->AddLink(m_revsph);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPitmanArm::Synchronize(double time, double steering) {
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(m_revolute->GetAngleFunction());
    fun->SetSetpoint(getMaxAngle() * steering, time);
}

// -----------------------------------------------------------------------------
// Get the total mass of the steering subsystem
// -----------------------------------------------------------------------------
double ChPitmanArm::GetMass() const {
    return getSteeringLinkMass() + getPitmanArmMass();
}

// -----------------------------------------------------------------------------
// Get the current COM location of the steering subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChPitmanArm::GetCOMPos() const {
    ChVector<> com = getSteeringLinkMass() * m_link->GetPos() + getPitmanArmMass() * m_arm->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPitmanArm::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Visualization for link
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pP;
        cyl->GetCylinderGeometry().p2 = m_pI;
        cyl->GetCylinderGeometry().rad = getSteeringLinkRadius();
        m_link->AddAsset(cyl);

        auto cyl_P = std::make_shared<ChCylinderShape>();
        cyl_P->GetCylinderGeometry().p1 = m_pP;
        cyl_P->GetCylinderGeometry().p2 = m_pTP;
        cyl_P->GetCylinderGeometry().rad = getSteeringLinkRadius();
        m_link->AddAsset(cyl_P);

        auto cyl_I = std::make_shared<ChCylinderShape>();
        cyl_I->GetCylinderGeometry().p1 = m_pI;
        cyl_I->GetCylinderGeometry().p2 = m_pTI;
        cyl_I->GetCylinderGeometry().rad = getSteeringLinkRadius();
        m_link->AddAsset(cyl_I);

        auto col = std::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.2f, 0.7f, 0.7f));
        m_link->AddAsset(col);
    }

    // Visualization for arm
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pC;
        cyl->GetCylinderGeometry().p2 = m_pL;
        cyl->GetCylinderGeometry().rad = getPitmanArmRadius();
        m_arm->AddAsset(cyl);

        auto col = std::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.7f, 0.7f, 0.2f));
        m_arm->AddAsset(col);
    }

    // Visualization for rev-sph link
    m_revsph->AddAsset(std::make_shared<ChPointPointSegment>());
}

void ChPitmanArm::RemoveVisualizationAssets() {
    m_link->GetAssets().clear();
    m_arm->GetAssets().clear();
    m_revsph->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPitmanArm::LogConstraintViolations() {
    // Revolute joint
    ////{
    ////    ChMatrix<>* C = m_revolute->GetC();
    ////    GetLog() << "Revolute              ";
    ////    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    ////    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    ////    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    ////    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    ////    GetLog() << "  " << C->GetElement(4, 0) << "\n";
    ////}

    // Universal joint
    {
        ChMatrix<>* C = m_universal->GetC();
        GetLog() << "Universal             ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }

    // Revolute-spherical joint
    {
        ChMatrix<>* C = m_revsph->GetC();
        GetLog() << "Revolute-spherical    ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPitmanArm::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    bodies.push_back(m_arm);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_revsph);
    joints.push_back(m_universal);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChPitmanArm::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    bodies.push_back(m_arm);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_revsph);
    joints.push_back(m_universal);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
