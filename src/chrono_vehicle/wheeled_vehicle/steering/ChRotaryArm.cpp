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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Base class for a rotary arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// =============================================================================

#include <vector>

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRotaryArm::ChRotaryArm(const std::string& name, bool vehicle_frame_inertia)
    : ChSteering(name), m_vehicle_frame_inertia(vehicle_frame_inertia) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRotaryArm::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
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

    // Create and initialize the Pitman arm body
    m_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_link->SetNameString(m_name + "_arm");
    m_link->SetPos(points[PITMANARM]);
    m_link->SetRot(steering_to_abs.GetRot());
    m_link->SetMass(getPitmanArmMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getPitmanArmInertiaMoments(), getPitmanArmInertiaProducts(),
                                                      chassisRot, steering_to_abs.GetRot());
        m_link->SetInertia(inertia);
    } else {
        m_link->SetInertiaXX(getPitmanArmInertiaMoments());
        m_link->SetInertiaXY(getPitmanArmInertiaProducts());
    }
    chassis->GetSystem()->AddBody(m_link);

    // Cache points for arm visualization (expressed in the arm frame)
    m_pC = m_link->TransformPointParentToLocal(points[REV]);
    m_pL = m_link->TransformPointParentToLocal(points[PITMANARM]);

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
    m_revolute->Initialize(chassis, m_link, ChFrame<>(points[REV], rot.Get_A_quaternion()));
    auto motor_fun = std::make_shared<ChFunction_Setpoint>();
    m_revolute->SetAngleFunction(motor_fun);
    chassis->GetSystem()->AddLink(m_revolute);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRotaryArm::Synchronize(double time, double steering) {
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(m_revolute->GetAngleFunction());
    fun->SetSetpoint(getMaxAngle() * steering, time);
}

// -----------------------------------------------------------------------------
// Get the total mass of the steering subsystem
// -----------------------------------------------------------------------------
double ChRotaryArm::GetMass() const {
    return getPitmanArmMass();
}

// -----------------------------------------------------------------------------
// Get the current COM location of the steering subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChRotaryArm::GetCOMPos() const {
    ChVector<> com = getPitmanArmMass() * m_link->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRotaryArm::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Visualization for arm
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pC;
        cyl->GetCylinderGeometry().p2 = m_pL;
        cyl->GetCylinderGeometry().rad = getPitmanArmRadius();
        m_link->AddAsset(cyl);

        auto col = std::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.7f, 0.7f, 0.2f));
        m_link->AddAsset(col);
    }
}

void ChRotaryArm::RemoveVisualizationAssets() {
    m_link->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRotaryArm::LogConstraintViolations() {
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
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRotaryArm::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChRotaryArm::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
