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
ChRotaryArm::ChRotaryArm(const std::string& name, bool vehicle_frame_inertia)
    : ChSteering(name), m_vehicle_frame_inertia(vehicle_frame_inertia) {}

ChRotaryArm::~ChRotaryArm() {
    auto sys = m_revolute->GetSystem();
    if (sys) {
        sys->Remove(m_revolute);
    }
}

// -----------------------------------------------------------------------------
void ChRotaryArm::Initialize(std::shared_ptr<ChChassis> chassis,
                             const ChVector<>& location,
                             const ChQuaternion<>& rotation) {
    m_parent = chassis;
    m_rel_xform = ChFrame<>(location, rotation);

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassisBody->GetFrame_REF_to_abs().GetRot();

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassisBody->GetFrame_REF_to_abs());

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
    m_link = std::shared_ptr<ChBody>(sys->NewBody());
    m_link->SetNameString(m_name + "_arm");
    m_link->SetPos(0.5 * (points[ARM_L] + points[ARM_C]));
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
    sys->AddBody(m_link);

    // Cache points for arm visualization (expressed in the arm frame)
    m_pC = m_link->TransformPointParentToLocal(points[ARM_C]);
    m_pL = m_link->TransformPointParentToLocal(points[ARM_L]);

    // Create and initialize the revolute joint between chassis and Pitman arm.
    // Note that this is modeled as a rotational motor to allow driving it with
    // imposed rotation (steering input).
    // The z direction of the joint orientation matrix is dirs[REV_AXIS], assumed
    // to be a unit vector.
    u = points[ARM_L] - points[ARM_C];
    v = Vcross(dirs[REV_AXIS], u);
    v.Normalize();
    u = Vcross(v, dirs[REV_AXIS]);
    rot.Set_A_axis(u, v, dirs[REV_AXIS]);

    m_revolute = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassisBody, m_link, ChFrame<>(points[ARM_C], rot.Get_A_quaternion()));
    auto motor_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    m_revolute->SetAngleFunction(motor_fun);
    sys->AddLink(m_revolute);
}

// -----------------------------------------------------------------------------
void ChRotaryArm::Synchronize(double time, double steering) {
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(m_revolute->GetAngleFunction());
    fun->SetSetpoint(getMaxAngle() * steering, time);
}


void ChRotaryArm::InitializeInertiaProperties() {
    m_mass = m_link->GetMass();
    m_com = ChFrame<>(0.5 * (getLocation(ARM_L) + getLocation(ARM_C)), QUNIT);
    m_inertia = m_link->GetInertia();
}

void ChRotaryArm::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(m_rel_xform, m_xform);
}

// -----------------------------------------------------------------------------
void ChRotaryArm::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Visualization for arm
    {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pC;
        cyl->GetCylinderGeometry().p2 = m_pL;
        cyl->GetCylinderGeometry().rad = getPitmanArmRadius();
        m_link->AddAsset(cyl);

        auto col = chrono_types::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.7f, 0.7f, 0.2f));
        m_link->AddAsset(col);
    }
}

void ChRotaryArm::RemoveVisualizationAssets() {
    m_link->GetAssets().clear();
}

// -----------------------------------------------------------------------------
void ChRotaryArm::LogConstraintViolations() {
    // Revolute joint
    ////{
    ////    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    ////    GetLog() << "Revolute              ";
    ////    GetLog() << "  " << C(0) << "  ";
    ////    GetLog() << "  " << C(1) << "  ";
    ////    GetLog() << "  " << C(2) << "  ";
    ////    GetLog() << "  " << C(3) << "  ";
    ////    GetLog() << "  " << C(4) << "\n";
    ////}
}

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
