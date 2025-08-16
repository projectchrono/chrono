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

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChRotaryArm::ChRotaryArm(const std::string& name, bool vehicle_frame_inertia)
    : ChSteering(name), m_vehicle_frame_inertia(vehicle_frame_inertia) {}

ChRotaryArm::~ChRotaryArm() {
    if (!m_initialized)
        return;

    auto sys = m_revolute->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_revolute);
}

// -----------------------------------------------------------------------------
void ChRotaryArm::Construct(std::shared_ptr<ChChassis> chassis,
                            const ChVector3d& location,
                            const ChQuaternion<>& rotation) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassisBody->GetFrameRefToAbs().GetRot();

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassisBody->GetFrameRefToAbs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector3d> points(NUM_POINTS);
    std::vector<ChVector3d> dirs(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = getLocation(static_cast<PointId>(i));
        points[i] = steering_to_abs.TransformPointLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector3d rel_dir = getDirection(static_cast<DirectionId>(i));
        dirs[i] = steering_to_abs.TransformDirectionLocalToParent(rel_dir);
    }

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Create and initialize the Pitman arm body
    m_link = chrono_types::make_shared<ChBody>();
    m_link->SetName(m_name + "_arm");
    m_link->SetTag(m_obj_tag);
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
    rot.SetFromDirectionAxes(u, v, dirs[REV_AXIS]);

    m_revolute = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_revolute->SetName(m_name + "_revolute");
    m_revolute->SetTag(m_obj_tag);
    m_revolute->Initialize(chassisBody, m_link, ChFrame<>(points[ARM_C], rot.GetQuaternion()));
    auto motor_fun = chrono_types::make_shared<ChFunctionSetpoint>();
    m_revolute->SetAngleFunction(motor_fun);
    sys->AddLink(m_revolute);
}

// -----------------------------------------------------------------------------
void ChRotaryArm::Synchronize(double time, const DriverInputs& driver_inputs) {
    auto fun = std::static_pointer_cast<ChFunctionSetpoint>(m_revolute->GetAngleFunction());
    fun->SetSetpoint(getMaxAngle() * driver_inputs.m_steering, time);
}

void ChRotaryArm::InitializeInertiaProperties() {
    m_mass = m_link->GetMass();
    m_com = ChFrame<>(0.5 * (getLocation(ARM_L) + getLocation(ARM_C)), QUNIT);
    m_inertia = m_link->GetInertia();
}

void ChRotaryArm::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(m_rel_xform);
}

// -----------------------------------------------------------------------------
void ChRotaryArm::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Visualization for arm
    utils::ChBodyGeometry::AddVisualizationCylinder(m_link, m_pC, m_pL, getPitmanArmRadius());
}

void ChRotaryArm::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_link);
}

// -----------------------------------------------------------------------------
void ChRotaryArm::LogConstraintViolations() {
    // Revolute joint
    ////{
    ////    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    ////    std::cout << "Revolute              ";
    ////    std::cout << "  " << C(0) << "  ";
    ////    std::cout << "  " << C(1) << "  ";
    ////    std::cout << "  " << C(2) << "  ";
    ////    std::cout << "  " << C(3) << "  ";
    ////    std::cout << "  " << C(4) << "\n";
    ////}
}

// -----------------------------------------------------------------------------
void ChRotaryArm::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ExportJointList(jsonDocument, joints);
}

void ChRotaryArm::Output(ChOutput& database) const {
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
