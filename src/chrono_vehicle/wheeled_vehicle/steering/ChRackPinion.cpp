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
// Base class for a Rack-Pinion steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// The steering subsystem is modeled with respect to a right-handed frame with
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The steering link translates along the Y axis. We do not explicitly model the
// pinion but instead use the implied rack-pinion constraint to calculate the
// rack displacement from a given pinion rotation angle.
//
// =============================================================================

#include <vector>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChRackPinion::ChRackPinion(const std::string& name) : ChSteering(name) {}

ChRackPinion::~ChRackPinion() {
    if (!m_initialized)
        return;

    auto sys = m_prismatic->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_prismatic);
    sys->Remove(m_actuator);
}

// -----------------------------------------------------------------------------
void ChRackPinion::Initialize(std::shared_ptr<ChChassis> chassis,
                              const ChVector3d& location,
                              const ChQuaternion<>& rotation) {
    ChSteering::Initialize(chassis, location, rotation);

    m_parent = chassis;
    m_rel_xform = ChFrame<>(location, rotation);

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassisBody->GetFrameRefToAbs());

    // Create and initialize the steering link body
    ChVector3d link_pos = steering_to_abs.TransformPointLocalToParent(ChVector3d(0, GetSteeringLinkCOM(), 0));
    ChQuaternion<> link_rot = steering_to_abs.GetRot().GetNormalized();

    m_link = chrono_types::make_shared<ChBody>();
    m_link->SetName(m_name + "_link");
    m_link->SetPos(link_pos);
    m_link->SetRot(link_rot);
    m_link->SetMass(GetSteeringLinkMass());
    m_link->SetInertiaXX(GetSteeringLinkInertia());
    sys->AddBody(m_link);

    // Create and initialize the prismatic joint between chassis and link.
    m_prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prismatic->SetName(m_name + "_prismatic");
    m_prismatic->Initialize(chassisBody, m_link, ChFrame<>(link_pos, link_rot * QuatFromAngleX(CH_PI_2)));
    sys->AddLink(m_prismatic);

    // Create and initialize the linear actuator.
    // The offset value here must be larger than any possible displacement of the steering link body (the rack) so that
    // we do not reach the singular configuration of the ChLinkLockLinActuator (when the distance between the two
    // markers becomes zero).
    double offset = 2;
    ChVector3d pt1 = link_pos;
    ChVector3d pt2 = link_pos - offset * link_rot.GetAxisY();

    m_actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
    m_actuator->SetName(m_name + "_actuator");
    m_actuator->Initialize(chassisBody, m_link, false, ChFrame<>(pt1, link_rot), ChFrame<>(pt2, link_rot));
    m_actuator->SetDistanceOffset(offset);
    sys->AddLink(m_actuator);
}

// -----------------------------------------------------------------------------
void ChRackPinion::Synchronize(double time, const DriverInputs& driver_inputs) {
    // Convert the steering input into an angle of the pinion and then into a
    // displacement of the rack.
    double angle = driver_inputs.m_steering * GetMaxAngle();
    double displ = angle * GetPinionRadius();

    if (auto fun = std::dynamic_pointer_cast<ChFunctionConst>(m_actuator->GetActuatorFunction()))
        fun->SetConstant(displ);
}

void ChRackPinion::InitializeInertiaProperties() {
    m_mass = GetSteeringLinkMass();
    m_com = ChFrame<>(GetSteeringLinkCOM(), QUNIT);
    m_inertia.setZero();
    m_inertia.diagonal() = GetSteeringLinkInertia().eigen();
}

void ChRackPinion::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(m_rel_xform);
}

// -----------------------------------------------------------------------------
void ChRackPinion::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double length = GetSteeringLinkLength();

    utils::ChBodyGeometry::AddVisualizationCylinder(m_link,                         //
                                                    ChVector3d(0, length / 2, 0),   //
                                                    ChVector3d(0, -length / 2, 0),  //
                                                    GetSteeringLinkRadius());
}

void ChRackPinion::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_link);
}

// -----------------------------------------------------------------------------
void ChRackPinion::LogConstraintViolations() {
    // Translational joint
    {
        ChVectorDynamic<> C = m_prismatic->GetConstraintViolation();
        std::cout << "Prismatic           ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    // Actuator
    {
        ChVectorDynamic<> C = m_actuator->GetConstraintViolation();
        std::cout << "Actuator            ";
        std::cout << "  " << C(0) << "  ";
    }
}

// -----------------------------------------------------------------------------
void ChRackPinion::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_prismatic);
    joints.push_back(m_actuator);
    ExportJointList(jsonDocument, joints);
}

void ChRackPinion::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_prismatic);
    joints.push_back(m_actuator);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
