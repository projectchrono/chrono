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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRackPinion::ChRackPinion(const std::string& name) : ChSteering(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                              const ChVector<>& location,
                              const ChQuaternion<>& rotation) {
    m_position = ChCoordsys<>(location, rotation);

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Create and initialize the steering link body
    ChVector<> link_local(0, GetSteeringLinkCOM(), 0);
    ChVector<> link_abs = steering_to_abs.TransformPointLocalToParent(link_local);

    m_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_link->SetNameString(m_name + "_link");
    m_link->SetPos(link_abs);
    m_link->SetRot(steering_to_abs.GetRot());
    m_link->SetMass(GetSteeringLinkMass());
    m_link->SetInertiaXX(GetSteeringLinkInertia());
    chassis->GetSystem()->AddBody(m_link);

    // Create and initialize the prismatic joint between chassis and link.
    m_prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prismatic->SetNameString(m_name + "_prismatic");
    m_prismatic->Initialize(chassis, m_link, ChCoordsys<>(link_abs, steering_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_prismatic);

    // Create and initialize the linear actuator.
    // The offset value here must be larger than any possible displacement of the
    // steering link body (the rack) so that we do not reach the singular
    // configuration of the ChLinkLinActuator (when the distance between the two
    // markers becomes zero).
    double offset = 10;
    ChVector<> pt1 = link_abs;
    ChVector<> pt2 = link_abs - offset * steering_to_abs.GetRot().GetYaxis();

    m_actuator = chrono_types::make_shared<ChLinkLinActuator>();
    m_actuator->SetNameString(m_name + "_actuator");
    m_actuator->Initialize(chassis, m_link, false, ChCoordsys<>(pt1, QUNIT), ChCoordsys<>(pt2, QUNIT));
    m_actuator->Set_lin_offset(offset);
    chassis->GetSystem()->AddLink(m_actuator);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::Synchronize(double time, double steering) {
    // Convert the steering input into an angle of the pinion and then into a
    // displacement of the rack.
    double angle = steering * GetMaxAngle();
    double displ = angle * GetPinionRadius();

    if (auto fun = std::dynamic_pointer_cast<ChFunction_Const>(m_actuator->Get_dist_funct()))
        fun->Set_yconst(displ);
}

// -----------------------------------------------------------------------------
// Get the total mass of the steering subsystem
// -----------------------------------------------------------------------------
double ChRackPinion::GetMass() const {
    return GetSteeringLinkMass();
}

// -----------------------------------------------------------------------------
// Get the current COM location of the steering subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChRackPinion::GetCOMPos() const {
    return m_link->GetPos();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double length = GetSteeringLinkLength();

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, length / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -length / 2, 0);
    cyl->GetCylinderGeometry().rad = GetSteeringLinkRadius();
    m_link->AddAsset(cyl);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.8f, 0.8f, 0.2f));
    m_link->AddAsset(col);
}

void ChRackPinion::RemoveVisualizationAssets() {
    m_link->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::LogConstraintViolations() {
    // Translational joint
    {
        ChVectorDynamic<> C = m_prismatic->GetC();
        GetLog() << "Prismatic           ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }

    // Actuator
    {
        ChVectorDynamic<> C = m_actuator->GetC();
        GetLog() << "Actuator            ";
        GetLog() << "  " << C(0) << "  ";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_prismatic);
    joints.push_back(m_actuator);
    ChPart::ExportJointList(jsonDocument, joints);
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
