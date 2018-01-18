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
// Base class for a road wheel.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChRoadWheel.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRoadWheel::ChRoadWheel(const std::string& name) : ChPart(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoadWheel::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                             std::shared_ptr<ChBody> carrier,
                             const ChVector<>& location) {
    // Express the road wheel reference frame in the absolute coordinate system.
    ChFrame<> wheel_to_abs(location);
    wheel_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Create and initialize the wheel body.
    m_wheel = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetPos(wheel_to_abs.GetPos());
    m_wheel->SetRot(wheel_to_abs.GetRot());
    m_wheel->SetMass(GetWheelMass());
    m_wheel->SetInertiaXX(GetWheelInertia());
    chassis->GetSystem()->AddBody(m_wheel);

    // Set wheel contact material properties.
    switch (m_wheel->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_wheel->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_wheel->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_wheel->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_wheel->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_wheel->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_wheel->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_wheel->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_wheel->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_wheel->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_wheel->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    // Create and initialize the revolute joint between wheel and carrier.
    // The axis of rotation is the y axis of the road wheel reference frame.
    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(carrier, m_wheel,
                           ChCoordsys<>(wheel_to_abs.GetPos(), wheel_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoadWheel::LogConstraintViolations() {
    ChMatrix<>* C = m_revolute->GetC();
    GetLog() << "  Road-wheel revolute\n";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoadWheel::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChRoadWheel::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
