// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a tracked vehicle roller.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChRoller.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRoller::ChRoller(const std::string& name) : ChPart(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoller::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Express the roller reference frame in the absolute coordinate system.
    ChFrame<> roller_to_abs(location);
    roller_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Create and initialize the roller body.
    m_wheel = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetPos(roller_to_abs.GetPos());
    m_wheel->SetRot(roller_to_abs.GetRot());
    m_wheel->SetMass(GetMass());
    m_wheel->SetInertiaXX(GetInertia());
    chassis->GetSystem()->AddBody(m_wheel);

    // Set roller contact material properties.
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

    // Create and initialize the revolute joint between roller and chassis.
    // The axis of rotation is the y axis of the road wheel reference frame.
    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_wheel,
                           ChCoordsys<>(roller_to_abs.GetPos(), roller_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoller::LogConstraintViolations() {
    ChMatrix<>* C = m_revolute->GetC();
    GetLog() << "  Road-wheel revolute\n";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
}

}  // end namespace vehicle
}  // end namespace chrono
