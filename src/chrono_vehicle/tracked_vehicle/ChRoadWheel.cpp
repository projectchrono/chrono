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
void ChRoadWheel::SetContactMaterial(float friction_coefficient,
                                     float restitution_coefficient,
                                     float young_modulus,
                                     float poisson_ratio) {
    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoadWheel::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                             ChSharedPtr<ChBody> carrier,
                             const ChVector<>& location) {
    // Express the road wheel reference frame in the absolute coordinate system.
    ChFrame<> wheel_to_abs(location);
    wheel_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Create and initialize the wheel body.
    m_wheel = ChSharedPtr<ChBody>(new ChBody(chassis->GetSystem()->GetContactMethod()));
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetPos(wheel_to_abs.GetPos());
    m_wheel->SetRot(wheel_to_abs.GetRot());
    m_wheel->SetMass(GetWheelMass());
    m_wheel->SetInertiaXX(GetWheelInertia());
    chassis->GetSystem()->AddBody(m_wheel);

    // Create and initialize the revolute joint between wheel and carrier.
    // The axis of rotation is the y axis of the road wheel reference frame.
    m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(carrier, m_wheel,
                           ChCoordsys<>(wheel_to_abs.GetPos(), wheel_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
}

}  // end namespace vehicle
}  // end namespace chrono
