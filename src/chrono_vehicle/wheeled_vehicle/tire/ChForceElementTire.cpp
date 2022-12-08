// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for a force element tire model
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

ChForceElementTire::ChForceElementTire(const std::string& name) : ChTire(name) {}

void ChForceElementTire::InitializeInertiaProperties() {
    m_mass = GetTireMass();
    m_inertia.setZero();
    m_inertia.diagonal() = GetTireInertia().eigen();
    m_com = ChFrame<>();
}

void ChForceElementTire::UpdateInertiaProperties() {
    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector<>(0, GetOffset(), 0)), spindle->GetRot());
}

double ChForceElementTire::GetAddedMass() const {
    return GetTireMass();
}

ChVector<> ChForceElementTire::GetAddedInertia() const {
    return GetTireInertia();
}

TerrainForce ChForceElementTire::ReportTireForce(ChTerrain* terrain) const {
    return GetTireForce();
}

TerrainForce ChForceElementTire::GetTireForce() const {
    if (!m_data.in_contact) {
        return TerrainForce();
    }

    TerrainForce tireforce;
    tireforce.point = m_wheel->GetPos();

    // Rotate into global coordinates
    tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
    tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

    // Move the tire forces from the contact patch to the wheel center
    tireforce.moment +=
        Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetZaxis()) - tireforce.point, tireforce.force);

    return tireforce;
}

}  // namespace vehicle
}  // namespace chrono
