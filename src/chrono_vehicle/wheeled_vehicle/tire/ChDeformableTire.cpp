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
// Base class for a deformable tire (FEA or multibody)
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChDeformableTire::ChDeformableTire(const std::string& name) : ChTire(name), m_pressure_enabled(true), m_contact_enabled(true) {}

// -----------------------------------------------------------------------------
void ChDeformableTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    // If pressure was not explicitly specified, fall back to the default value.
    if (m_pressure_enabled && m_pressure <= 0)
        m_pressure = GetDefaultPressure();
}

// -----------------------------------------------------------------------------
double ChDeformableTire::GetTireMass() const {
    return m_mass;
}

ChVector3d ChDeformableTire::GetTireInertia() const {
    return m_inertia.diagonal();
}

// -----------------------------------------------------------------------------
TerrainForce ChDeformableTire::GetTireForce() const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector3d(0, 0, 0);
    tire_force.moment = ChVector3d(0, 0, 0);
    return tire_force;
}

TerrainForce ChDeformableTire::ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    std::cerr << "ChDeformableTire::ReportTireForceLocal not implemented." << std::endl;
    throw std::runtime_error("ChDeformableTire::ReportTireForceLocal not implemented.");
}

}  // end namespace vehicle
}  // end namespace chrono
