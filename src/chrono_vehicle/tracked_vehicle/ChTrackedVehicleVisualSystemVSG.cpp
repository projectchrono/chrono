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
// Irrlicht-based visualization for wheeled vehicles.
// This class extends ChVehicleVisualSystemIrrlicht.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicleVisualSystemVSG::ChTrackedVehicleVisualSystemVSG() : ChVehicleVisualSystemVSG(), m_tvehicle(nullptr) {
    m_params->showTrackedVehicleState = true;
}

void ChTrackedVehicleVisualSystemVSG::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);
    m_tvehicle = dynamic_cast<ChTrackedVehicle*>(m_vehicle);
    assert(m_tvehicle);
}

double ChTrackedVehicleVisualSystemVSG::GetSprocketTorque(int side) {
    if (side == 0)
        return m_tvehicle->GetDriveline()->GetSprocketTorque(LEFT);
    else
        return m_tvehicle->GetDriveline()->GetSprocketTorque(RIGHT);
}

double ChTrackedVehicleVisualSystemVSG::GetSprocketSpeed(int side) {
    if(side == 0) {
        return m_tvehicle->GetDriveline()->GetSprocketSpeed(LEFT)*30.0/CH_C_PI;
    }
    else {
        return m_tvehicle->GetDriveline()->GetSprocketSpeed(RIGHT)*30.0/CH_C_PI;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
