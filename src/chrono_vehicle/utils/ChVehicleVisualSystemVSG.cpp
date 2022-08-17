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
// Irrlicht-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemIrrlicht and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#include "ChVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() {
    m_params->showVehicleState = true;
}

void ChVehicleVisualSystemVSG::Initialize() {
    ChVisualSystemVSG::Initialize();
}

double ChVehicleVisualSystemVSG::GetVehicleSpeed() {
    if (m_vehicle) {
        return m_vehicle->GetChassis()->GetSpeed();
    } else {
        return 0.0;
    }
}

    double ChVehicleVisualSystemVSG::GetEngineSpeed() {
        if (m_vehicle) {
            return m_vehicle->GetPowertrain()->GetMotorSpeed() * 30.0 / CH_C_PI;
        } else {
            return 0.0;
        }
    }

    double ChVehicleVisualSystemVSG::GetEngineTorque() {
        if (m_vehicle) {
            return m_vehicle->GetPowertrain()->GetMotorTorque();
        } else {
            return 0.0;
        }
    }

    int ChVehicleVisualSystemVSG::GetGear() {
        if (m_vehicle) {
            return m_vehicle->GetPowertrain()->GetCurrentTransmissionGear();
        } else {
            return 0;
        }
    }

    int ChVehicleVisualSystemVSG::GetMaxGear() {
        if (m_vehicle) {
            return 42;
        } else {
            return 0;
        }
    }

}  // end namespace vehicle
}  // end namespace chrono
