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

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline8WD.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChWheeledVehicleVisualSystemVSG::ChWheeledVehicleVisualSystemVSG() : ChVehicleVisualSystemVSG(), m_wvehicle(nullptr) {}

void ChWheeledVehicleVisualSystemVSG::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);
    m_wvehicle = dynamic_cast<ChWheeledVehicle*>(m_vehicle);
    assert(m_wvehicle);
    if (!m_wvehicle)
        return;
    if (auto driveline2 = std::dynamic_pointer_cast<ChShaftsDriveline2WD>(m_wvehicle->GetDriveline())) {
        m_drivenAxles = 1;
    } else if (auto driveline4 = std::dynamic_pointer_cast<ChShaftsDriveline4WD>(m_wvehicle->GetDriveline())) {
        m_drivenAxles = 2;
    } else if (auto driveline8 = std::dynamic_pointer_cast<ChShaftsDriveline8WD>(m_wvehicle->GetDriveline())) {
        m_drivenAxles = 4;
    }
}

double ChWheeledVehicleVisualSystemVSG::GetTireTorque(int draxle, int side) {
    if (!m_wvehicle)
        return 0.0;
    if (auto driveline2 = std::dynamic_pointer_cast<ChShaftsDriveline2WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline2->GetDrivenAxleIndexes()[0];
        if (side == 0)
            torque = driveline2->GetSpindleTorque(axle, LEFT);
        else
            torque = driveline2->GetSpindleTorque(axle, RIGHT);
        return torque;
    } else if (auto driveline4 = std::dynamic_pointer_cast<ChShaftsDriveline4WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline4->GetDrivenAxleIndexes()[draxle];
        if (side == 0)
            torque = driveline4->GetSpindleTorque(axle, LEFT);
        else
            torque = driveline4->GetSpindleTorque(axle, RIGHT);
        return torque;
    }
    if (auto driveline8 = std::dynamic_pointer_cast<ChShaftsDriveline8WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline8->GetDrivenAxleIndexes()[draxle];
        if (side == 0)
            torque = driveline8->GetSpindleTorque(axle, LEFT);
        else
            torque = driveline8->GetSpindleTorque(axle, RIGHT);
        return torque;
    } else {
        return 0.0;
    }
}
int ChWheeledVehicleVisualSystemVSG::GetNumDrivenAxles() {
    return m_drivenAxles;
}

}  // end namespace vehicle
}  // end namespace chrono
