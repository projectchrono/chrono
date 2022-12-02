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

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline8WD.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChWheeledVehicleVisualSystemIrrlicht::ChWheeledVehicleVisualSystemIrrlicht()
    : ChVehicleVisualSystemIrrlicht(), m_wvehicle(nullptr) {}

void ChWheeledVehicleVisualSystemIrrlicht::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemIrrlicht::AttachVehicle(vehicle);
    m_wvehicle = dynamic_cast<ChWheeledVehicle*>(m_vehicle);
    assert(m_wvehicle);
}

// -----------------------------------------------------------------------------
// Render stats for the vehicle driveline.
// -----------------------------------------------------------------------------
void ChWheeledVehicleVisualSystemIrrlicht::renderOtherStats(int left, int top) {
    char msg[100];

    if (auto driveline2 = std::dynamic_pointer_cast<ChShaftsDriveline2WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline2->GetDrivenAxleIndexes()[0];

        torque = driveline2->GetSpindleTorque(axle, LEFT);
        sprintf(msg, "T.axle 0 L: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top, 170, 15);

        torque = driveline2->GetSpindleTorque(axle, RIGHT);
        sprintf(msg, "T.axle 0 R: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top + 20, 170, 15);
    } else if (auto driveline4 = std::dynamic_pointer_cast<ChShaftsDriveline4WD>(m_wvehicle->GetDriveline())) {
        double torque;
        std::vector<int> axles = driveline4->GetDrivenAxleIndexes();

        for (int i = 0; i < 2; i++) {
            torque = driveline4->GetSpindleTorque(axles[i], LEFT);
            sprintf(msg, "T.axle %1d L: %+.2f", i, torque);
            renderLinGauge(std::string(msg), torque / 5000, false, left, top + 20 * i, 170, 15);

            torque = driveline4->GetSpindleTorque(axles[i], RIGHT);
            sprintf(msg, "T.axle %1d R: %+.2f", i, torque);
            renderLinGauge(std::string(msg), torque / 5000, false, left + 190, top + 20 * i, 170, 15);
        }
    } else if (auto driveline8 = std::dynamic_pointer_cast<ChShaftsDriveline8WD>(m_wvehicle->GetDriveline())) {
        double torque;
        std::vector<int> axles = driveline8->GetDrivenAxleIndexes();

        for (int i = 0; i < 4; i++) {
            torque = driveline8->GetSpindleTorque(axles[i], LEFT);
            sprintf(msg, "T.axle %1d L: %+.2f", i, torque);
            renderLinGauge(std::string(msg), torque / 5000, false, left, top + 20 * i, 170, 15);

            torque = driveline8->GetSpindleTorque(axles[i], RIGHT);
            sprintf(msg, "T.axle %1d R: %+.2f", i, torque);
            renderLinGauge(std::string(msg), torque / 5000, false, left + 190, top + 20 * i, 170, 15);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
