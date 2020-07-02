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
// Irrlicht-based visualization wrapper for wheeled vehicles.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChWheeledVehicleIrrApp::ChWheeledVehicleIrrApp(ChVehicle* vehicle,
                                               const std::wstring& title,
                                               const irr::core::dimension2d<irr::u32>& dims,
                                               irr::ELOG_LEVEL log_level)
    : ChVehicleIrrApp(vehicle, title, dims, log_level) {
    m_wvehicle = dynamic_cast<ChWheeledVehicle*>(vehicle);
    assert(m_wvehicle);
}

// -----------------------------------------------------------------------------
// Render stats for the vehicle driveline.
// -----------------------------------------------------------------------------
void ChWheeledVehicleIrrApp::renderOtherStats(int left, int top) {
    char msg[100];

    if (auto driveline = std::dynamic_pointer_cast<ChShaftsDriveline2WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline->GetDrivenAxleIndexes()[0];

        torque = driveline->GetSpindleTorque(axle, LEFT);
        sprintf(msg, "Torque wheel L: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top, 120, 15);

        torque = driveline->GetSpindleTorque(axle, RIGHT);
        sprintf(msg, "Torque wheel R: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top + 20, 120, 15);
    } else if (auto driveline = std::dynamic_pointer_cast<ChShaftsDriveline4WD>(m_wvehicle->GetDriveline())) {
        double torque;
        std::vector<int> axles = driveline->GetDrivenAxleIndexes();

        torque = driveline->GetSpindleTorque(axles[0], LEFT);
        sprintf(msg, "Torque wheel FL: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top, 120, 15);

        torque = driveline->GetSpindleTorque(axles[0], RIGHT);
        sprintf(msg, "Torque wheel FR: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top + 20, 120, 15);

        torque = driveline->GetSpindleTorque(axles[1], LEFT);
        sprintf(msg, "Torque wheel RL: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top + 40, 120, 15);

        torque = driveline->GetSpindleTorque(axles[1], RIGHT);
        sprintf(msg, "Torque wheel FR: %+.2f", torque);
        renderLinGauge(std::string(msg), torque / 5000, false, left, top + 60, 120, 15);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
