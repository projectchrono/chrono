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
// Utility classes for wrapping a wheeled vehicle and powertrain system into a
// single assembly.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constructor from given JSON specification files.
// The vehicle system is constructed within the specified system, using the
// given JSON specification files for the vehicle and powertrain subsystems.
// These files are assumed to be given relative to the Chrono::Vehicle data
// directory.
// -----------------------------------------------------------------------------
ChWheeledVehicleAssembly::ChWheeledVehicleAssembly(ChSystem* system,
                                                   const std::string& vehicle_def_filename,
                                                   const std::string& powertrain_def_filename)
    : m_driver_cb(NULL), m_tire_cb(NULL), m_chassis_cb(NULL) {
    // Create the vehicle and powertrain systems.
    m_vehicle = std::make_shared<WheeledVehicle>(system, vehicle::GetDataFile(vehicle_def_filename));
    m_powertrain = std::make_shared<SimplePowertrain>(vehicle::GetDataFile(powertrain_def_filename));

    // The vector of tire forces is required by the ChronoVehicle API. Since we
    // use rigid contact for tire-terrain interaction, these are always zero.
    m_tire_forces.resize(2 * m_vehicle->GetNumberAxles());
}

// -----------------------------------------------------------------------------
// Initialize the vehicle model at the specified location and orientation.
// -----------------------------------------------------------------------------
void ChWheeledVehicleAssembly::Initialize(const ChVector<>& init_loc, const ChQuaternion<>& init_rot) {
    // Initialize the vehicle and powertrain systems.
    m_vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));
    m_powertrain->Initialize(m_vehicle->GetChassisBody(), m_vehicle->GetDriveshaft());

    // If provided, invoke the user-specified callback to attach chassis contact
    // geometry.
    if (m_chassis_cb) {
        std::shared_ptr<ChBodyAuxRef> chassisBody = m_vehicle->GetChassisBody();

        m_chassis_cb->onCallback(chassisBody);
        chassisBody->SetCollide(true);
    }

    // If provided, invoke the user-specified callback to attach tire contact
    // geometry for each wheel of the vehicle.
    if (m_tire_cb) {
        for (int i = 0; i < 2 * m_vehicle->GetNumberAxles(); i++) {
            std::shared_ptr<ChBody> wheelBody = m_vehicle->GetWheelBody(i);

            m_tire_cb->onCallback(wheelBody);
            wheelBody->SetCollide(true);
        }
    }
}

// -----------------------------------------------------------------------------
// Update the vehicle model at the specified time.
// -----------------------------------------------------------------------------
void ChWheeledVehicleAssembly::Synchronize(double time) {
    // Invoke the user-provided callback to get driver inputs at current time.
    double throttle = 0;
    double steering = 0;
    double braking = 0;

    if (m_driver_cb)
        m_driver_cb->onCallback(time, throttle, steering, braking);

    // Update the powertrain system.
    m_powertrain->Synchronize(time, throttle, m_vehicle->GetDriveshaftSpeed());

    // Update the vehicle system.
    m_vehicle->Synchronize(time, steering, braking, m_powertrain->GetOutputTorque(), m_tire_forces);
}

}  // end namespace vehicle
}  // namespace chrono
