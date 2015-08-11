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
// Utility classes for wrapping a wheeled vehicle and powertrain system into a
// single assembly.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChWheeledVehicleAssembly.h"

namespace chrono {

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
    m_vehicle = ChSharedPtr<Vehicle>(new Vehicle(system, vehicle::GetDataFile(vehicle_def_filename)));
    m_powertrain = ChSharedPtr<SimplePowertrain>(new SimplePowertrain(vehicle::GetDataFile(powertrain_def_filename)));

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
    m_powertrain->Initialize();

    // If provided, invoke the user-specified callback to attach chassis contact
    // geometry.
    if (m_chassis_cb) {
        ChSharedPtr<ChBodyAuxRef> chassisBody = m_vehicle->GetChassis();

        m_chassis_cb->onCallback(chassisBody);
        chassisBody->SetCollide(true);
    }

    // If provided, invoke the user-specified callback to attach tire contact
    // geometry for each wheel of the vehicle.
    if (m_tire_cb) {
        for (int i = 0; i < 2 * m_vehicle->GetNumberAxles(); i++) {
            ChSharedPtr<ChBody> wheelBody = m_vehicle->GetWheelBody(i);
            double radius = m_vehicle->GetWheel(i)->GetRadius();
            double width = m_vehicle->GetWheel(i)->GetWidth();

            m_tire_cb->onCallback(wheelBody, radius, width);
            wheelBody->SetCollide(true);
        }
    }
}

// -----------------------------------------------------------------------------
// Update the vehicle model at the specified time.
// -----------------------------------------------------------------------------
void ChWheeledVehicleAssembly::Update(double time) {
    // Invoke the user-provided callback to get driver inputs at current time.
    double throttle = 0;
    double steering = 0;
    double braking = 0;

    if (m_driver_cb)
        m_driver_cb->onCallback(time, throttle, steering, braking);

    // Update the powertrain system.
    m_powertrain->Update(time, throttle, m_vehicle->GetDriveshaftSpeed());

    // Update the vehicle system.
    m_vehicle->Update(time, steering, braking, m_powertrain->GetOutputTorque(), m_tire_forces);
}

}  // namespace chrono
