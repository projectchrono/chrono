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
// Utility classes for using Chrono::Vehicle models in a ChronoParallel
// application.
//
// =============================================================================

#include "chrono_parallel/collision/ChCCollisionModelParallel.h"

#include "chrono_utils/ChUtilsVehicle.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// Constructor from given JSON specification files.
// The vehicle system is constructed within the specified system, using the
// given JSON specification files for the vehicle and powertrain subsystems.
// These files are assumed to be given relative to the Chrono::Vehicle data
// directory.
// -----------------------------------------------------------------------------
VehicleSystem::VehicleSystem(ChSystem* system,
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
void VehicleSystem::Initialize(const ChVector<>& init_loc, const ChQuaternion<>& init_rot) {
  // Initialize the vehicle and powertrain systems.
  m_vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));
  m_powertrain->Initialize();

  // If provided, invoke the user-provided callback to attach chassis contact
  // geometry.
  if (m_chassis_cb) {
    ChSharedPtr<ChBodyAuxRef> chassisBody = m_vehicle->GetChassis();

    chassisBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);
    m_chassis_cb->onCallback(chassisBody);
    chassisBody->SetCollide(true);
  }

  // Loop over all vehicle wheels and attach tire contact geometry.
  for (int i = 0; i < 2 * m_vehicle->GetNumberAxles(); i++) {
    ChSharedPtr<ChBody> wheelBody = m_vehicle->GetWheelBody(i);
    double radius = m_vehicle->GetWheel(i)->GetRadius();
    double width = m_vehicle->GetWheel(i)->GetWidth();

    wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

    if (m_tire_cb) {
      // Invoke the user-provided callback to attach tire contact geometry.
      m_tire_cb->onCallback(wheelBody, radius, width);
    } else {
      // Default to cylinders and mu = 0.8
      wheelBody->GetCollisionModel()->ClearModel();
      wheelBody->GetCollisionModel()->AddCylinder(radius, radius, width / 2);
      wheelBody->GetCollisionModel()->BuildModel();

      wheelBody->GetMaterialSurface()->SetFriction(0.8f);
    }

    wheelBody->SetCollide(true);
  }
}

// -----------------------------------------------------------------------------
// Update the vehicle model at the specified time.
// -----------------------------------------------------------------------------
void VehicleSystem::Update(double time) {
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

}  // namespace utils
}  // namespace chrono
