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
// Wrapper classes for modeling an entire M113 vehicle assembly
// (including the vehicle itself and the powertrain).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_models/vehicle/m113/M113_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
M113::M113()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

M113::M113(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

M113::~M113() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void M113::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void M113::Initialize() {
    // Create and initialize the M113 vehicle
    m_vehicle = m_system
                    ? new M113_Vehicle(m_fixed, m_shoe_type, m_brake_type, m_system, m_chassisCollisionType)
                    : new M113_Vehicle(m_fixed, m_shoe_type, m_brake_type, m_contactMethod, m_chassisCollisionType);

    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<M113_SimplePowertrain>("Powertrain");
    m_vehicle->InitializePowertrain(powertrain);
}

// -----------------------------------------------------------------------------
void M113::Synchronize(double time,
                       const ChDriver::Inputs& driver_inputs,
                       const TerrainForces& shoe_forces_left,
                       const TerrainForces& shoe_forces_right) {
    m_vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
}

// -----------------------------------------------------------------------------
void M113::Advance(double step) {
    m_vehicle->Advance(step);
}

// -----------------------------------------------------------------------------
double M113::GetTotalMass() const {
    return m_vehicle->GetVehicleMass();
}

}  // namespace m113
}  // end namespace vehicle
}  // end namespace chrono
