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

#include "chrono_models/vehicle/m113/M113_SimpleCVTPowertrain.h"
#include "chrono_models/vehicle/m113/M113_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/m113/M113_ShaftsPowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
M113::M113()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(collision::ChCollisionSystemType::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_create_track(true),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_shoe_topology(DoublePinTrackShoeType::TWO_CONNECTORS),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_powertrain_type(PowertrainModelType::SIMPLE_CVT),
      m_use_track_bushings(false),
      m_use_suspension_bushings(false),
      m_use_track_RSDA(false),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

M113::M113(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(collision::ChCollisionSystemType::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_fixed(false),
      m_create_track(true),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_shoe_topology(DoublePinTrackShoeType::TWO_CONNECTORS),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_powertrain_type(PowertrainModelType::SIMPLE_CVT),
      m_use_track_bushings(false),
      m_use_suspension_bushings(false),
      m_use_track_RSDA(false),
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
    if (m_system) {
        m_vehicle = new M113_Vehicle(m_fixed, m_shoe_type, m_shoe_topology, m_driveline_type, m_brake_type,
                                     m_use_track_bushings, m_use_suspension_bushings, m_use_track_RSDA, m_system,
                                     m_chassisCollisionType);
    } else {
        m_vehicle = new M113_Vehicle(m_fixed, m_shoe_type, m_shoe_topology, m_driveline_type, m_brake_type,
                                     m_use_track_bushings, m_use_suspension_bushings, m_use_track_RSDA, m_contactMethod,
                                     m_chassisCollisionType);
        m_vehicle->SetCollisionSystemType(m_collsysType);
    }
    m_vehicle->CreateTrack(m_create_track);
    m_vehicle->GetTrackAssembly(LEFT)->SetWheelCollisionType(m_wheel_cyl, m_idler_cyl, true);
    m_vehicle->GetTrackAssembly(RIGHT)->SetWheelCollisionType(m_wheel_cyl, m_idler_cyl, true);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    switch (m_powertrain_type) {
        default:
            std::cout << "Warning! M113 powertrain type not supported. Reverting to Simple CVT Powertrain" << std::endl;
        case PowertrainModelType::SIMPLE_CVT: {
            auto powertrain = chrono_types::make_shared<M113_SimpleCVTPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE_MAP: {
            auto powertrain = chrono_types::make_shared<M113_SimpleMapPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SHAFTS: {
            auto powertrain = chrono_types::make_shared<M113_ShaftsPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
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

}  // namespace m113
}  // end namespace vehicle
}  // end namespace chrono
