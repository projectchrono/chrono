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
// Authors: Rainer Gericke
// =============================================================================
//
// Wrapper classes for modeling an entire Marder vehicle assembly
// (including the vehicle itself and the powertrain).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder.h"

//#include "chrono_models/vehicle/marder/Marder_SimplePowertrain.h"
//#include "chrono_models/vehicle/marder/Marder_SimpleCVTPowertrain.h"
// #include "chrono_models/vehicle/marder/Marder_ShaftsPowertrain.h"
#include "chrono_models/vehicle/marder/powertrain/Marder_EngineShafts.h"
#include "chrono_models/vehicle/marder/powertrain/Marder_AutomaticTransmissionShafts.h"
#include "chrono_models/vehicle/marder/powertrain/Marder_EngineSimpleMap.h"
#include "chrono_models/vehicle/marder/powertrain/Marder_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/marder/powertrain/Marder_EngineSimple.h"
#include "chrono_models/vehicle/marder/powertrain/Marder_AutomaticTransmissionSimple.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
Marder::Marder()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_collsys_type(ChCollisionSystem::Type::BULLET),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_roller_cyl(true),
      m_fixed(false),
      m_create_track(true),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

Marder::Marder(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_collsys_type(ChCollisionSystem::Type::BULLET),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_roller_cyl(true),
      m_fixed(false),
      m_create_track(true),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_apply_drag(false) {}

Marder::~Marder() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void Marder::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void Marder::Initialize() {
    // Create and initialize the Marder vehicle
    if (m_system) {
        m_vehicle =
            new Marder_Vehicle(m_fixed, m_shoe_type, m_driveline_type, m_brake_type, m_system, m_chassisCollisionType);
    } else {
        m_vehicle = new Marder_Vehicle(m_fixed, m_shoe_type, m_driveline_type, m_brake_type, m_contactMethod,
                                       m_chassisCollisionType);
        m_vehicle->SetCollisionSystemType(m_collsys_type);
    }

    m_vehicle->CreateTrack(m_create_track);
    m_vehicle->GetTrackAssembly(LEFT)->SetWheelCollisionType(m_wheel_cyl, m_idler_cyl, m_roller_cyl);
    m_vehicle->GetTrackAssembly(RIGHT)->SetWheelCollisionType(m_wheel_cyl, m_idler_cyl, m_roller_cyl);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine;
    std::shared_ptr<ChTransmission> transmission;
    switch (m_engineType) {
        case EngineModelType::SHAFTS:
            engine = chrono_types::make_shared<Marder_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<Marder_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<Marder_EngineSimple>("Engine");
            transmission = chrono_types::make_shared<Marder_AutomaticTransmissionSimple>("Transmission");
            break;
    }

    if (!transmission) {
        switch (m_transmissionType) {
            case TransmissionModelType::AUTOMATIC_SHAFTS:
                transmission = chrono_types::make_shared<Marder_AutomaticTransmissionShafts>("Transmission");
                break;
            case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
                transmission = chrono_types::make_shared<Marder_AutomaticTransmissionSimpleMap>("Transmission");
                break;
        }
    }

    if (engine && transmission) {
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------

void Marder::Synchronize(double time,
                         const DriverInputs& driver_inputs) {
    m_vehicle->Synchronize(time, driver_inputs);
}

void Marder::Synchronize(double time,
                         const DriverInputs& driver_inputs,
                         const TerrainForces& shoe_forces_left,
                         const TerrainForces& shoe_forces_right) {
    m_vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
}

void Marder::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
