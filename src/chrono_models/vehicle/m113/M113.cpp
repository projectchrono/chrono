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

#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineShafts.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionShafts.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimpleMap.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimple.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimple.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
M113::M113()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_create_track(true),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_shoe_topology(DoublePinTrackShoeType::TWO_CONNECTORS),
      m_ancf_element_type(ChTrackShoeBandANCF::ElementType::ANCF_4),
      m_ancf_num_elements_length(3),
      m_ancf_num_elements_width(4),
      m_ancf_constrain_curvature(false),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_use_track_bushings(false),
      m_use_suspension_bushings(false),
      m_use_track_RSDA(false),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_gyration_mode(false),
      m_apply_drag(false) {}

M113::M113(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_wheel_cyl(true),
      m_idler_cyl(true),
      m_fixed(false),
      m_create_track(true),
      m_brake_type(BrakeType::SIMPLE),
      m_shoe_type(TrackShoeType::SINGLE_PIN),
      m_shoe_topology(DoublePinTrackShoeType::TWO_CONNECTORS),
      m_ancf_element_type(ChTrackShoeBandANCF::ElementType::ANCF_4),
      m_ancf_num_elements_length(3),
      m_ancf_num_elements_width(4),
      m_ancf_constrain_curvature(false),
      m_driveline_type(DrivelineTypeTV::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_use_track_bushings(false),
      m_use_suspension_bushings(false),
      m_use_track_RSDA(false),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_gyration_mode(false),
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
        m_vehicle = new M113_Vehicle(m_fixed, m_shoe_type, m_shoe_topology, m_ancf_element_type,
                                     m_ancf_constrain_curvature, m_ancf_num_elements_length, m_ancf_num_elements_width,
                                     m_driveline_type, m_brake_type, m_use_track_bushings, m_use_suspension_bushings,
                                     m_use_track_RSDA, m_system, m_chassisCollisionType);
    } else {
        m_vehicle = new M113_Vehicle(m_fixed, m_shoe_type, m_shoe_topology, m_ancf_element_type,
                                     m_ancf_constrain_curvature, m_ancf_num_elements_length, m_ancf_num_elements_width,
                                     m_driveline_type, m_brake_type, m_use_track_bushings, m_use_suspension_bushings,
                                     m_use_track_RSDA, m_contactMethod, m_chassisCollisionType);
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

    m_vehicle->GetDriveline()->SetGyrationMode(m_gyration_mode);

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine;
    std::shared_ptr<ChTransmission> transmission;
    switch (m_engineType) {
        case EngineModelType::SHAFTS:
            engine = chrono_types::make_shared<M113_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<M113_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<M113_EngineSimple>("Engine");
            transmission = chrono_types::make_shared<M113_AutomaticTransmissionSimple>("Transmission");
            break;
    }

    if(!transmission) {
        switch (m_transmissionType) {
            case TransmissionModelType::AUTOMATIC_SHAFTS:
                transmission = chrono_types::make_shared<M113_AutomaticTransmissionShafts>("Transmission");
                break;
            case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
                transmission = chrono_types::make_shared<M113_AutomaticTransmissionSimpleMap>("Transmission");
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

void M113::Synchronize(double time,
                       const DriverInputs& driver_inputs) {
    m_vehicle->Synchronize(time, driver_inputs);
}

void M113::Synchronize(double time,
                       const DriverInputs& driver_inputs,
                       const TerrainForces& shoe_forces_left,
                       const TerrainForces& shoe_forces_right) {
    m_vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
}

void M113::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace m113
}  // end namespace vehicle
}  // end namespace chrono
