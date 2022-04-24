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
// M113 vehicle model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder_Chassis.h"
//#include "chrono_models/vehicle/marder/Marder_DrivelineBDS.h"
#include "chrono_models/vehicle/marder/Marder_SimpleDriveline.h"
//#include "chrono_models/vehicle/marder/Marder_TrackAssemblyBandBushing.h"
//#include "chrono_models/vehicle/marder/Marder_TrackAssemblyDoublePin.h"
#include "chrono_models/vehicle/marder/Marder_TrackAssemblySinglePin.h"
//#include "chrono_models/vehicle/marder/Marder_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/marder/Marder_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Marder_Vehicle::Marder_Vehicle(bool fixed,
                               TrackShoeType shoe_type,
                               DrivelineTypeTV driveline_type,
                               BrakeType brake_type,
                               ChContactMethod contact_method,
                               CollisionType chassis_collision_type)
    : ChTrackedVehicle("Marder", contact_method), m_create_track(true) {
    Create(fixed, shoe_type, driveline_type, brake_type, chassis_collision_type);
}

Marder_Vehicle::Marder_Vehicle(bool fixed,
                               TrackShoeType shoe_type,
                               DrivelineTypeTV driveline_type,
                               BrakeType brake_type,
                               ChSystem* system,
                               CollisionType chassis_collision_type)
    : ChTrackedVehicle("Marder", system), m_create_track(true) {
    Create(fixed, shoe_type, driveline_type, brake_type, chassis_collision_type);
}

void Marder_Vehicle::Create(bool fixed,
                            TrackShoeType shoe_type,
                            DrivelineTypeTV driveline_type,
                            BrakeType brake_type,
                            CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Marder_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the track assembly subsystems
    switch (shoe_type) {
        case TrackShoeType::SINGLE_PIN:
            m_tracks[0] = chrono_types::make_shared<Marder_TrackAssemblySinglePin>(LEFT, brake_type);
            m_tracks[1] = chrono_types::make_shared<Marder_TrackAssemblySinglePin>(RIGHT, brake_type);
            break;
        case TrackShoeType::DOUBLE_PIN:
            ////m_tracks[0] = chrono_types::make_shared<Marder_TrackAssemblyDoublePin>(LEFT, brake_type);
            ////m_tracks[1] = chrono_types::make_shared<Marder_TrackAssemblyDoublePin>(RIGHT, brake_type);
            GetLog() << "Unimplemented track assembly model.\n";
            break;
        case TrackShoeType::BAND_BUSHING:
            ////m_tracks[0] = chrono_types::make_shared<Marder_TrackAssemblyBandBushing>(LEFT, brake_type);
            ////m_tracks[1] = chrono_types::make_shared<Marder_TrackAssemblyBandBushing>(RIGHT, brake_type);
            GetLog() << "Unimplemented track assembly model.\n";
            break;
        case TrackShoeType::BAND_ANCF:
            ////m_tracks[0] = chrono_types::make_shared<Marder_TrackAssemblyBandANCF>(LEFT, brake_type);
            ////m_tracks[1] = chrono_types::make_shared<Marder_TrackAssemblyBandANCF>(RIGHT, brake_type);
            GetLog() << "Unimplemented track assembly model.\n";
            break;
    }

    // Create the driveline
    switch (driveline_type) {
        case DrivelineTypeTV::SIMPLE:
            m_driveline = chrono_types::make_shared<Marder_SimpleDriveline>();
            break;
        case DrivelineTypeTV::BDS:
            ////m_driveline = chrono_types::make_shared<Marder_DrivelineBDS>();
            GetLog() << "Unimplemented driveline model.\n";
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Marder_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the left and right track assemblies.
    double track_offset = 1.36;
    m_tracks[0]->Initialize(m_chassis, ChVector<>(0, track_offset, 0), m_create_track);
    m_tracks[1]->Initialize(m_chassis, ChVector<>(0, -track_offset, 0), m_create_track);

    // Initialize the driveline subsystem
    m_driveline->Initialize(m_chassis, m_tracks[0], m_tracks[1]);

    // Invoke base class method
    ChTrackedVehicle::Initialize(chassisPos, chassisFwdVel);
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
