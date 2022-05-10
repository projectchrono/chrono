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

#include "chrono_models/vehicle/m113/M113_Chassis.h"
#include "chrono_models/vehicle/m113/M113_DrivelineBDS.h"
#include "chrono_models/vehicle/m113/M113_SimpleDriveline.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandBushing.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyDoublePin.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblySinglePin.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
M113_Vehicle::M113_Vehicle(bool fixed,
                           TrackShoeType shoe_type,
                           DoublePinTrackShoeType shoe_topology,
                           DrivelineTypeTV driveline_type,
                           BrakeType brake_type,
                           bool use_track_bushings,
                           bool use_suspension_bushings,
                           bool use_track_RSDA,
                           ChContactMethod contact_method,
                           CollisionType chassis_collision_type)
    : ChTrackedVehicle("M113", contact_method), m_create_track(true) {
    Create(fixed, shoe_type, shoe_topology, driveline_type, brake_type, use_track_bushings, use_suspension_bushings,
           use_track_RSDA, chassis_collision_type);
}

M113_Vehicle::M113_Vehicle(bool fixed,
                           TrackShoeType shoe_type,
                           DoublePinTrackShoeType shoe_topology,
                           DrivelineTypeTV driveline_type,
                           BrakeType brake_type,
                           bool use_track_bushings,
                           bool use_suspension_bushings,
                           bool use_track_RSDA,
                           ChSystem* system,
                           CollisionType chassis_collision_type)
    : ChTrackedVehicle("M113", system), m_create_track(true) {
    Create(fixed, shoe_type, shoe_topology, driveline_type, brake_type, use_track_bushings, use_suspension_bushings,
           use_track_RSDA, chassis_collision_type);
}

void M113_Vehicle::Create(bool fixed,
                          TrackShoeType shoe_type,
                          DoublePinTrackShoeType shoe_topology,
                          DrivelineTypeTV driveline_type,
                          BrakeType brake_type,
                          bool use_track_bushings,
                          bool use_suspension_bushings,
                          bool use_track_RSDA,
                          CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<M113_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the track assembly subsystems
    switch (shoe_type) {
        case TrackShoeType::SINGLE_PIN:
            m_tracks[0] = chrono_types::make_shared<M113_TrackAssemblySinglePin>(
                LEFT, brake_type, use_track_bushings, use_suspension_bushings, use_track_RSDA);
            m_tracks[1] = chrono_types::make_shared<M113_TrackAssemblySinglePin>(
                RIGHT, brake_type, use_track_bushings, use_suspension_bushings, use_track_RSDA);
            break;
        case TrackShoeType::DOUBLE_PIN:
            m_tracks[0] = chrono_types::make_shared<M113_TrackAssemblyDoublePin>(
                LEFT, shoe_topology, brake_type, use_track_bushings, use_suspension_bushings, use_track_RSDA);
            m_tracks[1] = chrono_types::make_shared<M113_TrackAssemblyDoublePin>(
                RIGHT, shoe_topology, brake_type, use_track_bushings, use_suspension_bushings, use_track_RSDA);
            break;
        case TrackShoeType::BAND_BUSHING:
            m_tracks[0] =
                chrono_types::make_shared<M113_TrackAssemblyBandBushing>(LEFT, brake_type, use_suspension_bushings);
            m_tracks[1] =
                chrono_types::make_shared<M113_TrackAssemblyBandBushing>(RIGHT, brake_type, use_suspension_bushings);
            break;
        case TrackShoeType::BAND_ANCF:
            m_tracks[0] =
                chrono_types::make_shared<M113_TrackAssemblyBandANCF>(LEFT, brake_type, use_suspension_bushings);
            m_tracks[1] =
                chrono_types::make_shared<M113_TrackAssemblyBandANCF>(RIGHT, brake_type, use_suspension_bushings);
            break;
    }

    // Create the driveline
    switch (driveline_type) {
        case DrivelineTypeTV::SIMPLE:
            m_driveline = chrono_types::make_shared<M113_SimpleDriveline>();
            break;
        case DrivelineTypeTV::BDS:
            m_driveline = chrono_types::make_shared<M113_DrivelineBDS>();
            break;
    }

    GetLog() << "M113 vehicle mass = " << GetMass() << " kg.\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the left and right track assemblies.
    double track_offset = 1.0795;
    m_tracks[0]->Initialize(m_chassis, ChVector<>(0, track_offset, 0), m_create_track);
    m_tracks[1]->Initialize(m_chassis, ChVector<>(0, -track_offset, 0), m_create_track);

    // Initialize the driveline subsystem
    m_driveline->Initialize(m_chassis, m_tracks[0], m_tracks[1]);

    // Invoke base class method
    ChTrackedVehicle::Initialize(chassisPos, chassisFwdVel);
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
