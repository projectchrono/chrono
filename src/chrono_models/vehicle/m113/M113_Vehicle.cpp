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
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

#ifdef CHRONO_FEA
#include "chrono_models/vehicle/m113/M113_TrackAssemblyBandANCF.h"
#endif

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
M113_Vehicle::M113_Vehicle(bool fixed,
                           TrackShoeType shoe_type,
                           ChMaterialSurface::ContactMethod contact_method,
                           ChassisCollisionType chassis_collision_type)
    : ChTrackedVehicle("M113", contact_method), m_type(shoe_type) {
    Create(fixed, chassis_collision_type);
}

M113_Vehicle::M113_Vehicle(bool fixed,
                           TrackShoeType shoe_type,
                           ChSystem* system,
                           ChassisCollisionType chassis_collision_type)
    : ChTrackedVehicle("M113", system), m_type(shoe_type) {
    Create(fixed, chassis_collision_type);
}

void M113_Vehicle::Create(bool fixed, ChassisCollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = std::make_shared<M113_Chassis>("Chassis", fixed, chassis_collision_type);

#ifndef CHRONO_FEA
    // If BAND_ANCF type track was selected but is not available, force BAND_BUSHING.
    if (m_type == TrackShoeType::BAND_ANCF)
        m_type = TrackShoeType::BAND_BUSHING;
#endif

    // Create the track assembly subsystems
    switch (m_type) {
        case TrackShoeType::SINGLE_PIN:
            m_tracks[0] = std::make_shared<M113_TrackAssemblySinglePin>(LEFT);
            m_tracks[1] = std::make_shared<M113_TrackAssemblySinglePin>(RIGHT);
            break;
        case TrackShoeType::DOUBLE_PIN:
            m_tracks[0] = std::make_shared<M113_TrackAssemblyDoublePin>(LEFT);
            m_tracks[1] = std::make_shared<M113_TrackAssemblyDoublePin>(RIGHT);
            break;
        case TrackShoeType::BAND_BUSHING:
            m_tracks[0] = std::make_shared<M113_TrackAssemblyBandBushing>(LEFT);
            m_tracks[1] = std::make_shared<M113_TrackAssemblyBandBushing>(RIGHT);
            break;
        case TrackShoeType::BAND_ANCF:
#ifdef CHRONO_FEA
            m_tracks[0] = std::make_shared<M113_TrackAssemblyBandANCF>(LEFT);
            m_tracks[1] = std::make_shared<M113_TrackAssemblyBandANCF>(RIGHT);
#endif
            break;
    }

    // Create the driveline
    m_driveline = std::make_shared<M113_SimpleDriveline>();
    ////m_driveline = std::make_shared<M113_DrivelineBDS>();

    GetLog() << "M113 vehicle mass = " << GetVehicleMass() << " kg.\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Invoke base class method to initialize the chassis.
    ChTrackedVehicle::Initialize(chassisPos, chassisFwdVel);

    // Initialize the left and right track assemblies.
    double track_offset = 1.0795;
    m_tracks[0]->Initialize(m_chassis->GetBody(), ChVector<>(0, track_offset, 0));
    m_tracks[1]->Initialize(m_chassis->GetBody(), ChVector<>(0, -track_offset, 0));

    // Initialize the driveline subsystem
    m_driveline->Initialize(m_chassis->GetBody(), m_tracks[0], m_tracks[1]);
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
