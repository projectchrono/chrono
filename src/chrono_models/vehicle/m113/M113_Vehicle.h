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

#ifndef M113_VEHICLE_H
#define M113_VEHICLE_H

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Definition of an M113 tracked vehicle with segmented tracks.
/// Both single-pin and double-pin track assemblies can be used with this vehicle model.
class CH_MODELS_API M113_Vehicle : public ChTrackedVehicle {
  public:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    M113_Vehicle(bool fixed,
                 TrackShoeType shoe_type,
                 DoublePinTrackShoeType shoe_topology,
                 DrivelineTypeTV driveline_type,
                 BrakeType brake_type,
                 bool use_track_bushings,
                 bool use_suspension_bushings,
                 bool use_track_RSDA,
                 ChContactMethod contact_method = ChContactMethod::NSC,
                 CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the M113 vehicle within the specified Chrono system.
    M113_Vehicle(bool fixed,
                 TrackShoeType shoe_type,
                 DoublePinTrackShoeType shoe_topology,
                 DrivelineTypeTV driveline_type,
                 BrakeType brake_type,
                 bool use_track_bushings,
                 bool use_suspension_bushings,
                 bool use_track_RSDA,
                 ChSystem* system,
                 CollisionType chassis_collision_type = CollisionType::NONE);

    ~M113_Vehicle() {}

    /// Create the track shoes (default: true).
    void CreateTrack(bool val) { m_create_track = val; }

    /// Initialize the M113 vehicle at the specified location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed,
                TrackShoeType shoe_type,
                DoublePinTrackShoeType shoe_topology,
                DrivelineTypeTV driveline_type,
                BrakeType brake_type,
                bool use_track_bushings,
                bool use_suspension_bushings,
                bool use_track_RSDA,
                CollisionType chassis_collision_type);

    bool m_create_track;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
