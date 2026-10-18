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

#include "chrono_vehicle/ChConfigVehicle.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#ifdef CHRONO_FEA
    #include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"
#endif

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Base class for an M113 tracked vehicle with segmented or band tracks.
class CH_MODELS_API M113_Vehicle : public ChTrackedVehicle {
  public:
    virtual ~M113_Vehicle() {}

    /// Create the track shoes (default: true).
    void CreateTrack(bool val) { m_create_track = val; }

    /// Initialize the M113 vehicle at the specified location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  protected:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    M113_Vehicle(ChContactMethod contact_method);

    /// Construct the M113 vehicle within the specified Chrono system.
    M113_Vehicle(ChSystem* system);

    bool m_create_track;
};

/// M113 vehicle with segmented single-pin track shoes.
class CH_MODELS_API M113_Vehicle_SinglePin : public M113_Vehicle {
  public:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    M113_Vehicle_SinglePin(bool fixed,
                           DrivelineTypeTV driveline_type,
                           BrakeType brake_type,
                           bool use_track_bushings,
                           bool use_suspension_bushings,
                           bool use_track_RSDA,
                           ChContactMethod contact_method,
                           CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the M113 vehicle within the specified Chrono system.
    M113_Vehicle_SinglePin(bool fixed,
                           DrivelineTypeTV driveline_type,
                           BrakeType brake_type,
                           bool use_track_bushings,
                           bool use_suspension_bushings,
                           bool use_track_RSDA,
                           ChSystem* system,
                           CollisionType chassis_collision_type = CollisionType::NONE);

  private:
    /// Create the single-pin M113 vehicle with given parameters.
    void Create(bool fixed,
                DrivelineTypeTV driveline_type,
                BrakeType brake_type,
                bool use_track_bushings,
                bool use_suspension_bushings,
                bool use_track_RSDA,
                CollisionType chassis_collision_type);
};

/// M113 vehicle with segmented double-pin track shoes.
class CH_MODELS_API M113_Vehicle_DoublePin : public M113_Vehicle {
  public:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    M113_Vehicle_DoublePin(bool fixed,
                           DoublePinTrackShoeType shoe_topology,
                           DrivelineTypeTV driveline_type,
                           BrakeType brake_type,
                           bool use_track_bushings,
                           bool use_suspension_bushings,
                           bool use_track_RSDA,
                           ChContactMethod contact_method,
                           CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the M113 vehicle within the specified Chrono system.
    M113_Vehicle_DoublePin(bool fixed,
                           DoublePinTrackShoeType shoe_topology,
                           DrivelineTypeTV driveline_type,
                           BrakeType brake_type,
                           bool use_track_bushings,
                           bool use_suspension_bushings,
                           bool use_track_RSDA,
                           ChSystem* system,
                           CollisionType chassis_collision_type = CollisionType::NONE);

  private:
    /// Create the double-pin M113 vehicle with given parameters.
    void Create(bool fixed,
                DoublePinTrackShoeType shoe_topology,
                DrivelineTypeTV driveline_type,
                BrakeType brake_type,
                bool use_track_bushings,
                bool use_suspension_bushings,
                bool use_track_RSDA,
                CollisionType chassis_collision_type);
};

/// M113 vehicle with bushings-based continuous tracks.
class CH_MODELS_API M113_Vehicle_BandBushing : public M113_Vehicle {
  public:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    M113_Vehicle_BandBushing(bool fixed,
                             DrivelineTypeTV driveline_type,
                             BrakeType brake_type,
                             bool use_suspension_bushings,
                             ChContactMethod contact_method,
                             CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the M113 vehicle within the specified Chrono system.
    M113_Vehicle_BandBushing(bool fixed,
                             DrivelineTypeTV driveline_type,
                             BrakeType brake_type,
                             bool use_suspension_bushings,
                             ChSystem* system,
                             CollisionType chassis_collision_type = CollisionType::NONE);

  private:
    /// Create the band-bushing M113 vehicle with given parameters.
    void Create(bool fixed, DrivelineTypeTV driveline_type, BrakeType brake_type, bool use_suspension_bushings, CollisionType chassis_collision_type);
};

#ifdef CHRONO_FEA

/// M113 vehicle with ANCF-based continuous tracks.
class CH_MODELS_API M113_Vehicle_BandANCF : public M113_Vehicle {
  public:
    /// Construct the M113 vehicle within an automatically created Chrono system.
    M113_Vehicle_BandANCF(bool fixed,
                          ChTrackShoeBandANCF::ElementType element_type,
                          bool constrain_curvature,
                          int num_elements_length,
                          int num_elements_width,
                          DrivelineTypeTV driveline_type,
                          BrakeType brake_type,
                          bool use_suspension_bushings,
                          ChContactMethod contact_method,
                          CollisionType chassis_collision_type = CollisionType::NONE);

    /// Construct the M113 vehicle within the specified Chrono system.
    M113_Vehicle_BandANCF(bool fixed,
                          ChTrackShoeBandANCF::ElementType element_type,
                          bool constrain_curvature,
                          int num_elements_length,
                          int num_elements_width,
                          DrivelineTypeTV driveline_type,
                          BrakeType brake_type,
                          bool use_suspension_bushings,
                          ChSystem* system,
                          CollisionType chassis_collision_type = CollisionType::NONE);

  private:
    /// Create the band-ANCF M113 vehicle with given parameters.
    void Create(bool fixed,
                ChTrackShoeBandANCF::ElementType element_type,
                bool constrain_curvature,
                int num_elements_length,
                int num_elements_width,
                DrivelineTypeTV driveline_type,
                BrakeType brake_type,
                bool use_suspension_bushings,
                CollisionType chassis_collision_type);
};

#endif

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
