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
// Authors: Justin Madsen,
// =============================================================================
//
// Tracked vehicle model built from subsystems specified with hardcoded values
// as static const variables in the subsystem .cpp files
//
// =============================================================================

#ifndef TRACKVEHICLE_H
#define TRACKVEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"
#include "ModelDefs.h"

#include "subsys/ChTrackVehicle.h"
#include "subsys/trackSystem/TrackSystem.h"
#include "subsys/powertrain/TrackPowertrain.h"
// #include "subsys/driveline/TrackDriveline.h"

namespace chrono {

/// Model a tracked vehicle using a subsystem based approach. A single chassis/hull
/// has a number of track systems, (e.g. 2). Each track system contains the track chain
/// and all subsystems that attach to the chassis/hull. Each subsystem is currently defined
/// in the respective .cpp file as static const values. Takes a set of throttle and brake
/// user-inputs for each track system. Steering is handled by modifying the throttle and brake
/// values directly.
///
/// Usage:
///   >>  TrackVehicle tankA("tank_BravoDelta");
///   >>  tankA.Initialize( ChCoordsys<>(x0,q0));
///   >>  while ( simulate )
///   >>    tankA.Update( time, throttle, braking);
///   >>    tankA.Advance( step_size );
class CH_SUBSYS_API TrackVehicle : public ChTrackVehicle {
  public:
    TrackVehicle(
        const std::string& name,
        VisualizationType::Enum vis = VisualizationType::None,
        CollisionType::Enum collide = CollisionType::None,
        double pin_damping_coef = 0.0,
        double tensioner_preload = 1e5,
        double mass = 5489.2,                                          // default for M113
        const ChVector<>& Ixx = ChVector<>(1786.9, 10449.7, 10721.2),  // default for M113
        const ChVector<>& COG_to_REF = ChVector<>(),
        const ChVector<>& left_pos_rel = ChVector<>(0.23644, -0.4780, 0.83475),   // relative to chassis REF c-sys
        const ChVector<>& right_pos_rel = ChVector<>(0.23644, -0.4780, -0.83475)  // relative to chassis REF c-sys
        );

    ~TrackVehicle();

    /// Initialize the tracked vehicle REF frame with the specified Coordinate system.
    /// This initial transform is inherited by all vehicle subsystems.
    /// Will add the collision geometry and add modeling elements to the ChSystem.
    virtual void Initialize(const ChCoordsys<>& chassis_Csys);  ///< initial config of vehicle REF frame

    /// Update the vehicle with the new settings for throttle and brake
    virtual void Update(double time,                          ///< [in] current time
        const std::vector<double>& throttle,  ///< [in] current steering input [-1,+1]
        const std::vector<double>& braking    ///< [in] current braking input [0,1]
        );

    /// Advance the vehicle (and the ChSystem)
    virtual void Advance(double step);

    /// set the pin friction as a damping value
    virtual void SetShoePinDamping(double damping);

    // Accessors
    virtual double GetShoePinDamping() const { return m_pin_damping; }

    virtual double GetDriveshaftSpeed(size_t idx) const;

    /// pointer to the powertrain
    virtual ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const;

    /// current value of the integration step size for the vehicle system.
    double GetStepsize() const { return m_stepsize; }

    /// number of track chain systems attached to the vehicle
    int GetNum_TrackSystems() const { return m_num_tracks; }

  private:
    // private variables
    std::vector<ChVector<> > m_TrackSystem_locs;  ///< location of each tracksystem, relative to chassis REF c-sys
    std::vector<ChSharedPtr<TrackSystem> > m_TrackSystems;  ///< handles to track systems

    const size_t m_num_tracks;  ///< how many track systems to build
    double m_pin_damping;
    const double m_tensioner_preload;

    // static variables
    static const ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis
};

}  // end namespace chrono

#endif
