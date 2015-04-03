// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen 
// =============================================================================
//
// Tracked vehicle model built from subsystems specified with hardcoded values
// Driven by directly applying rotational motion to sprocket bodies
//
// =============================================================================

#ifndef TRACKVEHICLEM113_H
#define TRACKVEHICLEM113_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"
#include "ModelDefs.h"

#include "subsys/base/ChTrackVehicle.h"
#include "subsys/trackSystem/TrackSystemM113.h"

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
class CH_SUBSYS_API TrackVehicleM113 : public ChTrackVehicle
{
public:

  TrackVehicleM113(const std::string& name,
    VisualizationType::Enum vis = VisualizationType::None,
    CollisionType::Enum collide = CollisionType::None,
    double mass = 5489.2, // default for M113 APC
    const ChVector<>& Ixx = ChVector<>(1786.9, 10449.7, 10721.2), // default for M113 APC
    double pin_damping_coef = 0.5, // non-zero, since pins between connected shoes always are pin/rubber bushing type elements.
    double tensioner_preload = 1e4, // idler tensioner preload
    const ChVector<>& left_pos_rel = ChVector<>(0.23644, -0.4780, 0.83475),  // relative to chassis REF c-sys
    const ChVector<>& right_pos_rel = ChVector<>(0.23644, -0.4780, -0.83475)// relative to chassis REF c-sys,
    );

  ~TrackVehicleM113();

  /// Initialize the tracked vehicle REF frame with the specified Coordinate system.
  /// This initial transform is inherited by all vehicle subsystems.
  /// Will add the collision geometry and add modeling elements to the ChSystem.
  virtual void Initialize(const ChCoordsys<>& chassis_Csys);  ///< initial config of vehicle REF frame
  
  /// Update the vehicle with the new settings for throttle and brake
  virtual void Update(double	time,
    const std::vector<double>&  throttle, 
    const std::vector<double>&  braking);

  /// Advance the vehicle (and the ChSystem)
  virtual void Advance(double step);

  /// set the pin friction as a damping value
  virtual void SetShoePinDamping(double damping);

  // Accessors
  /// current value of the integration step size for the vehicle system.
  double GetStepsize() const { return m_stepsize; }

  /// number of track chain systems attached to the vehicle
  int GetNum_TrackSystems() const { return m_num_tracks; }
  
  // Accessors
  // not really relevant, but required
  virtual double GetDriveshaftSpeed(size_t idx) const;

  /// pointer to the powertrain
  virtual const ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const;

  /// return the contant part of the damping (if any)
  virtual double GetShoePinDamping() const {return m_damping;}

private:

  // private variables
  std::vector<ChVector<> > m_TrackSystem_locs;   ///< location of each tracksystem, relative to chassis REF c-sys
  std::vector<ChSharedPtr<TrackSystemM113> > m_TrackSystems;	///< handles to track systems
  const ChVector<> m_trackSys_L;  ///< where to place left track system origin, relative to chassis REF c-sys
  const ChVector<> m_trackSys_R;  ///< where to place right track system origin, relative to chassis REF c-sys
  const size_t m_num_tracks;      ///< how many track systems to build
  double m_damping;         ///< damping coef. applied between shoe bodies, to the rev. constraint
  double m_tensioner_preload;

  // static variables
  static const double     mass_override;     // override chassis mass input
  static const ChVector<> COM_override;      // location of the chassis COM in the local ref frame
  static const ChVector<> inertia_override;  // symmetric moments of inertia of the chassis

  static const ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis

  // placeholder
  ChSharedPtr<ChShaft> m_axle;  // dummy shaft
};


} // end namespace chrono


#endif
