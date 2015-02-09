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

#include "subsys/base/ChTrackVehicle.h"
#include "subsys/trackSystem/TrackSystem.h"
#include "subsys/powertrain/TrackPowertrain.h"
#include "subsys/driveline/TrackDriveline.h"

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
class CH_SUBSYS_API TrackVehicle : public ChTrackVehicle
{
public:

  TrackVehicle(const std::string& name,
    VisualizationType chassisVis = VisualizationType::PRIMITIVES,
    CollisionType chassisCollide = CollisionType::NONE);

  ~TrackVehicle();

  // multiply lateral directions to get correct coord
  enum TrackVehicleSide{
    LEFT = -1,
    RIGHT = 1
  };

  /// Initialize the tracked vehicle REF frame with the specified Coordinate system.
  /// This initial transform is inherited by all vehicle subsystems.
  /// Will add the collision geometry and add modeling elements to the ChSystem.
  virtual void Initialize(const ChCoordsys<>& chassis_Csys);  ///< [in] initial config of vehicle REF frame
  
  /// Update the vehicle with the new settings for throttle and brake
  virtual void Update(double	time,
    const std::vector<double>&  throttle, 
    const std::vector<double>&  braking);

  /// Advance the vehicle (and the ChSystem)
  virtual void Advance(double step);


  // Accessors
  virtual double GetDriveshaftSpeed(size_t idx) const;

  /// pointer to the powertrain
  virtual const ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const;

  /// vehicle's driveline subsystem.
  const ChSharedPtr<TrackDriveline> GetDriveline(int idx) const { return m_drivelines[idx]; }

  /// vehicle's driveshaft body.
  const ChSharedPtr<ChShaft> GetDriveshaft(size_t idx) const {return m_drivelines[idx]->GetDriveshaft(); }

  /// current value of the integration step size for the vehicle system.
  double GetStepsize() const { return m_stepsize; }


  /// number of track chain systems attached to the vehicle
  int GetNum_TrackSystems() const { return m_num_tracks; }

  // not really relevant, since it's a static system
  // ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

private:

  // private variables
  std::vector<ChVector<> > m_TrackSystem_locs;   // locations of the track system c-sys relative to chassis
  std::vector<ChSharedPtr<TrackSystem> > m_TrackSystems;	// list of track systems
  int m_num_tracks; ///< how many track systems to build

  std::vector<ChSharedPtr<TrackDriveline>>   m_drivelines;    ///< handle to the driveline subsystem, one for each powertrain/drivegear pair

  // static variables
  static const ChVector<> m_trackPos_Left;  // relative to chassis c-sys
  static const ChVector<> m_trackPos_Right;

  static const double     m_mass;                   // chassis mass
  static const ChVector<> m_COM;                    // location of the chassis COM in the local ref frame
  static const ChVector<> m_inertia;                // symmetric moments of inertia of the chassis

  static const ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis

};


} // end namespace chrono


#endif
