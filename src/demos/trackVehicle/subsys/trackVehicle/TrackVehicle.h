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
class CH_SUBSYS_API TrackVehicle : public ChSystem
{
public:

  TrackVehicle(const std::string& name,
    VisualizationType chassisVis = VisualizationType::PRIMITIVES,
    CollisionType chassisCollide = CollisionType::PRIMITIVES);

  ~TrackVehicle();

  // multiply lateral directions to get correct coord
  enum TrackVehicleSide{
    LEFT = -1,
    RIGHT = 1
  };

  /// Initialize the tracked vehicle REF frame with the specified Coordinate system.
  /// This initial transform is inherited by all vehicle subsystems.
  /// Will add the collision geometry and add modeling elements to the ChSystem.
  void Initialize(const ChCoordsys<>& chassis_Csys);  ///< [in] initial config of vehicle REF frame
  
  /// Update the vehicle with the new settings for throttle and brake
  void Update(double	time,
              const std::vector<double>&  throttle,
			        const std::vector<double>&  braking);

  /// Advance the vehicle (and the ChSystem)
  void Advance(double step);

  /// integration step size for the vehicle system.
  void SetStepsize(double val) { m_stepsize = val; }

  // Accessors

  /// global location of the chassis reference frame origin.
  const ChVector<>& GetChassisPos() const { return m_chassis->GetFrame_REF_to_abs().GetPos(); }

  /// orientation of the chassis reference frame.
  /// The chassis orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetChassisRot() const { return m_chassis->GetFrame_REF_to_abs().GetRot(); }

  /// global location of the chassis center of mass.
  const ChVector<>& GetChassisPosCOM() const { return m_chassis->GetPos(); }

  /// orientation of the chassis centroidal frame.
  /// The chassis orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetChassisRotCOM() const { return m_chassis->GetRot(); }

  /// vehicle speed.
  /// Return the speed measured at the origin of the chassis reference frame.
  double GetVehicleSpeed() const { return m_chassis->GetFrame_REF_to_abs().GetPos_dt().Length(); }

  /// speed of the chassis COM.
  /// Return the speed measured at the chassis center of mass.
  double GetVehicleSpeedCOM() const { return m_chassis->GetPos_dt().Length(); }

  /// vehicle's driveline subsystem.
  const ChSharedPtr<TrackDriveline> GetDriveline(int idx) const { return m_drivelines[idx]; }

  /// vehicle's driveshaft body.
  const ChSharedPtr<ChShaft> GetDriveshaft(size_t idx) const {return m_drivelines[idx]->GetDriveshaft(); }

  /// current value of the integration step size for the vehicle system.
  double GetStepsize() const { return m_stepsize; }

  /// shared pointer to chassis body
  ChSharedPtr<ChBody> GetChassis() { return m_chassis; }

  /// shared pointer to the powertrain
  ChSharedPtr<TrackPowertrain> GetPowertrain(int idx) { return m_ptrains[idx]; }

  /// number of track chain systems attached to the vehicle
  int GetNum_TrackSystems() const { return m_num_tracks; }

  ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

private:

  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  void AddVisualization();
  void AddCollisionGeometry();

  // private variables

  ChSharedPtr<ChBodyAuxRef> m_chassis;  ///< hull body

  std::vector<ChVector<> > m_TrackSystem_locs;   // locations of the track system c-sys relative to chassis
  std::vector<ChSharedPtr<TrackSystem> > m_TrackSystems;	// list of track systems

  std::vector<ChSharedPtr<TrackDriveline>>   m_drivelines;    ///< handle to the driveline subsystem, one for each powertrain/drivegear pair
  std::vector<ChSharedPtr<TrackPowertrain>>  m_ptrains;  ///< powertrain system, one per track system

  VisualizationType m_vis;  // how to visualize chassis geometry
  CollisionType m_collide;  // how to handle chassis collision geometry

  // static variables
  static const size_t m_num_tracks;  // number of tracks for this vehicle
  static const size_t m_num_engines; // number of engines (powertrains and drivelines)

  static const ChVector<> m_trackPos_Left;  // relative to chassis c-sys
  static const ChVector<> m_trackPos_Right;

  static const double     m_mass;                   // chassis mass
  static const ChVector<> m_COM;                    // location of the chassis COM in the local ref frame
  static const ChVector<> m_inertia;                // symmetric moments of inertia of the chassis

  static const ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis
  static const std::string m_meshName;
  static const std::string  m_meshFile;
  static const ChVector<> m_chassisBoxSize; // length, height, width of chassis collision box (if collisiontype = PRIMITIVES)

  double m_stepsize;          ///< integration time step for tracked vehicle system

protected:
  ChSystem*                  m_system;       ///< pointer to the Chrono system
  bool                       m_ownsSystem;   ///< true if system created at construction

  // friend class irrDriver
};


} // end namespace chrono


#endif
