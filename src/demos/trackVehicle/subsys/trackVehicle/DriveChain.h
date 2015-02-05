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
// Authors: Justin Madsen
// =============================================================================
//
// Use some track vehicle components to model a drive gear, attached to a powertrain
// via a driveline. Wrap a chain around the drive gear and idler, to which an
// inertial load will be applied.
// It would be easy to turn this into a motorcycle dynamometer.
//
// =============================================================================

#ifndef DRIVECHAIN_H
#define DRIVECHAIN_H

#include "core/ChCoordsys.h"
// #include "physics/ChSystem.h"
#include "ModelDefs.h"

#include "subsys/base/ChTrackVehicle.h"
#include "subsys/driveGear/DriveGear.h"
#include "subsys/idler/IdlerSimple.h"
#include "subsys/trackChain/TrackChain.h"
#include "subsys/powertrain/TrackPowertrain.h"
#include "subsys/idler/SupportRoller.h"

namespace chrono {

/// Model a static DriveChain, with a powered Gear and
/// an idler attached to some large inertial load
class CH_SUBSYS_API DriveChain : public ChTrackVehicle
{
public:

  DriveChain(const std::string& name,
    VisualizationType chassisVis = VisualizationType::PRIMITIVES,
    CollisionType chassisCollide = CollisionType::PRIMITIVES);

  ~DriveChain();
  
  /// Initialize the drive chain system
  void Initialize(const ChCoordsys<>& gear_Csys);  ///< [in] initial config of gear
  
  /// Update the vehicle with the new settings for throttle and brake
  virtual void Update(double time,
              double throttle,
			  double braking);

  /// Advance the vehicle (and the ChSystem)
  virtual void Advance(double step);


  // Accessors

  /// Get the angular speed of the driveshaft.
  virtual double GetDriveshaftSpeed(size_t idx) const { return m_gear->GetAxle()->GetPos_dt(); }

  /// pointer to the powertrain
  virtual const ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const { return m_ptrain; }

  /// vehicle's driveshaft body.
  const ChSharedPtr<ChShaft> GetDriveshaft(size_t idx) const {return m_gear->GetAxle(); }

  /// current value of the integration step size for the vehicle system.
  double GetStepsize() const { return m_stepsize; }


  /// return the force exerted by the idler subsystem on the idler body
  double GetIdlerForce(size_t side);

  /// just return the COG of the gear
  ChCoordsys<> GetLocalDriverCoordsys() const { return ChCoordsys<>(m_chassis->GetPos(), m_chassis->GetRot()); }

  /// vehicle speed, doesn't matter
  double GetVehicleSpeed() const { return 0; }
  int GetNum_Engines() const { return 1;}

private:

  // private variables
  // <ChBodyAuxRef> m_chassis   in base class
  ChSharedPtr<DriveGear> m_gear;  		///< drive gear
  ChSharedPtr<IdlerSimple>	m_idler;	///< idler wheel
  ChSharedPtr<TrackChain> m_chain;    ///< chain

  ChVector<> m_idlerPosRel;	///< position of idler COG relative to local c-sys
  
  ChSharedPtr<TrackPowertrain>  m_ptrain;  ///< powertrain system

  std::vector<ChSharedPtr<SupportRoller>> m_rollers;  ///< passive support rollers
  size_t m_num_rollers;
	


  // static variables. hard-coded for now
  static const ChVector<> m_idlerPos; // relative to chassis frame, which is the same as the gear's (initially)
  static const ChQuaternion<> m_idlerRot; 

};


} // end namespace chrono


#endif
