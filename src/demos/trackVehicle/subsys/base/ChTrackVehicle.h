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
// Base class for a Tracked vehicle system.
//
// Chassis ref frame is Y- up, X-forward.
// 
// =============================================================================

#ifndef CH_TRACKVEHICLE_H
#define CH_TRACKVEHICLE_H

#include <vector>

#include "core/ChVector.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "subsys/powertrain/TrackPowertrain.h"

#include "subsys/ChApiSubsys.h"


namespace chrono {

///
/// Base class for chrono Track vehicle system.
/// This class provides the interface between the vehicle system and a driver
///
class CH_SUBSYS_API ChTrackVehicle : public ChShared
{
public:

  /// Construct with a default ChSystem.
  ChTrackVehicle();

  /// Construct using the specified ChSystem.
  ChTrackVehicle(ChSystem* system);

  /// Destructor.
  virtual ~ChTrackVehicle();

  /// Get a pointer to the Chrono ChSystem.
  ChSystem* GetSystem() { return m_system; }

  /// Get a handle to the chassis body.
  ChSharedPtr<ChBodyAuxRef> GetChassis() const { return m_chassis; }

  /// Get the global location of the chassis reference frame origin.
  const ChVector<>& GetChassisPos() const { return m_chassis->GetFrame_REF_to_abs().GetPos(); }

  /// Get the orientation of the chassis reference frame.
  const ChQuaternion<>& GetChassisRot() const { return m_chassis->GetFrame_REF_to_abs().GetRot(); }

  /// Get the global location of the chassis center of mass.
  const ChVector<>& GetChassisPosCOM() const { return m_chassis->GetPos(); }

  /// Get the orientation of the chassis centroidal frame.
  const ChQuaternion<>& GetChassisRotCOM() const { return m_chassis->GetRot(); }

  /// Get the vehicle speed, measured at the origin of the chassis reference frame.
  double GetVehicleSpeed() const { return m_chassis->GetFrame_REF_to_abs().GetPos_dt().Length(); }

  /// Get the speed of the chassis COM.
  double GetVehicleSpeedCOM() const { return m_chassis->GetPos_dt().Length(); }

  /// Get the angular speed of the driveshaft.
  virtual double GetDriveshaftSpeed(size_t idx) const = 0;

    /// pointer to the powertrain
  TrackPowertrain* GetPowertrain(int idx) { return (m_num_engines > 0 ? m_ptrains[idx].get_ptr(): NULL); }

  /// Get the local driver position and orientation, relative to the chassis reference frame.
  virtual ChCoordsys<> GetLocalDriverCoordsys() const = 0;

  /// Get the global location of the driver.
  ChVector<> GetDriverPos() const;

  /// number of track chain systems attached to the vehicle
  int GetNum_Engines() const { return m_num_engines; }


  /// Initialize at the specified global location and orientation.
  virtual void Initialize(
    const ChCoordsys<>& chassis_Csys   ///< [in] initial config of vehicle REF frame
    ) {}

  /// Update the state at the current time, driver inputs between 0 and 1.
  virtual void Update(double    time,       ///< [in] current time
    const std::vector<double>&  throttle,   ///< [in] current steering input [-1,+1]
		const std::vector<double>&  braking     ///< [in] current braking input [0,1]
	) {}

  /// Advance the system by the specified time step.
  virtual void Advance(double step);

  /// Set the integration step size
  void SetStepsize(double val) { m_stepsize = val; }

  /// Get the current value of the integration step size
  double GetStepsize() const { return m_stepsize; }

  /// TODO: Log current constraint violations.
  // void LogConstraintViolations();

protected:

  ChSystem*                  m_system;       ///< pointer to the Chrono system
  bool                       m_ownsSystem;   ///< true if system created at construction

  ChSharedPtr<ChBodyAuxRef>  m_chassis;      ///< handle to the chassis body
  int m_num_engines;  ///< can support multiple powertrain/drivetrains
  std::vector<ChSharedPtr<TrackPowertrain>>  m_ptrains;  ///< powertrain system, one per track system

  double                     m_stepsize;   ///< integration step-size for the vehicle system
};


} // end namespace chrono


#endif
