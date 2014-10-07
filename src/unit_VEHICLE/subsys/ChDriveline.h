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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle driveline.
//
// =============================================================================

#ifndef CH_DRIVELINE_H
#define CH_DRIVELINE_H

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChShaft.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {

///
/// Base class for a driveline subsystem.
///
class CH_SUBSYS_API ChDriveline : public ChShared
{
public:

  enum DriveType {
    TWO_WHEEL_DRIVE,    ///< front or rear wheel drive
    FOUR_WHEEL_DRIVE    ///< All wheel drive
  };

  ChDriveline(
    DriveType  type    ///< [in] driveline type
    );
  virtual ~ChDriveline() {}

  /// Initialize the driveline subsystem.
  /// This function connects this driveline subsystem to the axles of the
  /// provided suspension subsystems.  Note that it is the responsibility of
  /// the caller to provide a number of suspension subsystems consistent with
  /// the driveline type.
  virtual void Initialize(
    ChSharedPtr<ChBody>     chassis,     ///< handle to the chassis body
    const ChSuspensionList& suspensions  ///< list of driven suspension subsystems
    ) = 0;

  /// Get a handle to the driveshaft.
  /// Return a shared pointer to the shaft that connects this driveline to a
  /// powertrain system (i.e., right after the transmission box).
  ChSharedPtr<ChShaft> GetDriveshaft() const { return m_driveshaft; }

  /// Get the angular speed of the driveshaft.
  /// This represents the output from the driveline subsystem that is passed to
  /// the powertrain system.
  double GetDriveshaftSpeed() const { return m_driveshaft->GetPos_dt(); }

  /// Apply the specified motor torque.
  /// This represents the input to the driveline subsystem from the powertrain
  /// system.
  void ApplyDriveshaftTorque(double torque)  { m_driveshaft->SetAppliedTorque(torque); }

  /// Get the motor torque to be applied to the specified wheel.
  virtual double GetWheelTorque(const ChWheelID& wheel_id) const = 0;

protected:

  DriveType             m_type;         ///< type of driveline

  ChSharedPtr<ChShaft>  m_driveshaft;   ///< handle to the shaft connection to the powertrain
};


} // end namespace chrono


#endif
