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

  ChDriveline();
  virtual ~ChDriveline() {}

  /// Return the number of driven axles.
  virtual int GetNumDrivenAxles() const = 0;

  /// Initialize the driveline subsystem.
  /// This function connects this driveline subsystem to the axles of the
  /// specified suspension subsystems.
  virtual void Initialize(
    ChSharedPtr<ChBody>     chassis,     ///< handle to the chassis body
    const ChSuspensionList& suspensions, ///< list of all vehicle suspension subsystems
    const std::vector<int>& driven_axles ///< indexes of the driven vehicle axles
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

  ChSharedPtr<ChShaft>  m_driveshaft;   ///< handle to the shaft connection to the powertrain

  std::vector<int>      m_driven_axles; ///< indexes of the driven vehicle axles
};


} // end namespace chrono


#endif
