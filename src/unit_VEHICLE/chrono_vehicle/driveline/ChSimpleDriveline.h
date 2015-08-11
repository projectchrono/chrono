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
// Authors: Radu Serban
// =============================================================================
//
// Simple driveline model. This template can be used to model a 4WD driveline.
// It uses a constant front/rear torque split (a value between 0 and 1) and a
// simple model for Torsen limited-slip differentials.
//
// =============================================================================

#ifndef CH_SIMPLE_DRIVELINE_H
#define CH_SIMPLE_DRIVELINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriveline.h"

namespace chrono {

///
/// Simple driveline model. This template can be used to model a 4WD driveline.
/// It uses a constant front/rear torque split (a value between 0 and 1) and a
/// simple model for Torsen limited-slip differentials.
///
class CH_VEHICLE_API ChSimpleDriveline : public ChDriveline
{
public:

  ChSimpleDriveline();
  ~ChSimpleDriveline() {}

  /// Return the number of driven axles.
  virtual int GetNumDrivenAxles() const { return 2; }

  /// Initialize the driveline subsystem.
  /// This function connects this driveline subsystem to the axles of the
  /// specified suspension subsystems.
  virtual void Initialize(
    ChSharedPtr<ChBody>     chassis,     ///< handle to the chassis body
    const ChSuspensionList& suspensions, ///< list of all vehicle suspension subsystems
    const std::vector<int>& driven_axles ///< indexes of the driven vehicle axles
    );

  /// Get the angular speed of the driveshaft.
  /// This represents the output from the driveline subsystem that is passed to
  /// the powertrain system.
  virtual double GetDriveshaftSpeed() const;

  /// Update the driveline subsystem: apply the specified motor torque.
  /// This represents the input to the driveline subsystem from the powertrain
  /// system.
  virtual void Update(double torque);

  /// Get the motor torque to be applied to the specified wheel.
  virtual double GetWheelTorque(const ChWheelID& wheel_id) const;

protected:

  /// Return the front torque fraction [0,1].
  virtual double GetFrontTorqueFraction() const = 0;

  /// Return the torque bias ratio for the front differential.
  /// This is a simple model of a Torsen limited-slip differential.
  virtual double GetFrontDifferentialMaxBias() const = 0;

  /// Return the torque bias ratio for the rear differential.
  /// This is a simple model of a Torsen limited-slip differential.
  virtual double GetRearDifferentialMaxBias() const = 0;

private:
  ChSharedPtr<ChShaft>    m_front_left;
  ChSharedPtr<ChShaft>    m_front_right;
  ChSharedPtr<ChShaft>    m_rear_left;
  ChSharedPtr<ChShaft>    m_rear_right;
};


} // end namespace chrono


#endif
