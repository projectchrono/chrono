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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// 4WD driveline model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_4WD_H
#define CH_SHAFTS_DRIVELINE_4WD_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChDriveline.h"
#include "subsys/ChVehicle.h"

#include "physics/ChShaftsGear.h" 
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsMotor.h"
#include "physics/ChShaftsTorque.h"

namespace chrono {

// Forward reference
class ChVehicle;

///
/// 4WD driveline model template based on ChShaft objects.
///
class CH_SUBSYS_API ChShaftsDriveline4WD : public ChDriveline
{
public:

  ChShaftsDriveline4WD(
    ChVehicle* car         ///< [in] the vehicle subsystem
    );

  ~ChShaftsDriveline4WD() {}

  /// Set the direction of the motor block.
  /// This direction is a unit vector, relative to the chassis frame (for the
  /// ISO coordinate system, this is [1, 0, 0] for a longitudinal engine and
  /// [0, 1, 0] for a transversal engine).
  void SetMotorBlockDirection(const ChVector<>& dir) { m_dir_motor_block = dir; }

  /// Set the direction of the wheel axles.
  /// This direction is a unit vector, relative to the chassis frame. It must be
  /// specified for the design configuration (for the ISO vehicle coordinate
  /// system, this is typically [0, 1, 0]).
  void SetAxleDirection(const ChVector<>& dir) { m_dir_axle = dir; }

  /// Initialize the driveline subsystem.
  /// This function connects this driveline subsystem to the axles of the
  /// provided suspension subsystems.  Note that it is the responsibility of
  /// the caller to provide a number of suspension subsystems consistent with
  /// the driveline type (in this case two suspension subsystems, with the first
  /// one being the front suspension and the second one the rear suspension).
  virtual void Initialize(
    ChSharedPtr<ChBody>     chassis,     ///< handle to the chassis body
    const ChSuspensionList& suspensions  ///< list of driven suspension subsystems
    );

  /// Get the motor torque to be applied to the specified wheel.
  virtual double GetWheelTorque(ChWheelId which) const;

protected:

  /// Return the inertia of the driveshaft.
  virtual double GetDriveshaftInertia() const = 0;
  /// Return the inertia of the differential box.
  virtual double GetCentralDifferentialBoxInertia() const = 0;
  /// Return the inertia of the front driveshaft.
  virtual double GetToFrontDiffShaftInertia() const = 0;
  /// Return the inertia of the rear driveshaft.
  virtual double GetToRearDiffShaftInertia() const = 0;
  /// Return the inertia of the rear differential box.
  virtual double GetRearDifferentialBoxInertia() const = 0;
  /// Return the inertia of the front differential box.
  virtual double GetFrontDifferentialBoxInertia() const = 0;

  /// Return the gear ratio for the rear conical gear.
  virtual double GetRearConicalGearRatio() const = 0;
  /// Return the gear ratio for the front conical gear.
  virtual double GetFrontConicalGearRatio() const = 0;
  /// Return the gear ratio for the rear differential.
  virtual double GetRearDifferentialRatio() const = 0;
  /// Return the gear ratio for the front differential.
  virtual double GetFrontDifferentialRatio() const = 0;
  /// Return the gear ratio for the central differential.
  virtual double GetCentralDifferentialRatio() const = 0;

private:

  ChSharedPtr<ChShaftsPlanetary>        m_central_differential;
  ChSharedPtr<ChShaft>                  m_front_shaft;
  ChSharedPtr<ChShaft>                  m_rear_shaft;
  ChSharedPtr<ChShaftsGearboxAngled>    m_rear_conicalgear;
  ChSharedPtr<ChShaftsPlanetary>        m_rear_differential;
  ChSharedPtr<ChShaft>                  m_rear_differentialbox;
  ChSharedPtr<ChShaftsGearboxAngled>    m_front_conicalgear;
  ChSharedPtr<ChShaftsPlanetary>        m_front_differential;
  ChSharedPtr<ChShaft>                  m_front_differentialbox;

  ChVector<> m_dir_motor_block;
  ChVector<> m_dir_axle;

  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
