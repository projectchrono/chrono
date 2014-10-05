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
// 2WD driveline model template based on ChShaft objects. This template can be
// used to model either a FWD or a RWD driveline.
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_2WD_H
#define CH_SHAFTS_DRIVELINE_2WD_H

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
/// 2WD driveline model template based on ChShaft objects. This template can be
/// used to model either a FWD or a RWD driveline.
///
class CH_SUBSYS_API ChShaftsDriveline2WD : public ChDriveline
{
public:

  ChShaftsDriveline2WD(
    ChVehicle* car         ///< [in] the vehicle subsystem
    );

  ~ChShaftsDriveline2WD() {}

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
  /// To be called after creation, to create all the wrapped ChShaft objects 
  /// and their constraints, torques etc. 
  void Initialize(
    ChSharedPtr<ChBody>  chassis,    ///< handle to the chassis body
    ChSharedPtr<ChShaft> axle_L,     ///< handle to the left driven wheel axle
    ChSharedPtr<ChShaft> axle_R      ///< handle to the right driven wheel axle
    );

  /// Get the motor torque to be applied to the specified wheel.
  virtual double GetWheelTorque(ChWheelId which) const;

protected:

  /// Return the inertia of the driveshaft.
  virtual double GetDriveshaftInertia() const = 0;
  /// Return the inertia of the differential box.
  virtual double GetDifferentialBoxInertia() const = 0;

  /// Return the gear ratio for the conical gear.
  virtual double GetConicalGearRatio() const = 0;

  /// Return the gear ratio for the differential.
  virtual double GetDifferentialRatio() const = 0;

private:

  ChSharedPtr<ChShaftsGearboxAngled>    m_conicalgear;
  ChSharedPtr<ChShaft>                  m_differentialbox;
  ChSharedPtr<ChShaftsPlanetary>        m_differential;

  ChVector<> m_dir_motor_block;
  ChVector<> m_dir_axle;

  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
