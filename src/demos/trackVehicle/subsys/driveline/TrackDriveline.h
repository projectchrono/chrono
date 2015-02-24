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
// Authors: Justin Madsen, Alessandro Tasora, Radu Serban
// =============================================================================
//
// Driveline model template based on ChShaft objects. Models the left and right
// driveline to the drive sprockets of a tracked vehicle
//
// =============================================================================

#ifndef TRACK_DRIVELINE_H
#define TRACK_DRIVELINE_H

#include "subsys/ChApiSubsys.h"
#include "core/ChShared.h"
#include "subsys/driveGear/DriveGear.h"

#include "physics/ChShaftsGear.h" 
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsMotor.h"
#include "physics/ChShaftsTorque.h"

namespace chrono{

// Driveline model template based on ChShaft objects. Models the left and right
// driveline to the drive sprockets of a tracked vehicle
class CH_SUBSYS_API TrackDriveline : public ChShared
{
public:

  TrackDriveline(const std::string& name);

  ~TrackDriveline() {}

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

  /// Return the number of driven axles.
  // int GetNumDrivenAxles() const { return 1; }

  /// Initialize the driveline subsystem, connecting it to the drive gears
  void Initialize(
    ChSharedPtr<ChBody>     chassis,      ///< handle to the chassis body
    ChSharedPtr<DriveGear>  drivegear_L,	///< handle to the drive gear on the left side
    ChSharedPtr<DriveGear>  drivegear_R   ///< handle to the drive gear on the right side
    );

  /// Apply the specified motor torque.
  void ApplyDriveshaftTorque(double torque)  { m_driveshaft->SetAppliedTorque(torque);		 }
	
	
  // Acessors
  
  /// handle to the driveshaft.
  ChSharedPtr<ChShaft> GetDriveshaft() const { return m_driveshaft; }
	
  /// angular speed of the driveshaft.
  double GetDriveshaftSpeed() const { return m_driveshaft->GetPos_dt(); }	
	
  /// Get the indexes of the vehicle's axles driven by this driveline subsystem.
  const std::vector<int>& GetDrivenAxleIndexes() const { return m_driven_axles; }	
	
  /// motor torque to be applied to the specified drive gear.
  double GetGearTorque(const int gear_id) const;

private:

  double GetDriveshaftInertia() const      { return m_driveshaft_inertia; }
  double GetDifferentialBoxInertia() const { return m_differentialbox_inertia; }

  double GetConicalGearRatio() const       { return m_conicalgear_ratio; }
  double GetDifferentialRatio() const      { return m_differential_ratio; }


  ChSharedPtr<ChShaftsGearboxAngled>    m_conicalgear;
  ChSharedPtr<ChShaft>                  m_differentialbox;
  ChSharedPtr<ChShaftsPlanetary>        m_differential;

  ChVector<> m_dir_motor_block;
  ChVector<> m_dir_axle;

  // friend class ChIrrGuiDriver;
  // from abstract base
  ChSharedPtr<ChShaft>  m_driveshaft;   ///< handle to the shaft connection to the powertrain

  std::vector<int>      m_driven_axles; ///< indexes of the driven vehicle axles
  
  // from concrete class
  // Shaft inertias.
  static const double  m_driveshaft_inertia;
  static const double  m_differentialbox_inertia;

  // Gear ratios.
  static const double  m_conicalgear_ratio;
  static const double  m_differential_ratio;
};


} // end namespace chrono

#endif
