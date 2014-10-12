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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Generic vehicle model with solid axle suspensions
//
// =============================================================================

#ifndef GENERIC_VEHICLESOLIDAXLE_H
#define GENERIC_VEHICLESOLIDAXLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"
#include "subsys/suspension/ChSolidAxle.h"

#include "models/ModelDefs.h"
#include "models/generic/Generic_Wheel.h"
#include "models/generic/Generic_RackPinion.h"
#include "models/generic/Generic_Driveline2WD.h"
#include "models/generic/Generic_BrakeSimple.h"

class Generic_VehicleSolidAxle : public chrono::ChVehicle
{
public:

  Generic_VehicleSolidAxle(const bool        fixed = false,
                           VisualizationType wheelVis = PRIMITIVES);

  ~Generic_VehicleSolidAxle() {}

  virtual int GetNumberAxles() const { return 2; }

  virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  double GetSpringForce(const chrono::ChWheelID& wheel_id) const;
  double GetSpringLength(const chrono::ChWheelID& wheel_id) const;
  double GetSpringDeformation(const chrono::ChWheelID& wheel_id) const;

  double GetShockForce(const chrono::ChWheelID& wheel_id) const;
  double GetShockLength(const chrono::ChWheelID& wheel_id) const;
  double GetShockVelocity(const chrono::ChWheelID& wheel_id) const;

  virtual void Initialize(const chrono::ChCoordsys<>& chassisPos);
  virtual void Update(double                      time,
                      double                      steering,
                      double                      braking,
                      double                      powertrain_torque,
                      const chrono::ChTireForces& tire_forces);

  // Log debugging information
  void LogHardpointLocations(); /// suspension hardpoints at design
  void DebugLog(int what);      /// shock forces and lengths, constraints, etc.

private:

  chrono::ChSharedPtr<Generic_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<Generic_Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<Generic_Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<Generic_Wheel> m_rear_left_wheel;

  chrono::ChSharedPtr<Generic_BrakeSimple> m_front_right_brake;
  chrono::ChSharedPtr<Generic_BrakeSimple> m_front_left_brake;
  chrono::ChSharedPtr<Generic_BrakeSimple> m_rear_right_brake;
  chrono::ChSharedPtr<Generic_BrakeSimple> m_rear_left_brake;

  // Chassis mass properties
  static const double             m_chassisMass;
  static const chrono::ChVector<> m_chassisCOM;
  static const chrono::ChVector<> m_chassisInertia;

  // Driver local coordinate system
  static const chrono::ChCoordsys<> m_driverCsys;
};


#endif
