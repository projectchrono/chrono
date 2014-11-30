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
// Generic 2-axle vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a generic rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#ifndef GENERIC_VEHICLE_H
#define GENERIC_VEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"

#include "models/ModelDefs.h"

class Generic_Vehicle : public chrono::ChVehicle
{
public:

  Generic_Vehicle(const bool        fixed,
                  SuspensionType    suspType,
                  VisualizationType wheelVis);

  ~Generic_Vehicle() {}

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

  SuspensionType m_suspType;

  // Chassis mass properties
  static const double             m_chassisMass;
  static const chrono::ChVector<> m_chassisCOM;
  static const chrono::ChVector<> m_chassisInertia;

  // Driver local coordinate system
  static const chrono::ChCoordsys<> m_driverCsys;
};


#endif
