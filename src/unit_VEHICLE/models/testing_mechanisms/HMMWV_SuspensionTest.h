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
// Authors: Justin Madsen, Daniel Melanz, Radu Serban
// =============================================================================
//
// Suspension testing mechanism with a double wishbone/pitman arm steering combo
//
// =============================================================================

#ifndef HMMWV_SuspensionTest_H
#define HMMWV_SuspensionTest_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChSuspensionTest.h"

#include "models/ModelDefs.h"
#include "models/hmmwv/wheel/HMMWV_Wheel.h"
#include "models/hmmwv/suspension/HMMWV_DoubleWishbone.h"
#include "models/hmmwv/steering/HMMWV_PitmanArm.h"


class HMMWV_SuspensionTest : public chrono::ChSuspensionTest
{
public:

  HMMWV_SuspensionTest(VisualizationType wheelVis = PRIMITIVES,
                        bool use_motion = true);

  ~HMMWV_SuspensionTest();

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
                      const chrono::ChTireForces& tire_forces );

  // Log debugging information
  void LogHardpointLocations(); /// suspension hardpoints at design
  void DebugLog(int what);      /// shock forces and lengths, constraints, etc.

private:

  chrono::ChSharedPtr<hmmwv::HMMWV_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<hmmwv::HMMWV_Wheel> m_front_left_wheel;

  // Chassis mass properties
  // static const chrono::ChVector<> m_chassisCOM;

  // Driver local coordinate system
  chrono::ChCoordsys<> m_driverCsys;
};


#endif
