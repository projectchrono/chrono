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
// HMMWV full vehicle model...
//
// =============================================================================

#ifndef HMMWV_VEHICLE_H
#define HMMWV_VEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"

#include "models/hmmwv/HMMWV.h"
#include "models/hmmwv/HMMWV_Wheel.h"
#include "models/hmmwv/suspension/HMMWV_DoubleWishbone.h"
#include "models/hmmwv/steering/HMMWV_PitmanArm.h"
#include "models/hmmwv/driveline/HMMWV_Driveline2WD.h"
#include "models/hmmwv/brake/HMMWV_BrakeSimple.h"

namespace hmmwv {

// Forward reference
class HMMWV_Powertrain;

class HMMWV_Vehicle : public chrono::ChVehicle {
public:

  HMMWV_Vehicle(const bool        fixed = false,
                VisualizationType chassisVis = NONE,
                VisualizationType wheelVis = PRIMITIVES);

  ~HMMWV_Vehicle();

  virtual chrono::ChSharedBodyPtr GetWheelBody(chrono::ChWheelId which) const;

  virtual const chrono::ChVector<>& GetWheelPos(chrono::ChWheelId which) const;
  virtual const chrono::ChQuaternion<>& GetWheelRot(chrono::ChWheelId which) const;
  virtual const chrono::ChVector<>& GetWheelLinVel(chrono::ChWheelId which) const;
  virtual chrono::ChVector<> GetWheelAngVel(chrono::ChWheelId which) const;
  virtual double GetWheelOmega(chrono::ChWheelId which) const;

  virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  double GetSpringForce(chrono::ChWheelId which) const;
  double GetSpringLength(chrono::ChWheelId which) const;
  double GetSpringDeformation(chrono::ChWheelId which) const;

  double GetShockForce(chrono::ChWheelId which) const;
  double GetShockLength(chrono::ChWheelId which) const;
  double GetShockVelocity(chrono::ChWheelId which) const;

  virtual void Initialize(const chrono::ChCoordsys<>& chassisPos);
  virtual void Update(double                      time,
                      double                      throttle,
                      double                      steering,
                      double                      braking,
                      double                      powertrain_torque,
                      const chrono::ChTireForces& tire_forces);

  static void ExportMeshPovray(const std::string& out_dir);

  // Log debugging information
  void LogHardpointLocations(); /// suspension hardpoints at design
  void DebugLog(int what);      /// shock forces and lengths, constraints, etc.

private:

  chrono::ChSharedPtr<HMMWV_DoubleWishboneFront> m_front_susp;
  chrono::ChSharedPtr<HMMWV_DoubleWishboneRear>  m_rear_susp;

  chrono::ChSharedPtr<HMMWV_PitmanArm> m_steering;

  chrono::ChSharedPtr<HMMWV_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_rear_left_wheel;

  chrono::ChSharedPtr<HMMWV_Driveline2WD> m_driveline;

  chrono::ChSharedPtr<HMMWV_BrakeSimple> m_front_right_brake;
  chrono::ChSharedPtr<HMMWV_BrakeSimple> m_front_left_brake;
  chrono::ChSharedPtr<HMMWV_BrakeSimple> m_rear_right_brake;
  chrono::ChSharedPtr<HMMWV_BrakeSimple> m_rear_left_brake;

  // Chassis visualization mesh
  static const std::string m_chassisMeshName;
  static const std::string m_chassisMeshFile;

  // Chassis mass properties
  static const double             m_chassisMass;
  static const chrono::ChVector<> m_chassisCOM;
  static const chrono::ChVector<> m_chassisInertia;

  // Driver local coordinate system
  static const chrono::ChCoordsys<> m_driverCsys;
};


} // end namespace hmmwv


#endif
