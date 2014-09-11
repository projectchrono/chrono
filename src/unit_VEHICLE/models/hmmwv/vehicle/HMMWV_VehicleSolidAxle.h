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
// HMMWV full vehicle model with solid axle suspension...
//
// =============================================================================

#ifndef HMMWV_VEHICLESOLIDAXLE_H
#define HMMWV_VEHICLESOLIDAXLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"

#include "models/hmmwv/HMMWV.h"
#include "models/hmmwv/HMMWV_Wheel.h"
#include "models/hmmwv/suspension/HMMWV_SolidAxle.h"
#include "models/hmmwv/driveline/HMMWV_Driveline2WD.h"
#include "models/hmmwv/powertrain/HMMWV_Powertrain.h"
#include "models/hmmwv/brake/HMMWV_BrakeSimple.h"

namespace hmmwv {

// Forward reference
class HMMWV_Powertrain;

class HMMWV_VehicleSolidAxle : public chrono::ChVehicle {
public:

  HMMWV_VehicleSolidAxle(const bool        fixed = false,
                VisualizationType chassisVis = NONE,
                VisualizationType wheelVis = PRIMITIVES);

  ~HMMWV_VehicleSolidAxle();

  virtual chrono::ChSharedBodyPtr GetWheelBody(chrono::ChWheelId which) const;

  virtual const chrono::ChVector<>& GetWheelPos(chrono::ChWheelId which) const;
  virtual const chrono::ChQuaternion<>& GetWheelRot(chrono::ChWheelId which) const;
  virtual const chrono::ChVector<>& GetWheelLinVel(chrono::ChWheelId which) const;
  virtual chrono::ChVector<> GetWheelAngVel(chrono::ChWheelId which) const;
  virtual double GetWheelOmega(chrono::ChWheelId which) const;

  double GetSpringForce(chrono::ChWheelId which);
  double GetSpringLength(chrono::ChWheelId which);

  virtual void Initialize(const chrono::ChCoordsys<>& chassisPos);
  virtual void Update(double                      time,
                      double                      throttle,
                      double                      steering,
                      double                      braking,
                      const chrono::ChTireForces& tire_forces);

  static void ExportMeshPovray(const std::string& out_dir);

  // Log debugging information
  void LogHardpointLocations(); /// suspension hardpoints at design
  void DebugLog(int what);      /// shock forces and lengths, constraints, etc.

private:

  chrono::ChSharedPtr<HMMWV_SolidAxleFront> m_front_susp;
  chrono::ChSharedPtr<HMMWV_SolidAxleRear>  m_rear_susp;

  chrono::ChSharedPtr<HMMWV_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_rear_left_wheel;

  chrono::ChSharedPtr<HMMWV_Driveline2WD> m_driveline;
  chrono::ChSharedPtr<HMMWV_Powertrain>   m_powertrain;

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
};


} // end namespace hmmwv


#endif
