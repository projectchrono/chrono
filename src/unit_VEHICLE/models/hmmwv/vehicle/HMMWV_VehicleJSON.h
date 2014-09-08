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
// Uses JSON input files for subsystem templates
//
// =============================================================================

#ifndef HMMWV_VEHICLE_JSON_H
#define HMMWV_VEHICLE_JSON_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"
#include "subsys/suspension/DoubleWishbone.h"
#include "subsys/driveline/ShaftsDriveline2WD.h"
#include "subsys/wheel/Wheel.h"
#include "subsys/brake/BrakeSimple.h"

#include "models/hmmwv/HMMWV.h"
#include "models/hmmwv/powertrain/HMMWV_Powertrain.h"

namespace hmmwv {

// Forward reference
class HMMWV_Powertrain;

class HMMWV_VehicleJSON : public chrono::ChVehicle {
public:

  HMMWV_VehicleJSON(const bool        fixed = false,
                    VisualizationType chassisVis = NONE);

  ~HMMWV_VehicleJSON();

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

  chrono::ChSharedPtr<chrono::DoubleWishbone> m_front_susp;
  chrono::ChSharedPtr<chrono::DoubleWishbone>  m_rear_susp;

  chrono::ChSharedPtr<chrono::Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<chrono::Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<chrono::Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<chrono::Wheel> m_rear_left_wheel;

  chrono::ChSharedPtr<chrono::ShaftsDriveline2WD> m_driveline;
  chrono::ChSharedPtr<HMMWV_Powertrain>   m_powertrain;

  chrono::ChSharedPtr<chrono::BrakeSimple> m_front_right_brake;
  chrono::ChSharedPtr<chrono::BrakeSimple> m_front_left_brake;
  chrono::ChSharedPtr<chrono::BrakeSimple> m_rear_right_brake;
  chrono::ChSharedPtr<chrono::BrakeSimple> m_rear_left_brake;

  // Chassis visualization mesh
  static const std::string m_chassisMeshName;
  static const std::string m_chassisMeshFile;

  // Chassis mass properties
  static const double             m_chassisMass;
  static const chrono::ChVector<> m_chassisInertia;
};


} // end namespace hmmwv


#endif
