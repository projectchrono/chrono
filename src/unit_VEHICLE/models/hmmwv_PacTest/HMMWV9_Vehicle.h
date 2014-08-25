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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV 9-body vehicle model...
//
// =============================================================================

#ifndef HMMWV9_VEHICLE_H
#define HMMWV9_VEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"

#include "HMMWV9.h"
#include "HMMWV9_DoubleWishbone.h"
#include "HMMWV9_Wheel.h"
#include "HMMWV9_SimplePowertrain.h"
#include "HMMWV9_Powertrain.h"

namespace pactest {

// Forward reference
class HMMWV9_SimplePowertrain;
class HMMWV9_Powertrain;

class HMMWV9_Vehicle : public chrono::ChVehicle {
public:

  HMMWV9_Vehicle(const bool                   fixed = false,
                 VisualizationType            chassisVis = NONE,
                 VisualizationType            wheelVis = PRIMITIVES);

  ~HMMWV9_Vehicle();

  virtual chrono::ChSharedBodyPtr GetWheelBody(chrono::ChWheelId which) const;

  virtual const chrono::ChVector<>& GetWheelPos(chrono::ChWheelId which) const;
  virtual const chrono::ChQuaternion<>& GetWheelRot(chrono::ChWheelId which) const;
  virtual const chrono::ChVector<>& GetWheelLinVel(chrono::ChWheelId which) const;
  virtual chrono::ChVector<> GetWheelAngVel(chrono::ChWheelId which) const;
  virtual double GetWheelOmega(chrono::ChWheelId which) const;

  virtual void Initialize(const chrono::ChCoordsys<>& chassisPos);
  virtual void Update(double                      time,
                      double                      throttle,
                      double                      steering,
                      const chrono::ChTireForces& tire_forces);

  static void ExportMeshPovray(const std::string& out_dir);

private:

  chrono::ChSharedPtr<HMMWV9_DoubleWishboneFront>   m_front_right_susp;
  chrono::ChSharedPtr<HMMWV9_DoubleWishboneFront>   m_front_left_susp;
  chrono::ChSharedPtr<HMMWV9_DoubleWishboneRear>    m_rear_right_susp;
  chrono::ChSharedPtr<HMMWV9_DoubleWishboneRear>    m_rear_left_susp;

  chrono::ChSharedPtr<HMMWV9_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<HMMWV9_Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<HMMWV9_Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<HMMWV9_Wheel> m_rear_left_wheel;

  ////chrono::ChSharedPtr<HMMWV9_SimplePowertrain>  m_powertrain;
  chrono::ChSharedPtr<HMMWV9_Powertrain>  m_powertrain;

  // Chassis visualization mesh
  static const std::string m_chassisMeshName;
  static const std::string m_chassisMeshFile;

  // Chassis mass properties
  static const double  m_chassisMass;
  static const chrono::ChVector<>  m_chassisInertia;
};


} // end namespace pactest


#endif
