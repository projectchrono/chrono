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

#include "HMMWV_DoubleWishbone.h"

#include "../hmmwv_common/HMMWV.h"
#include "../hmmwv_common/HMMWV_Wheel.h"
#include "../hmmwv_common/HMMWV_SimplePowertrain.h"
#include "../hmmwv_common/HMMWV_Powertrain.h"

namespace hmmwv {

// Forward reference
class HMMWV_SimplePowertrain;
class HMMWV_Powertrain;

class HMMWV_Vehicle : public chrono::ChVehicle {
public:

  HMMWV_Vehicle(const bool                   fixed = false,
                VisualizationType            chassisVis = NONE,
                VisualizationType            wheelVis = PRIMITIVES);

  ~HMMWV_Vehicle();

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

	void CheckShocks(const size_t step_num, const double simTime);


private:

  chrono::ChSharedPtr<HMMWV_DoubleWishboneFront>   m_front_right_susp;
  chrono::ChSharedPtr<HMMWV_DoubleWishboneFront>   m_front_left_susp;
  chrono::ChSharedPtr<HMMWV_DoubleWishboneRear>    m_rear_right_susp;
  chrono::ChSharedPtr<HMMWV_DoubleWishboneRear>    m_rear_left_susp;

  chrono::ChSharedPtr<HMMWV_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<HMMWV_Wheel> m_rear_left_wheel;

  ////chrono::ChSharedPtr<HMMWV_SimplePowertrain>  m_powertrain;
  chrono::ChSharedPtr<HMMWV_Powertrain>  m_powertrain;

  // Chassis visualization mesh
  static const std::string m_chassisMeshName;
  static const std::string m_chassisMeshFile;

  // Chassis mass properties
  static const double  m_chassisMass;
  static const chrono::ChVector<>  m_chassisInertia;

	// debugging spring/shock values
	double GetSpringForce(chrono::ChWheelId which);
	double GetSpringLength(chrono::ChWheelId which);
};


} // end namespace hmmwv


#endif