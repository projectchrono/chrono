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
// Authors: Alessandro Tasora
// =============================================================================
//
// Trailer for articulated vehicle model. 
//
// =============================================================================

#ifndef ARTICULATED_TRAILER_H
#define ARTICULATED_TRAILER_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"
#include "subsys/suspension/ChSolidAxle.h"

#include "models/ModelDefs.h"
#include "models/articulated/Articulated_Wheel.h"
#include "models/articulated/Articulated_BrakeSimple.h"

class Articulated_Trailer 
{
public:

	Articulated_Trailer(chrono::ChSystem& mysystem, 
					const bool        fixed,
                  SuspensionType    suspType,
                  VisualizationType wheelVis);

  ~Articulated_Trailer() {}

  virtual int GetNumberAxles() const { return 2; }

  double GetSpringForce(const chrono::ChWheelID& wheel_id) const;
  double GetSpringLength(const chrono::ChWheelID& wheel_id) const;
  double GetSpringDeformation(const chrono::ChWheelID& wheel_id) const;

  double GetShockForce(const chrono::ChWheelID& wheel_id) const;
  double GetShockLength(const chrono::ChWheelID& wheel_id) const;
  double GetShockVelocity(const chrono::ChWheelID& wheel_id) const;

  virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, 
						  const bool        connect_to_puller,
						  chrono::ChSharedPtr<chrono::ChBodyAuxRef> pulling_vehicle);

  virtual void Update(double                      time,
                      double                      braking,
                      const chrono::ChTireForces& tire_forces);

  // Log debugging information
  void LogHardpointLocations(); /// suspension hardpoints at design
  void DebugLog(int what);      /// shock forces and lengths, constraints, etc.

  /// Get a handle to the specified wheel body.
  chrono::ChSharedPtr<chrono::ChBody> GetWheelBody(const  chrono::ChWheelID& wheelID) const;

private:

  SuspensionType m_suspType;

  chrono::ChSharedPtr<Articulated_Wheel> m_front_right_wheel;
  chrono::ChSharedPtr<Articulated_Wheel> m_front_left_wheel;
  chrono::ChSharedPtr<Articulated_Wheel> m_rear_right_wheel;
  chrono::ChSharedPtr<Articulated_Wheel> m_rear_left_wheel;

  chrono::ChSharedPtr<Articulated_BrakeSimple> m_front_right_brake;
  chrono::ChSharedPtr<Articulated_BrakeSimple> m_front_left_brake;
  chrono::ChSharedPtr<Articulated_BrakeSimple> m_rear_right_brake;
  chrono::ChSharedPtr<Articulated_BrakeSimple> m_rear_left_brake;

  chrono::ChSharedPtr<chrono::ChBodyAuxRef>  m_chassis;      ///< handle to the chassis body
  chrono::ChSharedPtr<chrono::ChBodyAuxRef>  m_frontaxle;      ///< handle to the steering axle
  chrono::ChSuspensionList    m_suspensions;  ///< list of handles to suspension subsystems
  chrono::ChWheelList         m_wheels;       ///< list of handles to wheel subsystems
  chrono::ChBrakeList         m_brakes;       ///< list of handles to brake subsystems

  chrono::ChSharedPtr<chrono::ChLinkLockSpherical>  m_joint;      ///< handle to the joint between chassis and front axle
  chrono::ChSharedPtr<chrono::ChLinkLockSpherical>  m_puller;      ///< handle to the joint between trailer and pulling vehicle (optional)
  
  // Chassis mass properties
  static const double             m_chassisMass;
  static const chrono::ChVector<> m_chassisCOM;
  static const chrono::ChVector<> m_chassisInertia;
  static const double             m_frontaxleMass;
  static const chrono::ChVector<> m_frontaxleCOM;
  static const chrono::ChVector<> m_frontaxleREF;
  static const chrono::ChVector<> m_frontaxleInertia;

  static const chrono::ChVector<> m_frontaxleSphericalJoint;
  static const chrono::ChVector<> m_frontaxlePullerJoint;
};


#endif
