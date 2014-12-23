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
// Authors: Justin Madsen
// =============================================================================
//
// A simple suspension/road wheel system, that uses a torsional spring and rigid arm
//
// =============================================================================

#ifndef TORSION_ARM_SUSPENSION_H
#define TORSION_ARM_SUSPENSION_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

namespace chrono {

/// Consists of two bodies, the torsion arm link and the road-wheel.
/// Arm constrained to chassis via revolute joint, as is the arm to wheel.
/// Rotational spring damper between arm and chassis
class CH_SUBSYS_API TorsionArmSuspension : public ChShared
{
public:

  TorsionArmSuspension();

  ~TorsionArmSuspension() {}

  /// 
  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
				  const ChVector<>&         location,
				  const ChQuaternion<>&     rotation);
  
  
  double getSpringCoefficient() const { return m_springK; }
  double getDampingCoefficient() const { return m_springC; }

  ChSharedPtr<ChBody> GetArmBody() { return m_arm; }
  ChSharedPtr<ChBody> GetWheelBody() { return m_wheel; }

private:

  // private functions
  void Create();
  void AddVisualization();
  
  // private variables
  ChSharedPtr<ChBody> m_arm;  ///< arm body
  ChSharedPtr<ChLinkLockRevolute> m_armChassis_rev; ///< arm-chassis revolute joint
  ChSharedPtr<ChBody> m_wheel;  ///< wheel body
  ChSharedPtr<ChLinkLockRevolute> m_armWheel_rev; ///< arm-wheel revolute joint

  ChFrame<> m_Loc; // location of subsystem, relative to trackSystem ref c-sys

  // static variables
  static const double m_armMass;
  static const ChVector<> m_armInertia;
  static const double m_armRadius;

  static const double m_wheelMass;
  static const ChVector<> m_wheelInertia;
  static const double m_wheelWidth;
  static const double m_wheelRadius;

  static const double m_springK;	// torsional spring constant
  static const double m_springC;	// torsional damping constant
  static const ChVector<> m_TorquePreload;	// preload torque (x,y,z) in local coords, on the spring/damper
  
};


} // end namespace chrono


#endif
