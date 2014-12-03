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
#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API TorsionArmSuspension
{
public:

  TorsionArmSuspension(const std::string& filename);
  TorsionArmSuspension(const rapidjson::Document& d);

  ~TorsionArmSuspension() {}

  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
				  const ChVector<>&         location,
				  const ChQuaternion<>&     rotation);
  
  
  double getSpringCoefficient() const { return m_springK; }
  double getDampingCoefficient() const { return m_springC; }
  double getSpringRestLength() const { return m_springRestLength; }
  ChSharedPtr<ChBody> GetArmBody() { return m_arm; }
  ChSharedPtr<ChBody> GetWheelBody() { return m_wheel; }

private:

  // private functions
  void Create(const rapidjson::Document& d);
  void AddVisualization();
  
  // private variables
  ChSharedPtr<ChBody> m_arm;
  double m_armMass;
  ChVector<> m_armInertia;
  double m_armRadius;
  
  ChSharedPtr<ChBody> m_wheel;
  double m_wheelMass;
  ChVector<> m_wheelInertia;
  double m_wheelWidth;
  double m_wheelRadius;
  ChVector<> m_wheelRelLoc;	// location relative to the arm attachment point to the chassis
  
  double m_springK;	// torsional spring constant
  double m_springC;	// torsional damping constant
  ChVector<> m_TorquePreload;	// preload torque (x,y,z) in local coords, on the spring/damper
  
};


} // end namespace chrono


#endif
