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
// model a single track chain system, as part of a tracked vehicle. Uses JSON input files
//
// =============================================================================

#include <cstdio>

#include "subsys/suspension/TorsionArmSuspension.h"

namespace chrono {

TorsionArmSuspension::TorsionArmSuspension(const std::string& filename)
{
   FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];

  fclose(fp);


  Create();
}


TorsionArmSuspension::Create()
{
/*
  // load data for the arm
  m_armMass = d["Arm"]["Mass"].GetDouble();
  m_armInertia = loadVector(d["Arm"]["Inertia"]);
  m_armRadius = d["Arm"]["Radius"].GetDouble();
  
  // load data for the wheel
  m_wheelMass = d["Wheel"]["Mass"].GetDouble();
  m_wheelInertia = loadVector(d["Wheel"]["Inertia"]);
  m_wheelRadius = d["Wheel"]["Radius"].GetDouble();
  m_wheelWidth = d["Wheel"]["Width"].GetDouble();
  m_wheelRelLoc = loadVector(d["Wheel"]["Location"]);
  
  // load data for the torsion bar
  m_springK = d["Torsion Bar"]["Stiffness"].GetDouble();
  m_springC = d["Torsion Bar"]["Damping"].GetDouble();

  
  m_arm = ChSharedPtr<ChBody>(new ChBody);
  
  m_wheel = ChSharedPtr<ChBody>(new ChBody);
  */
  return 0;
}

void TorsionArmSuspension::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                             const ChVector<>&         location,
                             const ChQuaternion<>&     rotation)
{
  // Express the steering reference frame in the absolute coordinate system.
  ChFrame<> loc_to_abs(location, rotation);
  loc_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  
  
}

/// add a cylinder to model the torsion bar arm and the wheel
void TorsionArmSuspension::AddVisualization()
{

}

} // end namespace chrono
