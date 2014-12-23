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
// model a single track chain system, as part of a tracked vehicle.
// Static variable values are based on a M113 model in the report by Shabana
//
// =============================================================================

#include <cstdio>

#include "subsys/suspension/TorsionArmSuspension.h"

namespace chrono {

// static variables
const double TorsionArmSuspension::m_armMass = 75.26; // [kg]
const ChVector<> TorsionArmSuspension::m_armInertia(0.77, 0.37, 0.77);  // [kg-m2]
const double TorsionArmSuspension::m_armRadius = 0.2; // [m]

const double TorsionArmSuspension::m_wheelMass = 561.1; // [kg]
const ChVector<> TorsionArmSuspension::m_wheelInertia(26.06, 19.82, 19.82); // [kg-m2]
const double TorsionArmSuspension::m_wheelWidth = 0.4;  // [m]
const double TorsionArmSuspension::m_wheelRadius = 0.7; // [m]


const double TorsionArmSuspension::m_springK = 100000;	// torsional spring constant
const double TorsionArmSuspension::m_springC = 1000;	// torsional damping constant
const ChVector<> m_TorquePreload;



TorsionArmSuspension::TorsionArmSuspension()
{
  // FILE* fp = fopen(filename.c_str(), "r");
  // char readBuffer[65536];
  // fclose(fp);

  Create();
}


void TorsionArmSuspension::Create()
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

  */

  // create the bodies
  m_arm = ChSharedPtr<ChBody>(new ChBody);
  m_wheel = ChSharedPtr<ChBody>(new ChBody);

  // create the constraints
  m_armChassis_rev = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_armWheel_rev = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);

  // create the torsional spring damper 

  
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
