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
// Chain drive.
//
// =============================================================================

#include <cstdio>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"
#include "physics/ChGlobal.h"

#include "DriveChain.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
// idler, relative to gear/chassis
const ChVector<> DriveChain::m_idlerPos(-2.1904, -0.1443, 0.2447); // relative to local csys
const ChQuaternion<> DriveChain::m_idlerRot(QUNIT);


/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
DriveChain::DriveChain(const std::string& name, VisualizationType vis, CollisionType collide)
  : m_ownsSystem(true),
  m_stepsize(1e-3),
  m_num_idlers(1)
{
  // Integration and Solver settings
  SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  SetIterLCPmaxItersSpeed(150);
  SetIterLCPmaxItersStab(150);
  SetMaxPenetrationRecoverySpeed(2.0);
  // SetIterLCPomega(2.0);
  // SetIterLCPsharpnessLambda(2.0);
  

  // create the chassis body    
  m_chassis = ChSharedPtr<ChBody>(new ChBody);
  m_chassis->SetIdentifier(0);
  m_chassis->SetNameString(name);
  // basic body info. Not relevant since it's fixed.
  m_chassis->SetMass(100);
  m_chassis->SetInertiaXX(ChVector<>(10,10,10));
  // chassis is fixed to ground
  m_chassis->SetBodyFixed(true);
    
  // add the chassis body to the system
  Add(m_chassis);
  
  // build one of each of the following subsystems. 
  m_gear = ChSharedPtr<DriveGear>(new DriveGear("drive gear",
    vis,	//VisualizationType::PRIMITIVES,
    collide));	//CollisionType::PRIMITIVES) );

  m_idler = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler",
    vis,	// VisualizationType::PRIMITIVES,
    collide));	// CollisionType::PRIMITIVES) );

  m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain",
    vis,	// VisualizationType::PRIMITIVES,
    collide));	// CollisionType::PRIMITIVES) );
  
  
  // create the powertrain and drivelines
  m_driveline = ChSharedPtr<TrackDriveline_1WD>(new TrackDriveline_1WD("driveline ") );
  m_ptrain = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain ") );


}


DriveChain::~DriveChain()
{
  if(m_ownsSystem)
    delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void DriveChain::Initialize(const ChCoordsys<>& gear_Csys)
{
  // initialize the drive gear, idler and track chain
  m_idlerPosRel = m_idlerPos;
  m_chassis->SetPos(gear_Csys.pos);
  m_chassis->SetRot(gear_Csys.rot);
  
  // initialize 1 of each of the following subsystems.
  // will use the chassis ref frame to do the transforms, since the TrackSystem
  // local ref. frame has same rot (just difference in position)
  m_gear->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    ChCoordsys<>());

  m_idler->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    ChCoordsys<>(m_idlerPosRel, QUNIT) );

  // Create list of the center location of the rolling elements and their clearance.
  // Clearance is a sphere shaped envelope at each center location, where it can
  //  be guaranteed that the track chain geometry will not penetrate the sphere.
  std::vector<ChVector<>> rolling_elem_locs; // w.r.t. chassis ref. frame
  std::vector<double> clearance;  // 1 per rolling elem  
  
  // drive sprocket is First added to the lists passed into TrackChain Init()
  rolling_elem_locs.push_back(gear_Csys.pos );
  clearance.push_back(m_gear->GetRadius() );

  // add to the lists passed into the track chain Init()
  rolling_elem_locs.push_back(m_idlerPosRel );
  clearance.push_back(m_idler->GetRadius() );

  // Assumption: start_pos should lie close to where the actual track chain would 
  //             pass between the idler and driveGears.
  // MUST be on the top part of the chain so the chain wrap rotation direction can be assumed.
  // rolling_elem_locs, start_pos w.r.t. chassis c-sys
  m_chain->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    rolling_elem_locs, clearance, ChVector<>(m_gear->GetRadius(),0,0) );
  
  // initialize the powertrain, drivelines
  m_driveline->Initialize(m_chassis, m_gear);
  m_ptrain->Initialize(m_chassis, m_driveline->GetDriveshaft() );
}


void DriveChain::Update(double time,
                        double throttle,
                        double  braking)
{
  // update left and right powertrains, with the new left and right throttle/shaftspeed
  m_ptrain->Update(time, throttle, m_driveline->GetDriveshaftSpeed() );

}

void DriveChain::Advance(double step)
{
  double t = 0;
  double settlePhaseA = 0.001;
  double settlePhaseB = 0.1;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( GetChTime() < settlePhaseA )
    {
      h = 1e-5;
    } else if ( GetChTime() < settlePhaseB )
    {
      h = 1e-4;
    }
    DoStepDynamics(h);
    t += h;
  }
}


double DriveChain::GetIdlerForce(size_t idler_idx)
{
  assert(idler_idx < m_num_idlers);

  // only 1 idler, for now
  ChVector<> out_force = m_idler->GetSpringForce();

  return out_force.Length();
}

} // end namespace chrono
