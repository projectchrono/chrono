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
// custom collision detection classes
#include "subsys/collision/TrackCollisionCallback.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
// idler, relative to gear/chassis
const ChVector<> DriveChain::m_idlerPos(-2.1904, -0.1443, 0.2447); // relative to local csys
const ChQuaternion<> DriveChain::m_idlerRot(QUNIT);


/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
DriveChain::DriveChain(const std::string& name,
                       VisualizationType gearVis,
                       CollisionType gearCollide,
                       size_t num_idlers,
                       size_t num_rollers)
  : ChTrackVehicle(1e-3, 1, gearVis, gearCollide),
  m_num_rollers(num_rollers),
  m_num_idlers(num_idlers)
{
  // ---------------------------------------------------------------------------
  // Set the base class variables
  m_num_engines = 1;

  // Integration and Solver settings set in ChTrackVehicle
  GetSystem()->SetIterLCPmaxItersStab(75);
  GetSystem()->SetIterLCPmaxItersSpeed(75);

  // doesn't matter for the chassis, since no visuals used
  m_meshName = "na";
  m_meshFile = "none";
  m_chassisBoxSize = ChVector<>(2.0, 0.6, 0.75);

  // create the chassis body    
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);
  m_chassis->SetIdentifier(0);
  m_chassis->SetNameString(name);
  // basic body info. Not relevant since it's fixed.
  m_chassis->SetFrame_COG_to_REF(ChFrame<>() );
  m_chassis->SetMass(100);
  m_chassis->SetInertiaXX(ChVector<>(10,10,10) );
  // chassis is fixed to ground
  m_chassis->SetBodyFixed(true);
    
  // add the chassis body to the system
  m_system->Add(m_chassis);
  
  // --------------------------------------------------------------------------
  // BUILD THE SUBSYSTEMS
  // drive gear, inherits drivechain's visual and collision types
  double gear_mass = 100.0; // 436.7
  ChVector<> gear_Ixx(12.22/4.0, 12.22/4.0, 13.87/4.0);  // 12.22, 12.22, 13.87
  m_gear = ChSharedPtr<DriveGear>(new DriveGear("drive gear",
    gear_mass,
    gear_Ixx,
    m_vis,
    m_collide));

 // idlers, if m ore than 1
  m_idlers.clear();
  m_idlers.resize(m_num_idlers);
  double idler_mass = 100.0; // 429.6
  ChVector<> idler_Ixx(gear_Ixx);    // 12.55, 12.55, 14.7
  m_idlers[0] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler",
    idler_mass,
    idler_Ixx,
    //vis,
    VisualizationType::MESH,
    CollisionType::PRIMITIVES) );

  // track chain system
  double shoe_mass = 3.0; // 18.03
  ChVector<> shoe_Ixx(0.22/6.0, 0.25/6.0, 0.04/6.0);  // 0.22, 0.25, 0.04
  m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain",
    shoe_mass,
    shoe_Ixx,
    VisualizationType::COMPOUNDPRIMITIVES,
    CollisionType::PRIMITIVES) );

  // create the powertrain, connect transmission shaft directly to gear shaft
  m_ptrain = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain ") );

  // support rollers, if any
  m_rollers.clear();
  m_rollers.resize(m_num_rollers);
  for(int j = 0; j < m_num_rollers; j++)
  {
    m_rollers[j] = ChSharedPtr<SupportRoller>(new SupportRoller("support roller " +std::to_string(j),
      VisualizationType::PRIMITIVES,
      CollisionType::PRIMITIVES));
  }

  if(m_num_idlers > 1)
  {
    // for now, just create 1 more idler
    m_idlers[1] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler 2",
    idler_mass,
    idler_Ixx,
    VisualizationType::MESH,
    CollisionType::PRIMITIVES) );
  }
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
  double idler_preload = 60000;
  // m_idlerPosRel = m_idlerPos;
  m_idlerPosRel = ChVector<>(-2.5, 0, 0);
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(gear_Csys.pos, gear_Csys.rot));
  
  // Create list of the center location of the rolling elements and their clearance.
  // Clearance is a sphere shaped envelope at each center location, where it can
  //  be guaranteed that the track chain geometry will not penetrate the sphere.
  std::vector<ChVector<>> rolling_elem_locs; // w.r.t. chassis ref. frame
  std::vector<double> clearance;  // 1 per rolling elem  
  std::vector<ChVector<>> rolling_elem_spin_axis; /// w.r.t. abs. frame
  

  // initialize 1 of each of the following subsystems.
  // will use the chassis ref frame to do the transforms, since the TrackSystem
  // local ref. frame has same rot (just difference in position)
  m_gear->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    ChCoordsys<>());

  // drive sprocket is First added to the lists passed into TrackChain Init()
  rolling_elem_locs.push_back(ChVector<>() );
  clearance.push_back(m_gear->GetRadius() );
  rolling_elem_spin_axis.push_back(m_gear->GetBody()->GetRot().GetZaxis() );

  // initialize the support rollers.
  for(int r_idx = 0; r_idx < m_num_rollers; r_idx++)
  {
    ChVector<> roller_loc = m_chassis->GetPos();
    roller_loc.y = -1.0;
    roller_loc.x -= 0.3*(0 + r_idx);

    m_rollers[r_idx]->Initialize(m_chassis,
      m_chassis->GetFrame_REF_to_abs(),
      ChCoordsys<>(roller_loc, QUNIT) );

    // add to the points passed into the track chain
    rolling_elem_locs.push_back( roller_loc );
    clearance.push_back(m_rollers[r_idx]->GetRadius() );
    rolling_elem_spin_axis.push_back( m_rollers[r_idx]->GetBody()->GetRot().GetZaxis() );
  }

  // init the idler last
  m_idlers[0]->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    ChCoordsys<>(m_idlerPosRel, Q_from_AngAxis(CH_C_PI, VECT_Z) ),
    idler_preload);

  // add to the lists passed into the track chain Init()
  rolling_elem_locs.push_back(m_idlerPosRel );
  clearance.push_back(m_idlers[0]->GetRadius() );
  rolling_elem_spin_axis.push_back(m_idlers[0]->GetBody()->GetRot().GetZaxis() );

  // when there's only 2 rolling elements, the above locations will repeat
  // the first rolling element at the front and end of the vector.
  // So, just use the mid-point between the first two rolling elements.

  ChVector<> start_pos;
  if( clearance.size() <3 )
  {
    start_pos = (rolling_elem_locs[0] + rolling_elem_locs[1])/2.0;
    start_pos.y += (clearance[0] + clearance[1])/2.0;
  }
  else
  {
    start_pos = (rolling_elem_locs.front() + rolling_elem_locs.back())/2.0;
    start_pos.y += (clearance.front() + clearance.back() )/2.0;
  }
  start_pos.x += 0.04;
  // NOTE: start_pos needs to somewhere on the top length of chain.
  // Rolling_elem_locs MUST be ordered from front-most to the rear, 
  //  w.r.t. the chassis x-dir, so chain links created in a clockwise direction.
  m_chain->Initialize(m_chassis, 
    m_chassis->GetFrame_REF_to_abs(),
    rolling_elem_locs, clearance,
    rolling_elem_spin_axis,
    start_pos );
  
  // can set pin friction between adjoining shoes by activing damping on the DOF
  m_chain->Set_pin_friction(2.0); // [N-m-sec] ???

  // initialize the powertrain, drivelines
  m_ptrain->Initialize(m_chassis, m_gear->GetAxle() );

  // extra idlers?
  if(m_num_idlers > 1)
  {
    // init this idler, and position it so
    // it snaps into place
    double preload_2 = 120000;
    double idler2_xoff = -1.5;
    double idler2_yoff = -2.1;
    double rotation_ang = CH_C_PI_4;

    // get the absolute c-sys of the gear, and set position relative to that point.
    ChCoordsys<> idler2_csys(m_gear->GetBody()->GetPos(), m_gear->GetBody()->GetRot());
    idler2_csys.pos.x += idler2_xoff;
    idler2_csys.pos.y += idler2_yoff;
    // rotation should set the -x dir 
    idler2_csys.rot = Q_from_AngAxis(rotation_ang, VECT_Z);

    // set the idler relative to the chassis/gear origin
    m_idlers[1]->Initialize(m_chassis, 
      m_chassis->GetFrame_REF_to_abs(),
      idler2_csys,
      preload_2);

  }
}

void DriveChain::Update(double time,
                        double throttle,
                        double braking)
{
  // update left and right powertrains, with the new left and right throttle/shaftspeed
  m_ptrain->Update(time, throttle, GetDriveshaftSpeed(0) );

}

void DriveChain::Advance(double step)
{
  double t = 0;
  m_system->SetIterLCPmaxItersStab(60);
  m_system->SetIterLCPmaxItersSpeed(75);
  double settlePhaseA = 0.3;
  double settlePhaseB = 1.0;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( m_system->GetChTime() < settlePhaseA )
    {
      h = 5e-4;
      m_system->SetIterLCPmaxItersStab(100);
      m_system->SetIterLCPmaxItersSpeed(100);
    } else if ( m_system->GetChTime() < settlePhaseB )
    {
      h = 1e-3;
      m_system->SetIterLCPmaxItersStab(75);
      m_system->SetIterLCPmaxItersSpeed(75);
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


double DriveChain::GetIdlerForce() const
{

  // only 1 idler, for now
  ChVector<> out_force = m_idlers[0]->GetSpringForce();

  return out_force.Length();
}


} // end namespace chrono
