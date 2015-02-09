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
  double shoe_mass = 18.03/4.0; // 18.03
  ChVector<> shoe_Ixx(0.22/4.0, 0.25/4.0, 0.04/4.0);  // 0.22, 0.25, 0.04
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
  // m_chain->Set_pin_friction(2.0); // [N-m-sec] ???

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
  m_system->SetIterLCPmaxItersStab(80);
  m_system->SetIterLCPmaxItersSpeed(95);
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
      m_system->SetIterLCPmaxItersSpeed(150);
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


// call the chain function to update the constant damping coef.
void DriveChain::SetShoePinDamping(double damping)
{
  m_chain->Set_pin_friction(damping);
}

// Log constraint violations
// -----------------------------------------------------------------------------
void DriveChain::LogConstraintViolations(bool include_chain)
{
  GetLog().SetNumFormat("%16.4e");

  // Report constraint violations for the gear revolute joint
  GetLog() << "\n---- Gear constraint violations\n\n";
  m_gear->LogConstraintViolations();

  // report violations for idler constraints
  for(int id = 0; id < m_num_idlers; id++)
  {
    GetLog() << "\n---- idler # " << id << " constraint violations\n\n";
    m_idlers[id]->LogConstraintViolations();
  }

  // violations of the roller revolute joints
  for(int roller = 0; roller < m_num_rollers; roller++)
  {
    GetLog() << "\n---- Roller # " << roller << " constrain violations\n\n";
    m_rollers[roller]->LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");
}




// write output


// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void DriveChain::Save_DebugLog(int what,
                                   const std::string& filename)
{
  m_log_file_name = filename;
  m_save_log_to_file = true;
  m_log_what = what;
  
  create_fileHeader(what);
  m_log_file_exists = true;

  // initialize the rig input values to zero
}


void DriveChain::DebugLog(int console_what)
{
  GetLog().SetNumFormat("%10.2f");

  if (console_what & DBG_FIRSTSHOE)
  {




    GetLog() << "\n---- Spring (left, right)\n";
    GetLog() << "Length [inch]       "
      << GetSpringLength(FRONT_LEFT) / in2m << "  "
      << GetSpringLength(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Deformation [inch]  "
      << GetSpringDeformation(FRONT_LEFT) / in2m << "  "
      << GetSpringDeformation(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Force [lbf]         "
      << GetSpringForce(FRONT_LEFT) / lbf2N << "  "
      << GetSpringForce(FRONT_RIGHT) / lbf2N << "\n";
  }

  if (console_what & DBG_GEAR)
  {
    GetLog() << "\n---- Shock (left, right,)\n";
    GetLog() << "Length [inch]       "
      << GetShockLength(FRONT_LEFT) / in2m << "  "
      << GetShockLength(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Velocity [inch/s]   "
      << GetShockVelocity(FRONT_LEFT) / in2m << "  "
      << GetShockVelocity(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Force [lbf]         "
      << GetShockForce(FRONT_LEFT) / lbf2N << "  "
      << GetShockForce(FRONT_RIGHT) / lbf2N << "\n";
  }

  if (console_what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations();
  }

  if (console_what & DBG_PTRAIN)
  {
    GetLog() << "\n---- suspension test (left, right)\n";
    /*
    GetLog() << "Actuator Displacement [in] "
      << GetActuatorDisp(FRONT_LEFT) / in2m << "  "
      << GetActuatorDisp(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Actuator Force [N] "
      << GetActuatorForce(FRONT_LEFT) / in2m << "  "
      << GetActuatorForce(FRONT_RIGHT) / in2m << "\n";
    GetLog() << "Actuator marker dist [in] "
      << GetActuatorMarkerDist(FRONT_LEFT) / in2m << "  "
      << GetActuatorMarkerDist(FRONT_RIGHT) / in2m << "\n";
    
  
    
    GetLog() << "Kingpin angle [deg] "
      << Get_KingpinAng(LEFT) * rad2deg << "  "
      << Get_KingpinAng(RIGHT) * rad2deg << "\n";
    GetLog() << "Kingpin offset [in] "
      << Get_KingpinOffset(LEFT) / in2m << "  "
      << Get_KingpinOffset(RIGHT) / in2m << "\n";
    GetLog() << "Caster angle [deg] "
      << Get_CasterAng(LEFT) * rad2deg << "  "
      << Get_CasterAng(RIGHT) * rad2deg << "\n";
    GetLog() << "Caster offset [in] "
      << Get_CasterOffset(LEFT) / in2m << "  "
      << Get_CasterOffset(RIGHT) / in2m << "\n";
    GetLog() << "Toe angle [deg] "
      << Get_ToeAng(LEFT) * rad2deg << "  "
      << Get_ToeAng(RIGHT) * rad2deg << "\n";
    GetLog() << "suspension roll angle [deg] "
      << Get_LCArollAng() << "\n";

      */
  }

  GetLog().SetNumFormat("%g");

}




void DriveChain::SaveLog()
{
  // told to save the data?
  if( m_save_log_to_file )
  {
    if( !m_log_file_exists ) 
    {
      std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
    }
    // open the file to append
    // open the data file for writing the header
  ChStreamOutAsciiFile ofile(m_log_file_name.c_str(), std::ios::app);

    // write the simulation time first, and rig inputs
    std::stringstream ss;
    ss << GetChTime() <<","<< m_steer <<","<< m_postDisp[LEFT] /in2m <<","<< m_postDisp[RIGHT] / in2m;

    // python pandas expects csv w/ no whitespace
    if( m_log_what & DBG_SPRINGS )
    {
      ss << "," << GetSpringLength(FRONT_LEFT) / in2m << ","
        << GetSpringLength(FRONT_RIGHT) / in2m << ","
        << GetSpringDeformation(FRONT_LEFT) / in2m << ","
        << GetSpringDeformation(FRONT_RIGHT) / in2m << ","
        << GetSpringForce(FRONT_LEFT) / lbf2N << ","
        << GetSpringForce(FRONT_RIGHT) / lbf2N;
  
    }
    if (m_log_what & DBG_SHOCKS)
    {
      ss << "," << GetShockLength(FRONT_LEFT) / in2m << ","
        << GetShockLength(FRONT_RIGHT) / in2m << ","
        << GetShockVelocity(FRONT_LEFT) / in2m << ","
        << GetShockVelocity(FRONT_RIGHT) / in2m << ","
        << GetShockForce(FRONT_LEFT) / lbf2N << ","
        << GetShockForce(FRONT_RIGHT) / lbf2N;
    }

    if (m_log_what & DBG_CONSTRAINTS)
    {
      // Report constraint violations for all joints
      LogConstraintViolations();
    }
    
    // ",KA_L,KA_R,Koff_L,Koff_R,CA_L,CA_R,Coff_L,Coff_R,TA_L,TA_R,LCA_roll";
    if (m_log_what & DBG_SUSPENSIONTEST)
    {
      ss <<","<< Get_KingpinAng(LEFT)*rad2deg <<","<< Get_KingpinAng(RIGHT)*rad2deg 
        <<","<< Get_KingpinOffset(LEFT)/in2m <<","<< Get_KingpinOffset(RIGHT)/in2m
        <<","<< Get_CasterAng(LEFT)*rad2deg <<","<< Get_CasterAng(RIGHT)*rad2deg 
        <<","<< Get_CasterOffset(LEFT)/in2m <<","<< Get_CasterOffset(RIGHT)/in2m
        <<","<< Get_ToeAng(LEFT)*rad2deg <<","<< Get_ToeAng(RIGHT)*rad2deg
        <<","<< Get_LCArollAng();
      /*
      ss << "," << GetActuatorDisp(FRONT_LEFT) / in2m << ","
        << GetActuatorDisp(FRONT_RIGHT) / in2m << ","
        << GetActuatorForce(FRONT_LEFT) / in2m << ","
        << GetActuatorForce(FRONT_RIGHT) / in2m << ","
        << GetActuatorMarkerDist(FRONT_LEFT) / in2m << ","
        << GetActuatorMarkerDist(FRONT_RIGHT) / in2m;
       */
    }
    // next line last, then write to file
    ss << "\n";
    ofile << ss.str().c_str();
  }
}



void DriveChain::create_fileHeader(int what)
{
  // open the data file for writing the header
  ChStreamOutAsciiFile ofile(m_log_file_name.c_str());
  // write the headers, output types specified by "what"
  std::stringstream ss;
  ss << "time,steer,postDisp_L,postDisp_R";
  if(what & DBG_SPRINGS)
  {
    // L/R spring length, delta x, force
    ss << ",k_len_L,k_len_R,k_dx_L,k_dx_R,k_F_L,k_F_R";
  }
  if(what & DBG_SHOCKS)
  {
    ss << ",d_len_L,d_len_R,d_vel_L,d_vel_R,d_F_L,d_F_R";
  }
  if(what & DBG_CONSTRAINTS)
  {
    // TODO:
  }
  if(what & DBG_SUSPENSIONTEST)
  {
    ss << ",KA_L,KA_R,Koff_L,Koff_R,CA_L,CA_R,Coff_L,Coff_R,TA_L,TA_R,LCA_roll";
  }

  // write to file, go to next line in file in prep. for next step.
  ofile << ss.str().c_str();
  ofile << "\n";
}




} // end namespace chrono
