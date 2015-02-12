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


// to help write vectors and quats to file by overloading stringstream

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
                       size_t num_rollers,
                       double gear_mass,
                       const ChVector<>& gear_inertia
): ChTrackVehicle(gearVis, gearCollide, gear_mass, gear_inertia, 1),
  m_num_rollers(num_rollers),
  m_num_idlers(num_idlers)
{
  // ---------------------------------------------------------------------------
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
  // drive gear, inherits drivechain's visual and collision types, mass, inertia, etc.
  m_gear = ChSharedPtr<DriveGear>(new DriveGear("drive gear",
    m_vis,
    m_collide,
    0,
    m_mass,
    m_inertia) );

 // idlers
  m_idlers.clear();
  m_idlers.resize(m_num_idlers);
  double idler_mass = 100.0; // 429.6
  ChVector<> idler_Ixx(m_inertia);    // 12.55, 12.55, 14.7
  double tensioner_K = 80e3;
  double tensioner_C = tensioner_K * 0.08;
  m_idlers[0] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler",
    VisualizationType::MESH,
    // VisualizationType::PRIMITIVES,
    CollisionType::PRIMITIVES,
    0,
    idler_mass,
    idler_Ixx,
    tensioner_K,
    tensioner_C) );

  // track chain system
  double shoe_mass = 18.03/4.0; // 18.03
  ChVector<> shoe_Ixx(0.22/4.0, 0.25/4.0, 0.04/4.0);  // 0.22, 0.25, 0.04
  m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain",
    VisualizationType::COMPOUNDPRIMITIVES,
    CollisionType::PRIMITIVES,
    0,
    shoe_mass,
    shoe_Ixx) );

  // create the powertrain, connect transmission shaft directly to gear shaft
  m_ptrain = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain ") );

  // support rollers, if any
  m_rollers.clear();
  m_rollers.resize(m_num_rollers);
  double roller_mass = 50.0;

  for(int j = 0; j < m_num_rollers; j++)
  {
    double roller_r = m_rollers[j]->GetRadius();
    double roller_w = m_rollers[j]->GetWidth();
    // assume constant density cylinder
    ChVector<> roller_Ixx = roller_mass * ChVector<>((3.0*roller_r*roller_r + roller_w*roller_w)/12.0,
      (3.0*roller_r*roller_r + roller_w*roller_w)/12.0,
      roller_r*roller_r/2.0);
    GetLog() << " I just manually calculated inertia, uh-oh \n\n Ixx = " << roller_Ixx << "\n";
    m_rollers[j] = ChSharedPtr<SupportRoller>(new SupportRoller("support roller " +std::to_string(j),
      VisualizationType::PRIMITIVES,
      CollisionType::PRIMITIVES,
      0,
      roller_mass,
      roller_Ixx) );
  }

  if(m_num_idlers > 1)
  {
    // for now, just create 1 more idler
    m_idlers[1] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler 2",
    VisualizationType::MESH,
    // VisualizationType::PRIMITIVES,
    CollisionType::PRIMITIVES,
    0,
    idler_mass,
    idler_Ixx) );
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
  double idler_preload = 20000;
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
    double preload_2 = 60000;
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

// Write constraint violations of subsystems, in order, to the ostraem
// -----------------------------------------------------------------------------
void DriveChain::SaveConstraintViolations(bool include_chain)
{
  if( !m_log_file_exists ) 
  {
    std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
  }

  // call the subsystems in the same order as the headers are set up
  std::stringstream ss_g;
  m_gear->SaveConstraintViolations(ss_g);
  ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str(), std::ios::app);
  ofileGCV << ss_g.str().c_str();

  // report violations for idler constraints
  for(int id = 0; id < m_num_idlers; id++)
  {
    std::stringstream ss_id;
    m_idlers[id]->SaveConstraintViolations(ss_id);
    ChStreamOutAsciiFile ofileICV(m_filename_ICV[id].c_str(), std::ios::app);
    ofileICV << ss_id.str().c_str();
  }

  // violations of the roller revolute joints
  for(int roller = 0; roller < m_num_rollers; roller++)
  {
    std::stringstream ss_r;
    m_rollers[roller]->SaveConstraintViolations(ss_r);
    ChStreamOutAsciiFile ofileRCV(m_filename_RCV[roller].c_str(), std::ios::app);
    ofileRCV << ss_r.str().c_str();
  }

}




// write output


// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void DriveChain::Setup_log_to_file(int what,
                               const std::string& filename)
{
  m_save_log_to_file = false;
  m_log_file_name = filename;
  m_log_what_to_file = what;
  // has the chain been created?
  if(m_chain)
  {
    // have the chain, the last subsystem created, been initialized?
    if(m_chain->Get_numShoes() )
    {
      m_save_log_to_file = true;
      GetLog() << " SAVING OUTPUT DATA TO FILE: \n " << filename.c_str() << "\n";
      create_fileHeader(what);
      m_log_file_exists = true;
    }
    else
    {
      GetLog() << " no shoes were initialized, not saving data ";
    }
  }
  else
  {
    GetLog() << " chain subsystem not created yet, not saving data";
  }

  // initialize the rig input values to zero
}


void DriveChain::Log_to_console(int console_what)
{
  GetLog().SetNumFormat("%10.2f");

  if (console_what & DBG_FIRSTSHOE)
  {
    GetLog() << "\n---- shoe 0 : " << m_chain->GetShoeBody(0)->GetName() << "\n";
    // COG state data

    GetLog() << " COG Pos [m] : "  <<  m_chain->GetShoeBody(0)->GetPos() << "\n";
    GetLog() << " COG Vel [m/s] : "  <<  m_chain->GetShoeBody(0)->GetPos_dt() << "\n";
    GetLog() << " COG Acc [m/s2] : "  <<  m_chain->GetShoeBody(0)->GetPos_dtdt() << "\n";
    GetLog() << " COG omega [rad/s] : "  <<  m_chain->GetShoeBody(0)->GetRot_dt() << "\n";

    // shoe pin tension
    GetLog() << " pin 0 reaction force [N] : "  <<  m_chain->GetPinReactForce(0) << "\n";
    // GetLog() << "pin 0 reaction torque [N-m] : "  <<  m_chain->GetPinReactTorque(0) << "\n";
  }

  if (console_what & DBG_GEAR)
  {
    GetLog() << "\n---- Gear : " << m_gear->GetBody()->GetName() << "\n";
    // COG state data
    GetLog() << " COG Pos [m] : " << m_gear->GetBody()->GetPos() << "\n";
    GetLog() << " COG Vel [m/s] : " << m_gear->GetBody()->GetPos_dt() << "\n";
    GetLog() << " COG omega [rad/s] : " << m_gear->GetBody()->GetRot_dt() << "\n";

    /*
    // # of shoe pins in contact?
    GetLog() << "# of shoes in contact ? : " << m_gear->GetBody()->GetCollisionModel()->Get << "\n";

    
    // # of non-intermittant contact steps
    GetLog() << "cumulative contact steps : " <<  << "\n";
    */
  }

  if (console_what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations(false);
  }

  if (console_what & DBG_PTRAIN)
  {
    GetLog() << "\n ---- powertrain : \n";
  
  }

  GetLog().SetNumFormat("%g");

}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void DriveChain::Log_to_file()
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

    // write the simulation time
    double t = m_system->GetChTime();

    // python pandas expects csv w/ no whitespace
    if( m_log_what_to_file & DBG_FIRSTSHOE )
    {
      std::stringstream ss;
      ss << t <<","<< m_chain->GetShoeBody(0)->GetPos() 
        <<","<<  m_chain->GetShoeBody(0)->GetPos_dt() 
        <<","<<  m_chain->GetShoeBody(0)->GetPos_dtdt()
        <<","<<  m_chain->GetShoeBody(0)->GetRot_dt().Q_to_NasaAngles()
        <<","<<  m_chain->GetPinReactForce(0);
        // <<","<<  m_chain->GetPinReactTorque(0);
      ChStreamOutAsciiFile ofile(m_filename_DBG_FIRSTSHOE.c_str(), std::ios::app);
      ofile << ss.str().c_str();
  
    }
    if (m_log_what_to_file & DBG_GEAR)
    {
      std::stringstream ss_g;
      ss_g << t <<","<< m_gear->GetBody()->GetPos() 
        <<","<< m_gear->GetBody()->GetPos_dt() 
        <<","<< m_gear->GetBody()->GetRot_dt().Q_to_NasaAngles();
      ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str(), std::ios::app);
      ofileDBG_GEAR << ss_g.str().c_str();
    }

    if (m_log_what_to_file & DBG_CONSTRAINTS)
    {
      // Report constraint violations for all joints
      SaveConstraintViolations();
    }
    
    if (m_log_what_to_file & DBG_PTRAIN)
    {
      std::stringstream ss_pt;
      // motor speed, mot torque, out torque
      ss_pt << "," << m_ptrain->GetMotorSpeed()
        << "," << m_ptrain->GetMotorTorque()
        << "," << m_ptrain->GetOutputTorque();
      ChStreamOutAsciiFile ofilePT(m_filename_DBG_PTRAIN.c_str(), std::ios::app);
      ofilePT << ss_pt.str().c_str();
    }

  }
}



void DriveChain::create_fileHeader(int what)
{
  if(what & DBG_FIRSTSHOE)
  {
    // Shoe 0 : S0, Pin0: P0
    m_filename_DBG_FIRSTSHOE = m_log_file_name+"_shoe0.csv";
    ChStreamOutAsciiFile ofileDBG_FIRSTSHOE(m_filename_DBG_FIRSTSHOE.c_str());
    std::stringstream ss;
    ss << "time,S0x,S0y,S0z,S0vx,S0vy,S0vz,S0ax,S0ay,S0az,S0wx,S0wy,S0wz,P0fx,P0fy,P0fz";
    ofileDBG_FIRSTSHOE << ss.str().c_str();
  }
  if(what & DBG_GEAR)
  {
    m_filename_DBG_GEAR = m_log_file_name+"_gear.csv";
    ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str());
    std::stringstream ss;
    ss << "time,Gx,Gy,Gz,Gvx,Gvy,Gvz,Gwx,Gwy,Gwz";
    ofileDBG_GEAR << ss.str().c_str();
  }
  if(what & DBG_CONSTRAINTS)
  {
    // in the same order as listed in the header
    m_filename_GCV = m_log_file_name+"_gear.csv";
    ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str());
    std::stringstream ss;
    ss << m_gear->getFileHeader_ConstraintViolations(0);
    ofileGCV << ss.str().c_str();

    for(int id = 0; id < m_num_idlers; id++)
    {
      m_filename_ICV.push_back(m_log_file_name+"_idler"+std::to_string(id)+".csv");
      ChStreamOutAsciiFile ofileICV(m_filename_ICV.back().c_str());
      std::stringstream ss_id;
      ss_id << m_idlers[id]->getFileHeader_ConstraintViolations(id);
      ofileICV << ss_id.str().c_str();
    }

    // violations of the roller revolute joints
    for(int roller = 0; roller < m_num_rollers; roller++)
    {
      m_filename_RCV.push_back(m_log_file_name+"_roller"+std::to_string(roller)+".csv");
      ChStreamOutAsciiFile ofileRCV(m_filename_ICV.back().c_str());
      std::stringstream ss_r;
      ss_r << m_rollers[roller]->getFileHeader_ConstraintViolations(roller);
      ofileRCV << ss_r.str().c_str();
    }
  }

  if(what & DBG_PTRAIN)
  {
    m_filename_DBG_PTRAIN = m_log_file_name+"_ptrain.csv";
    ChStreamOutAsciiFile ofileDBG_PTRAIN(m_filename_DBG_PTRAIN.c_str());
    std::stringstream ss_pt;
    ss_pt << ",motSpeed,motT,outT";
    ofileDBG_PTRAIN << ss_pt.str().c_str();
  }


}


} // end namespace chrono
