// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// Tracked vehicle model built from subsystems.
//  Location of subsystems hard-coded for M113 vehicle
//  Sprocket body driven with a specified motion.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "physics/ChGlobal.h"
#include "core/ChFileutils.h"

#include "TrackVehicleM113.h"

#include "subsys/trackSystem/TrackSystemM113.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"


namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const double     TrackVehicleM113::mass_override = 5489.2 / 5.0; // chassis sprung mass override
const ChVector<> TrackVehicleM113::COM_override = ChVector<>(0., 0.0, 0.);  // COM location, relative to body Csys REF frame
const ChVector<> TrackVehicleM113::inertia_override(1786.9/5.0, 10449.7/5.0, 10721.2/5.0);  // chassis inertia (roll,yaw,pitch)

const ChCoordsys<> TrackVehicleM113::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicleM113::TrackVehicleM113(const std::string& name,
                           VisualizationType::Enum chassisVis,
                           CollisionType::Enum chassisCollide,
                           double mass,
                           const ChVector<>& Ixx,
                           double pin_damping_coef,
                           double tensioner_preload,
                           const ChVector<>& left_pos_rel,
                           const ChVector<>& right_pos_rel
): ChTrackVehicle(name, chassisVis, chassisCollide, mass, Ixx, 1),
  m_num_tracks(2),
  m_trackSys_L(left_pos_rel),
  m_trackSys_R(right_pos_rel),
  m_damping(pin_damping_coef),
  m_tensioner_preload(tensioner_preload)
{
  // Solver variables
  m_system->SetIterLCPomega(0.9);
  m_system->SetIterLCPsharpnessLambda(0.9);
  m_system->SetMaxPenetrationRecoverySpeed(1.5);

  // ---------------------------------------------------------------------------
  // Set the base class variables not created by constructor, if we plan to use them.
  m_meshName = "M113_chassis";
  m_meshFile = utils::GetModelDataFile("M113/Chassis_XforwardYup.obj");
  m_chassisBoxSize = ChVector<>(4.0, 1.2, 1.5); // full length, height, width of chassis box

  // setup the chassis body
  m_chassis->SetIdentifier(0);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(COM_override, ChQuaternion<>(1, 0, 0, 0)));
  // add visualization assets to the chassis
  AddVisualization();

  // resize all vectors for the number of track systems
  m_TrackSystems.resize(m_num_tracks);
  m_TrackSystem_locs.resize(m_num_tracks);
  m_filename_WCV.resize(m_num_tracks);

  // Right and Left track System relative locations, respectively
  m_TrackSystem_locs[0] = m_trackSys_L;
  m_TrackSystem_locs[1] = m_trackSys_R;

  // two drive Gears, like a 2WD driven vehicle.
  // m_drivelines.resize(m_num_engines);
  // m_ptrains.resize(m_num_engines); // done by base vehicle class

  // create track systems
  for (int i = 0; i < m_num_tracks; i++) {
    std::stringstream t_ss;
    t_ss << "track chain " << i;
    m_TrackSystems[i] = ChSharedPtr<TrackSystemM113>(new TrackSystemM113(t_ss.str(), i, m_tensioner_preload) );
  }
  
  // create the powertrain and drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    std::stringstream dl_ss;
    dl_ss << "driveline " << j;
    std::stringstream pt_ss;
    pt_ss << "powertrain " << j;
    // m_drivelines[j] = ChSharedPtr<TrackDriveline>(new TrackDriveline(dl_ss.str() ) );
    m_ptrains[j] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain(pt_ss.str() ) );
  }

  // add a dummy shaft
  m_axle = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle->SetName("dummy shaft");
  m_system->Add(m_axle);
}


TrackVehicleM113::~TrackVehicleM113()
{
  if(m_ownsSystem)
    delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void TrackVehicleM113::Initialize(const ChCoordsys<>& chassis_Csys)
{
  // move the chassis REF frame to the specified initial position/orientation
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassis_Csys));

  // add collision geometry to the chassis
  AddCollisionGeometry();

  // initialize the subsystems with the initial c-sys and specified offsets
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i],
      dynamic_cast<ChTrackVehicle*>(this),
      m_damping);
  }

  // initialize the powertrain, drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    size_t driveGear_R_idx = 2*j;
    size_t driveGear_L_idx = 2*j + 1;
    m_ptrains[j]->Initialize(m_chassis, m_axle );
  }

}

// 
void TrackVehicleM113::Update(double	time,
                          const std::vector<double>&  throttle,
                          const std::vector<double>&  braking)
{
  assert( throttle.size() == m_num_tracks);
  
  // throttle is applied as the rotational speed on the gears
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Update(time, throttle[i]);
  }
}

void TrackVehicleM113::Advance(double step)
{
  double t = 0;
  double settlePhaseA = 0.25;
  double settlePhaseB = 0.5;
  m_system->SetIterLCPmaxItersStab(100);
  m_system->SetIterLCPmaxItersSpeed(150);
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( m_system->GetChTime() < settlePhaseA )
    {
      m_system->SetIterLCPmaxItersStab(100);
      m_system->SetIterLCPmaxItersSpeed(100);
      // h = step/2.0;
    } else if ( m_system->GetChTime() < settlePhaseB )
    {
      m_system->SetIterLCPmaxItersStab(150);
      m_system->SetIterLCPmaxItersSpeed(200);
      // h = step/2.0;
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


// call the chain function to update the constant damping coef.
void TrackVehicleM113::SetShoePinDamping(double damping)
{
  m_damping = damping;
  for( int i = 0; i < m_num_tracks; i++)
  {
    (m_TrackSystems[i]->GetTrackChain())->Set_pin_friction(damping);
  }
}

double TrackVehicleM113::GetDriveshaftSpeed(size_t idx) const
{
  assert(idx < m_num_tracks );
  return m_axle->GetPos_dt();
}


const ChSharedPtr<TrackPowertrain> TrackVehicleM113::GetPowertrain(size_t idx) const
{ 
  assert( idx < m_num_engines );
  return  m_ptrains[idx];
}


// Log constraint violations
// -----------------------------------------------------------------------------
void TrackVehicleM113::LogConstraintViolations()
{
  GetLog().SetNumFormat("%16.4e");

   GetLog() << " \n\n  Constraint Violations \n";
  // log constraint violation on each chain system
  for(size_t chain_id = 0; chain_id < m_num_tracks; chain_id++)
  {
    GetLog() << "\n  ******************************** \n track system #: " << chain_id << "\n";
    // Report constraint violations for the gear revolute joint
    GetLog() << "\n\n---- Gear constraint violations\n";
    m_TrackSystems[chain_id]->GetDriveGear()->LogConstraintViolations();

    GetLog() << "\n\n---- Idler constraint violations\n";
    m_TrackSystems[chain_id]->GetIdler()->LogConstraintViolations();

    // violations of the bogie revolute joints in the suspensions
    for(size_t susp_id = 0; susp_id < m_TrackSystems[chain_id]->Get_NumWheels(); susp_id++)
    {
      GetLog() << "\n\n---- Suspension #: "<< susp_id << " constrain violations\n";
      m_TrackSystems[chain_id]->GetSuspension(susp_id)->LogConstraintViolations();
    }

    // TODO: track chain inter-shoe revolute constraint violation
  }
  GetLog().SetNumFormat("%g");
}

// Write constraint violations of subsystems, in order, to the ostraem
// -----------------------------------------------------------------------------
void TrackVehicleM113::SaveConstraintViolations()
{
  if( !m_log_file_exists ) 
  {
    std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
  }
  double t = m_system->GetChTime();

  // save constraint violation on each chain system
  for(size_t chain_id = 0; chain_id < m_num_tracks; chain_id++)
  {
    // call the subsystems in the same order as the headers are set up
    // save violations for gear revolute constraint
    std::stringstream ss_g;
    ss_g << t;
    m_TrackSystems[chain_id]->GetDriveGear()->SaveConstraintViolations(ss_g);
    ChStreamOutAsciiFile ofileGCV(m_filename_GCV[chain_id].c_str(), std::ios::app);
    ofileGCV << ss_g.str().c_str();

    // save violations for idler constraint
    std::stringstream ss_id;
    ss_id << t;
    m_TrackSystems[chain_id]->GetIdler()->SaveConstraintViolations(ss_id);
    ChStreamOutAsciiFile ofileICV(m_filename_ICV[chain_id].c_str(), std::ios::app);
    ofileICV << ss_id.str().c_str();

    // save violations of the bogie wheel revolute joints
    for(size_t w_id = 0; w_id < m_TrackSystems[chain_id]->Get_NumWheels(); w_id++)
    {
      std::stringstream ss_r;
      ss_r << t;
      m_TrackSystems[chain_id]->GetSuspension(w_id)->SaveConstraintViolations(ss_r);
      ChStreamOutAsciiFile ofileRCV(m_filename_WCV[chain_id][w_id].c_str(), std::ios::app);
      ofileRCV << ss_r.str().c_str();
    }

    // TODO: save the violations of the inter-shoe body revolute constraints

  }
}

// write output, console functions

// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void TrackVehicleM113::Setup_log_to_file(int what,
                               const std::string& filename,
                               const std::string& data_dirname)
{
  m_save_log_to_file = false;
  m_log_what_to_file = what;

  // create the directory for the data files
  if(ChFileutils::MakeDirectory(data_dirname.c_str()) < 0) {
    std::cout << "Error creating directory " << data_dirname << std::endl;
  }
  m_log_file_name = data_dirname + "/" + filename;

  // have the chainSystems been created?
  if(m_TrackSystems.size() > 0)
  {
    // have the chain, the last subsystem created, been initialized?
    m_save_log_to_file = true;
    // write the headers to all the files as necessary
    create_fileHeaders(what);
    m_log_file_exists = true;

    // write the system heirarchy and ChSystem data also
    GetLog() << " SAVING model heirarchy and ChSystem details \n";
    ChStreamOutAsciiFile ofile_hier( (m_log_file_name +"_Heirarchy.csv").c_str() );
    m_system->ShowHierarchy(ofile_hier); 
    ChStreamOutAsciiFile ofile_system( (m_log_file_name +"_ChSystem.csv").c_str() );
    m_system->StreamOUT( ofile_system );

  }
  else
  {
    GetLog() << " chain subsystem not created yet, not saving data";
  }

  // initialize the rig input values to zero
}


void TrackVehicleM113::Log_to_console(int console_what)
{
  GetLog().SetNumFormat("%f");//%10.4f");

  if(console_what & DBG_CHASSIS)
  {
    GetLog() << "\n---- Chassis : " << m_chassis->GetName()
      << "\n COG Pos [m]: " << m_chassis->GetPos()
      << "\n COG Vel [m/s]: " << m_chassis->GetPos_dt()
      << "\n COG omega [rad/s]: " << m_chassis->GetWvel_loc()
      << "\n RightSide Applied Motion [rad/s]: " << m_TrackSystems[(int)RIGHTSIDE]->GetDriveGear()->GetGearMotion()
      << "\n LeftSide Applied Motion [rad/s]: " << m_TrackSystems[(int)LEFTSIDE]->GetDriveGear()->GetGearMotion()
      << "\n";
  }

  // log to console all the subsystem data in each chain system
  for(size_t chain_id = 0; chain_id < m_num_tracks; chain_id++)
  {
    if (console_what & DBG_GEAR)
    {
      // gear state data, contact info
      GetLog() << "\n---- Gear : " << m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetName() 
        << "\n COG Pos [m]: " << m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetPos() 
        << "\n COG Vel [m/s]: " << m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetPos_dt() 
        << "\n COG omega [rad/s]: " << m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetRot_dt().Q_to_NasaAngles()
        << "\n";

    }

    if (console_what & DBG_IDLER)
    {
      GetLog() << "\n---- Idler : " << m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetName() 
        <<"\n COG Pos [m]: " << m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetPos() 
        <<"\n COG Vel [m/s]: " << m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetPos_dt()
        <<"\n COG omega [rad/s]: " << m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetRot_dt().Q_to_NasaAngles()
        <<"\n spring react F [N]: " << m_TrackSystems[chain_id]->GetIdler()->GetSpringForce()
        <<"\n react from K [N]: " << m_TrackSystems[chain_id]->GetIdler()->Get_SpringReact_Deform()
        <<"\n react from C [N] " << m_TrackSystems[chain_id]->GetIdler()->Get_SpringReact_Deform_dt();
    }

    if (console_what & DBG_CONSTRAINTS)
    {
      // Report constraint violations for all joints
      LogConstraintViolations();
    }

    // TODO:
    /*
    if(console_what & DBG_ALL_CONTACTS)
    {
      // use the reporter class with the console log
      _contact_reporter m_report(GetLog() );
      // add it to the system, should be called next timestep
      m_system->GetContactContainer()->ReportAllContacts(&m_report);
    }
    */
  }

  GetLog().SetNumFormat("%g");

}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void TrackVehicleM113::Log_to_file()
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

    double t = m_system->GetChTime();


    if(m_log_what_to_file & DBG_CHASSIS)
    {
      std::stringstream ss_c;
      // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,throttleR,throttleL
      ss_c << t <<","<< m_chassis->GetPos()
        <<","<< m_chassis->GetPos_dt()
        <<","<< m_chassis->GetWvel_loc()
        <<","<< m_TrackSystems[(int)RIGHTSIDE]->GetDriveGear()->GetGearMotion()
        <<","<< m_TrackSystems[(int)LEFTSIDE]->GetDriveGear()->GetGearMotion()
        <<"\n";
      ChStreamOutAsciiFile ofileDBG_CHASSIS(m_filename_DBG_CHASSIS.c_str(), std::ios::app);
      ofileDBG_CHASSIS << ss_c.str().c_str();
    }


    // create headers for all the subsystems in each chain system
    for(size_t chain_id = 0; chain_id < m_num_tracks; chain_id++ )
    {
      if (m_log_what_to_file & DBG_GEAR)
      {
        std::stringstream ss_g;
        // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz
        ss_g << t <<","<< m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetPos() 
          <<","<< m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetPos_dt() 
          <<","<< m_TrackSystems[chain_id]->GetDriveGear()->GetBody()->GetWvel_loc()
          <<"\n";
        ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR[chain_id].c_str(), std::ios::app);
        ofileDBG_GEAR << ss_g.str().c_str();
      }

      if (m_log_what_to_file & DBG_IDLER)
      {
        std::stringstream ss_id;
        // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c
        ss_id << t <<","<< m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetPos()
          <<","<< m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetPos_dt()
          <<","<< m_TrackSystems[chain_id]->GetIdler()->GetBody()->GetWvel_loc()
          <<","<< m_TrackSystems[chain_id]->GetIdler()->GetSpringForce()
          <<","<< m_TrackSystems[chain_id]->GetIdler()->Get_SpringReact_Deform()
          <<","<< m_TrackSystems[chain_id]->GetIdler()->Get_SpringReact_Deform_dt()
          <<"\n";
        ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER[chain_id].c_str(), std::ios::app);
        ofileDBG_IDLER << ss_id.str().c_str();
      }

      if (m_log_what_to_file & DBG_CONSTRAINTS)
      {
        // Report constraint violations for all joints
        SaveConstraintViolations();
      }
    }

    // increment step number
    m_cnt_Log_to_file++;
  }
}



void TrackVehicleM113::create_fileHeaders(int what)
{
  // creating files, reset counter of # of times Log_to_file was called
  m_cnt_Log_to_file = 0;

  GetLog() << " ------ Output Data ------------ \n\n";

  if(what & DBG_CHASSIS)
  {
    std::stringstream ss_c_fn;
    ss_c_fn << m_log_file_name << "_chassis.csv";
    m_filename_DBG_CHASSIS = ss_c_fn.str();
    ChStreamOutAsciiFile ofileDBG_CHASSIS(m_filename_DBG_CHASSIS.c_str() );
    std::stringstream ss_c;
    ss_c << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,throttleR,throttleL\n";
    ofileDBG_CHASSIS << ss_c.str().c_str();
    GetLog() << " writing to file: " << m_filename_DBG_CHASSIS
      << "\n        data: " << ss_c.str().c_str() << "\n";
  }

  // create headers for all the subsystems in each chain system
  for(size_t chain_id = 0; chain_id < m_num_tracks; chain_id++)
  {
    if(what & DBG_GEAR)
    {
      std::stringstream ss_gear_fn;
      ss_gear_fn << m_log_file_name  << "_Side" << chain_id << "_gear.csv";
      m_filename_DBG_GEAR.push_back( ss_gear_fn.str() );
      // open the file and write the header
      ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.back().c_str() );
      std::stringstream ss_g;
      ss_g << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz\n";
      ofileDBG_GEAR << ss_g.str().c_str();
      GetLog() << " writing to file: " << m_filename_DBG_GEAR.back() 
        << "\n          data: " << ss_g.str().c_str() <<"\n";

    }

    if(what & DBG_IDLER)
    {
      std::stringstream ss_idler_fn;
      ss_idler_fn <<  m_log_file_name << "_Side" << chain_id << "_idler.csv";
      m_filename_DBG_IDLER.push_back(ss_idler_fn.str() );
      // open the file, write header
      ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.back().c_str() );
      std::stringstream ss_id;
      ss_id << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c\n";
      ofileDBG_IDLER << ss_id.str().c_str();
      GetLog() << " writing to file: " << m_filename_DBG_IDLER.back() 
        << "\n          data:" << ss_id.str().c_str() <<"\n";
    }

    // write the data for each subsystem's constraint violation
    if(what & DBG_CONSTRAINTS)
    {
      // in the same order as listed in the header
      // open the file, write header for gear CV
      std::stringstream ss_gcv_fn;
      ss_gcv_fn << m_log_file_name << "_Side" << chain_id <<"_GearCV.csv";
      m_filename_GCV.push_back(ss_gcv_fn.str() );
      ChStreamOutAsciiFile ofileGCV(m_filename_GCV.back().c_str() );
      std::stringstream ss_gCV;
      ss_gCV << m_TrackSystems[chain_id]->GetDriveGear()->getFileHeader_ConstraintViolations(0);
      ofileGCV << ss_gCV.str().c_str();
      GetLog() << " writing to file: " << m_filename_GCV.back() 
        << "\n          data: " << ss_gCV.str().c_str() <<"\n";

      // open the file, write header for idler CV
      std::stringstream ss_icv_fn;
      ss_icv_fn << m_log_file_name << "_Side" << chain_id << "_idlerCV.csv";
      m_filename_ICV.push_back(ss_icv_fn.str() );
      ChStreamOutAsciiFile ofileICV(m_filename_ICV.back().c_str());
      std::stringstream ss_header;
      ss_header << m_TrackSystems[chain_id]->GetIdler()->getFileHeader_ConstraintViolations();
      ofileICV << ss_header.str().c_str();
      GetLog() << " writing to file: " << m_filename_ICV.back() 
        << "\n          data: " << ss_header.str().c_str() <<"\n";

      // open the file, write headers for violations of the bogie wheels revolute joints
      for(size_t w_id = 0; w_id < m_TrackSystems[chain_id]->Get_NumWheels(); w_id++)
      {
        std::stringstream ss_wCV;
        ss_wCV << m_log_file_name << "_Side" << chain_id << "_roller" << w_id << "CV.csv";
        m_filename_WCV[chain_id].push_back(ss_wCV.str());
        ChStreamOutAsciiFile ofileWCV(m_filename_WCV[chain_id].back().c_str());
        std::stringstream ss_header;
        ofileWCV << ss_wCV.str().c_str();
        GetLog() << " writing to file: " << m_filename_WCV[chain_id].back() 
        << "\n         data: " << ss_header.str().c_str() <<"\n";
      }
    }

/*
    // write all contact info to a new file each step
    if(what & DBG_ALL_CONTACTS)
    {
      m_filename_DBG_ALL_CONTACTS = m_log_file_name+"_allContacts";
      // write a file each step, so we'll write a header then.
      GetLog() << " writing contact info to file name: " << m_filename_DBG_ALL_CONTACTS << "\n\n";
    }
*/

  }

}


} // end namespace chrono
