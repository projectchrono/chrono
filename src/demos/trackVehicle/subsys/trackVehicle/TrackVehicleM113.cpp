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

#include "TrackVehicleM113.h"

#include "physics/ChGlobal.h"
#include "core/ChFileutils.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/trackSystem/TrackSystemM113.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const ChCoordsys<> TrackVehicleM113::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicleM113::TrackVehicleM113(const std::string& name,
                                   VisualizationType::Enum chassisVis,
                                   CollisionType::Enum chassisCollide,
                                   double mass,
                                   const ChVector<>& Ixx,
                                   double pin_damping_coef,
                                   double tensioner_preload,
                                   double omega_max,
                                   const ChVector<>& right_pos_rel,
                                   const ChVector<>& left_pos_rel,
                                   const ChVector<>& COG_to_REF)
    : ChTrackVehicle(name, chassisVis, chassisCollide, mass, Ixx, 1),
      m_num_tracks(2),
      m_damping(pin_damping_coef),
      m_tensioner_preload(tensioner_preload) {

    // Solver variables - DEFAULTS SET IN PARENT CLASS CONSTRUCTOR
    m_system->SetSolverOverrelaxationParam(0.9);
    m_system->SetSolverSharpnessParam(0.9);
    m_system->SetMaxPenetrationRecoverySpeed(1.5);

    // ---------------------------------------------------------------------------
    // Set the base class variables not created by constructor, if we plan to use them.
    m_meshName = "M113_chassis";
    m_meshFile = vehicle::GetDataFile("M113/Chassis_XforwardYup.obj");
    m_chassisBoxSize = ChVector<>(4.0, 1.2, 1.5);  // full length, height, width of chassis box

    // setup the chassis body
    m_chassis->SetIdentifier(0);
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(COG_to_REF, ChQuaternion<>(1, 0, 0, 0)));
    // add visualization assets to the chassis
    AddVisualization();

    // Right and Left track System relative locations, respectively
    m_TrackSystem_locs.push_back(right_pos_rel);
    m_TrackSystem_locs.push_back(left_pos_rel);

    // create track systems
    for (int i = 0; i < m_num_tracks; i++) {
        std::stringstream t_ss;
        t_ss << "track chain " << i;
        m_TrackSystems.push_back(
            ChSharedPtr<TrackSystemM113>(new TrackSystemM113(t_ss.str(), i, m_tensioner_preload, omega_max)));
    }

    // create the powertrain and drivelines
    for (int j = 0; j < m_num_engines; j++) {
        std::stringstream dl_ss;
        dl_ss << "driveline " << j;
        std::stringstream pt_ss;
        pt_ss << "powertrain " << j;
        m_ptrains.push_back(ChSharedPtr<TrackPowertrain>(new TrackPowertrain(pt_ss.str())));
    }

    // add a dummy shaft
    m_axle = ChSharedPtr<ChShaft>(new ChShaft);
    m_axle->SetName("dummy shaft");
    m_system->Add(m_axle);
}

TrackVehicleM113::~TrackVehicleM113() {
    if (m_ownsSystem)
        delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void TrackVehicleM113::Initialize(const ChCoordsys<>& chassis_Csys) {
    // move the chassis REF frame to the specified initial position/orientation
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassis_Csys));

    // add collision geometry to the chassis
    AddCollisionGeometry();

    // initialize the subsystems with the initial c-sys and specified offsets
    for (int i = 0; i < m_num_tracks; i++) {
        m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i], dynamic_cast<ChTrackVehicle*>(this), m_damping);
    }

    // initialize the powertrain, drivelines
    for (int j = 0; j < m_num_engines; j++) {
        size_t driveGear_R_idx = 2 * j;
        size_t driveGear_L_idx = 2 * j + 1;
        m_ptrains[j]->Initialize(m_chassis, m_axle);
    }
}

//
void TrackVehicleM113::Update(double time, const std::vector<double>& throttle, const std::vector<double>& braking) {
    assert(throttle.size() == m_num_tracks);

    // throttle is applied as the rotational speed on the gears
    for (int i = 0; i < m_num_tracks; i++) {
        m_TrackSystems[i]->Update(time, throttle[i]);
    }
}

void TrackVehicleM113::Advance(double step) {
    double t = 0;
    double settlePhaseA = 1.0;
    double settlePhaseB = 1.0;
    m_system->SetMaxItersSolverStab(100);
    m_system->SetMaxItersSolverSpeed(200);
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        if (m_system->GetChTime() < settlePhaseA) {
            m_system->SetMaxItersSolverStab(100);
            m_system->SetMaxItersSolverSpeed(100);
            // h = step/2.0;
        } else if (m_system->GetChTime() < settlePhaseB) {
            m_system->SetMaxItersSolverStab(100);
            m_system->SetMaxItersSolverSpeed(100);
            // h = step/2.0;
        }
        m_system->DoStepDynamics(h);
        t += h;
    }
}

// call the chain function to update the constant damping coef.
void TrackVehicleM113::SetShoePinDamping(double damping) {
    m_damping = damping;
    for (int i = 0; i < m_num_tracks; i++) {
        (m_TrackSystems[i]->GetTrackChain())->Set_pin_friction(damping);
    }
}

double TrackVehicleM113::GetDriveshaftSpeed(size_t idx) const {
    assert(idx < m_num_tracks);
    return GetSprocketSpeed(idx);
}

ChSharedPtr<TrackPowertrain> TrackVehicleM113::GetPowertrain(size_t idx) const {
    assert(idx < m_num_engines);
    return m_ptrains[idx];
}

// write output, console functions

// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void TrackVehicleM113::Setup_logger(int what_subsys,
                                    int debug_type,
                                    const std::string& filename,
                                    const std::string& data_dirname) {
    m_save_log_to_file = false;
    m_log_what_to_file = what_subsys;
    m_log_debug_type = debug_type;

    // create the directory for the data files
    if (ChFileutils::MakeDirectory(data_dirname.c_str()) < 0) {
        std::cout << "Error creating directory " << data_dirname << std::endl;
    }
    m_log_file_name = data_dirname + "/" + filename;

    // have the chainSystems been created?
    if (m_TrackSystems.size() > 0) {
        // have the chain, the last subsystem created, been initialized?
        m_save_log_to_file = true;
        // write the headers to all the files as necessary
        create_fileHeaders();
        m_log_file_exists = true;

        // write the system heirarchy and ChSystem data also
        GetLog() << " SAVING model heirarchy and ChSystem details \n";
        ChStreamOutAsciiFile ofile_hier((m_log_file_name + "_Heirarchy.csv").c_str());
        m_system->ShowHierarchy(ofile_hier);
        ChStreamOutAsciiFile ofile_system((m_log_file_name + "_ChSystem.csv").c_str());
        ofile_system << m_system;
    } else {
        GetLog() << " chain subsystem not created yet, not saving data";
    }

    // initialize the rig input values to zero
}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void TrackVehicleM113::Log_to_file() {
    // told to save the data?
    if (m_save_log_to_file) {
        if (!m_log_file_exists) {
            std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
        }
        // open the file to append
        // open the data file for writing the header

        double t = m_system->GetChTime();

        // python pandas expects csv w/ no whitespace
        if (m_log_what_to_file & DBG_CHASSIS) {
            std::stringstream ss_c;
            // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,throttleR,throttleL
            ss_c << t << "," << m_chassis->GetPos() << "," << m_chassis->GetPos_dt() << "," << m_chassis->GetWvel_loc()
                 << "," << m_TrackSystems[(int)RIGHTSIDE]->GetDriveGear()->GetGearMotion() << ","
                 << m_TrackSystems[(int)LEFTSIDE]->GetDriveGear()->GetGearMotion() << "\n";
            ChStreamOutAsciiFile ofileDBG_CHASSIS(m_filename_DBG_CHASSIS.c_str(), std::ios::app);
            ofileDBG_CHASSIS << ss_c.str().c_str();
        }

        // output data for each trackChain system
        for (size_t chain_id = 0; chain_id < m_num_tracks; chain_id++) {
            m_TrackSystems[chain_id]->Write_subsys_data(t, m_chassis);
        }

        if (m_log_what_to_file & DBG_PTRAIN) {
            std::stringstream ss_pt;
            // motor speed, mot torque, out torque
            ss_pt << t << "," << m_ptrains[0]->GetThrottle() << ","
                  << m_ptrains[0]->GetMotorSpeed() * 60.0 / (CH_C_2PI)  // RPM
                  << "," << m_ptrains[0]->GetMotorTorque() << "," << m_ptrains[0]->GetOutputTorque() << "\n";
            ChStreamOutAsciiFile ofilePT(m_filename_DBG_PTRAIN.c_str(), std::ios::app);
            ofilePT << ss_pt.str().c_str();
        }

        if (m_log_what_to_file & DBG_COLLISIONCALLBACK) {
            std::stringstream ss_cc;
            // report # of contacts detected this step between shoe pins # gear.
            // time,Ncontacts,Nbroadphase,NcPz,NcNz
            ss_cc << t << "," << GetCollisionCallback()->GetNcontacts() << ","
                  << GetCollisionCallback()->GetNbroadPhasePassed() << ","
                  << GetCollisionCallback()->Get_sum_Pz_contacts() << ","
                  << GetCollisionCallback()->Get_sum_Nz_contacts() << "\n";
            ChStreamOutAsciiFile ofileCCBACK(m_filename_DBG_COLLISIONCALLBACK.c_str(), std::ios::app);
            ofileCCBACK << ss_cc.str().c_str();
        }

        if (m_log_what_to_file & DBG_ALL_CONTACTS) {
            // use the reporter class with the console log
            std::stringstream ss_fn;  // filename
            ss_fn << m_filename_DBG_ALL_CONTACTS << m_cnt_Log_to_file << ".csv";

            // new file created for each step
            ChStreamOutAsciiFile ofileContacts(ss_fn.str().c_str());

            // write headers to the file first
            std::stringstream ss_header;
            ss_header << "bodyA,bodyB,pAx,pAy,pAz,pBx,pBy,pBz,Nx,Ny,Nz,dist,Fx,Fy,Fz\n";
            ofileContacts << ss_header.str().c_str();

            _contact_reporter m_report(ofileContacts);
            // add it to the system, should be called next timestep
            m_system->GetContactContainer()->ReportAllContacts(&m_report);
        }

        // increment step number
        m_cnt_Log_to_file++;
    }
}

void TrackVehicleM113::create_fileHeaders() {
    // creating files, reset counter of # of times Log_to_file was called
    m_cnt_Log_to_file = 0;

    GetLog() << " ------ Output Data ------------ \n\n";

    if (m_log_what_to_file & DBG_CHASSIS && m_log_debug_type & DBG_BODY) {
        m_filename_DBG_CHASSIS = m_log_file_name + "_chassis.csv";
        ChStreamOutAsciiFile ofileDBG_CHASSIS(m_filename_DBG_CHASSIS.c_str());
        std::stringstream ss_c;
        ss_c << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,throttleR,throttleL\n";
        ofileDBG_CHASSIS << ss_c.str().c_str();
    }

    // write broadphase, narrow phase contact info
    if (m_log_what_to_file & DBG_COLLISIONCALLBACK) {
        // filename, collision callback
        m_filename_DBG_COLLISIONCALLBACK = m_log_file_name + "_Ccallback.csv";
        ChStreamOutAsciiFile ofile(m_filename_DBG_COLLISIONCALLBACK.c_str());
        // headers
        std::stringstream ss_callback_head;
        ss_callback_head << "time,Ncontacts,Nbroadphase,NcPz,NcNz\n";
        ofile << ss_callback_head.str().c_str();
    }

    if (m_log_what_to_file & DBG_ALL_CONTACTS) {
        m_filename_DBG_ALL_CONTACTS = m_log_file_name + "_AllContacts";
    }
    // create headers for all the subsystems in each chain system
    for (size_t chain_id = 0; chain_id < m_num_tracks; chain_id++) {
        // don't forget to provide the pointer to the custom collision callback ptr
        m_TrackSystems[chain_id]->Write_subsys_headers(m_log_what_to_file, m_log_debug_type, m_log_file_name);
    }
}

}  // end namespace chrono
