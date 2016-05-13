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
#include <algorithm>

#include "DriveChain.h"

#include "physics/ChGlobal.h"
#include "core/ChFileutils.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/trackSystem/TrackSystem.h"

// -----------------------------------------------------------------------------
// Static variables
// const ChCoordsys<> DriveChain::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

namespace chrono {

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
/// chassis is static, so use idler info to create the gear body, instead.
DriveChain::DriveChain(const std::string& name,
                       VisualizationType::Enum vis,
                       CollisionType::Enum collide,
                       double pin_damping_coef,   ///< inter-shoe body revolute joint damping coef., [N-s/m]
                       double tensioner_preload,  ///< idler tensioner-spring preload [N]
                       const ChVector<>& right_pos_rel,
                       const ChVector<>& COG_to_REF)
    : ChTrackVehicle(name, vis, collide, 1), m_damping(pin_damping_coef), m_tensioner_preload(tensioner_preload), m_TrackSystem_loc(right_pos_rel) {
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

    // create a single track system
    m_TrackSystem = ChSharedPtr<TrackSystem>(new TrackSystem("Track system", 0));
   
    // create the powertrain
    m_ptrains.push_back(ChSharedPtr<TrackPowertrain>(new TrackPowertrain("Powertrain")));
}

DriveChain::~DriveChain() {
    if (m_ownsSystem)
        delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void DriveChain::Initialize(const ChCoordsys<>& chassis_Csys) {

    // move the chassis REF frame to the specified initial position/orientation
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassis_Csys));

    // add collision geometry to the chassis
    AddCollisionGeometry();
    
    // initialize the track system
    m_TrackSystem->Initialize(m_chassis, m_TrackSystem_loc, dynamic_cast<ChTrackVehicle*>(this),
        m_damping);

    // init. the powertrain
    m_ptrains[0]->Initialize(m_chassis, m_TrackSystem->GetDriveGear()->GetAxle());
}

void DriveChain::Update(double time,    ///< [in] current time
    const std::vector<double>& throttle,  ///< [in] current steering input [-1,+1]
    const std::vector<double>& braking    ///< [in] current braking input [0,1]
    ) {
    // just use the right throttle for input
    m_ptrains[0]->Update(time, throttle[0], GetDriveshaftSpeed(0));
}

void DriveChain::Advance(double step) {
    double t = 0;
    double settlePhaseA = 0.05;
    double settlePhaseB = 0.25;
    m_system->SetMaxItersSolverStab(75);
    m_system->SetMaxItersSolverSpeed(100);
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        if (m_system->GetChTime() < settlePhaseA) {
            m_system->SetMaxItersSolverStab(100);
            m_system->SetMaxItersSolverSpeed(150);
        }
        else if (m_system->GetChTime() < settlePhaseB) {
            m_system->SetMaxItersSolverStab(100);
            m_system->SetMaxItersSolverSpeed(150);
        }
        m_system->DoStepDynamics(h);
        t += h;
    }
}


// call the chain function to update the constant damping coef.
void DriveChain::SetShoePinDamping(double damping) {
    m_damping = damping;
    m_TrackSystem->GetTrackChain()->Set_pin_friction(damping);
}


// write output
/*
// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
virtual void Setup_logger(int what_subsys,  /// which vehicle objects (e.g. subsystems) to save data for?
                          int debug_type,   /// data types: _BODY, _CONSTRAINTS, _CONTACTS
                          const std::string& out_filename,
                          const std::string& data_dirname = "data_test") {
    m_save_log_to_file = false;
    m_log_what_to_file = what_subsys;
    m_log_debug_type = debug_type;

    // create the directory for the data files
    if (ChFileutils::MakeDirectory(data_dirname.c_str()) < 0) {
        std::cout << "Error creating directory " << data_dirname << std::endl;
    }
    m_log_file_name = data_dirname + "/" + filename;

    // has the chain been created?
    if (m_chain) {
        // have the chain, the last subsystem created, been initialized?
        if (m_chain->Get_numShoes()) {
            m_save_log_to_file = true;
            GetLog() << " SAVING OUTPUT DATA TO FILE: \n " << filename.c_str() << "\n";
            create_fileHeaders(what);
            m_log_file_exists = true;

            // write the system heirarchy and ChSystem data also
            GetLog() << " SAVING model heirarchy and ChSystem details \n";
            ChStreamOutAsciiFile ofile_hier((m_log_file_name + "_Heirarchy.csv").c_str());
            m_system->ShowHierarchy(ofile_hier);
            ChStreamOutAsciiFile ofile_system((m_log_file_name + "_ChSystem.csv").c_str());
            m_system->StreamOUT(ofile_system);

        } else {
            GetLog() << " no shoes were initialized, not saving data ";
        }
    } else {
        GetLog() << " chain subsystem not created yet, not saving data";
    }

    // initialize the rig input values to zero
}

void DriveChain::Log_to_console(int console_what) {
    GetLog().SetNumFormat("%f");  //%10.4f");

    if (console_what & DBG_FIRSTSHOE) {
        // first shoe COG state data, and first pin force/torque
        GetLog() << "\n---- shoe 0 : " << m_chain->GetShoeBody(0)->GetName()
                 << "\n COG Pos [m] : " << m_chain->GetShoeBody(0)->GetPos()
                 << "\n COG Vel [m/s] : " << m_chain->GetShoeBody(0)->GetPos_dt()
                 << "\n COG Acc [m/s2] : " << m_chain->GetShoeBody(0)->GetPos_dtdt()
                 << "\n COG omega [rad/s] : " << m_chain->GetShoeBody(0)->GetRot_dt() << "\n";

        // shoe pin tension
        GetLog() << " pin 0 reaction force [N] : " << m_chain->GetPinReactForce(0) << "\n";
        GetLog() << "pin 0 reaction torque [N-m] : " << m_chain->GetPinReactTorque(0) << "\n";

        // shoe - gear contact details
        if (1) {
            //  specify some collision info with the gear
            std::vector<ChVector<> > sg_info;             // output data set
            std::vector<ChVector<> > Force_mag_info;      // contact forces, (Fn, Ft, 0),
            std::vector<ChVector<> > Ft_info;             // per step friction contact force statistics
            std::vector<ChVector<> > PosRel_contact;      // location of contact point, relative to gear c-sysss
            std::vector<ChVector<> > VRel_contact;        // velocity of contact point, relative to gear c-sys
            std::vector<ChVector<> > NormDirRel_contact;  // tracked contact normal dir., w.r.t. gear c-sys
            // sg_info = (Num_contacts, t_persist, t_persist_max)
            reportShoeGearContact(m_chain->GetShoeBody(0)->GetNameString(), sg_info, Force_mag_info, PosRel_contact,
                                  VRel_contact, NormDirRel_contact);

            GetLog() << "\n ---- Shoe - gear contact info, INDEX 0 :"
                     << "\n (# contacts, time_persist, t_persist_max) : " << sg_info[0]
                     << "\n force magnitude, (Fn, Ft, 0) : " << Force_mag_info[0]
                     << "\n contact point rel pos : " << PosRel_contact[0]
                     << "\n contact point rel vel : " << VRel_contact[0]
                     << "\n contact point rel norm. dir : " << NormDirRel_contact[0] << "\n";
        }
    }

    if (console_what & DBG_GEAR) {
        // gear state data, contact info
        GetLog() << "\n---- Gear : " << m_gear->GetBody()->GetName()
                 << "\n COG Pos [m] : " << m_gear->GetBody()->GetPos()
                 << "\n COG Vel [m/s] : " << m_gear->GetBody()->GetPos_dt()
                 << "\n COG omega [rad/s] : " << m_gear->GetBody()->GetRot_dt().Q_to_NasaAngles() << "\n";

        // find what's in contact with the gear by processing all collisions with a special callback function
        ChVector<> Fn_info = ChVector<>();
        ChVector<> Ft_info = ChVector<>();
        // info is: (max, avg., variance)
        int num_gear_contacts = reportGearContact(Fn_info, Ft_info);

        GetLog() << "\n     Gear Contact info"
                 << "\n # of contacts : " << num_gear_contacts << "\n normal (max, avg, variance) : " << Fn_info
                 << "\n tangent (max, avg, variance) : " << Ft_info << "\n";
    }

    if (console_what & DBG_IDLER) {
        GetLog() << "\n---- Idler : " << m_idlers[0]->GetBody()->GetName()
                 << "\n COG Pos [m] : " << m_idlers[0]->GetBody()->GetPos()
                 << "\n COG Vel [m/s] : " << m_idlers[0]->GetBody()->GetPos_dt()
                 << "\n COG omega [rad/s] : " << m_idlers[0]->GetBody()->GetRot_dt().Q_to_NasaAngles()
                 << "\n spring react F [N] : " << m_idlers[0]->GetSpringForce()
                 << "\n react from K [N] : " << m_idlers[0]->Get_SpringReact_Deform()
                 << "\n react from C [N] : " << m_idlers[0]->Get_SpringReact_Deform_dt();
    }

    if (console_what & DBG_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    if (console_what & DBG_PTRAIN) {
        GetLog() << "\n ---- powertrain \n throttle : " << m_ptrains[0]->GetThrottle()
                 << "\n motor speed [RPM] : " << m_ptrains[0]->GetMotorSpeed() * 60.0 / (CH_C_2PI)
                 << "\n motor torque [N-m] : " << m_ptrains[0]->GetMotorTorque()
                 << "\n output torque [N-m : " << m_ptrains[0]->GetOutputTorque() << "\n";
    }

    if (console_what & DBG_COLLISIONCALLBACK & (GetCollisionCallback() != NULL)) {
        GetLog() << "\n ---- collision callback info :"
                 << "\n Contacts this step: " << GetCollisionCallback()->GetNcontacts()
                 << "\n Broadphase passed this step: " << GetCollisionCallback()->GetNbroadPhasePassed()
                 << "\n Sum contacts, +z side: " << GetCollisionCallback()->Get_sum_Pz_contacts()
                 << "\n Sum contacts, -z side: " << GetCollisionCallback()->Get_sum_Nz_contacts() << "\n";
    }

    if (console_what & DBG_ALL_CONTACTS) {
        // use the reporter class with the console log
        _contact_reporter m_report(GetLog());
        // add it to the system, should be called next timestep
        m_system->GetContactContainer()->ReportAllContacts(&m_report);
    }

    GetLog().SetNumFormat("%g");
}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void DriveChain::Log_to_file() {
    // told to save the data?
    if (m_save_log_to_file) {
        if (!m_log_file_exists) {
            std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
        }
        // open the file to append
        // open the data file for writing the header

        // write the simulation time
        double t = m_system->GetChTime();

        // python pandas expects csv w/ no whitespace
        if (m_log_what_to_file & DBG_FIRSTSHOE) {
            std::stringstream ss;
            // time,x,y,z,vx,vy,vz,ax,ay,az,wx,wy,wz,fx,fy,fz
            ss << t << "," << m_chain->GetShoeBody(0)->GetPos() << "," << m_chain->GetShoeBody(0)->GetPos_dt() << ","
               << m_chain->GetShoeBody(0)->GetPos_dtdt() << "," << m_chain->GetShoeBody(0)->GetWvel_loc() << ","
               << m_chain->GetPinReactForce(0) << "," << m_chain->GetPinReactTorque(0) << "\n";
            // open the file for appending, write the data.
            ChStreamOutAsciiFile ofile(m_filename_DBG_FIRSTSHOE.c_str(), std::ios::app);
            ofile << ss.str().c_str();

            // second file, to specify some collision info with the gear
            double num_contacts = 0;
            std::vector<ChVector<> > sg_info;         // output data set
            std::vector<ChVector<> > Force_mag_info;  // per step contact force magnitude, (Fn, Ft, 0)
            std::vector<ChVector<> > PosRel_contact;  // location of a contact point relative to the gear c-sys
            std::vector<ChVector<> > VRel_contact;    // follow the vel. of a contact point relative to the gear c-sys
            std::vector<ChVector<> > NormDirRel_contact;  // tracked contact normal dir., w.r.t. gear c-sys
            // sg_info = (Num_contacts, t_persist, t_persist_max)
            num_contacts = reportShoeGearContact(m_chain->GetShoeBody(0)->GetNameString(), sg_info, Force_mag_info,
                                                 PosRel_contact, VRel_contact, NormDirRel_contact);

            // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
            // "time,Ncontacts,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,VzRelP,normDirRelxP,normDirRelyP,normDirRelzP
            //  ,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,normDirRelxN,normDirRelyN,normDirRelzN
            //  "
            std::stringstream ss_sg;
            ss_sg << t << "," << num_contacts << "," << sg_info[0] << "," << Force_mag_info[0].x << ","
                  << Force_mag_info[0].y << "," << PosRel_contact[0] << "," << VRel_contact[0] << ","
                  << NormDirRel_contact[0] << "," << sg_info[1] << "," << Force_mag_info[1].x << ","
                  << Force_mag_info[1].y << "," << PosRel_contact[1] << "," << VRel_contact[1] << ","
                  << NormDirRel_contact[1] << "\n";
            ChStreamOutAsciiFile ofile_shoeGear(m_filename_DBG_shoeGear.c_str(), std::ios::app);
            ofile_shoeGear << ss_sg.str().c_str();
        }
        if (m_log_what_to_file & DBG_GEAR) {
            std::stringstream ss_g;
            // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz
            ss_g << t << "," << m_gear->GetBody()->GetPos() << "," << m_gear->GetBody()->GetPos_dt() << ","
                 << m_gear->GetBody()->GetWvel_loc() << "\n";
            ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str(), std::ios::app);
            ofileDBG_GEAR << ss_g.str().c_str();

            // second file, for the specific contact info
            std::stringstream ss_gc;

            // find what's in contact with the gear by processing all collisions with a special callback function
            ChVector<> Fn_info = ChVector<>();
            ChVector<> Ft_info = ChVector<>();
            // info is: (max, avg., variance)
            int num_gear_contacts = reportGearContact(Fn_info, Ft_info);
            // time,Ncontacts,FnMax,FnAvg,FnVar,FtMax,FtAvg,FtVar
            ss_gc << t << "," << num_gear_contacts << "," << Fn_info << "," << std::sqrt(Fn_info.z) << "," << Ft_info
                  << "," << std::sqrt(Ft_info.z) << "\n";
            ChStreamOutAsciiFile ofileDBG_GEAR_CONTACT(m_filename_DBG_GEAR_CONTACT.c_str(), std::ios::app);
            ofileDBG_GEAR_CONTACT << ss_gc.str().c_str();
        }

        if (m_log_what_to_file & DBG_IDLER) {
            std::stringstream ss_id;
            // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c
            ss_id << t << "," << m_idlers[0]->GetBody()->GetPos() << "," << m_idlers[0]->GetBody()->GetPos_dt() << ","
                  << m_idlers[0]->GetBody()->GetWvel_loc() << "," << m_idlers[0]->GetSpringForce() << ","
                  << m_idlers[0]->Get_SpringReact_Deform() << "," << m_idlers[0]->Get_SpringReact_Deform_dt() << "\n";
            ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.c_str(), std::ios::app);
            ofileDBG_IDLER << ss_id.str().c_str();
        }

        if (m_log_what_to_file & DBG_CONSTRAINTS) {
            // Report constraint violations for all joints
            SaveConstraintViolations();
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

        if (m_log_what_to_file & DBG_COLLISIONCALLBACK & (GetCollisionCallback() != NULL)) {
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

void DriveChain::create_fileHeaders(int what) {
    // creating files, reset counter of # of times Log_to_file was called
    m_cnt_Log_to_file = 0;

    GetLog() << " ------ Output Data ------------ \n\n";

    if (what & DBG_FIRSTSHOE) {
        // Shoe 0 : S0, Pin0: P0
        m_filename_DBG_FIRSTSHOE = m_log_file_name + "_shoe0.csv";
        ChStreamOutAsciiFile ofileDBG_FIRSTSHOE(m_filename_DBG_FIRSTSHOE.c_str());
        std::stringstream ss;
        ss << "time,x,y,z,Vx,Vy,Vz,Ax,Ay,Az,Wx,Wy,Wz,Fx,Fy,Fz,Tx,Ty,Tz\n";
        ofileDBG_FIRSTSHOE << ss.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_FIRSTSHOE << "\n         data: " << ss.str().c_str() << "\n";

        // report the contact with the gear in a second file
        m_filename_DBG_shoeGear = m_log_file_name + "_shoe0GearContact.csv";
        ChStreamOutAsciiFile ofileDBG_shoeGear(m_filename_DBG_shoeGear.c_str());
        std::stringstream ss_sg;
        // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
        ss_sg << "time,Ncontacts,NcontactsP,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,"
                 "VzRelP,normDirRelxP,normDirRelyP,normDirRelzP"
              << ",NcontactsN,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,"
                 "normDirRelxN,normDirRelyN,normDirRelzN\n";
        ofileDBG_shoeGear << ss_sg.str().c_str();
    }

    if (what & DBG_GEAR) {
        m_filename_DBG_GEAR = m_log_file_name + "_gear.csv";
        ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str());
        std::stringstream ss_g;
        ss_g << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz\n";
        ofileDBG_GEAR << ss_g.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_GEAR << "\n          data: " << ss_g.str().c_str() << "\n";

        // report on some specific collision info in a separate file
        m_filename_DBG_GEAR_CONTACT = m_log_file_name + "_gearContact.csv";
        ChStreamOutAsciiFile ofileDBG_GEAR_CONTACT(m_filename_DBG_GEAR_CONTACT.c_str());
        std::stringstream ss_gc;
        ss_gc << "time,Ncontacts,FnMax,FnAvg,FnVar,FnSig,FtMax,FtAvg,FtVar,FtSig\n";
        ofileDBG_GEAR_CONTACT << ss_gc.str().c_str();
        GetLog() << " writing to file : " << m_filename_DBG_GEAR_CONTACT << "\n          data: " << ss_gc.str().c_str()
                 << "\n";
    }

    if (what & DBG_IDLER) {
        m_filename_DBG_IDLER = m_log_file_name + "_idler.csv";
        ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.c_str());
        std::stringstream ss_id;
        ss_id << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c\n";
        ofileDBG_IDLER << ss_id.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_IDLER << "\n          data:" << ss_id.str().c_str() << "\n";
    }

    // write the data for each subsystem's constraint violation
    if (what & DBG_CONSTRAINTS) {
        // in the same order as listed in the header
        m_filename_GCV = m_log_file_name + "_GearCV.csv";
        ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str());
        std::stringstream ss_gCV;
        ss_gCV << m_gear->getFileHeader_ConstraintViolations(0);
        ofileGCV << ss_gCV.str().c_str();
        GetLog() << " writing to file: " << m_filename_GCV << "\n          data: " << ss_gCV.str().c_str() << "\n";

        for (int id = 0; id < m_num_idlers; id++) {
            std::stringstream ss_iCV;
            ss_iCV << m_log_file_name << "_idler" << id << "CV.csv";
            m_filename_ICV.push_back(ss_iCV.str().c_str());
            ChStreamOutAsciiFile ofileICV(m_filename_ICV.back().c_str());
            std::stringstream ss_header;
            ss_header << m_idlers[id]->getFileHeader_ConstraintViolations();
            ofileICV << ss_header.str().c_str();
            GetLog() << " writing to file: " << m_filename_ICV[id] << "\n          data: " << ss_header.str().c_str()
                     << "\n";
        }

        // violations of the roller revolute joints
        for (int roller = 0; roller < m_num_wheels; roller++) {
            std::stringstream ss_rCV;
            ss_rCV << m_log_file_name << "_roller" << roller << "CV.csv";
            m_filename_RCV.push_back(ss_rCV.str());
            ChStreamOutAsciiFile ofileRCV(m_filename_RCV.back().c_str());
            std::stringstream ss_header;
            ofileRCV << ss_rCV.str().c_str();
            GetLog() << " writing to file: " << m_filename_RCV[roller] << "\n         data: " << ss_header.str().c_str()
                     << "\n";
        }
    }

    // write powertrian headers
    if (what & DBG_PTRAIN) {
        m_filename_DBG_PTRAIN = m_log_file_name + "_ptrain.csv";
        ChStreamOutAsciiFile ofileDBG_PTRAIN(m_filename_DBG_PTRAIN.c_str());
        std::stringstream ss_pt;
        ss_pt << "time,throttle,motSpeed,motT,outT\n";
        ofileDBG_PTRAIN << ss_pt.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_PTRAIN << "\n          data: " << ss_pt.str().c_str()
                 << "\n";
    }

    // write broadphase, narrow phase contact info
    if (what & DBG_COLLISIONCALLBACK & (GetCollisionCallback() != NULL)) {
        m_filename_DBG_COLLISIONCALLBACK = m_log_file_name + "_Ccallback.csv";
        ChStreamOutAsciiFile ofileDBG_COLLISIONCALLBACK(m_filename_DBG_COLLISIONCALLBACK.c_str());
        std::stringstream ss_cc;
        ss_cc << "time,Ncontacts,Nbroadphase,NcPz,NcNz\n";
        ofileDBG_COLLISIONCALLBACK << ss_cc.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_COLLISIONCALLBACK << "\n     data: " << ss_cc.str().c_str()
                 << "\n";
    }

    // write all contact info to a new file each step
    if (what & DBG_ALL_CONTACTS) {
        m_filename_DBG_ALL_CONTACTS = m_log_file_name + "_allContacts";
        // write a file each step, so we'll write a header then.
        GetLog() << " writing contact info to file name: " << m_filename_DBG_ALL_CONTACTS << "\n\n";
    }
}
*/

}  // end namespace chrono
