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
// Using a TrackSoilBin type system, attach the chassis to another ground body,
//	using an OldHam joint to keep the trach chain system in-plane.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"
#include "physics/ChGlobal.h"

#include "TrackSoilBin.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// collision mesh
#include "geometry/ChTriangleMeshSoup.h"
// custom collision detection classes
#include "subsys/collision/TrackCollisionCallback.h"

// to help write vectors and quats to file by overloading stringstream

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
// idler, relative to gear/chassis
const ChVector<> TrackSoilBin::m_idlerPos(-2.1904, -0.1443, 0.2447);  // relative to local csys
const ChQuaternion<> TrackSoilBin::m_idlerRot(QUNIT);

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackSoilBin::TrackSoilBin(const std::string& name,
                           VisualizationType::Enum chassisVis,
                           CollisionType::Enum chassisCollide,
                           double mass,
                           const ChVector<>& inertia,
                           size_t num_idlers,
                           size_t num_rollers)
    : ChTrackVehicle(name, chassisVis, chassisCollide, mass, inertia, 1),
      m_num_rollers(num_rollers),
      m_num_idlers(num_idlers) {
    // setup the chassis body
    m_chassis->SetIdentifier(0);
    // basic body info. Not relevant since it's fixed.
    m_chassis->SetFrame_COG_to_REF(ChFrame<>());
    // chassis is fixed to ground
    m_chassis->SetBodyFixed(true);

    // Set  base class variables here

    // --------------------------------------------------------------------------
    // BUILD THE SUBSYSTEMS
    // drive gear, inherits drivechain's visual and collision types
    double gear_mass = 100.0;                                    // 436.7
    ChVector<> gear_Ixx(12.22 / 4.0, 12.22 / 4.0, 13.87 / 4.0);  // 12.22, 12.22, 13.87
    m_gear = ChSharedPtr<DriveGear>(new DriveGear("drive gear", m_vis, m_collide, 0, gear_mass, gear_Ixx));

    // idlers, if m ore than 1
    m_idlers.clear();
    m_idlers.resize(m_num_idlers);
    double idler_mass = 100.0;       // 429.6
    ChVector<> idler_Ixx(gear_Ixx);  // 12.55, 12.55, 14.7
    m_idlers[0] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler", VisualizationType::Mesh,
                                                           // VisualizationType::Primitives,
                                                           CollisionType::Primitives, 0, idler_mass, idler_Ixx));

    // track chain system
    double shoe_mass = 18.03 / 4.0;                           // 18.03
    ChVector<> shoe_Ixx(0.22 / 4.0, 0.25 / 4.0, 0.04 / 4.0);  // 0.22, 0.25, 0.04
    m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain", VisualizationType::CompoundPrimitives,
                                                     CollisionType::Primitives, 0, shoe_mass, shoe_Ixx));

    // create the powertrain, connect transmission shaft directly to gear shaft
    m_ptrain = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain "));

    // support rollers, if any
    m_rollers.clear();
    m_rollers.resize(m_num_rollers);
    for (int j = 0; j < m_num_rollers; j++) {
        std::stringstream r_ss;
        r_ss << "support roller " << j;
        m_rollers[j] = ChSharedPtr<SupportRoller>(
            new SupportRoller(r_ss.str(), VisualizationType::Primitives, CollisionType::Primitives));
    }

    if (m_num_idlers > 1) {
        // for now, just create 1 more idler
        m_idlers[1] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler 2", VisualizationType::Mesh,
                                                               // VisualizationType::Primitives,
                                                               CollisionType::Primitives, 0, idler_mass, idler_Ixx));
    }
}

TrackSoilBin::~TrackSoilBin() {
    if (m_ownsSystem)
        delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void TrackSoilBin::Initialize(const ChCoordsys<>& gear_Csys) {
    // initialize the drive gear, idler and track chain
    double idler_preload = 60000;
    // m_idlerPosRel = m_idlerPos;
    m_idlerPosRel = ChVector<>(-2.5, 0, 0);
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(gear_Csys.pos, gear_Csys.rot));

    // Create list of the center location of the rolling elements and their clearance.
    // Clearance is a sphere shaped envelope at each center location, where it can
    //  be guaranteed that the track chain geometry will not penetrate the sphere.
    std::vector<ChVector<> > rolling_elem_locs;       // w.r.t. chassis ref. frame
    std::vector<double> clearance;                    // 1 per rolling elem
    std::vector<ChVector<> > rolling_elem_spin_axis;  /// w.r.t. abs. frame

    // initialize 1 of each of the following subsystems.
    // will use the chassis ref frame to do the transforms, since the TrackSystem
    // local ref. frame has same rot (just difference in position)
    // NOTE: Gear Init() function moved to after TrackChain() Init
    //      However, still add rolling element info for gear to list first

    // drive sprocket is First added to the lists passed into TrackChain Init()
    rolling_elem_locs.push_back(ChVector<>());
    clearance.push_back(m_gear->GetRadius());
    rolling_elem_spin_axis.push_back(m_gear->GetBody()->GetRot().GetZaxis());

    // initialize the support rollers.
    for (int r_idx = 0; r_idx < m_num_rollers; r_idx++) {
        ChVector<> roller_loc = m_chassis->GetPos();
        roller_loc.y = -1.0;
        roller_loc.x -= 0.3 * (0 + r_idx);

        m_rollers[r_idx]->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), ChCoordsys<>(roller_loc, QUNIT));

        // add to the points passed into the track chain
        rolling_elem_locs.push_back(roller_loc);
        clearance.push_back(m_rollers[r_idx]->GetRadius());
        rolling_elem_spin_axis.push_back(m_rollers[r_idx]->GetBody()->GetRot().GetZaxis());
    }

    // init the idler last
    m_idlers[0]->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(),
                            ChCoordsys<>(m_idlerPosRel, Q_from_AngAxis(CH_C_PI, VECT_Z)), idler_preload);

    // add to the lists passed into the track chain Init()
    rolling_elem_locs.push_back(m_idlerPosRel);
    clearance.push_back(m_idlers[0]->GetRadius());
    rolling_elem_spin_axis.push_back(m_idlers[0]->GetBody()->GetRot().GetZaxis());

    // when there's only 2 rolling elements, the above locations will repeat
    // the first rolling element at the front and end of the vector.
    // So, just use the mid-point between the first two rolling elements.

    ChVector<> start_pos;
    if (clearance.size() < 3) {
        start_pos = (rolling_elem_locs[0] + rolling_elem_locs[1]) / 2.0;
        start_pos.y += (clearance[0] + clearance[1]) / 2.0;
    } else {
        start_pos = (rolling_elem_locs.front() + rolling_elem_locs.back()) / 2.0;
        start_pos.y += (clearance.front() + clearance.back()) / 2.0;
    }
    start_pos.x += 0.04;
    // NOTE: start_pos needs to somewhere on the top length of chain.
    // Rolling_elem_locs MUST be ordered from front-most to the rear,
    //  w.r.t. the chassis x-dir, so chain links created in a clockwise direction.
    m_chain->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), rolling_elem_locs, clearance,
                        rolling_elem_spin_axis, start_pos);

    // can set pin friction between adjoining shoes by activing damping on the DOF
    // m_chain->Set_pin_friction(2.0); // [N-m-sec] ???

    // Now, the gear can be initialized
    m_gear->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), ChCoordsys<>(), m_chain->GetShoeBody());

    // initialize the powertrain, drivelines
    m_ptrain->Initialize(m_chassis, m_gear->GetAxle());

    // extra idlers?
    if (m_num_idlers > 1) {
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
        m_idlers[1]->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), idler2_csys, preload_2);
    }
}

void TrackSoilBin::Update(double time, double throttle, double braking) {
    // update left and right powertrains, with the new left and right throttle/shaftspeed
    m_ptrain->Update(time, throttle, GetDriveshaftSpeed(0));
}

void TrackSoilBin::Advance(double step) {
    double t = 0;
    m_system->SetMaxItersSolverStab(80);
    m_system->SetMaxItersSolverSpeed(95);
    double settlePhaseA = 0.3;
    double settlePhaseB = 1.0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        if (m_system->GetChTime() < settlePhaseA) {
            h = 5e-4;
            m_system->SetMaxItersSolverStab(100);
            m_system->SetMaxItersSolverSpeed(100);
        } else if (m_system->GetChTime() < settlePhaseB) {
            h = 1e-3;
            m_system->SetMaxItersSolverStab(75);
            m_system->SetMaxItersSolverSpeed(150);
        }
        m_system->DoStepDynamics(h);
        t += h;
    }
}

// call the chain function to update the constant damping coef.
void TrackSoilBin::SetShoePinDamping(double damping) {
    m_chain->Set_pin_friction(damping);
}

// Log constraint violations
// -----------------------------------------------------------------------------
void TrackSoilBin::LogConstraintViolations(bool include_chain) {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the gear revolute joint
    GetLog() << "\n---- Gear constraint violations\n\n";
    m_gear->LogConstraintViolations();

    // report violations for idler constraints
    for (int id = 0; id < m_num_idlers; id++) {
        GetLog() << "\n---- idler # " << id << " constraint violations\n\n";
        m_idlers[id]->LogConstraintViolations();
    }

    // violations of the roller revolute joints
    for (int roller = 0; roller < m_num_rollers; roller++) {
        GetLog() << "\n---- Roller # " << roller << " constrain violations\n\n";
        m_rollers[roller]->LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// Write constraint violations of subsystems, in order, to the ostraem
// -----------------------------------------------------------------------------
void TrackSoilBin::SaveConstraintViolations(std::stringstream& ss, bool include_chain) {
    if (!m_log_file_exists) {
        std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
    }
    // call the subsystems in the same order as the headers are set up
    m_gear->SaveConstraintViolations(ss);

    // report violations for idler constraints
    for (int id = 0; id < m_num_idlers; id++) {
        m_idlers[id]->SaveConstraintViolations(ss);
    }

    // violations of the roller revolute joints
    for (int roller = 0; roller < m_num_rollers; roller++) {
        m_rollers[roller]->SaveConstraintViolations(ss);
    }

    GetLog().SetNumFormat("%g");
}

// write output

// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
void TrackSoilBin::Setup_log_to_file(int what, const std::string& filename) {
    m_save_log_to_file = false;
    m_log_file_name = filename;
    m_log_what_to_file = what;
    // has the chain been created?
    if (m_chain) {
        // have the chain, the last subsystem created, been initialized?
        if (m_chain->Get_numShoes()) {
            m_save_log_to_file = true;
            GetLog() << " SAVING OUTPUT DATA TO FILE: \n " << filename.c_str() << "\n";
            create_fileHeader(what);
            m_log_file_exists = true;
        } else {
            GetLog() << " no shoes were initialized, not saving data ";
        }
    } else {
        GetLog() << " chain subsystem not created yet, not saving data";
    }

    // initialize the rig input values to zero
}

void TrackSoilBin::Log_to_console(int console_what) {
    GetLog().SetNumFormat("%10.2f");

    if (console_what & DBG_FIRSTSHOE) {
        GetLog() << "\n-- shoe 0 : " << m_chain->GetShoeBody(0)->GetName() << "\n";
        // COG state data

        GetLog() << "COG Pos [m] : " << m_chain->GetShoeBody(0)->GetPos() << "\n";
        GetLog() << "COG Vel [m/s] : " << m_chain->GetShoeBody(0)->GetPos_dt() << "\n";
        GetLog() << "COG Acc [m/s2] : " << m_chain->GetShoeBody(0)->GetPos_dtdt() << "\n";
        GetLog() << "COG omega [rad/s] : " << m_chain->GetShoeBody(0)->GetRot_dt() << "\n";

        // shoe pin tension
        GetLog() << "pin 0 reaction force [N] : " << m_chain->GetPinReactForce(0) << "\n";
        // GetLog() << "pin 0 reaction torque [N-m] : "  <<  m_chain->GetPinReactTorque(0) << "\n";
    }

    if (console_what & DBG_GEAR) {
        GetLog() << "\n---- Gear " << m_gear->GetBody()->GetName() << "\n";
        // COG state data
        GetLog() << "COG Pos [m] : " << m_gear->GetBody()->GetPos() << "\n";
        GetLog() << "COG Vel [m/s] : " << m_gear->GetBody()->GetPos_dt() << "\n";
        GetLog() << "COG omega [rad/s] : " << m_gear->GetBody()->GetRot_dt() << "\n";

        /*
        // # of shoe pins in contact?
        GetLog() << "# of shoes in contact ? : " << m_gear->GetBody()->GetCollisionModel()->Get << "\n";


        // # of non-intermittant contact steps
        GetLog() << "cumulative contact steps : " <<  << "\n";
        */
    }

    if (console_what & DBG_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations(false);
    }

    if (console_what & DBG_PTRAIN) {
        GetLog() << "\n---- powertrain \n";
    }

    GetLog().SetNumFormat("%g");
}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void TrackSoilBin::Log_to_file() {
    // told to save the data?
    if (m_save_log_to_file) {
        if (!m_log_file_exists) {
            std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
        }
        // open the file to append
        // open the data file for writing the header
        ChStreamOutAsciiFile ofile(m_log_file_name.c_str(), std::ios::app);

        // write the simulation time
        std::stringstream ss;
        ss << m_system->GetChTime();

        // python pandas expects csv w/ no whitespace
        if (m_log_what_to_file & DBG_FIRSTSHOE) {
            ss << "," << m_chain->GetShoeBody(0)->GetPos() << "," << m_chain->GetShoeBody(0)->GetPos_dt() << ","
               << m_chain->GetShoeBody(0)->GetPos_dtdt() << ","
               << m_chain->GetShoeBody(0)->GetRot_dt().Q_to_NasaAngles() << "," << m_chain->GetPinReactForce(0);
            // << "," <<  m_chain->GetPinReactTorque(0);
        }
        if (m_log_what_to_file & DBG_GEAR) {
            ss << "," << m_gear->GetBody()->GetPos() << "," << m_gear->GetBody()->GetPos_dt() << ","
               << m_gear->GetBody()->GetRot_dt().Q_to_NasaAngles();
        }

        if (m_log_what_to_file & DBG_CONSTRAINTS) {
            // Report constraint violations for all joints
            SaveConstraintViolations(ss);
        }

        if (m_log_what_to_file & DBG_PTRAIN) {
            // motor speed, mot torque, out torque
            ss << "," << m_ptrain->GetMotorSpeed() << "," << m_ptrain->GetMotorTorque() << ","
               << m_ptrain->GetOutputTorque();
        }
        // next line last, then write to file
        ss << "\n";
        ofile << ss.str().c_str();
    }
}

void TrackSoilBin::create_fileHeader(int what) {
    // open the data file for writing the header
    ChStreamOutAsciiFile ofile(m_log_file_name.c_str());
    // write the headers, output types specified by "what"
    std::stringstream ss;
    ss << "time";
    if (what & DBG_FIRSTSHOE) {
        // Shoe 0 : S0, Pin0: P0
        ss << ",S0x,S0y,S0z,S0vx,S0vy,S0vz,S0ax,S0ay,S0az,S0wx,S0wy,S0wz,P0fx,P0fy,P0fz";
    }
    if (what & DBG_GEAR) {
        ss << ",Gx,Gy,Gz,Gvx,Gvy,Gvz,Gwx,Gwy,Gwz";
    }
    if (what & DBG_CONSTRAINTS) {
        // in the same order as listed in the header
        ss << m_gear->getFileHeader_ConstraintViolations(0);

        for (int id = 0; id < m_num_idlers; id++) {
            ss << m_idlers[id]->getFileHeader_ConstraintViolations(id);
        }

        // violations of the roller revolute joints
        for (int roller = 0; roller < m_num_rollers; roller++) {
            ss << m_rollers[roller]->getFileHeader_ConstraintViolations(roller);
        }
    }
    if (what & DBG_PTRAIN) {
        ss << ",motSpeed,motT,outT";
    }

    // write to file, go to next line in file in prep. for next step.
    ofile << ss.str().c_str();
    ofile << "\n";
}

}  // end namespace chrono
