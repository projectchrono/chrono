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
// model a single track chain system, as part of a tracked vehicle.
// Sprocket drive gear body driven with a motion, i.e. no powertrain.
//
// =============================================================================

#include <cstdio>
#include <sstream>

#include "subsys/trackSystem/TrackSystemM113.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables

// idler, right side
const ChVector<> TrackSystemM113::m_idlerPos(-2.1904, -0.1443, 0.2447);  // relative to local csys
const ChQuaternion<> TrackSystemM113::m_idlerRot(QUNIT);

// drive gear, right side
const ChVector<> TrackSystemM113::m_gearPos(1.7741, -0.0099, 0.2447);  // relative to local csys
const ChQuaternion<> TrackSystemM113::m_gearRot(QUNIT);

// suspension
const int TrackSystemM113::m_numSuspensions = 5;

TrackSystemM113::TrackSystemM113(const std::string& name,
                                 const size_t track_idx,
                                 const double tensioner_preload,
                                 const double omega_max)
    : m_track_idx(track_idx), m_name(name), m_idler_preload(tensioner_preload) {
    // FILE* fp = fopen(filename.c_str(), "r");
    // char readBuffer[65536];
    // fclose(fp);

    Create(track_idx, omega_max);
}

// Create: 1) load/set the subsystem data, resize vectors 2) BuildSubsystems()
// TODO: replace hard-coded junk with JSON input files for each subsystem
void TrackSystemM113::Create(const size_t track_idx, const double omega_max) {
    /*

    // read idler info
    assert(d.HasMember("Idler"));
    m_idlerMass = d["Idler"]["Mass"].GetDouble();
    m_idlerPos = loadVector(d["Idler"]["Location"]);
    m_idlerInertia = loadVector(d["Idler"]["Inertia"]);
    m_idlerRadius = d["Spindle"]["Radius"].GetDouble();
    m_idlerWidth = d["Spindle"]["Width"].GetDouble();
    m_idler_K = d["Idler"]["SpringK"].GetDouble();
    m_idler_C = d["Idler"]["SpringC"].GetDouble();

    // Read Drive Gear data
    assert(d.HasMember("Drive Gear"));
    assert(d["Drive Gear"].IsObject());

    m_gearMass = d["Drive Gear"]["Mass"].GetDouble();
    m_gearPos = loadVector(d["Drive Gear"]["Location"]);
    m_gearInertia = loadVector(d["Drive Gear"]["Inertia"]);
    m_gearRadius = d["Drive Gear"]["Radius"].GetDouble();
    m_gearWidth = d["Drive Gear"]["Width"].GetDouble();

    // Read Suspension data
    assert(d.HasMember("Suspension"));
    assert(d["Suspension"].IsObject());
    assert(d["Suspension"]["Location"].IsArray() );

    m_suspensionFilename = d["Suspension"]["Input File"].GetString();
    m_NumSuspensions = d["Suspension"]["Location"].Size();

   */

    m_suspensions.resize(m_numSuspensions);
    m_suspensionLocs.resize(m_numSuspensions);
    // hard-code positions relative to TrackSystemM113 csys. Start w/ one nearest sprocket

    m_suspensionLocs[0] = ChVector<>(1.3336, 0, 0);
    m_suspensionLocs[1] = ChVector<>(0.6668, 0, 0);
    // TrackSystemM113 c-sys aligned with middle suspension subsystem arm/chassis revolute constraint position
    m_suspensionLocs[2] = ChVector<>(0, 0, 0);
    m_suspensionLocs[3] = ChVector<>(-0.6682, 0, 0);
    m_suspensionLocs[4] = ChVector<>(-1.3368, 0, 0);

    /*

    for(int j = 0; j < m_numSuspensions; j++)
    {
      m_suspensionLocs[j] = loadVector(d["Suspension"]["Locaiton"][j]);
    }

    // Read Track Chain data
    assert(d.HasMember("Track Chain"));
    assert(d["Track Chain"].IsObject());
    m_trackChainFilename = d["Track Chain"]["Input File"].GetString()

    */

    // create the various subsystems, from the hardcoded static variables in each subsystem class
    BuildSubsystems(omega_max);
}

void TrackSystemM113::BuildSubsystems(const double omega_max) {
    std::stringstream gearName;
    gearName << "drive gear " << m_track_idx;
    // build one of each of the following subsystems. VisualizationType and CollisionType defaults are PRIMITIVES
    m_driveGear = ChSharedPtr<DriveGearMotion>(new DriveGearMotion(gearName.str(), VisualizationType::Mesh,
                                                                   // VisualizationType::Primitives,
                                                                   //  CollisionType::Primitives,
                                                                   CollisionType::CallbackFunction, m_track_idx, 436.7,
                                                                   ChVector<>(12.22, 12.22, 13.87), 
                                                                   0.2, // mu
                                                                   omega_max));

    std::stringstream idlerName;
    idlerName << "idler " << m_track_idx;
    m_idler = ChSharedPtr<IdlerSimple>(new IdlerSimple(idlerName.str(), VisualizationType::Mesh,
                                                       CollisionType::Primitives, m_track_idx,
                                                       429.6,                           // idler mass
                                                       ChVector<>(12.55, 12.55, 14.7),  // idler Ixx
                                                       1e6, 1.5e4, 1.0,   // tensioner spring K, C
                                                       0.1));       // mu

    std::stringstream chainname;
    chainname << "chain " << m_track_idx;
    m_chain = ChSharedPtr<TrackChain>(new TrackChain(chainname.str(),
                                                     // VisualizationType::Primitives,
                                                     VisualizationType::CompoundPrimitives, CollisionType::Primitives,
                                                     m_track_idx,
                                                     18.02,                         // shoe mass
                                                     ChVector<>(0.22, 0.25, 0.04),  // shoe Ixx
                                                     0.1));                         // mu
    // CollisionType::CompoundPrimitives) );

    // build suspension/road wheel subsystems
    for (int i = 0; i < m_numSuspensions; i++) {
        std::stringstream susp_name;
        susp_name << "suspension " << i << ", chain " << m_track_idx;
        m_suspensions[i] = ChSharedPtr<TorsionArmSuspension>(
            new TorsionArmSuspension(susp_name.str(), VisualizationType::Mesh, CollisionType::Primitives, m_track_idx,
                                     561.1,                            // wheel mass
                                     ChVector<>(19.82, 19.82, 26.06),  // wheel Ixx
                                     75.26,                            // arm mass
                                     ChVector<>(0.77, 0.37, 0.77),     // arm Ixx
                                     1e6, 1.5e4, 0,                    //  5e3, 1e2, 3e2));   // K, C, preload
                                     0.1));                            // mu
    }
}

void TrackSystemM113::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                                 const ChVector<>& local_pos,
                                 ChTrackVehicle* vehicle,
                                 double pin_damping) {
    m_local_pos = local_pos;
    m_gearPosRel = m_gearPos;
    m_idlerPosRel = m_idlerPos;

    // if we're on the left side of the vehicle, switch lateral z-axis on all relative positions
    if (m_local_pos.z < 0) {
        m_gearPosRel.z *= -1;
        m_idlerPosRel.z *= -1;
    }

    // Create list of the center location of the rolling elements and their clearance.
    // Clearance is a sphere shaped envelope at each center location, where it can
    //  be guaranteed that the track chain geometry will not penetrate the sphere.
    std::vector<ChVector<> > rolling_elem_locs;       // w.r.t. chassis ref. frame
    std::vector<double> clearance;                    // 1 per rolling elem
    std::vector<ChVector<> > rolling_elem_spin_axis;  /// w.r.t. abs. frame

    // initialize 1 of each of the following subsystems.
    // will use the chassis ref frame to do the transforms, since the TrackSystemM113
    // local ref. frame has same rot (just difference in position)
    // NOTE: move drive Gear Init() AFTER the chain of shoes is created, since
    //        need the list of shoes to be passed in to create custom collision w/ gear
    // HOWEVER, still add the info to the rolling element lists passed into TrackChain Init().

    // drive sprocket is First added to the lists passed into TrackChain Init()
    rolling_elem_locs.push_back(m_local_pos + Get_GearPosRel());
    clearance.push_back(m_driveGear->GetRadius());
    rolling_elem_spin_axis.push_back(m_driveGear->GetBody()->GetRot().GetZaxis());

    // initialize the torsion arm suspension subsystems
    for (int s_idx = 0; s_idx < m_suspensionLocs.size(); s_idx++) {
        m_suspensions[s_idx]->Initialize(chassis, chassis->GetFrame_REF_to_abs(),
                                         ChCoordsys<>(m_local_pos + m_suspensionLocs[s_idx], QUNIT));

        // add to the lists passed into the track chain, find location of each wheel center w.r.t. chassis coords.
        rolling_elem_locs.push_back(m_local_pos + m_suspensionLocs[s_idx] + m_suspensions[s_idx]->GetWheelPosRel());
        clearance.push_back(m_suspensions[s_idx]->GetWheelRadius());
        rolling_elem_spin_axis.push_back(m_suspensions[s_idx]->GetWheelBody()->GetRot().GetZaxis());
    }

    // last control point: the idler body
    m_idler->Initialize(chassis, chassis->GetFrame_REF_to_abs(),
                        ChCoordsys<>(m_local_pos + Get_IdlerPosRel(), Q_from_AngAxis(CH_C_PI, VECT_Z)),
                        m_idler_preload);

    // add to the lists passed into the track chain Init()
    rolling_elem_locs.push_back(m_local_pos + Get_IdlerPosRel());
    clearance.push_back(m_idler->GetRadius());
    rolling_elem_spin_axis.push_back(m_idler->GetBody()->GetRot().GetZaxis());

    // After all rolling elements have been initialized, now able to setup the TrackChain.
    // Assumed that start_pos is between idler and gear control points, e.g., on the top
    //   of the track chain.
    ChVector<> start_pos = (rolling_elem_locs.front() + rolling_elem_locs.back()) / 2.0;
    start_pos.y += (clearance.front() + clearance.back()) / 2.0;

    // Assumption: start_pos should lie close to where the actual track chain would
    //             pass between the idler and driveGears.
    // MUST be on the top part of the chain so the chain wrap rotation direction can be assumed.
    m_chain->Initialize(chassis, chassis->GetFrame_REF_to_abs(), rolling_elem_locs, clearance, rolling_elem_spin_axis,
                        start_pos);

    // add some initial damping to the inter-shoe pin joints
    if (pin_damping > 0)
        m_chain->Set_pin_friction(pin_damping);

    // chain of shoes available for gear init
    m_driveGear->Initialize(chassis, chassis->GetFrame_REF_to_abs(),
                            ChCoordsys<>(m_local_pos + Get_GearPosRel(), QUNIT), m_chain->GetShoeBody(), vehicle);
}

void TrackSystemM113::Update(double time, double throttle) {
    // gear will apply a rot. vel. according to a set max omega
    m_driveGear->Update(time, throttle);
}

const ChVector<> TrackSystemM113::Get_Idler_spring_react() {
    return m_idler->m_shock->Get_react_force();
}

void TrackSystemM113::Write_subsys_headers(int what_subsys, int debug_type, const std::string& filename) {
    m_log_subsys = what_subsys;
    m_debug_type = debug_type;
    // write headers for each debug type
    if (m_debug_type & DBG_BODY) {
        // TrackChain subsystem, body info is a single shoe in the chain.
        if (m_log_subsys & DBG_FIRSTSHOE) {
            // Shoe 0 : S0, Pin0: P0
            std::stringstream ss_fs;
            ss_fs << filename << "_Side" << m_track_idx << "_shoe0.csv";
            GetTrackChain()->Write_header(ss_fs.str(), DBG_BODY);
        }
        if (m_log_subsys & DBG_GEAR) {
            // filename
            std::stringstream ss_gf;
            ss_gf << filename << "_Side" << m_track_idx << "_gear.csv";
            GetDriveGear()->Write_header(ss_gf.str(), DBG_BODY);
        }
        if (m_log_subsys & DBG_IDLER) {
            // idler filename
            std::stringstream ss_i;
            ss_i << filename << "_Side" << m_track_idx << "_idler.csv";
            GetIdler()->Write_header(ss_i.str(), DBG_BODY);
        }
        if (m_log_subsys & DBG_SUSPENSION) {
            // wheel body info
            for (int wheel = 0; wheel < m_numSuspensions; wheel++) {
                std::stringstream ss;
                ss << filename << "_Side" << m_track_idx << "_wheel" << wheel << ".csv";
                GetSuspension(wheel)->Write_header(ss.str(), DBG_BODY);
            }
        }
    }

    // write the data for each subsystem's constraint violation
    if (m_debug_type & DBG_CONSTRAINTS) {
        if (m_log_subsys & DBG_FIRSTSHOE) {
            // filename, first shoe CV w/ to adjoining shoes
            std::stringstream ss_shoeCV;
            ss_shoeCV << filename << "_side" << m_track_idx << "_shoe0CV.csv";
            GetTrackChain()->Write_header(ss_shoeCV.str(), DBG_CONSTRAINTS);
        }
        if (m_log_subsys & DBG_GEAR) {
            // filename, gear CV
            std::stringstream ss_gcv;
            ss_gcv << filename << "_side" << m_track_idx << "_GearCV.csv";
            GetDriveGear()->Write_header(ss_gcv.str(), DBG_CONSTRAINTS);
        }
        if (m_log_subsys & DBG_IDLER) {
            // filename, idler CV
            std::stringstream ss_iCV;
            ss_iCV << filename << "_Side" << m_track_idx << "_idlerCV.csv";
            GetIdler()->Write_header(ss_iCV.str(), DBG_CONSTRAINTS);
        }
        if (m_log_subsys & DBG_SUSPENSION) {
            // violations of the roller revolute joints on torsion arm suspension
            for (int roller = 0; roller < Get_NumWheels(); roller++) {
                // filename, roller CV
                std::stringstream ss_rCV;
                ss_rCV << filename << "_Side" << m_track_idx << "_roller" << roller << "CV.csv";
                GetSuspension(roller)->Write_header(ss_rCV.str(), DBG_CONSTRAINTS);
            }
        }
    }

    if (m_debug_type & DBG_CONTACTS) {
        if (m_log_subsys & DBG_FIRSTSHOE) {
            // report the contact with the gear in a second file
            std::stringstream ss;
            ss << filename << "_Side" << m_track_idx << "_shoe0GearContact.csv";
            GetTrackChain()->Write_header(ss.str(), DBG_CONTACTS);
        }
        if (m_log_subsys & DBG_GEAR) {
            // report on some specific collision info
            std::stringstream ss;
            ss << filename << "_Side" << m_track_idx << "_gearContact.csv";
            GetDriveGear()->Write_header(ss.str(), DBG_CONTACTS);
        }
        if (m_log_subsys & DBG_IDLER) {
            // todo, idler contact info
        }
        if (m_log_subsys & DBG_SUSPENSION) {
            // todo, suspension wheel contact info
        }
    }
}

void TrackSystemM113::Write_subsys_data(const double t, const ChSharedPtr<ChBody> chassis) {
    // write headers for each debug type
    if (m_debug_type & DBG_BODY) {
        if (m_log_subsys & DBG_FIRSTSHOE) {
            GetTrackChain()->Write_data(t, chassis, DBG_BODY);
        }
        if (m_log_subsys & DBG_GEAR) {
            // write to body data file,
            // second file, for the specific contact info
            GetDriveGear()->Write_data(t, chassis->GetSystem(), DBG_BODY);
        }
        if (m_log_subsys & DBG_IDLER) {
            // write body and tensioner data
            GetIdler()->Write_data(t, DBG_BODY);
        }
        if (m_log_subsys & DBG_SUSPENSION)
            for (int wheel = 0; wheel < m_numSuspensions; wheel++) {
                GetSuspension(wheel)->Write_data(t, DBG_BODY);
            }
    }
    if (m_debug_type & DBG_CONSTRAINTS) {
        // constraint violation for specified subsystems
        if (m_log_subsys & DBG_FIRSTSHOE) {
            // write constraint violation between this and adjacent shoes
            GetTrackChain()->Write_data(t, chassis, DBG_CONSTRAINTS);
        }
        if (m_log_subsys & DBG_GEAR) {
            // write constraint violation for gear revolute constraint
            GetDriveGear()->Write_data(t, chassis->GetSystem(), DBG_CONSTRAINTS);
        }
        if (m_log_subsys & DBG_IDLER) {
            // write CV for idler constraint (2 DOFS, 4 constrained vars)
            GetIdler()->Write_data(t, DBG_CONSTRAINTS);
        }
        if (m_log_subsys & DBG_SUSPENSION) {
            // write CV for wheel on each suspension
            for (int wheel = 0; wheel < m_numSuspensions; wheel++) {
                GetSuspension(wheel)->Write_data(t, DBG_CONSTRAINTS);
            }
        }
    }

    if (m_debug_type & DBG_CONTACTS) {
        // todo
    }
}

}  // end namespace chrono
