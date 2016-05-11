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
// Tracked vehicle model built from subsystems.
//  Location of subsystems hard-coded for M113 vehicle
//  TODO: specify this w/ JSON input data file
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "physics/ChGlobal.h"

#include "utils/ChUtilsInputOutput.h"

#include "TrackVehicle.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/trackSystem/TrackSystem.h"
// #include "subsys/driveline/TrackDriveline.h"


namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const ChCoordsys<> TrackVehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicle::TrackVehicle(const std::string& name,
                           VisualizationType::Enum vis,
                           CollisionType::Enum collide,
                           double pin_damping_coef,
                           double tensioner_preload,
                           double mass,
                           const ChVector<>& Ixx,
                           const ChVector<>& COG_to_REF,
                           const ChVector<>& left_pos_rel,
                           const ChVector<>& right_pos_rel)
    : ChTrackVehicle(name, vis, collide, mass, Ixx, 1),
      m_num_tracks(2),
      m_pin_damping(pin_damping_coef),
      m_tensioner_preload(tensioner_preload) {
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

    // two drive Gears, like a 2WD driven vehicle.
    // m_drivelines.resize(m_num_engines);
    // m_ptrains.resize(m_num_engines); // done by base vehicle class

    // create track systems
    for (int i = 0; i < m_num_tracks; i++) {
        std::stringstream t_ss;
        t_ss << "track chain " << i;
        m_TrackSystems.push_back(ChSharedPtr<TrackSystem>(new TrackSystem(t_ss.str(), i)) );
    }

    // create the powertrain and drivelines
    for (int j = 0; j < m_num_engines; j++) {
        std::stringstream dl_ss;
        dl_ss << "driveline " << j;
        std::stringstream pt_ss;
        pt_ss << "powertrain " << j;
        // m_drivelines[j] = ChSharedPtr<TrackDriveline>(new TrackDriveline(dl_ss.str() ) );
        m_ptrains.push_back( ChSharedPtr<TrackPowertrain>(new TrackPowertrain(pt_ss.str())) );
    }

    // TODO: add brakes. Perhaps they are a part of the suspension subsystem?
}

TrackVehicle::~TrackVehicle() {
    if (m_ownsSystem)
        delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void TrackVehicle::Initialize(const ChCoordsys<>& chassis_Csys) {
    // move the chassis REF frame to the specified initial position/orientation
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassis_Csys));

    // add collision geometry to the chassis
    AddCollisionGeometry();

    // initialize the subsystems with the initial c-sys and specified offsets
    for (int i = 0; i < m_num_tracks; i++) {
        m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i], dynamic_cast<ChTrackVehicle*>(this),
                                      m_pin_damping);
    }

    // initialize the powertrain, drivelines
    for (int j = 0; j < m_num_engines; j++) {

        m_ptrains[j]->Initialize(m_chassis, m_TrackSystems[j]->GetDriveGear()->GetAxle());
    }
}

void TrackVehicle::Update(double time, const std::vector<double>& throttle, const std::vector<double>& braking) {
    assert(throttle.size() >= m_num_tracks);
    assert(braking.size() >= m_num_tracks);
    // update powertrains
    for (int i = 0; i < m_num_engines; i++) {
        m_ptrains[i]->Update(time, throttle[0], GetDriveshaftSpeed(i));
    }
}

void TrackVehicle::Advance(double step) {
    double t = 0;
    double settlePhaseA = 0.05;
    double settlePhaseB = 0.1;
    m_system->SetMaxItersSolverStab(100);
    m_system->SetMaxItersSolverSpeed(150);
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        if (m_system->GetChTime() < settlePhaseA) {
            m_system->SetMaxItersSolverStab(80);
            m_system->SetMaxItersSolverSpeed(100);
        } else if (m_system->GetChTime() < settlePhaseB) {
            m_system->SetMaxItersSolverStab(100);
            m_system->SetMaxItersSolverSpeed(150);
        }
        m_system->DoStepDynamics(h);
        t += h;
    }
}

// call the chain function to update the constant damping coef.
void TrackVehicle::SetShoePinDamping(double damping) {
    m_pin_damping = damping;
    for (int i = 0; i < m_num_tracks; i++) {
        m_TrackSystems[i]->m_chain->Set_pin_friction(damping);
    }
}

double TrackVehicle::GetDriveshaftSpeed(size_t idx) const {
    assert(idx < m_num_tracks);
    return m_TrackSystems[idx]->GetDriveGear()->GetAxle()->GetPos_dt();
}

ChSharedPtr<TrackPowertrain> TrackVehicle::GetPowertrain(size_t idx) const {
    assert(idx < m_num_engines);
    return m_ptrains[idx];
}

}  // end namespace chrono
