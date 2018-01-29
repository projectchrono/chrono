// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// M113 track assembly subsystem.
//
// =============================================================================

#include "chrono_models/vehicle/m113a/M113a_BrakeSimple.h"
#include "chrono_models/vehicle/m113a/M113a_Idler.h"
#include "chrono_models/vehicle/m113a/M113a_RoadWheel.h"
#include "chrono_models/vehicle/m113a/M113a_SprocketSinglePin.h"
#include "chrono_models/vehicle/m113a/M113a_Suspension.h"
#include "chrono_models/vehicle/m113a/M113a_TrackAssemblySinglePin.h"
#include "chrono_models/vehicle/m113a/M113a_TrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const ChVector<> M113a_TrackAssemblySinglePin::m_sprocket_loc(0, 0, 0);
const ChVector<> M113a_TrackAssemblySinglePin::m_idler_loc_L(-3.7613, 0, -0.151);
const ChVector<> M113a_TrackAssemblySinglePin::m_idler_loc_R(-3.8395, 0, -0.151);
const ChVector<> M113a_TrackAssemblySinglePin::m_susp_locs_L[5] = {
    ChVector<>(-0.629, 0, -0.304),
    ChVector<>(-1.296, 0, -0.304),
    ChVector<>(-1.963, 0, -0.304),
    ChVector<>(-2.629, 0, -0.304),
    ChVector<>(-3.296, 0, -0.304)
};
const ChVector<> M113a_TrackAssemblySinglePin::m_susp_locs_R[5] = {
    ChVector<>(-0.731, 0, -0.304),
    ChVector<>(-1.397, 0, -0.304),
    ChVector<>(-2.064, 0, -0.304),
    ChVector<>(-2.731, 0, -0.304),
    ChVector<>(-3.398, 0, -0.304)
};

// -----------------------------------------------------------------------------
// Constructor for M113 track assembly class.
// Create the idler, brake, suspension, and sprocket subsystems.
// Create the track shoes.
// -----------------------------------------------------------------------------
M113a_TrackAssemblySinglePin::M113a_TrackAssemblySinglePin(VehicleSide side) : ChTrackAssemblySinglePin("", side) {
    size_t num_shoes;
    std::string suspName("M113_Suspension");
    std::string shoeName("M113_TrackShoe");
    switch (side) {
        case LEFT:
            SetName("M113a_TrackAssemblyLeft");
            m_idler = std::make_shared<M113a_IdlerLeft>();
            m_brake = std::make_shared<M113a_BrakeSimple>("M113_BrakeLeft");
            m_sprocket = std::make_shared<M113a_SprocketSinglePinLeft>();
            num_shoes = 63;
            suspName += "Left_";
            shoeName += "Left_";
            break;
        case RIGHT:
            SetName("M113a_TrackAssemblyRight");
            m_idler = std::make_shared<M113a_IdlerRight>();
            m_brake = std::make_shared<M113a_BrakeSimple>("M113_BrakeRight");
            m_sprocket = std::make_shared<M113a_SprocketSinglePinRight>();
            num_shoes = 64;
            suspName += "Right_";
            shoeName += "Right_";
            break;
    }

    m_suspensions.resize(5);
    m_suspensions[0] = std::make_shared<M113a_Suspension>(suspName + "0", side, 0, true);
    m_suspensions[1] = std::make_shared<M113a_Suspension>(suspName + "1", side, 1, false);
    m_suspensions[2] = std::make_shared<M113a_Suspension>(suspName + "2", side, 2, false);
    m_suspensions[3] = std::make_shared<M113a_Suspension>(suspName + "3", side, 3, false);
    m_suspensions[4] = std::make_shared<M113a_Suspension>(suspName + "4", side, 4, true);

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(std::make_shared<M113a_TrackShoeSinglePin>(shoeName + std::to_string(it)));
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<> M113a_TrackAssemblySinglePin::GetSprocketLocation() const {
    return m_sprocket_loc;
}

const ChVector<> M113a_TrackAssemblySinglePin::GetIdlerLocation() const {
    return (m_side == LEFT) ? m_idler_loc_L : m_idler_loc_R;
}

const ChVector<> M113a_TrackAssemblySinglePin::GetRoadWhelAssemblyLocation(int which) const {
    return (m_side == LEFT) ? m_susp_locs_L[which] : m_susp_locs_R[which];
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
