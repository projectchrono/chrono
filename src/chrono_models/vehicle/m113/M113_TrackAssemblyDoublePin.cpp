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
// M113 double-pin track assembly subsystem.
//
// =============================================================================

#include "chrono_models/vehicle/m113/M113_BrakeSimple.h"
#include "chrono_models/vehicle/m113/M113_BrakeShafts.h"
#include "chrono_models/vehicle/m113/M113_Idler.h"
#include "chrono_models/vehicle/m113/M113_RoadWheel.h"
#include "chrono_models/vehicle/m113/M113_SprocketDoublePin.h"
#include "chrono_models/vehicle/m113/M113_Suspension.h"
#include "chrono_models/vehicle/m113/M113_TrackAssemblyDoublePin.h"
#include "chrono_models/vehicle/m113/M113_TrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const ChVector<> M113_TrackAssemblyDoublePin::m_sprocket_loc(0, 0, 0);
const ChVector<> M113_TrackAssemblyDoublePin::m_idler_loc(-3.83, 0, -0.12);
const ChVector<> M113_TrackAssemblyDoublePin::m_susp_locs_L[5] = {
    ChVector<>(-0.655, 0, -0.215),
    ChVector<>(-1.322, 0, -0.215),
    ChVector<>(-1.989, 0, -0.215),
    ChVector<>(-2.656, 0, -0.215),
    ChVector<>(-3.322, 0, -0.215)
};
const ChVector<> M113_TrackAssemblyDoublePin::m_susp_locs_R[5] = {
    ChVector<>(-0.740, 0, -0.215),
    ChVector<>(-1.407, 0, -0.215),
    ChVector<>(-2.074, 0, -0.215),
    ChVector<>(-2.740, 0, -0.215),
    ChVector<>(-3.407, 0, -0.215)
};

// -----------------------------------------------------------------------------
// Constructor for the M113 track assembly using double-pin track shoes.
// Create the suspensions, idler, brake, sprocket, and track shoes.
// -----------------------------------------------------------------------------
M113_TrackAssemblyDoublePin::M113_TrackAssemblyDoublePin(VehicleSide side, BrakeType brake_type)
    : ChTrackAssemblyDoublePin("", side) {
    size_t num_shoes;
    std::string suspName("M113_Suspension");
    std::string shoeName("M113_TrackShoe");
    switch (side) {
        case LEFT:
            SetName("M113_TrackAssemblyLeft");
            m_idler = chrono_types::make_shared<M113_IdlerLeft>();
            m_brake = chrono_types::make_shared<M113_BrakeSimple>("M113_BrakeLeft");
            m_sprocket = chrono_types::make_shared<M113_SprocketDoublePinLeft>();
            num_shoes = 63;
            suspName += "Left_";
            shoeName += "Left_";
            break;
        case RIGHT:
            SetName("M113_TrackAssemblyRight");
            m_idler = chrono_types::make_shared<M113_IdlerRight>();
            m_brake = chrono_types::make_shared<M113_BrakeSimple>("M113_BrakeRight");
            m_sprocket = chrono_types::make_shared<M113_SprocketDoublePinRight>();
            num_shoes = 64;
            suspName += "Right_";
            shoeName += "Right_";
            break;
    }

    m_suspensions.resize(5);
    m_suspensions[0] = chrono_types::make_shared<M113_Suspension>(suspName + "0", side, 0, true);
    m_suspensions[1] = chrono_types::make_shared<M113_Suspension>(suspName + "1", side, 1, true);
    m_suspensions[2] = chrono_types::make_shared<M113_Suspension>(suspName + "2", side, 2, false);
    m_suspensions[3] = chrono_types::make_shared<M113_Suspension>(suspName + "3", side, 3, false);
    m_suspensions[4] = chrono_types::make_shared<M113_Suspension>(suspName + "4", side, 4, true);

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(chrono_types::make_shared<M113_TrackShoeDoublePin>(shoeName + std::to_string(it)));
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<> M113_TrackAssemblyDoublePin::GetSprocketLocation() const {
    return m_sprocket_loc;
}

const ChVector<> M113_TrackAssemblyDoublePin::GetIdlerLocation() const {
    return m_idler_loc;
}

const ChVector<> M113_TrackAssemblyDoublePin::GetRoadWhelAssemblyLocation(int which) const {
    return (m_side == LEFT) ? m_susp_locs_L[which] : m_susp_locs_R[which];
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
