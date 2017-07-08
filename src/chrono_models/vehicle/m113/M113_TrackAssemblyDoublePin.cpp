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
M113_TrackAssemblyDoublePin::M113_TrackAssemblyDoublePin(VehicleSide side) : ChTrackAssemblyDoublePin("", side) {
    m_suspensions.resize(5);
    m_suspensions[0] = std::make_shared<M113_Suspension>(side, true);
    m_suspensions[1] = std::make_shared<M113_Suspension>(side, true);
    m_suspensions[2] = std::make_shared<M113_Suspension>(side, false);
    m_suspensions[3] = std::make_shared<M113_Suspension>(side, false);
    m_suspensions[4] = std::make_shared<M113_Suspension>(side, true);

    switch (side) {
        case LEFT:
            SetName("M113_TrackAssemblyLeft");
            m_idler = std::make_shared<M113_IdlerLeft>();
            m_brake = std::make_shared<M113_BrakeSimple>();
            break;
        case RIGHT:
            SetName("M113_TrackAssemblyRight");
            m_idler = std::make_shared<M113_IdlerRight>();
            m_brake = std::make_shared<M113_BrakeSimple>();
            break;
    }

    size_t num_shoes;
    switch (side) {
        case LEFT:
            m_sprocket = std::make_shared<M113_SprocketDoublePinLeft>();
            num_shoes = 63;
            break;
        case RIGHT:
            m_sprocket = std::make_shared<M113_SprocketDoublePinRight>();
            num_shoes = 64;
            break;
    }

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(std::make_shared<M113_TrackShoeDoublePin>());
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
