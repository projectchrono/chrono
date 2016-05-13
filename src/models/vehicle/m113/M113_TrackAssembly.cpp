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
// Authors: Radu Serban
// =============================================================================
//
// M113 track assembly subsystem.
//
// =============================================================================

#include "models/vehicle/m113/M113_Sprocket.h"
#include "models/vehicle/m113/M113_Idler.h"
#include "models/vehicle/m113/M113_BrakeSimple.h"
#include "models/vehicle/m113/M113_RoadWheel.h"
#include "models/vehicle/m113/M113_Suspension.h"
#include "models/vehicle/m113/M113_TrackShoe.h"
#include "models/vehicle/m113/M113_TrackAssembly.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackAssembly::M113_TrackAssembly(VehicleSide side)
    : ChTrackAssembly("", side) {
    size_t num_shoes;
    if (side == LEFT) {
        SetName("M113_TrackAssemblyLeft");
        m_sprocket = std::make_shared<M113_SprocketLeft>();
        m_idler = std::make_shared<M113_IdlerLeft>();
        m_brake = std::make_shared<M113_BrakeSimple>();
        num_shoes = 63;
    } else {
        SetName("M113_TrackAssemblyRight");
        m_sprocket = std::make_shared<M113_SprocketRight>();
        m_idler = std::make_shared<M113_IdlerRight>();
        m_brake = std::make_shared<M113_BrakeSimple>();
        num_shoes = 64;
    }

    m_suspensions.resize(5);
    m_suspensions[0] = std::make_shared<M113_Suspension>(side, true);
    m_suspensions[1] = std::make_shared<M113_Suspension>(side, true);
    m_suspensions[2] = std::make_shared<M113_Suspension>(side, false);
    m_suspensions[3] = std::make_shared<M113_Suspension>(side, false);
    m_suspensions[4] = std::make_shared<M113_Suspension>(side, true);

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(std::make_shared<M113_TrackShoe>());
    }
}

}  // end namespace m113
