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

#include "m113/M113_Sprocket.h"
#include "m113/M113_Idler.h"
#include "m113/M113_BrakeSimple.h"
#include "m113/M113_RoadWheel.h"
#include "m113/M113_Suspension.h"
#include "m113/M113_TrackShoe.h"
#include "m113/M113_TrackAssembly.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackAssembly::M113_TrackAssembly(VehicleSide side, VisualizationType vis_type)
    : ChTrackAssembly("") {
    size_t num_shoes;
    if (side == LEFT) {
        SetName("M113_TrackAssemblyLeft");
        m_sprocket = ChSharedPtr<M113_Sprocket>(new M113_SprocketLeft(vis_type));
        m_idler = ChSharedPtr<M113_Idler>(new M113_IdlerLeft(vis_type));
        m_brake = ChSharedPtr<M113_BrakeSimple>(new M113_BrakeSimple());
        num_shoes = 63;
    } else {
        SetName("M113_TrackAssemblyRight");
        m_sprocket = ChSharedPtr<M113_Sprocket>(new M113_SprocketRight(vis_type));
        m_idler = ChSharedPtr<M113_Idler>(new M113_IdlerRight(vis_type));
        m_brake = ChSharedPtr<M113_BrakeSimple>(new M113_BrakeSimple());
        num_shoes = 64;
    }

    m_suspensions.resize(5);
    m_suspensions[0] = ChSharedPtr<M113_Suspension>(new M113_Suspension(side, true, vis_type));
    m_suspensions[1] = ChSharedPtr<M113_Suspension>(new M113_Suspension(side, true, vis_type));
    m_suspensions[2] = ChSharedPtr<M113_Suspension>(new M113_Suspension(side, false, vis_type));
    m_suspensions[3] = ChSharedPtr<M113_Suspension>(new M113_Suspension(side, false, vis_type));
    m_suspensions[4] = ChSharedPtr<M113_Suspension>(new M113_Suspension(side, true, vis_type));

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(ChSharedPtr<M113_TrackShoe>(new M113_TrackShoe(vis_type)));
    }
}

}  // end namespace m113
