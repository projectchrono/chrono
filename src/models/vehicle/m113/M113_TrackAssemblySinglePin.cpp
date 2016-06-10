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
// M113 single-pin track assembly subsystem.
//
// =============================================================================

#include "models/vehicle/m113/M113_BrakeSimple.h"
#include "models/vehicle/m113/M113_Idler.h"
#include "models/vehicle/m113/M113_RoadWheel.h"
#include "models/vehicle/m113/M113_SprocketSinglePin.h"
#include "models/vehicle/m113/M113_Suspension.h"
#include "models/vehicle/m113/M113_TrackAssemblySinglePin.h"
#include "models/vehicle/m113/M113_TrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// Constructor for the M113 track assembly using single-pin track shoes.
// Create the suspensions, idler, brake, sprocket, and track shoes.
M113_TrackAssemblySinglePin::M113_TrackAssemblySinglePin(VehicleSide side) : ChTrackAssemblySinglePin("", side) {
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
            m_sprocket = std::make_shared<M113_SprocketSinglePinLeft>();
            num_shoes = 63;
            break;
        case RIGHT:
            m_sprocket = std::make_shared<M113_SprocketSinglePinRight>();
            num_shoes = 64;
            break;
    }

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(std::make_shared<M113_TrackShoeSinglePin>());
    }
}

void M113_TrackAssemblySinglePin::SetIdlerVisType(VisualizationType vis) {
    std::static_pointer_cast<M113_Idler>(GetIdler())->SetVisType(vis);
}

void M113_TrackAssemblySinglePin::SetRoadWheelVisType(VisualizationType vis) {
    for (int iw = 0; iw < GetNumRoadWheelAssemblies(); iw++) {
        std::static_pointer_cast<M113_RoadWheel>(GetRoadWheel(iw))->SetVisType(vis);
    }
}

void M113_TrackAssemblySinglePin::SetSprocketVisType(VisualizationType vis) {
    std::static_pointer_cast<M113_SprocketSinglePin>(GetSprocket())->SetVisType(vis);
}

void M113_TrackAssemblySinglePin::SetTrackShoeVisType(VisualizationType vis) {
    for (size_t is = 0; is < GetNumTrackShoes(); is++) {
        std::static_pointer_cast<M113_TrackShoeSinglePin>(GetTrackShoe(is))->SetVisType(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
