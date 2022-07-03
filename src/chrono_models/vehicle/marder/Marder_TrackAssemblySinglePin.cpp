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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder single-pin track assembly subsystem.
//
// =============================================================================

#include "chrono_models/vehicle/marder/Marder_BrakeSimple.h"
#include "chrono_models/vehicle/marder/Marder_BrakeShafts.h"
#include "chrono_models/vehicle/marder/Marder_Idler.h"
#include "chrono_models/vehicle/marder/Marder_IdlerWheel.h"
#include "chrono_models/vehicle/marder/Marder_RoadWheel.h"
#include "chrono_models/vehicle/marder/Marder_SupportRoller.h"
#include "chrono_models/vehicle/marder/Marder_SprocketSinglePin.h"
#include "chrono_models/vehicle/marder/Marder_Suspension.h"
#include "chrono_models/vehicle/marder/Marder_TrackAssemblySinglePin.h"
#include "chrono_models/vehicle/marder/Marder_TrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
static const double supp_z_offset = 0.02;

const double Marder_TrackAssemblySinglePin::m_right_x_offset = 0.1;
const ChVector<> Marder_TrackAssemblySinglePin::m_sprocket_loc(0, 0, 0);
const ChVector<> Marder_TrackAssemblySinglePin::m_idler_loc(-5.4, 0, -0.0647);
const ChVector<> Marder_TrackAssemblySinglePin::m_susp_locs_L[6] = {
    ChVector<>(-0.8458, 0, -0.3759), ChVector<>(-1.6258, 0, -0.3759), ChVector<>(-2.4058, 0, -0.3759),
    ChVector<>(-3.1858, 0, -0.3759), ChVector<>(-3.9658, 0, -0.3759), ChVector<>(-4.7458, 0, -0.3759)};
const ChVector<> Marder_TrackAssemblySinglePin::m_susp_locs_R[6] = {
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 0.8458, 0, -0.3759),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 1.6258, 0, -0.3759),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 2.4058, 0, -0.3759),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 3.1858, 0, -0.3759),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 3.9658, 0, -0.3759),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 4.7458, 0, -0.3759)};
const ChVector<> Marder_TrackAssemblySinglePin::m_supp_locs_L[3] = {ChVector<>(-1.2358, 0, 0.1561 + supp_z_offset),
                                                                    ChVector<>(-2.7958, 0, 0.1561 + supp_z_offset),
                                                                    ChVector<>(-4.3106, 0, 0.1561 + supp_z_offset)};
const ChVector<> Marder_TrackAssemblySinglePin::m_supp_locs_R[3] = {
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 1.2358, 0, 0.1561 + supp_z_offset),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 2.7958, 0, 0.1561 + supp_z_offset),
    ChVector<>(Marder_TrackAssemblySinglePin::m_right_x_offset - 4.3106, 0, 0.1561 + supp_z_offset)};

// -----------------------------------------------------------------------------
// Constructor for the M113 track assembly using single-pin track shoes.
// Create the suspensions, idler, brake, sprocket, and track shoes.
// -----------------------------------------------------------------------------
Marder_TrackAssemblySinglePin::Marder_TrackAssemblySinglePin(VehicleSide side, BrakeType brake_type)
    : ChTrackAssemblySinglePin("", side) {
    size_t num_shoes = 0;
    std::string suspName("Marder_Suspension");
    std::string shoeName("Marder_TrackShoe");
    m_rollers.resize(3);
    switch (side) {
        case LEFT:
            SetName("Marder_TrackAssemblyLeft");
            m_idler = chrono_types::make_shared<Marder_Idler>("Marder_Idler_Left", side);
            m_brake = chrono_types::make_shared<Marder_BrakeShafts>("Marder_BrakeLeft");
            m_sprocket = chrono_types::make_shared<Marder_SprocketSinglePinLeft>();
            num_shoes = 77;
            suspName += "Left_";
            shoeName += "Left_";
            m_rollers[0] = chrono_types::make_shared<Marder_SupportRollerLeft>(0);
            m_rollers[1] = chrono_types::make_shared<Marder_SupportRollerLeft>(1);
            m_rollers[2] = chrono_types::make_shared<Marder_SupportRollerLeft>(2);
            break;
        case RIGHT:
            SetName("Marder_TrackAssemblyRight");
            m_idler = chrono_types::make_shared<Marder_Idler>("Marder_Idler_Right", side);
            m_brake = chrono_types::make_shared<Marder_BrakeShafts>("Marder_BrakeRight");
            m_sprocket = chrono_types::make_shared<Marder_SprocketSinglePinRight>();
            num_shoes = 78;
            suspName += "Right_";
            shoeName += "Right_";
            m_rollers[0] = chrono_types::make_shared<Marder_SupportRollerRight>(0);
            m_rollers[1] = chrono_types::make_shared<Marder_SupportRollerRight>(1);
            m_rollers[2] = chrono_types::make_shared<Marder_SupportRollerRight>(2);
            break;
    }

    m_suspensions.resize(6);
    m_suspensions[0] = chrono_types::make_shared<Marder_Suspension>(suspName + "0", side, 0, true);
    m_suspensions[1] = chrono_types::make_shared<Marder_Suspension>(suspName + "1", side, 1, true);
    m_suspensions[2] = chrono_types::make_shared<Marder_Suspension>(suspName + "2", side, 2, false);
    m_suspensions[3] = chrono_types::make_shared<Marder_Suspension>(suspName + "3", side, 3, false);
    m_suspensions[4] = chrono_types::make_shared<Marder_Suspension>(suspName + "4", side, 4, true);
    m_suspensions[5] = chrono_types::make_shared<Marder_Suspension>(suspName + "5", side, 5, true);

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(chrono_types::make_shared<Marder_TrackShoeSinglePin>(shoeName + std::to_string(it)));
    }
}

// -----------------------------------------------------------------------------
const ChVector<> Marder_TrackAssemblySinglePin::GetSprocketLocation() const {
    return m_sprocket_loc;
}

const ChVector<> Marder_TrackAssemblySinglePin::GetIdlerLocation() const {
    return m_idler_loc;
}

const ChVector<> Marder_TrackAssemblySinglePin::GetRoadWhelAssemblyLocation(int which) const {
    return (m_side == LEFT) ? m_susp_locs_L[which] : m_susp_locs_R[which];
}

const ChVector<> Marder_TrackAssemblySinglePin::GetRollerLocation(int which) const {
    return (m_side == LEFT) ? m_supp_locs_L[which] : m_supp_locs_R[which];
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
