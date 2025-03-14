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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// M113 continuous band track assembly subsystem using rigid-link track shoes.
//
// =============================================================================

#include "chrono_models/vehicle/m113/track_assembly/M113_TrackAssemblyBandANCF.h"
#include "chrono_models/vehicle/m113/M113_BrakeSimple.h"
#include "chrono_models/vehicle/m113/M113_BrakeShafts.h"
#include "chrono_models/vehicle/m113/M113_Idler.h"
#include "chrono_models/vehicle/m113/M113_IdlerWheel.h"
#include "chrono_models/vehicle/m113/M113_RoadWheel.h"
#include "chrono_models/vehicle/m113/sprocket/M113_SprocketBand.h"
#include "chrono_models/vehicle/m113/M113_Suspension.h"
#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandANCF.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const ChVector3d M113_TrackAssemblyBandANCF::m_sprocket_loc(0, 0, 0);
const ChVector3d M113_TrackAssemblyBandANCF::m_idler_loc(-3.83, 0, -0.12);
const ChVector3d M113_TrackAssemblyBandANCF::m_susp_locs_L[5] = {  //
    ChVector3d(-0.655, 0, -0.215),                                 //
    ChVector3d(-1.322, 0, -0.215),                                 //
    ChVector3d(-1.989, 0, -0.215),                                 //
    ChVector3d(-2.656, 0, -0.215),                                 //
    ChVector3d(-3.322, 0, -0.215)};
const ChVector3d M113_TrackAssemblyBandANCF::m_susp_locs_R[5] = {  //
    ChVector3d(-0.740, 0, -0.215),                                 //
    ChVector3d(-1.407, 0, -0.215),                                 //
    ChVector3d(-2.074, 0, -0.215),                                 //
    ChVector3d(-2.740, 0, -0.215),                                 //
    ChVector3d(-3.407, 0, -0.215)};

// -----------------------------------------------------------------------------
// Constructor for the M113 continuous band track assembly using rigid-link
// track shoes.
// Create the suspensions, idler, brake, sprocket, and track shoes.
// -----------------------------------------------------------------------------
M113_TrackAssemblyBandANCF::M113_TrackAssemblyBandANCF(VehicleSide side,
                                                       BrakeType brake_type,
                                                       ChTrackShoeBandANCF::ElementType element_type,
                                                       bool constrain_curvature,
                                                       int num_elements_length,
                                                       int num_elements_width,
                                                       bool use_suspension_bushings)
    : ChTrackAssemblyBandANCF("", side) {
    size_t num_shoes = 0;
    std::string suspName("M113_Suspension");
    std::string shoeName("M113_TrackShoe");
    switch (side) {
        case LEFT:
            SetName("M113_TrackAssemblyLeft");
            m_idler = chrono_types::make_shared<M113_Idler>("M113_Idler_Left", side);
            m_brake = chrono_types::make_shared<M113_BrakeSimple>("M113_BrakeLeft");
            m_sprocket = chrono_types::make_shared<M113_SprocketBandLeft>();
            num_shoes = 105;
            suspName += "Left_";
            shoeName += "Left_";
            break;
        case RIGHT:
            SetName("M113_TrackAssemblyRight");
            m_idler = chrono_types::make_shared<M113_Idler>("M113_Idler_Right", side);
            m_brake = chrono_types::make_shared<M113_BrakeSimple>("M113_BrakeRight");
            m_sprocket = chrono_types::make_shared<M113_SprocketBandRight>();
            num_shoes = 106;
            suspName += "Right_";
            shoeName += "Right_";
            break;
    }

    m_suspensions.resize(5);
    m_suspensions[0] =
        chrono_types::make_shared<M113_Suspension>(suspName + "0", side, 0, use_suspension_bushings, true);
    m_suspensions[1] =
        chrono_types::make_shared<M113_Suspension>(suspName + "1", side, 0, use_suspension_bushings, true);
    m_suspensions[2] =
        chrono_types::make_shared<M113_Suspension>(suspName + "2", side, 2, use_suspension_bushings, false);
    m_suspensions[3] =
        chrono_types::make_shared<M113_Suspension>(suspName + "3", side, 2, use_suspension_bushings, false);
    m_suspensions[4] =
        chrono_types::make_shared<M113_Suspension>(suspName + "4", side, 0, use_suspension_bushings, true);

    for (size_t it = 0; it < num_shoes; it++) {
        m_shoes.push_back(chrono_types::make_shared<M113_TrackShoeBandANCF>(
            shoeName + std::to_string(it), element_type, constrain_curvature, num_elements_length, num_elements_width));
    }

    // Specify material properties for the web mesh
    double E_rubber = 0.01e9;
    double nu_rubber = 0.3;
    double G_rubber = 0.5 * E_rubber / (1 + 0.49);
    SetRubberLayerMaterial(1100, ChVector3d(E_rubber), ChVector3d(nu_rubber), ChVector3d(G_rubber));

    double E_steel = 210e9;
    double nu_steel = 0.3;
    double G_steel = 0.5 * E_steel / (1 + 0.3);
    SetSteelLayerMaterial(7900, ChVector3d(E_steel), ChVector3d(nu_steel), ChVector3d(G_steel));

    SetLayerFiberAngles(0 * CH_DEG_TO_RAD, 0 * CH_DEG_TO_RAD, 0 * CH_DEG_TO_RAD);

    SetElementStructuralDamping(0.15);

    // Specify contact properties for the web mesh
    SetContactSurfaceType(ChTrackAssemblyBandANCF::ContactSurfaceType::TRIANGLE_MESH);
}

void M113_TrackAssemblyBandANCF::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.1f;
    minfo.Y = 1e7f;
    m_contact_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
const ChVector3d M113_TrackAssemblyBandANCF::GetSprocketLocation() const {
    return m_sprocket_loc;
}

const ChVector3d M113_TrackAssemblyBandANCF::GetIdlerLocation() const {
    return m_idler_loc;
}

const ChVector3d M113_TrackAssemblyBandANCF::GetRoadWhelAssemblyLocation(int which) const {
    return (m_side == LEFT) ? m_susp_locs_L[which] : m_susp_locs_R[which];
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
