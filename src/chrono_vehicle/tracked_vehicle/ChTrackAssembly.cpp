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
// Base class for a track assembly which consists of one sprocket, one idler,
// a collection of road wheel assemblies (suspensions), and a collection of
// track shoes.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Get the complete state for the specified track shoe.
// -----------------------------------------------------------------------------
BodyState ChTrackAssembly::GetTrackShoeState(size_t id) const {
    BodyState state;

    state.pos = GetTrackShoePos(id);
    state.rot = GetTrackShoeRot(id);
    state.lin_vel = GetTrackShoeLinVel(id);
    state.ang_vel = GetTrackShoeAngVel(id);

    return state;
}

// -----------------------------------------------------------------------------
// Initialize this track assembly subsystem.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                                 const ChVector<>& sprocket_loc,
                                 const ChVector<>& idler_loc,
                                 const std::vector<ChVector<> >& suspension_locs) {
    m_sprocket->Initialize(chassis, sprocket_loc);
    m_idler->Initialize(chassis, idler_loc);

    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->Initialize(chassis, suspension_locs[i]);
    }
}

// -----------------------------------------------------------------------------
// Assemble track shoes over wheels.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Assemble() {
    //// TODO
}

// -----------------------------------------------------------------------------
// Update the state of this track assembly at the current time.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Update(double time, const TrackShoeForces& shoe_forces) {
    // Apply track shoe forces
    for (size_t i = 0; i < m_shoes.size(); ++i) {
        m_shoes[i]->m_shoe->Empty_forces_accumulators();
        m_shoes[i]->m_shoe->Accumulate_force(shoe_forces[i].force, shoe_forces[i].point, false);
        m_shoes[i]->m_shoe->Accumulate_torque(shoe_forces[i].moment, false);
    }
    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
