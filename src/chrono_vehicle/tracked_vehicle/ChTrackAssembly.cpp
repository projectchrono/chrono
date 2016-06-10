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

#include <cmath>

#include "chrono/core/ChLog.h"

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
// Get the complete states for all track shoes.
// -----------------------------------------------------------------------------
void ChTrackAssembly::GetTrackShoeStates(BodyStates& states) const {
    size_t num_shoes = GetNumTrackShoes();
    assert(states.size() == num_shoes);

    for (size_t i = 0; i < num_shoes; ++i)
        states[i] = GetTrackShoeState(i);
}

// -----------------------------------------------------------------------------
// Initialize this track assembly subsystem.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                 const ChVector<>& sprocket_loc,
                                 const ChVector<>& idler_loc,
                                 const std::vector<ChVector<> >& suspension_locs) {
    GetSprocket()->Initialize(chassis, sprocket_loc, this);
    m_idler->Initialize(chassis, idler_loc);
    m_brake->Initialize(GetSprocket()->GetRevolute());

    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->Initialize(chassis, suspension_locs[i]);
    }

    // Assemble the track. This positions all track shoes around the sprocket,
    // road wheels, and idler. (Implemented by derived classes)
    bool ccw = Assemble(chassis);

    // Loop over all track shoes and allow them to connect themselves to their
    // neighbor.
    size_t num_shoes = GetNumTrackShoes();
    std::shared_ptr<ChTrackShoe> next;
    for (size_t i = 0; i < num_shoes; ++i) {
        if (ccw)
            next = (i == num_shoes - 1) ? GetTrackShoe(0) : GetTrackShoe(i + 1);
        else
            next = (i == 0) ? GetTrackShoe(num_shoes - 1) : GetTrackShoe(i - 1);
        GetTrackShoe(i)->Connect(next);
    }
}

// -----------------------------------------------------------------------------
// Calculate and return the total mass of the track assembly
// -----------------------------------------------------------------------------
double ChTrackAssembly::GetMass() const {
    double mass = GetSprocket()->GetMass() + m_idler->GetMass();
    for (size_t i = 0; i < m_suspensions.size(); i++)
        mass += m_suspensions[i]->GetMass();
    for (size_t i = 0; i < GetNumTrackShoes(); ++i)
        mass += GetTrackShoe(i)->GetMass();

    return mass;
}

// -----------------------------------------------------------------------------
// Update the state of this track assembly at the current time.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Synchronize(double time, double braking, const TrackShoeForces& shoe_forces) {
    // Apply track shoe forces
    for (size_t i = 0; i < GetNumTrackShoes(); ++i) {
        GetTrackShoe(i)->m_shoe->Empty_forces_accumulators();
        GetTrackShoe(i)->m_shoe->Accumulate_force(shoe_forces[i].force, shoe_forces[i].point, false);
        GetTrackShoe(i)->m_shoe->Accumulate_torque(shoe_forces[i].moment, false);
    }

    // Apply braking input
    m_brake->Synchronize(braking);
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChTrackAssembly::LogConstraintViolations() {
    GetLog() << "SPROCKET constraint violations\n";
    GetSprocket()->LogConstraintViolations();
    GetLog() << "IDLER constraint violations\n";
    m_idler->LogConstraintViolations();
    for (size_t i = 0; i < m_suspensions.size(); i++) {
        GetLog() << "SUSPENSION #" << i << " constraint violations\n";
        m_suspensions[i]->LogConstraintViolations();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
