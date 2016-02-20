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
// Base class for a tracked vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicle::ChTrackedVehicle(const std::string& name, ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method), m_name(name), m_contacts(new ChTrackContactManager) {
}

ChTrackedVehicle::ChTrackedVehicle(const std::string& name, ChSystem* system)
    : ChVehicle(system), m_name(name), m_contacts(new ChTrackContactManager) {
}

ChTrackedVehicle::~ChTrackedVehicle() {
    delete m_contacts;
}

// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between
// 0 and 1, steering between -1 and +1, braking between 0 and 1), the torque
// from the powertrain, and tire forces (expressed in the global reference
// frame).
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Synchronize(double time,
                                   double steering,
                                   double braking,
                                   double powertrain_torque,
                                   const TrackShoeForces& shoe_forces_left,
                                   const TrackShoeForces& shoe_forces_right) {
    // Apply powertrain torque to the driveline's input shaft.
    m_driveline->Synchronize(steering, powertrain_torque);

    // Apply contact track shoe forces.
    m_tracks[LEFT]->Synchronize(time, braking, shoe_forces_left);
    m_tracks[RIGHT]->Synchronize(time, braking, shoe_forces_right);
}

// -----------------------------------------------------------------------------
// Advance the state of this vehicle by the specified time step.
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Advance(double step) {
    // Invoke the base class method to perform the actual work.
    ChVehicle::Advance(step);

    // Process contacts.
    m_contacts->Process(this);
}

// -----------------------------------------------------------------------------
// Override collision flags for various subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicle::SetCollide(int flags) {
    m_tracks[0]->GetIdler()->SetCollide((flags & static_cast<int>(TrackCollide::IDLER_LEFT)) != 0);
    m_tracks[1]->GetIdler()->SetCollide((flags & static_cast<int>(TrackCollide::IDLER_RIGHT)) != 0);

    m_tracks[0]->GetSprocket()->SetCollide((flags & static_cast<int>(TrackCollide::SPROCKET_LEFT)) != 0);
    m_tracks[1]->GetSprocket()->SetCollide((flags & static_cast<int>(TrackCollide::SPROCKET_RIGHT)) != 0);

    bool collide_wheelsL = (flags & static_cast<int>(TrackCollide::WHEELS_LEFT)) != 0;
    bool collide_wheelsR = (flags & static_cast<int>(TrackCollide::WHEELS_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumRoadWheelAssemblies(); ++i)
        m_tracks[0]->GetRoadWheel(i)->SetCollide(collide_wheelsL);
    for (size_t i = 0; i < m_tracks[1]->GetNumRoadWheelAssemblies(); ++i)
        m_tracks[1]->GetRoadWheel(i)->SetCollide(collide_wheelsR);

    bool collide_shoesL = (flags & static_cast<int>(TrackCollide::SHOES_LEFT)) != 0;
    bool collide_shoesR = (flags & static_cast<int>(TrackCollide::SHOES_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackShoes(); ++i)
        m_tracks[0]->GetTrackShoe(i)->SetCollide(collide_shoesL);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackShoes(); ++i)
        m_tracks[1]->GetTrackShoe(i)->SetCollide(collide_shoesR);
}

// -----------------------------------------------------------------------------
// Calculate and return the total vehicle mass
// -----------------------------------------------------------------------------
double ChTrackedVehicle::GetVehicleMass() const {
    return m_chassis->GetMass() + m_tracks[0]->GetMass() + m_tracks[1]->GetMass();
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChTrackedVehicle::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the track assemblies.
    GetLog() << "\n---- LEFT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[0]->LogConstraintViolations();
    GetLog() << "\n---- RIGHT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[1]->LogConstraintViolations();

    GetLog().SetNumFormat("%g");
}

}  // end namespace vehicle
}  // end namespace chrono
