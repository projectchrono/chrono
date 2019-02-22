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
// Classes for monitoring contacts of tracked vehicle subsystems.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackContactManager.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackContactManager::ChTrackContactManager()
    : m_initialized(false), m_flags(0), m_collect(false), m_shoe_index_L(0), m_shoe_index_R(0) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackContactManager::Process(ChTrackedVehicle* vehicle) {
    if (m_flags == 0)
        return;

    // Initialize the manager if not already done.
    if (!m_initialized) {
        m_chassis = vehicle->GetChassis();

        m_sprocket_L = vehicle->GetTrackAssembly(LEFT)->GetSprocket();
        m_sprocket_R = vehicle->GetTrackAssembly(RIGHT)->GetSprocket();

        m_shoe_L = vehicle->GetTrackAssembly(LEFT)->GetTrackShoe(m_shoe_index_L);
        m_shoe_R = vehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(m_shoe_index_R);

        m_idler_L = vehicle->GetTrackAssembly(LEFT)->GetIdler();
        m_idler_R = vehicle->GetTrackAssembly(RIGHT)->GetIdler();

        m_initialized = true;
    }

    // Clear lists
    m_chassis_contacts.clear();
    m_sprocket_L_contacts.clear();
    m_sprocket_R_contacts.clear();
    m_shoe_L_contacts.clear();
    m_shoe_R_contacts.clear();
    m_idler_L_contacts.clear();
    m_idler_R_contacts.clear();

    // Traverse all system contacts and extract information.
    vehicle->GetSystem()->GetContactContainer()->ReportAllContacts(this);

    // Collect contact information data.
    // Print current time, and number of contacts involving the chassis, left/right sprockets,
    // left/right idlers, left/right track shoes, followed by the location of the contacts, in the
    // same order as above, expressed in the local frame of the respective body.
    if (m_collect) {
        // Get number of contacts in all lists;
        size_t n_chassis = m_chassis_contacts.size();
        size_t n_sprocket_L = m_sprocket_L_contacts.size();
        size_t n_sprocket_R = m_sprocket_R_contacts.size();
        size_t n_idler_L = m_idler_L_contacts.size();
        size_t n_idler_R = m_idler_R_contacts.size();
        size_t n_shoe_L = m_shoe_L_contacts.size();
        size_t n_shoe_R = m_shoe_R_contacts.size();

        // Only collect data at this time if there is at least one monitored contact
        size_t n_contacts = n_chassis + n_sprocket_L + n_sprocket_R + n_idler_L + n_idler_R + n_shoe_L + n_shoe_R;

        if (n_contacts != 0) {
            // Current simulation time
            m_csv << vehicle->GetChTime();

            // Number of contacts on vehicle parts
            m_csv << m_chassis_contacts.size();
            m_csv << m_sprocket_L_contacts.size();
            m_csv << m_sprocket_R_contacts.size();
            m_csv << m_idler_L_contacts.size();
            m_csv << m_idler_R_contacts.size();
            m_csv << m_shoe_L_contacts.size();
            m_csv << m_shoe_R_contacts.size();

            // Chassis contact points
            for (auto it = m_chassis_contacts.begin(); it != m_chassis_contacts.end(); ++it) {
                m_csv << m_chassis->GetBody()->TransformPointParentToLocal(it->m_point);
            }

            // Left sprocket contact points
            for (auto it = m_sprocket_L_contacts.begin(); it != m_sprocket_L_contacts.end(); ++it) {
                m_csv << m_sprocket_L->GetGearBody()->TransformPointParentToLocal(it->m_point);
            }

            // Right sprocket contact points
            for (auto it = m_sprocket_R_contacts.begin(); it != m_sprocket_R_contacts.end(); ++it) {
                m_csv << m_sprocket_R->GetGearBody()->TransformPointParentToLocal(it->m_point);
            }

            // Left idler contact points
            for (auto it = m_idler_L_contacts.begin(); it != m_idler_L_contacts.end(); ++it) {
                m_csv << m_idler_L->GetWheelBody()->TransformPointParentToLocal(it->m_point);
            }

            // Right idler contact points
            for (auto it = m_idler_R_contacts.begin(); it != m_idler_R_contacts.end(); ++it) {
                m_csv << m_idler_R->GetWheelBody()->TransformPointParentToLocal(it->m_point);
            }

            // Left track shoe contact points
            for (auto it = m_shoe_L_contacts.begin(); it != m_shoe_L_contacts.end(); ++it) {
                m_csv << m_shoe_L->GetShoeBody()->TransformPointParentToLocal(it->m_point);
            }

            // Right track shoe contact points
            for (auto it = m_shoe_R_contacts.begin(); it != m_shoe_R_contacts.end(); ++it) {
                m_csv << m_shoe_R->GetShoeBody()->TransformPointParentToLocal(it->m_point);
            }

            m_csv << std::endl;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChTrackContactManager::InContact(TrackedCollisionFlag::Enum part) const {
    switch (part) {
        case TrackedCollisionFlag::CHASSIS:
            return m_chassis_contacts.size() != 0;
        case TrackedCollisionFlag::SPROCKET_LEFT:
            return m_sprocket_L_contacts.size() != 0;
        case TrackedCollisionFlag::SPROCKET_RIGHT:
            return m_sprocket_R_contacts.size() != 0;
        case TrackedCollisionFlag::IDLER_LEFT:
            return m_idler_L_contacts.size() != 0;
        case TrackedCollisionFlag::IDLER_RIGHT:
            return m_idler_R_contacts.size() != 0;
        case TrackedCollisionFlag::SHOES_LEFT:
            return m_shoe_L_contacts.size() != 0;
        case TrackedCollisionFlag::SHOES_RIGHT:
            return m_shoe_R_contacts.size() != 0;
        default:
            return false;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChTrackContactManager::OnReportContact(const ChVector<>& pA,
                                            const ChVector<>& pB,
                                            const ChMatrix33<>& plane_coord,
                                            const double& distance,
                                            const double& eff_radius,
                                            const ChVector<>& react_forces,
                                            const ChVector<>& react_torques,
                                            ChContactable* modA,
                                            ChContactable* modB) {
    ChTrackContactInfo info;

    // Ignore contacts with zero force or positive separation.
    if (distance > 0 || react_forces.IsNull())
        return true;

    // Extract contacts on chassis.
    if (IsFlagSet(TrackedCollisionFlag::CHASSIS)) {
        if (modA == m_chassis->GetBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_chassis_contacts.push_back(info);
        }
        if (modB == m_chassis->GetBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_chassis_contacts.push_back(info);
        }
    }

    // Extract contacts on sprockets.
    if (IsFlagSet(TrackedCollisionFlag::SPROCKET_LEFT)) {
        if (modA == m_sprocket_L->GetGearBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_sprocket_L_contacts.push_back(info);
        }
        if (modB == m_sprocket_L->GetGearBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_sprocket_L_contacts.push_back(info);
        }
    }

    if (IsFlagSet(TrackedCollisionFlag::SPROCKET_RIGHT)) {
        if (modA == m_sprocket_R->GetGearBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_sprocket_R_contacts.push_back(info);
        }
        if (modB == m_sprocket_R->GetGearBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_sprocket_R_contacts.push_back(info);
        }
    }

    // Extract contacts on track shoes (discard contacts with sprockets)
    if (IsFlagSet(TrackedCollisionFlag::SHOES_LEFT)) {
        if (modA == m_shoe_L->GetShoeBody().get() && modB != m_sprocket_L->GetGearBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_shoe_L_contacts.push_back(info);
        }
        if (modB == m_shoe_L->GetShoeBody().get() && modA != m_sprocket_L->GetGearBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_shoe_L_contacts.push_back(info);
        }
    }

    if (IsFlagSet(TrackedCollisionFlag::SHOES_RIGHT)) {
        if (modA == m_shoe_R->GetShoeBody().get() && modB != m_sprocket_R->GetGearBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_shoe_R_contacts.push_back(info);
        }
        if (modB == m_shoe_R->GetShoeBody().get() && modA != m_sprocket_R->GetGearBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_shoe_R_contacts.push_back(info);
        }
    }

    // Extract contacts on idler wheels.
    if (IsFlagSet(TrackedCollisionFlag::IDLER_LEFT)) {
        if (modA == m_idler_L->GetWheelBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_idler_L_contacts.push_back(info);
        }
        if (modB == m_idler_L->GetWheelBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_idler_L_contacts.push_back(info);
        }
    }

    if (IsFlagSet(TrackedCollisionFlag::IDLER_RIGHT)) {
        if (modA == m_idler_R->GetWheelBody().get()) {
            info.m_point = pA;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_idler_R_contacts.push_back(info);
        }
        if (modB == m_idler_R->GetWheelBody().get()) {
            info.m_point = pB;
            info.m_csys = plane_coord;
            info.m_force = react_forces;
            info.m_torque = react_torques;
            m_idler_R_contacts.push_back(info);
        }
    }

    // Continue scanning contacts
    return true;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackContactManager::WriteContacts(const std::string& filename) {
    if (m_collect && m_flags != 0)
        m_csv.write_to_file(filename);
}

}  // end namespace vehicle
}  // end namespace chrono
