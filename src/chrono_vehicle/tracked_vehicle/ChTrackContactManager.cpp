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
        m_sprocket_L = vehicle->GetTrackAssembly(LEFT)->GetSprocket();
        m_sprocket_R = vehicle->GetTrackAssembly(RIGHT)->GetSprocket();

        m_shoe_L = vehicle->GetTrackAssembly(LEFT)->GetTrackShoe(m_shoe_index_L);
        m_shoe_R = vehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(m_shoe_index_R);

        m_idler_L = vehicle->GetTrackAssembly(LEFT)->GetIdler();
        m_idler_R = vehicle->GetTrackAssembly(RIGHT)->GetIdler();

        m_initialized = true;
    }

    // Clear lists
    m_sprocket_L_contacts.clear();
    m_sprocket_R_contacts.clear();
    m_shoe_L_contacts.clear();
    m_shoe_R_contacts.clear();
    m_idler_L_contacts.clear();
    m_idler_R_contacts.clear();

    // Traverse all system contacts and extract information.
    vehicle->GetSystem()->GetContactContainer()->ReportAllContacts(this);

    // Collect contact information data.

    //// TODO...

    if (m_collect) {
        m_csv << vehicle->GetChTime();
        // Left sprocket
        m_csv << m_sprocket_L_contacts.size();
        for (auto it = m_sprocket_L_contacts.begin(); it != m_sprocket_L_contacts.end(); ++it) {
            m_csv << m_sprocket_L->GetGearBody()->TransformPointParentToLocal(it->m_point);
        }
        m_csv << std::endl;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChTrackContactManager::ReportContactCallback(const ChVector<>& pA,
                                                  const ChVector<>& pB,
                                                  const ChMatrix33<>& plane_coord,
                                                  const double& distance,
                                                  const ChVector<>& react_forces,
                                                  const ChVector<>& react_torques,
                                                  ChContactable* modA,
                                                  ChContactable* modB) {
    ChTrackContactInfo info;

    // Ignore contacts with zero force.
    if (react_forces.IsNull())
        return true;

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
