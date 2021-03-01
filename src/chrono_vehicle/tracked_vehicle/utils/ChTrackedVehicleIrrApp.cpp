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
// Irrlicht-based visualization wrapper for tracked vehicles.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicleIrrApp::ChTrackedVehicleIrrApp(ChVehicle* vehicle,
                                               const std::wstring& title,
                                               const irr::core::dimension2d<irr::u32>& dims,
                                               irr::ELOG_LEVEL log_level)
    : ChVehicleIrrApp(vehicle, title, dims, log_level) {
    m_tvehicle = dynamic_cast<ChTrackedVehicle*>(vehicle);
    assert(m_tvehicle);
}

// -----------------------------------------------------------------------------
// Render stats for the vehicle driveline subsystem.
// -----------------------------------------------------------------------------
void ChTrackedVehicleIrrApp::renderOtherStats(int left, int top) {
    char msg[100];

    auto driveline = m_tvehicle->GetDriveline();
    double toRPM = 30 / CH_C_PI;

    double shaft_speed = driveline->GetDriveshaftSpeed() * toRPM;
    sprintf(msg, "Driveshaft (RPM): %+.2f", shaft_speed);
    renderLinGauge(std::string(msg), shaft_speed / 2000, true, left, top, 120, 15);

    double speedL = driveline->GetSprocketSpeed(LEFT) * toRPM;
    sprintf(msg, "Sprocket L (RPM): %+.2f", speedL);
    renderLinGauge(std::string(msg), speedL / 2000, true, left, top + 30, 120, 15);

    double torqueL = driveline->GetSprocketTorque(LEFT);
    sprintf(msg, "Torque sprocket L: %+.2f", torqueL);
    renderLinGauge(std::string(msg), torqueL / 5000, true, left, top + 50, 120, 15);

    double speedR = driveline->GetSprocketSpeed(RIGHT) * toRPM;
    sprintf(msg, "Sprocket R (RPM): %+.2f", speedR);
    renderLinGauge(std::string(msg), speedR / 2000, true, left, top + 80, 120, 15);

    double torqueR = driveline->GetSprocketTorque(RIGHT);
    sprintf(msg, "Torque sprocket R: %+.2f", torqueR);
    renderLinGauge(std::string(msg), torqueR / 5000, true, left, top + 100, 120, 15);
}

// -----------------------------------------------------------------------------
// Render contact normals for monitored subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicleIrrApp::renderOtherGraphics() {
    // Contact normals on left sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_tvehicle->m_contact_manager->m_sprocket_L_contacts) {
        ChVector<> v1 = c.m_point;
        ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis();

        if (v1.y() > m_tvehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos().y())
            irrlicht::tools::drawSegment(GetVideoDriver(), v1, v2, video::SColor(255, 180, 0, 0), false);
    }

    // Contact normals on rear sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_tvehicle->m_contact_manager->m_sprocket_R_contacts) {
        ChVector<> v1 = c.m_point;
        ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis();

        if (v1.y() < m_tvehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos().y())
            irrlicht::tools::drawSegment(GetVideoDriver(), v1, v2, video::SColor(255, 180, 0, 0), false);
    }

    // Contact normals on monitored track shoes.
    renderContactNormals(m_tvehicle->m_contact_manager->m_shoe_L_contacts, video::SColor(255, 180, 180, 0));
    renderContactNormals(m_tvehicle->m_contact_manager->m_shoe_R_contacts, video::SColor(255, 180, 180, 0));

    // Contact normals on idler wheels.
    renderContactNormals(m_tvehicle->m_contact_manager->m_idler_L_contacts, video::SColor(255, 0, 0, 180));
    renderContactNormals(m_tvehicle->m_contact_manager->m_idler_R_contacts, video::SColor(255, 0, 0, 180));

    // Contact normals on chassis.
    renderContactNormals(m_tvehicle->m_contact_manager->m_chassis_contacts, video::SColor(255, 0, 180, 0));
}

// Render normal for all contacts in the specified list, using the given color.
void ChTrackedVehicleIrrApp::renderContactNormals(const std::list<ChTrackContactManager::ContactInfo>& lst,
                                                  const video::SColor& col) {
    for (const auto& c : lst) {
        ChVector<> v1 = c.m_point;
        ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis();

        irrlicht::tools::drawSegment(GetVideoDriver(), v1, v2, col, false);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
