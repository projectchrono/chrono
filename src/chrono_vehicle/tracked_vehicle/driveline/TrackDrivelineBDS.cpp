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
// Track driveline model template based on ChShaft objects using data from file
// (JSON format). 
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/driveline/TrackDrivelineBDS.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackDrivelineBDS::TrackDrivelineBDS(const std::string& filename) : ChTrackDrivelineBDS("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackDrivelineBDS::TrackDrivelineBDS(const rapidjson::Document& d) : ChTrackDrivelineBDS("") {
    Create(d);
}

void TrackDrivelineBDS::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // The direction of the motor block is along the X axis, while the directions of
    // the axles is along the Y axis (relative to the chassis coordinate frame).
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));

    m_driveshaft_inertia = d["Driveshaft Inertia"].GetDouble();
    m_differentialbox_inertia = d["Differential Box Inertia"].GetDouble();

    m_conicalgear_ratio = d["Conical Gear Ratio"].GetDouble();

    m_differential_locking_limit = 100;
    if (d.HasMember("Differential Locking Limit")) {
        m_differential_locking_limit = d["Differential Locking Limit"].GetDouble();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
