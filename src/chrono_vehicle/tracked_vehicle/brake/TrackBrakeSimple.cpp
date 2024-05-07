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
// Tracked vehicle simple brake model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackBrakeSimple::TrackBrakeSimple(const std::string& filename) : ChTrackBrakeSimple("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

TrackBrakeSimple::TrackBrakeSimple(const rapidjson::Document& d) : ChTrackBrakeSimple("") {
    Create(d);
}

void TrackBrakeSimple::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    // Read maximum braking torque
    m_maxtorque = d["Maximum Torque"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
