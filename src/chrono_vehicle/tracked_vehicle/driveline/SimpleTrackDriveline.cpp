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
// Simple tracked vehicle driveline model template using data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/driveline/SimpleTrackDriveline.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SimpleTrackDriveline::SimpleTrackDriveline(const std::string& filename) : ChSimpleTrackDriveline("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

SimpleTrackDriveline::SimpleTrackDriveline(const rapidjson::Document& d) : ChSimpleTrackDriveline("") {
    Create(d);
}

void SimpleTrackDriveline::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_diff_bias = d["Differential Max Bias"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
