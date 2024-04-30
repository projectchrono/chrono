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
// Simple driveline model template using data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SimpleDriveline::SimpleDriveline(const std::string& filename) : ChSimpleDriveline("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

SimpleDriveline::SimpleDriveline(const rapidjson::Document& d) : ChSimpleDriveline("") {
    Create(d);
}

void SimpleDriveline::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_front_torque_frac = d["Front Torque Fraction"].GetDouble();
    m_front_diff_bias = d["Front Differential Max Bias"].GetDouble();
    m_rear_diff_bias = d["Rear Differential Max Bias"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
