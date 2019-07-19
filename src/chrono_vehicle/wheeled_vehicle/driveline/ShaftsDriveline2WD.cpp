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
// 2WD driveline model template based on ChShaft objects using data from file
// (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ShaftsDriveline2WD::ShaftsDriveline2WD(const std::string& filename) : ChShaftsDriveline2WD("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ShaftsDriveline2WD::ShaftsDriveline2WD(const rapidjson::Document& d) : ChShaftsDriveline2WD("") {
    Create(d);
}

void ShaftsDriveline2WD::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Get shaft directions.
    assert(d.HasMember("Shaft Direction"));
    SetMotorBlockDirection(ReadVectorJSON(d["Shaft Direction"]["Motor Block"]));
    SetAxleDirection(ReadVectorJSON(d["Shaft Direction"]["Axle"]));

    // Read shaft inertias.
    assert(d.HasMember("Shaft Inertia"));
    m_driveshaft_inertia = d["Shaft Inertia"]["Driveshaft"].GetDouble();
    m_differentialbox_inertia = d["Shaft Inertia"]["Differential Box"].GetDouble();

    // Read gear ratios.
    assert(d.HasMember("Gear Ratio"));
    m_conicalgear_ratio = d["Gear Ratio"]["Conical Gear"].GetDouble();
    m_differential_ratio = d["Gear Ratio"]["Differential"].GetDouble();

    m_axle_differential_locking_limit = 100;
    if (d.HasMember("Axle Differential Locking Limit")) {
        m_axle_differential_locking_limit = d["Axle Differential Locking Limit"].GetDouble();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
