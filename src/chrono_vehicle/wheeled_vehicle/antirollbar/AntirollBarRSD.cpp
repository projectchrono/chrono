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
// RSD antirollbar model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/antirollbar/AntirollBarRSD.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
AntirollBarRSD::AntirollBarRSD(const std::string& filename) : ChAntirollBarRSD("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

AntirollBarRSD::AntirollBarRSD(const rapidjson::Document& d) : ChAntirollBarRSD("") {
    Create(d);
}

void AntirollBarRSD::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read arm data
    m_arm_mass = d["Arm"]["Mass"].GetDouble();
    m_arm_inertia = ReadVectorJSON(d["Arm"]["Inertia"]);
    m_arm_length = d["Arm"]["Length"].GetDouble();
    m_arm_width = d["Arm"]["Width"].GetDouble();
    m_arm_radius = d["Arm"]["Radius"].GetDouble();

    // Read droplink data
    m_link_height = d["Droplink"]["Height"].GetDouble();

    // Read RSD data
    m_spring_coef = d["RSD"]["Spring Coefficient"].GetDouble();
    m_damping_coef = d["RSD"]["Damping Coefficient"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
