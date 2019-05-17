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
// Rack-pinion steering model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RackPinion::RackPinion(const std::string& filename) : ChRackPinion("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RackPinion::RackPinion(const rapidjson::Document& d) : ChRackPinion("") {
    Create(d);
}

void RackPinion::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read steering link data
    m_steeringLinkMass = d["Steering Link"]["Mass"].GetDouble();
    m_steeringLinkInertia = ReadVectorJSON(d["Steering Link"]["Inertia"]);
    m_steeringLinkCOM = d["Steering Link"]["COM"].GetDouble();
    m_steeringLinkRadius = d["Steering Link"]["Radius"].GetDouble();
    m_steeringLinkLength = d["Steering Link"]["Length"].GetDouble();

    // Pinion radius
    m_pinionRadius = d["Pinion"]["Radius"].GetDouble();
    m_maxAngle = d["Pinion"]["Maximum Angle"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
