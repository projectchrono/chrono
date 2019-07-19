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
// Pitman arm steering model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
PitmanArm::PitmanArm(const std::string& filename) : ChPitmanArm("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

PitmanArm::PitmanArm(const rapidjson::Document& d) : ChPitmanArm("") {
    Create(d);
}

void PitmanArm::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read flag indicating that inertia matrices are expressed in
    // vehicle-aligned centroidal frame.
    if (d.HasMember("Vehicle-Frame Inertia")) {
        bool flag = d["Vehicle-Frame Inertia"].GetBool();
        SetVehicleFrameInertiaFlag(flag);
    }

    // Read steering link data
    m_steeringLinkMass = d["Steering Link"]["Mass"].GetDouble();
    m_points[STEERINGLINK] = ReadVectorJSON(d["Steering Link"]["COM"]);
    m_steeringLinkInertiaMoments = ReadVectorJSON(d["Steering Link"]["Moments of Inertia"]);
    m_steeringLinkInertiaProducts = ReadVectorJSON(d["Steering Link"]["Products of Inertia"]);
    m_steeringLinkRadius = d["Steering Link"]["Radius"].GetDouble();

    // Read Pitman arm data
    m_pitmanArmMass = d["Pitman Arm"]["Mass"].GetDouble();
    m_points[PITMANARM] = ReadVectorJSON(d["Pitman Arm"]["COM"]);
    m_pitmanArmInertiaMoments = ReadVectorJSON(d["Pitman Arm"]["Moments of Inertia"]);
    m_pitmanArmInertiaProducts = ReadVectorJSON(d["Pitman Arm"]["Products of Inertia"]);
    m_pitmanArmRadius = d["Pitman Arm"]["Radius"].GetDouble();

    // Read data for the revolute joint (Pitman arm - chassis)
    m_points[REV] = ReadVectorJSON(d["Revolute Joint"]["Location"]);
    m_dirs[REV_AXIS] = ReadVectorJSON(d["Revolute Joint"]["Direction"]);
    m_maxAngle = d["Revolute Joint"]["Maximum Angle"].GetDouble() * (CH_C_PI / 180);

    // Read data for the universal joint (Pitman arm - steering link)
    m_points[UNIV] = ReadVectorJSON(d["Universal Joint"]["Location"]);
    m_dirs[UNIV_AXIS_ARM] = ReadVectorJSON(d["Universal Joint"]["Direction Arm"]);
    m_dirs[UNIV_AXIS_LINK] = ReadVectorJSON(d["Universal Joint"]["Direction Link"]);

    // Read data for the revolute-spherical joint (chassis - steering link)
    m_points[REVSPH_R] = ReadVectorJSON(d["Revolute-Spherical Joint"]["Location Chassis"]);
    m_points[REVSPH_S] = ReadVectorJSON(d["Revolute-Spherical Joint"]["Location Link"]);
    m_dirs[REVSPH_AXIS] = ReadVectorJSON(d["Revolute-Spherical Joint"]["Direction"]);

    // Read data for tierod connection points
    m_points[TIEROD_PA] = ReadVectorJSON(d["Tierod Locations"]["Pitman Side"]);
    m_points[TIEROD_IA] = ReadVectorJSON(d["Tierod Locations"]["Idler Side"]);
}

}  // end namespace vehicle
}  // end namespace chrono
