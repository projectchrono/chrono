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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// RotaryArm steering model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/steering/RotaryArm.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RotaryArm::RotaryArm(const std::string& filename) : ChRotaryArm("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RotaryArm::RotaryArm(const rapidjson::Document& d) : ChRotaryArm("") {
    Create(d);
}

void RotaryArm::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_pitmanArmMass = d["Pitman Arm"]["Mass"].GetDouble();
    m_pitmanArmRadius = d["Pitman Arm"]["Radius"].GetDouble();
    m_pitmanArmInertiaMoments = ReadVectorJSON(d["Pitman Arm"]["Inertia"]);
    m_pitmanArmInertiaProducts = ReadVectorJSON(d["Pitman Arm"]["Inertia Products"]);
    m_points[ARM_C] = ReadVectorJSON(d["Pitman Arm"]["Point of Rotation"]);
    m_points[ARM_L] = ReadVectorJSON(d["Pitman Arm"]["Point to Draglink"]);
    m_dirs[REV_AXIS] = ReadVectorJSON(d["Pitman Arm"]["Axis of Rotation"]);
    m_maxAngle = d["Pitman Arm"]["Maximum Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
}

}  // end namespace vehicle
}  // end namespace chrono
