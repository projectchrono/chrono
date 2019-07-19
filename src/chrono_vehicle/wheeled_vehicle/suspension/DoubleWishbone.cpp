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
// Double-A arm suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a double wishbone suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
DoubleWishbone::DoubleWishbone(const std::string& filename)
    : ChDoubleWishbone(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

DoubleWishbone::DoubleWishbone(const rapidjson::Document& d)
    : ChDoubleWishbone(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DoubleWishbone::~DoubleWishbone() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a DoubleWishbone suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void DoubleWishbone::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read flag indicating that inertia matrices are expressed in
    // vehicle-aligned centroidal frame.
    if (d.HasMember("Vehicle-Frame Inertia")) {
        bool flag = d["Vehicle-Frame Inertia"].GetBool();
        SetVehicleFrameInertiaFlag(flag);
    }

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read Upright data
    assert(d.HasMember("Upright"));
    assert(d["Upright"].IsObject());

    m_uprightMass = d["Upright"]["Mass"].GetDouble();
    m_points[UPRIGHT] = ReadVectorJSON(d["Upright"]["COM"]);
    m_uprightInertiaMoments = ReadVectorJSON(d["Upright"]["Moments of Inertia"]);
    m_uprightInertiaProducts = ReadVectorJSON(d["Upright"]["Products of Inertia"]);
    m_uprightRadius = d["Upright"]["Radius"].GetDouble();

    // Read UCA data
    assert(d.HasMember("Upper Control Arm"));
    assert(d["Upper Control Arm"].IsObject());

    m_UCAMass = d["Upper Control Arm"]["Mass"].GetDouble();
    m_points[UCA_CM] = ReadVectorJSON(d["Upper Control Arm"]["COM"]);
    m_UCAInertiaMoments = ReadVectorJSON(d["Upper Control Arm"]["Moments of Inertia"]);
    m_UCAInertiaProducts = ReadVectorJSON(d["Upper Control Arm"]["Products of Inertia"]);
    m_UCARadius = d["Upper Control Arm"]["Radius"].GetDouble();
    m_points[UCA_F] = ReadVectorJSON(d["Upper Control Arm"]["Location Chassis Front"]);
    m_points[UCA_B] = ReadVectorJSON(d["Upper Control Arm"]["Location Chassis Back"]);
    m_points[UCA_U] = ReadVectorJSON(d["Upper Control Arm"]["Location Upright"]);

    // Read LCA data
    assert(d.HasMember("Lower Control Arm"));
    assert(d["Lower Control Arm"].IsObject());

    m_LCAMass = d["Lower Control Arm"]["Mass"].GetDouble();
    m_points[LCA_CM] = ReadVectorJSON(d["Lower Control Arm"]["COM"]);
    m_LCAInertiaMoments = ReadVectorJSON(d["Lower Control Arm"]["Moments of Inertia"]);
    m_LCAInertiaProducts = ReadVectorJSON(d["Lower Control Arm"]["Products of Inertia"]);
    m_LCARadius = d["Lower Control Arm"]["Radius"].GetDouble();
    m_points[LCA_F] = ReadVectorJSON(d["Lower Control Arm"]["Location Chassis Front"]);
    m_points[LCA_B] = ReadVectorJSON(d["Lower Control Arm"]["Location Chassis Back"]);
    m_points[LCA_U] = ReadVectorJSON(d["Lower Control Arm"]["Location Upright"]);

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    m_points[TIEROD_C] = ReadVectorJSON(d["Tierod"]["Location Chassis"]);
    m_points[TIEROD_U] = ReadVectorJSON(d["Tierod"]["Location Upright"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Arm"]);
    m_springRestLength = d["Spring"]["Free Length"].GetDouble();

    if (d["Spring"].HasMember("Spring Coefficient")) {
        m_springForceCB = new LinearSpringForce(d["Spring"]["Spring Coefficient"].GetDouble());
    } else if (d["Spring"].HasMember("Curve Data")) {
        int num_points = d["Spring"]["Curve Data"].Size();
        MapSpringForce* springForceCB = new MapSpringForce();
        for (int i = 0; i < num_points; i++) {
            springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                     d["Spring"]["Curve Data"][i][1u].GetDouble());
        }
        m_springForceCB = springForceCB;
    }

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());

    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Arm"]);

    if (d["Shock"].HasMember("Damping Coefficient")) {
        m_shockForceCB = new LinearDamperForce(d["Shock"]["Damping Coefficient"].GetDouble());
    } else if (d["Shock"].HasMember("Curve Data")) {
        int num_points = d["Shock"]["Curve Data"].Size();
        MapDamperForce* shockForceCB = new MapDamperForce();
        for (int i = 0; i < num_points; i++) {
            shockForceCB->add_point(d["Shock"]["Curve Data"][i][0u].GetDouble(),
                                    d["Shock"]["Curve Data"][i][1u].GetDouble());
        }
        m_shockForceCB = shockForceCB;
    }

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());

    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
