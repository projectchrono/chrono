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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Multi-link suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a multi-link suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
MultiLink::MultiLink(const std::string& filename) : ChMultiLink(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MultiLink::MultiLink(const rapidjson::Document& d) : ChMultiLink(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

MultiLink::~MultiLink() {}

// -----------------------------------------------------------------------------
// Worker function for creating a MultiLink suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void MultiLink::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    if (d.HasMember("Camber Angle (deg)"))
        m_camber_angle = d["Camber Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    else
        m_camber_angle = 0;

    if (d.HasMember("Toe Angle (deg)"))
        m_toe_angle = d["Toe Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    else
        m_toe_angle = 0;

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
    m_uprightInertia = ReadVectorJSON(d["Upright"]["Inertia"]);
    m_uprightRadius = d["Upright"]["Radius"].GetDouble();

    // Read Upper Arm data
    assert(d.HasMember("Upper Arm"));
    assert(d["Upper Arm"].IsObject());

    m_upperArmMass = d["Upper Arm"]["Mass"].GetDouble();
    m_points[UA_CM] = ReadVectorJSON(d["Upper Arm"]["COM"]);
    m_upperArmInertia = ReadVectorJSON(d["Upper Arm"]["Inertia"]);
    m_upperArmRadius = d["Upper Arm"]["Radius"].GetDouble();
    m_points[UA_F] = ReadVectorJSON(d["Upper Arm"]["Location Chassis Front"]);
    m_points[UA_B] = ReadVectorJSON(d["Upper Arm"]["Location Chassis Back"]);
    m_points[UA_U] = ReadVectorJSON(d["Upper Arm"]["Location Upright"]);

    // Read Lateral data
    assert(d.HasMember("Lateral"));
    assert(d["Lateral"].IsObject());

    m_lateralMass = d["Lateral"]["Mass"].GetDouble();
    m_points[LAT_CM] = ReadVectorJSON(d["Lateral"]["COM"]);
    m_lateralInertia = ReadVectorJSON(d["Lateral"]["Inertia"]);
    m_lateralRadius = d["Lateral"]["Radius"].GetDouble();
    m_points[LAT_C] = ReadVectorJSON(d["Lateral"]["Location Chassis"]);
    m_points[LAT_U] = ReadVectorJSON(d["Lateral"]["Location Upright"]);
    m_directions[UNIV_AXIS_LINK_LAT] = ReadVectorJSON(d["Lateral"]["Universal Joint Axis Link"]);
    m_directions[UNIV_AXIS_CHASSIS_LAT] = ReadVectorJSON(d["Lateral"]["Universal Joint Axis Chassis"]);

    // Read Trailing Link data
    assert(d.HasMember("Trailing Link"));
    assert(d["Trailing Link"].IsObject());

    m_trailingLinkMass = d["Trailing Link"]["Mass"].GetDouble();
    m_points[TL_CM] = ReadVectorJSON(d["Trailing Link"]["COM"]);
    m_trailingLinkInertia = ReadVectorJSON(d["Trailing Link"]["Inertia"]);
    m_trailingLinkRadius = d["Trailing Link"]["Radius"].GetDouble();
    m_points[TL_C] = ReadVectorJSON(d["Trailing Link"]["Location Chassis"]);
    m_points[TL_U] = ReadVectorJSON(d["Trailing Link"]["Location Upright"]);
    m_directions[UNIV_AXIS_LINK_TL] = ReadVectorJSON(d["Trailing Link"]["Universal Joint Axis Link"]);
    m_directions[UNIV_AXIS_CHASSIS_TL] = ReadVectorJSON(d["Trailing Link"]["Universal Joint Axis Chassis"]);

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    if (d["Tierod"].HasMember("Mass")) {
        assert(d["Tierod"].HasMember("Inertia"));
        assert(d["Tierod"].HasMember("Radius"));
        m_tierodMass = d["Tierod"]["Mass"].GetDouble();
        m_tierodRadius = d["Tierod"]["Radius"].GetDouble();
        m_tierodInertia = ReadVectorJSON(d["Tierod"]["Inertia"]);
        m_use_tierod_bodies = true;
        if (d["Tierod"].HasMember("Bushing Data")) {
            m_tierodBushingData = ReadBushingDataJSON(d["Tierod"]["Bushing Data"]);
        }
    } else {
        m_tierodMass = 0;
        m_tierodRadius = 0;
        m_tierodInertia = ChVector<>(0);
        m_use_tierod_bodies = false;
    }

    m_points[TIEROD_C] = ReadVectorJSON(d["Tierod"]["Location Chassis"]);
    m_points[TIEROD_U] = ReadVectorJSON(d["Tierod"]["Location Upright"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_L] = ReadVectorJSON(d["Spring"]["Location Link"]);
    m_springRestLength = d["Spring"]["Free Length"].GetDouble();
    double preload = 0;
    if (d["Spring"].HasMember("Preload"))
        preload = d["Spring"]["Preload"].GetDouble();

    if (d["Spring"].HasMember("Minimum Length") && d["Spring"].HasMember("Maximum Length")) {
        // Use bump stops
        if (d["Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB = chrono_types::make_shared<LinearSpringBistopForce>(
                d["Spring"]["Spring Coefficient"].GetDouble(), d["Spring"]["Minimum Length"].GetDouble(),
                d["Spring"]["Maximum Length"].GetDouble(), preload);
        } else if (d["Spring"].HasMember("Curve Data")) {
            int num_points = d["Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringBistopForce>(
                d["Spring"]["Minimum Length"].GetDouble(), d["Spring"]["Maximum Length"].GetDouble(), preload);
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Spring"]["Curve Data"][i][1u].GetDouble());
            }
            m_springForceCB = springForceCB;
        }
    } else {
        // No bump stops
        if (d["Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB =
                chrono_types::make_shared<LinearSpringForce>(d["Spring"]["Spring Coefficient"].GetDouble(), preload);
        } else if (d["Spring"].HasMember("Curve Data")) {
            int num_points = d["Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringForce>(preload);
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Spring"]["Curve Data"][i][1u].GetDouble());
            }
            m_springForceCB = springForceCB;
        }
    }

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());

    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_L] = ReadVectorJSON(d["Shock"]["Location Link"]);

    if (d["Shock"].HasMember("Damping Coefficient")) {
        m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(d["Shock"]["Damping Coefficient"].GetDouble());
    } else if (d["Shock"].HasMember("Curve Data")) {
        int num_points = d["Shock"]["Curve Data"].Size();
        auto shockForceCB = chrono_types::make_shared<MapDamperForce>();
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
