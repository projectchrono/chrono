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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Hendrickson PRIMAXX suspension constructed with data from file.
//
// TODO: properly specify universal joint axes
//       (when base class ChHendricksonPRIMAXX is completed)
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/HendricksonPRIMAXX.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a double wishbone suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
HendricksonPRIMAXX::HendricksonPRIMAXX(const std::string& filename) : ChHendricksonPRIMAXX("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

HendricksonPRIMAXX::HendricksonPRIMAXX(const rapidjson::Document& d) : ChHendricksonPRIMAXX("") {
    Create(d);
}

HendricksonPRIMAXX::~HendricksonPRIMAXX() {}

// -----------------------------------------------------------------------------
// Worker function for creating a HendricksonPRIMAXX suspension using data in
// the specified RapidJSON document.
// -----------------------------------------------------------------------------
void HendricksonPRIMAXX::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read Axle Housing data
    assert(d.HasMember("Axle Housing"));
    assert(d["Axle Housing"].IsObject());

    m_axlehousingMass = d["Axle Housing"]["Mass"].GetDouble();
    m_axlehousingCOM = ReadVectorJSON(d["Axle Housing"]["COM"]);
    m_axlehousingInertia = ReadVectorJSON(d["Axle Housing"]["Inertia"]);
    m_axlehousingRadius = d["Axle Housing"]["Radius"].GetDouble();

    // Read Transverse Beam data
    assert(d.HasMember("Transverse Beam"));
    assert(d["Transverse Beam"].IsObject());

    m_transversebeamMass = d["Transverse Beam"]["Mass"].GetDouble();
    m_transversebeamCOM = ReadVectorJSON(d["Transverse Beam"]["COM"]);
    m_transversebeamInertia = ReadVectorJSON(d["Transverse Beam"]["Inertia"]);
    m_transversebeamRadius = d["Transverse Beam"]["Radius"].GetDouble();

    // Read Lower Beam data
    assert(d.HasMember("Lower Beam"));
    assert(d["Lower Beam"].IsObject());

    m_lowerbeamMass = d["Lower Beam"]["Mass"].GetDouble();
    m_points[LOWERBEAM_CM] = ReadVectorJSON(d["Lower Beam"]["COM"]);
    m_lowerbeamInertia = ReadVectorJSON(d["Lower Beam"]["Inertia"]);
    m_lowerbeamRadius = d["Lower Beam"]["Radius"].GetDouble();
    m_points[LOWERBEAM_C] = ReadVectorJSON(d["Lower Beam"]["Chassis Attachment Point"]);
    m_points[LOWERBEAM_AH] = ReadVectorJSON(d["Lower Beam"]["Axle Housing Attachment Point"]);
    m_points[LOWERBEAM_TB] = ReadVectorJSON(d["Lower Beam"]["Transverse Beam Attachment Point"]);

    // Read Torque Rod data
    assert(d.HasMember("Torque Rod"));
    assert(d["Torque Rod"].IsObject());

    m_torquerodMass = d["Torque Rod"]["Mass"].GetDouble();
    m_points[TORQUEROD_CM] = ReadVectorJSON(d["Torque Rod"]["COM"]);
    m_torquerodInertia = ReadVectorJSON(d["Torque Rod"]["Inertia"]);
    m_torquerodRadius = d["Torque Rod"]["Radius"].GetDouble();
    m_points[TORQUEROD_C] = ReadVectorJSON(d["Torque Rod"]["Chassis Attachment Point"]);
    m_points[TORQUEROD_AH] = ReadVectorJSON(d["Torque Rod"]["Axle Housing Attachment Point"]);

    // Read Knuckle data
    assert(d.HasMember("Knuckle"));
    assert(d["Knuckle"].IsObject());

    m_knuckleMass = d["Knuckle"]["Mass"].GetDouble();
    m_points[KNUCKLE_CM] = ReadVectorJSON(d["Knuckle"]["COM"]);
    m_knuckleInertia = ReadVectorJSON(d["Knuckle"]["Inertia"]);
    m_knuckleRadius = d["Knuckle"]["Radius"].GetDouble();
    m_points[KNUCKLE_L] = ReadVectorJSON(d["Knuckle"]["Lower Attachment Point"]);
    m_points[KNUCKLE_U] = ReadVectorJSON(d["Knuckle"]["Upper Attachment Point"]);

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
    m_points[TIEROD_K] = ReadVectorJSON(d["Tierod"]["Location Knuckle"]);

    // Read spring/shock data and create force callbacks
    //// TODO:  add support for map-based spring-damper
    ////        add support for bump stops
    assert(d.HasMember("Shock Axle Housing"));
    assert(d["Shock Axle Housing"].IsObject());
    m_points[SHOCKAH_C] = ReadVectorJSON(d["Shock Axle Housing"]["Location Chassis"]);
    m_points[SHOCKAH_AH] = ReadVectorJSON(d["Shock Axle Housing"]["Location Axle Housing"]);
    m_shockAHForceCB = ReadTSDAFunctorJSON(d["Shock Axle Housing"], m_shockAH_restLength);

    assert(d.HasMember("Shock Lower Beam"));
    assert(d["Shock Lower Beam"].IsObject());
    m_points[SHOCKLB_C] = ReadVectorJSON(d["Shock Lower Beam"]["Location Chassis"]);
    m_points[SHOCKLB_LB] = ReadVectorJSON(d["Shock Lower Beam"]["Location Lower Beam"]);
    m_shockLBForceCB = ReadTSDAFunctorJSON(d["Shock Lower Beam"], m_shockLB_restLength);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();

    //// TODO
    m_dirs[UNIV_AXIS_TORQUEROD_ROD] = ChVector<>(1, 0, 0);
    m_dirs[UNIV_AXIS_TORQUEROD_CHASSIS] = ChVector<>(1, 0, 0);
    m_dirs[UNIV_AXIS_LOWERBEAM_BEAM] = ChVector<>(1, 0, 0);
    m_dirs[UNIV_AXIS_LOWERBEAM_CHASSIS] = ChVector<>(1, 0, 0);
}

}  // end namespace vehicle
}  // end namespace chrono
