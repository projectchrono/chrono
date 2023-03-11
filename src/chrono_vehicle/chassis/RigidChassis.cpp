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
// Vehicle rigid chassis model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

RigidChassis::RigidChassis(const std::string& filename) : ChRigidChassis("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidChassis::RigidChassis(const rapidjson::Document& d) : ChRigidChassis("") {
    Create(d);
}

void RigidChassis::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read inertia properties for all sub-components
    // and calculate composite inertia properties
    assert(d.HasMember("Components"));
    assert(d["Components"].IsArray());
    int num_comp = d["Components"].Size();

    utils::CompositeInertia composite;

    for (int i = 0; i < num_comp; i++) {
        const Value& comp = d["Components"][i];
        ChVector<> loc = ReadVectorJSON(comp["Centroidal Frame"]["Location"]);
        ChQuaternion<> rot = ReadQuaternionJSON(comp["Centroidal Frame"]["Orientation"]);
        double mass = comp["Mass"].GetDouble();
        ChVector<> inertiaXX = ReadVectorJSON(comp["Moments of Inertia"]);
        ChVector<> inertiaXY = ReadVectorJSON(comp["Products of Inertia"]);
        bool is_void = comp["Void"].GetBool();

        ChMatrix33<> inertia(inertiaXX, inertiaXY);
        composite.AddComponent(ChFrame<>(loc, rot), mass, inertia, is_void);
    }

    m_body_mass = composite.GetMass();
    m_body_inertia = composite.GetInertia();
    m_body_COM_loc = composite.GetCOM();

    // Extract driver position
    m_driverCsys.pos = ReadVectorJSON(d["Driver Position"]["Location"]);
    m_driverCsys.rot = ReadQuaternionJSON(d["Driver Position"]["Orientation"]);

    // Extract location of connector to a rear chassis (if present)
    if (d.HasMember("Rear Connector Location")) {
        m_connector_rear_loc = ReadVectorJSON(d["Rear Connector Location"]);
    } else {
        m_connector_rear_loc = ChVector<>(0, 0, 0);
    }

    // Read contact and visualization data.
    m_geometry = ReadVehicleGeometryJSON(d);
}

// -----------------------------------------------------------------------------

RigidChassisRear::RigidChassisRear(const std::string& filename) : ChRigidChassisRear("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidChassisRear::RigidChassisRear(const rapidjson::Document& d) : ChRigidChassisRear("") {
    Create(d);
}

void RigidChassisRear::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read inertia properties for all sub-components
    // and calculate composite inertia properties
    assert(d.HasMember("Components"));
    assert(d["Components"].IsArray());
    int num_comp = d["Components"].Size();

    utils::CompositeInertia composite;

    for (int i = 0; i < num_comp; i++) {
        const Value& comp = d["Components"][i];
        ChVector<> loc = ReadVectorJSON(comp["Centroidal Frame"]["Location"]);
        ChQuaternion<> rot = ReadQuaternionJSON(comp["Centroidal Frame"]["Orientation"]);
        double mass = comp["Mass"].GetDouble();
        ChVector<> inertiaXX = ReadVectorJSON(comp["Moments of Inertia"]);
        ChVector<> inertiaXY = ReadVectorJSON(comp["Products of Inertia"]);
        bool is_void = comp["Void"].GetBool();

        ChMatrix33<> inertia(inertiaXX, inertiaXY);
        composite.AddComponent(ChFrame<>(loc, rot), mass, inertia, is_void);
    }

    m_body_mass = composite.GetMass();
    m_body_inertia = composite.GetInertia();
    m_body_COM_loc = composite.GetCOM();

    // Extract location of connector to the front chassis
    assert(d.HasMember("Front Connector Location"));
    m_connector_front_loc = ReadVectorJSON(d["Front Connector Location"]);

    // Extract location of connector to a rear chassis (if present)
    if (d.HasMember("Rear Connector Location")) {
        m_connector_rear_loc = ReadVectorJSON(d["Rear Connector Location"]);
    } else {
        m_connector_rear_loc = ChVector<>(0, 0, 0);
    }

    // Read contact and visualization data.
    m_geometry = ReadVehicleGeometryJSON(d);
}

}  // end namespace vehicle
}  // end namespace chrono
