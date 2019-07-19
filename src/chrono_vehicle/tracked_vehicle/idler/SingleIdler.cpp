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
// Single idler model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/idler/SingleIdler.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SingleIdler::SingleIdler(const std::string& filename) :ChSingleIdler(""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SingleIdler::SingleIdler(const rapidjson::Document& d) : ChSingleIdler(""), m_has_mesh(false) {
    Create(d);
}

void SingleIdler::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read wheel geometry and mass properties
    assert(d.HasMember("Wheel"));
    m_wheel_radius = d["Wheel"]["Radius"].GetDouble();
    m_wheel_width = d["Wheel"]["Width"].GetDouble();
    m_wheel_mass = d["Wheel"]["Mass"].GetDouble();
    m_wheel_inertia = ReadVectorJSON(d["Wheel"]["Inertia"]);
    m_points[WHEEL] = ReadVectorJSON(d["Wheel"]["COM"]);

    // Read carrier geometry and mass properties
    assert(d.HasMember("Carrier"));
    m_carrier_mass = d["Carrier"]["Mass"].GetDouble();
    m_carrier_inertia = ReadVectorJSON(d["Carrier"]["Inertia"]);
    m_points[CARRIER] = ReadVectorJSON(d["Carrier"]["COM"]);
    m_points[CARRIER_CHASSIS] = ReadVectorJSON(d["Carrier"]["Location Chassis"]);
    m_carrier_vis_radius = d["Carrier"]["Visualization Radius"].GetDouble();
    m_pitch_angle = d["Carrier"]["Pitch Angle"].GetDouble();

    // Read tensioner data
    assert(d.HasMember("Tensioner"));
    m_points[TSDA_CARRIER] = ReadVectorJSON(d["Tensioner"]["Location Carrier"]);
    m_points[TSDA_CHASSIS] = ReadVectorJSON(d["Tensioner"]["Location Chassis"]);
    m_tensioner_l0 = d["Tensioner"]["Free Length"].GetDouble();
    double tensioner_f = d["Tensioner"]["Preload"].GetDouble();
    if (d["Tensioner"].HasMember("Spring Coefficient")) {
        // Linear spring-damper
        double tensioner_k = d["Tensioner"]["Spring Coefficient"].GetDouble();
        double tensioner_c = d["Tensioner"]["Damping Coefficient"].GetDouble();
        m_tensionerForceCB = new LinearSpringDamperActuatorForce(tensioner_k, tensioner_c, tensioner_f);
    } else if (d["Tensioner"].HasMember("Spring Curve Data")) {
        // Nonlinear (curves) spring-damper
        int num_pointsK = d["Tensioner"]["Spring Curve Data"].Size();
        int num_pointsC = d["Tensioner"]["Damper Curve Data"].Size();
        MapSpringDamperActuatorForce* tensionerForceCB = new MapSpringDamperActuatorForce();
        for (int i = 0; i < num_pointsK; i++) {
            tensionerForceCB->add_pointK(d["Tensioner"]["Spring Curve Data"][i][0u].GetDouble(),
                d["Tensioner"]["Spring Curve Data"][i][1u].GetDouble());
        }
        for (int i = 0; i < num_pointsC; i++) {
            tensionerForceCB->add_pointC(d["Tensioner"]["Damper Curve Data"][i][0u].GetDouble(),
                d["Tensioner"]["Damper Curve Data"][i][1u].GetDouble());
        }
        tensionerForceCB->set_f(tensioner_f);
        m_tensionerForceCB = tensionerForceCB;
    }

    // Read contact material data
    assert(d.HasMember("Contact Material"));

    float mu = d["Contact Material"]["Coefficient of Friction"].GetFloat();
    float cr = d["Contact Material"]["Coefficient of Restitution"].GetFloat();

    SetContactFrictionCoefficient(mu);
    SetContactRestitutionCoefficient(cr);

    if (d["Contact Material"].HasMember("Properties")) {
        float ym = d["Contact Material"]["Properties"]["Young Modulus"].GetFloat();
        float pr = d["Contact Material"]["Properties"]["Poisson Ratio"].GetFloat();
        //SetContactMaterialProperties(ym, pr);
    }
    if (d["Contact Material"].HasMember("Coefficients")) {
        float kn = d["Contact Material"]["Coefficients"]["Normal Stiffness"].GetFloat();
        float gn = d["Contact Material"]["Coefficients"]["Normal Damping"].GetFloat();
        float kt = d["Contact Material"]["Coefficients"]["Tangential Stiffness"].GetFloat();
        float gt = d["Contact Material"]["Coefficients"]["Tangential Damping"].GetFloat();
        SetContactMaterialCoefficients(kn, gn, kt, gt);
    }

    // Read wheel visualization
    if (d.HasMember("Visualization")) {
        assert(d["Visualization"].HasMember("Mesh Filename"));
        assert(d["Visualization"].HasMember("Mesh Name"));
        m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
        m_meshName = d["Visualization"]["Mesh Name"].GetString();
        m_has_mesh = true;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SingleIdler::AddVisualizationAssets(VisualizationType vis) {
    ChSingleIdler::AddVisualizationAssets(vis);

    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(trimesh_shape);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
