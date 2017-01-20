// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// FEA co-rotational tire constructed with data from file (JSON format).
// The mesh data is assumed to be provided through an Abaqus INP file.
//
// =============================================================================

#include "chrono_fea/ChMeshFileLoader.h"

#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace chrono::fea;
using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

// -----------------------------------------------------------------------------
// Constructors for FEATire
// -----------------------------------------------------------------------------
FEATire::FEATire(const std::string& filename) : ChFEATire("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    ProcessJSON(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

FEATire::FEATire(const rapidjson::Document& d) : ChFEATire("") {
    ProcessJSON(d);
}

// -----------------------------------------------------------------------------
// Process the specified JSON document and load tire specification
// -----------------------------------------------------------------------------
void FEATire::ProcessJSON(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());
    
    // Read geometric dimensions
    m_tire_radius = d["Tire Radius"].GetDouble();
    m_rim_radius = d["Rim Radius"].GetDouble();
    m_rim_width = d["Rim Width"].GetDouble();

    // Read contact material data
    assert(d.HasMember("Contact Material"));

    float mu = d["Contact Material"]["Coefficient of Friction"].GetFloat();
    float cr = d["Contact Material"]["Coefficient of Restitution"].GetFloat();

    SetContactFrictionCoefficient(mu);
    SetContactRestitutionCoefficient(cr);

    if (d["Contact Material"].HasMember("Properties")) {
        float ym = d["Contact Material"]["Properties"]["Young Modulus"].GetFloat();
        float pr = d["Contact Material"]["Properties"]["Poisson Ratio"].GetFloat();
        SetContactMaterialProperties(ym, pr);
    }
    if (d["Contact Material"].HasMember("Coefficients")) {
        float kn = d["Contact Material"]["Coefficients"]["Normal Stiffness"].GetFloat();
        float gn = d["Contact Material"]["Coefficients"]["Normal Damping"].GetFloat();
        float kt = d["Contact Material"]["Coefficients"]["Tangential Stiffness"].GetFloat();
        float gt = d["Contact Material"]["Coefficients"]["Tangential Damping"].GetFloat();
        SetContactMaterialCoefficients(kn, gn, kt, gt);
    }

    // Read continuum material data
    double E = d["Continuum Material"]["Elasticity Modulus"].GetDouble();
    double nu = d["Continuum Material"]["Poisson Ratio"].GetDouble();
    double rd = d["Continuum Material"]["Rayleigh Damping"].GetDouble();
    double density = d["Continuum Material"]["Density"].GetDouble();

    m_material = std::make_shared<ChContinuumElastic>();
    m_material->Set_E(E);
    m_material->Set_v(nu);
    m_material->Set_RayleighDampingK(rd);
    m_material->Set_density(density);

    // Default tire pressure
    m_default_pressure = d["Default Pressure"].GetDouble();

    // Name of Abaqus input file
    m_input_file = d["Abaqus Mesh Filename"].GetString();
}

// -----------------------------------------------------------------------------
// Create the FEA mesh
// -----------------------------------------------------------------------------
void FEATire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    //// TODO:
    ////    Currently, we assume that the INP file contains a tire with rotation axis along X
    ChMeshFileLoader::FromAbaqusFile(m_mesh, GetDataFile(m_input_file).c_str(), m_material, m_node_sets,
                                     wheel_frame.GetPos(),
                                     wheel_frame.GetA() * ChMatrix33<>(CH_C_PI_2, ChVector<>(0, 0, 1)));

    for (unsigned int i = 0; i < m_mesh->GetNnodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(i));
        // Node position (expressed in wheel frame)
        ChVector<> loc = wheel_frame.TransformPointParentToLocal(node->GetPos());
        // Node velocity (expressed in absolute frame)
        ChVector<> vel = wheel_frame.PointSpeedLocalToParent(loc);
        node->SetPos_dt(vel);
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> FEATire::GetInternalNodes() const {
    return m_node_sets[0];
}

std::vector<std::shared_ptr<fea::ChNodeFEAbase>> FEATire::GetConnectedNodes() const {
    return m_node_sets[1];
}

}  // end namespace vehicle
}  // end namespace chrono
