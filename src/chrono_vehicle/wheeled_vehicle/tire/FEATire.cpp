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
// FEA co-rotational tire constructed with data from file (JSON format).
// The mesh data is assumed to be provided through an Abaqus INP file.
//
// =============================================================================

#include "chrono/fea/ChMeshFileLoader.h"

#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono::fea;
using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constructors for FEATire
// -----------------------------------------------------------------------------
FEATire::FEATire(const std::string& filename) : ChFEATire("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    ProcessJSON(d);

    std::cout << "Loaded JSON " << filename << std::endl;
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
    m_mat_info = ReadMaterialInfoJSON(d["Contact Material"]);

    // Read continuum material data
    double E = d["Continuum Material"]["Elasticity Modulus"].GetDouble();
    double nu = d["Continuum Material"]["Poisson Ratio"].GetDouble();
    double rd = d["Continuum Material"]["Rayleigh Damping"].GetDouble();
    double density = d["Continuum Material"]["Density"].GetDouble();

    m_material = chrono_types::make_shared<ChContinuumElastic>();
    m_material->SetYoungModulus(E);
    m_material->SetPoissonRatio(nu);
    m_material->SetRayleighDampingBeta(rd);
    m_material->SetDensity(density);

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
    ChMeshFileLoader::FromAbaqusFile(m_mesh, GetVehicleDataFile(m_input_file).c_str(), m_material, m_node_sets,
                                     wheel_frame.GetPos(),
                                     wheel_frame.GetRotMat() * ChMatrix33<>(CH_PI_2, ChVector3d(0, 0, 1)));

    for (unsigned int i = 0; i < m_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(i));
        // Node position (expressed in wheel frame)
        ChVector3d loc = wheel_frame.TransformPointParentToLocal(node->GetPos());
        // Node velocity (expressed in absolute frame)
        ChVector3d vel = wheel_frame.PointSpeedLocalToParent(loc);
        node->SetPosDt(vel);
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> FEATire::GetInternalNodes() const {
    return m_node_sets.at("BC_INT");
}

std::vector<std::shared_ptr<fea::ChNodeFEAbase>> FEATire::GetConnectedNodes() const {
    return m_node_sets.at("BC_CONN");
}

void FEATire::CreateContactMaterial() {
    m_contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    m_contact_mat->SetFriction(m_mat_info.mu);
    m_contact_mat->SetRestitution(m_mat_info.cr);
    m_contact_mat->SetYoungModulus(m_mat_info.Y);
    m_contact_mat->SetPoissonRatio(m_mat_info.nu);
    m_contact_mat->SetKn(m_mat_info.kn);
    m_contact_mat->SetGn(m_mat_info.gn);
    m_contact_mat->SetKt(m_mat_info.kt);
    m_contact_mat->SetGt(m_mat_info.gt);
}

}  // end namespace vehicle
}  // end namespace chrono
