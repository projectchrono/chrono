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
// Band-bushing track shoe constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandBushing.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackShoeBandBushing::TrackShoeBandBushing(const std::string& filename)
    : ChTrackShoeBandBushing(""), m_has_mesh(false) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackShoeBandBushing::TrackShoeBandBushing(const rapidjson::Document& d)
    : ChTrackShoeBandBushing(""), m_has_mesh(false) {
    Create(d);
}

void TrackShoeBandBushing::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    assert(d.HasMember("Belt Width"));
    m_belt_width = d["Belt Width"].GetDouble();

    assert(d.HasMember("Shoe Height"));
    m_shoe_height = d["Shoe Height"].GetDouble();

    // Read tread body geometry and mass properties
    assert(d.HasMember("Tread"));
    m_tread_mass = d["Tread"]["Mass"].GetDouble();
    m_tread_inertias = ReadVectorJSON(d["Tread"]["Inertia"]);
    m_tread_length = d["Tread"]["Length"].GetDouble();
    m_tread_thickness = d["Tread"]["Thickness"].GetDouble();
    m_tooth_tip_length = d["Tread"]["Tooth Tip Length"].GetDouble();
    m_tooth_base_length = d["Tread"]["Tooth Base Length"].GetDouble();
    m_tooth_width = d["Tread"]["Tooth Width"].GetDouble();
    m_tooth_height = d["Tread"]["Tooth Height"].GetDouble();
    m_tooth_arc_radius = d["Tread"]["Tooth Arc Radius"].GetDouble();

    // Read web geometry and mass properties
    assert(d.HasMember("Web"));
    m_num_web_segments = d["Web"]["Number Segments"].GetInt();
    m_web_mass = d["Web"]["Mass"].GetDouble();
    m_web_inertias = ReadVectorJSON(d["Web"]["Inertia"]);
    m_web_length = d["Web"]["Length"].GetDouble();
    m_web_thickness = d["Web"]["Thickness"].GetDouble();

    // Read guide pin geometry
    assert(d.HasMember("Guide Pin"));
    m_guide_box_dims = ReadVectorJSON(d["Guide Pin"]["Dimensions"]);
    m_guide_box_offset_x = d["Guide Pin"]["Offset"].GetDouble();

    // Read bushing parameters
    assert(d.HasMember("Bushing Parameters"));
    double Klin = d["Bushing Parameters"]["Stiffness Linear"].GetDouble();
    double Krot_dof = d["Bushing Parameters"]["Stiffness Rotational DOF"].GetDouble();
    double Krot_other = d["Bushing Parameters"]["Stiffness Rotational non-DOF"].GetDouble();
    double Dlin = d["Bushing Parameters"]["Damping Linear"].GetDouble();
    double Drot_dof = d["Bushing Parameters"]["Damping Rotational DOF"].GetDouble();
    double Drot_other = d["Bushing Parameters"]["Damping Rotational non-DOF"].GetDouble();
    SetBushingParameters(Klin, Krot_dof, Krot_other, Dlin, Drot_dof, Drot_other);

    // Read contact material information (defer creating the materials until CreateContactMaterials.  Load default
    // values in MatInfo structures (in case not all are provided in the JSON file).
    assert(d.HasMember("Contact Materials"));

    {
        // Material for shoe pad (ground contact)
        m_pad_mat_info.mu = 0.7f;
        m_pad_mat_info.cr = 0.1f;
        m_pad_mat_info.Y = 1e7f;
        m_pad_mat_info.nu = 0.3f;
        m_pad_mat_info.kn = 2e6f;
        m_pad_mat_info.gn = 40.0f;
        m_pad_mat_info.kt = 2e5f;
        m_pad_mat_info.gt = 20.0f;

        assert(d["Contact Materials"].HasMember("Pad Material"));
        const Value& mat = d["Contact Materials"]["Pad Material"];

        m_pad_mat_info.mu = mat["Coefficient of Friction"].GetFloat();
        m_pad_mat_info.cr = mat["Coefficient of Restitution"].GetFloat();
        if (mat.HasMember("Properties")) {
            m_pad_mat_info.Y = mat["Properties"]["Young Modulus"].GetFloat();
            m_pad_mat_info.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
        }
        if (mat.HasMember("Coefficients")) {
            m_pad_mat_info.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
            m_pad_mat_info.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
            m_pad_mat_info.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
            m_pad_mat_info.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
        }
    }

    {
        // Material for shoe body (wheel contact)
        m_body_mat_info.mu = 0.7f;
        m_body_mat_info.cr = 0.1f;
        m_body_mat_info.Y = 1e7f;
        m_body_mat_info.nu = 0.3f;
        m_body_mat_info.kn = 2e6f;
        m_body_mat_info.gn = 40.0f;
        m_body_mat_info.kt = 2e5f;
        m_body_mat_info.gt = 20.0f;

        assert(d["Contact Materials"].HasMember("Body Material"));
        const Value& mat = d["Contact Materials"]["Body Material"];

        m_body_mat_info.mu = mat["Coefficient of Friction"].GetFloat();
        m_body_mat_info.cr = mat["Coefficient of Restitution"].GetFloat();
        if (mat.HasMember("Properties")) {
            m_body_mat_info.Y = mat["Properties"]["Young Modulus"].GetFloat();
            m_body_mat_info.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
        }
        if (mat.HasMember("Coefficients")) {
            m_body_mat_info.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
            m_body_mat_info.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
            m_body_mat_info.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
            m_body_mat_info.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
        }
    }

    {
        // Material for guide (wheel contact)
        m_guide_mat_info.mu = 0.7f;
        m_guide_mat_info.cr = 0.1f;
        m_guide_mat_info.Y = 1e7f;
        m_guide_mat_info.nu = 0.3f;
        m_guide_mat_info.kn = 2e6f;
        m_guide_mat_info.gn = 40.0f;
        m_guide_mat_info.kt = 2e5f;
        m_guide_mat_info.gt = 20.0f;

        assert(d["Contact Materials"].HasMember("Guide Material"));
        const Value& mat = d["Contact Materials"]["Guide Material"];

        m_guide_mat_info.mu = mat["Coefficient of Friction"].GetFloat();
        m_guide_mat_info.cr = mat["Coefficient of Restitution"].GetFloat();
        if (mat.HasMember("Properties")) {
            m_guide_mat_info.Y = mat["Properties"]["Young Modulus"].GetFloat();
            m_guide_mat_info.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
        }
        if (mat.HasMember("Coefficients")) {
            m_guide_mat_info.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
            m_guide_mat_info.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
            m_guide_mat_info.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
            m_guide_mat_info.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
        }
    }

    {
        // Material for teeth (sprocket contact)
        m_tooth_mat_info.mu = 0.7f;
        m_tooth_mat_info.cr = 0.1f;
        m_tooth_mat_info.Y = 1e7f;
        m_tooth_mat_info.nu = 0.3f;
        m_tooth_mat_info.kn = 2e6f;
        m_tooth_mat_info.gn = 40.0f;
        m_tooth_mat_info.kt = 2e5f;
        m_tooth_mat_info.gt = 20.0f;

        assert(d["Contact Materials"].HasMember("Tooth Material"));
        const Value& mat = d["Contact Materials"]["Tooth Material"];

        m_tooth_mat_info.mu = mat["Coefficient of Friction"].GetFloat();
        m_tooth_mat_info.cr = mat["Coefficient of Restitution"].GetFloat();
        if (mat.HasMember("Properties")) {
            m_tooth_mat_info.Y = mat["Properties"]["Young Modulus"].GetFloat();
            m_tooth_mat_info.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
        }
        if (mat.HasMember("Coefficients")) {
            m_tooth_mat_info.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
            m_tooth_mat_info.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
            m_tooth_mat_info.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
            m_tooth_mat_info.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
        }
    }

    // Read wheel visualization
    if (d.HasMember("Visualization")) {
        assert(d["Visualization"].HasMember("Mesh"));
        m_meshFile = d["Visualization"]["Mesh"].GetString();
        m_has_mesh = true;
    }

    // Set name for procedurally-generated tread visualization mesh.
    m_tread_meshName = GetName();
}

void TrackShoeBandBushing::CreateContactMaterials(ChContactMethod contact_method) {
    // Material for shoe pad (ground contact)
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_pad_mat_info.mu);
            matNSC->SetRestitution(m_pad_mat_info.cr);
            m_pad_material = matNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_pad_mat_info.mu);
            matSMC->SetRestitution(m_pad_mat_info.cr);
            matSMC->SetYoungModulus(m_pad_mat_info.Y);
            matSMC->SetPoissonRatio(m_pad_mat_info.nu);
            matSMC->SetKn(m_pad_mat_info.kn);
            matSMC->SetGn(m_pad_mat_info.gn);
            matSMC->SetKt(m_pad_mat_info.kt);
            matSMC->SetGt(m_pad_mat_info.gt);
            m_pad_material = matSMC;
            break;
        }
    }

    // Material for shoe body (wheel contact)
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_body_mat_info.mu);
            matNSC->SetRestitution(m_body_mat_info.cr);
            m_body_material = matNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_body_mat_info.mu);
            matSMC->SetRestitution(m_body_mat_info.cr);
            matSMC->SetYoungModulus(m_body_mat_info.Y);
            matSMC->SetPoissonRatio(m_body_mat_info.nu);
            matSMC->SetKn(m_body_mat_info.kn);
            matSMC->SetGn(m_body_mat_info.gn);
            matSMC->SetKt(m_body_mat_info.kt);
            matSMC->SetGt(m_body_mat_info.gt);
            m_body_material = matSMC;
            break;
        }
    }

    // Material for guide (wheel contact)
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_guide_mat_info.mu);
            matNSC->SetRestitution(m_guide_mat_info.cr);
            m_guide_material = matNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_guide_mat_info.mu);
            matSMC->SetRestitution(m_guide_mat_info.cr);
            matSMC->SetYoungModulus(m_guide_mat_info.Y);
            matSMC->SetPoissonRatio(m_guide_mat_info.nu);
            matSMC->SetKn(m_guide_mat_info.kn);
            matSMC->SetGn(m_guide_mat_info.gn);
            matSMC->SetKt(m_guide_mat_info.kt);
            matSMC->SetGt(m_guide_mat_info.gt);
            m_guide_material = matSMC;
            break;
        }
    }

    // Material for teeth (sprocket contact)
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_tooth_mat_info.mu);
            matNSC->SetRestitution(m_tooth_mat_info.cr);
            m_tooth_material = matNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_tooth_mat_info.mu);
            matSMC->SetRestitution(m_tooth_mat_info.cr);
            matSMC->SetYoungModulus(m_tooth_mat_info.Y);
            matSMC->SetPoissonRatio(m_tooth_mat_info.nu);
            matSMC->SetKn(m_tooth_mat_info.kn);
            matSMC->SetGn(m_tooth_mat_info.gn);
            matSMC->SetKt(m_tooth_mat_info.kt);
            matSMC->SetGt(m_tooth_mat_info.gt);
            m_tooth_material = matSMC;
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackShoeBandBushing::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        trimesh_shape->SetStatic(true);
        m_shoe->AddAsset(trimesh_shape);
    } else {
        ChTrackShoeBandBushing::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
