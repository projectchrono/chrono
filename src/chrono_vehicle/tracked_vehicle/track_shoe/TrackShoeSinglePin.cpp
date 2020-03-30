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
// Single-pin track shoe constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeSinglePin.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackShoeSinglePin::TrackShoeSinglePin(const std::string& filename) : ChTrackShoeSinglePin(""), m_has_mesh(false) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackShoeSinglePin::TrackShoeSinglePin(const rapidjson::Document& d) : ChTrackShoeSinglePin(""), m_has_mesh(false) {
    Create(d);
}

void TrackShoeSinglePin::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read shoe body geometry and mass properties

    assert(d.HasMember("Shoe"));
    m_shoe_height = d["Shoe"]["Height"].GetDouble();
    m_shoe_pitch = d["Shoe"]["Pitch"].GetDouble();
    m_shoe_mass = d["Shoe"]["Mass"].GetDouble();
    m_shoe_inertia = ReadVectorJSON(d["Shoe"]["Inertia"]);

    // Read contact data

    assert(d.HasMember("Contact"));
    assert(d["Contact"].HasMember("Cylinder Material"));
    assert(d["Contact"].HasMember("Shoe Materials"));
    assert(d["Contact"].HasMember("Cylinder Shape"));    
    assert(d["Contact"].HasMember("Shoe Shapes"));

    // Read contact material information (defer creating the materials until CreateCylContactMaterial and
    // CreateShoeContactMaterials).  Load default values in MatInfo structures (in case not all are provided in the
    // JSON file).

    m_cyl_mat_info.mu = 0.7f;
    m_cyl_mat_info.cr = 0.1f;
    m_cyl_mat_info.Y = 1e7f;
    m_cyl_mat_info.nu = 0.3f;
    m_cyl_mat_info.kn = 2e6f;
    m_cyl_mat_info.gn = 40.0f;
    m_cyl_mat_info.kt = 2e5f;
    m_cyl_mat_info.gt = 20.0f;

    const Value& mat = d["Contact"]["Cylinder Material"];

    m_cyl_mat_info.mu = mat["Coefficient of Friction"].GetFloat();
    m_cyl_mat_info.cr = mat["Coefficient of Restitution"].GetFloat();
    if (mat.HasMember("Properties")) {
        m_cyl_mat_info.Y = mat["Properties"]["Young Modulus"].GetFloat();
        m_cyl_mat_info.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
    }
    if (mat.HasMember("Coefficients")) {
        m_cyl_mat_info.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
        m_cyl_mat_info.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
        m_cyl_mat_info.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
        m_cyl_mat_info.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
    }

    assert(d["Contact"]["Shoe Materials"].IsArray());
    int num_mats = d["Contact"]["Shoe Materials"].Size();

    for (int i = 0; i < num_mats; i++) {
        MatInfo minfo;
        minfo.mu = 0.7f;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        minfo.nu = 0.3f;
        minfo.kn = 2e6f;
        minfo.gn = 40.0f;
        minfo.kt = 2e5f;
        minfo.gt = 20.0f;

        const Value& mat = d["Contact"]["Shoe Materials"][i];

        minfo.mu = mat["Coefficient of Friction"].GetFloat();
        minfo.cr = mat["Coefficient of Restitution"].GetFloat();
        if (mat.HasMember("Properties")) {
            minfo.Y = mat["Properties"]["Young Modulus"].GetFloat();
            minfo.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
        }
        if (mat.HasMember("Coefficients")) {
            minfo.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
            minfo.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
            minfo.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
            minfo.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
        }

        m_shoe_mat_info.push_back(minfo);
    }

    // Read geometric collison data

    m_cyl_radius = d["Contact"]["Cylinder Shape"]["Radius"].GetDouble();
    m_front_cyl_loc = d["Contact"]["Cylinder Shape"]["Front Offset"].GetDouble();
    m_rear_cyl_loc = d["Contact"]["Cylinder Shape"]["Rear Offset"].GetDouble();

    assert(d["Contact"]["Shoe Shapes"].IsArray());
    int num_shapes = d["Contact"]["Shoe Shapes"].Size();

    for (int i = 0; i < num_shapes; i++) {
        const Value& shape = d["Contact"]["Shoe Shapes"][i];

        std::string type = shape["Type"].GetString();
        int matID = shape["Material Index"].GetInt();
        assert(matID >= 0 && matID < num_mats);

        if (type.compare("BOX") == 0) {
            ChVector<> pos = ReadVectorJSON(shape["Location"]);
            ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
            ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
            m_coll_boxes.push_back(BoxShape(pos, rot, dims, matID));
        } else if (type.compare("CYLINDER") == 0) {
            ChVector<> pos = ReadVectorJSON(shape["Location"]);
            ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
            double radius = shape["Radius"].GetDouble();
            double length = shape["Length"].GetDouble();
            m_coll_cylinders.push_back(CylinderShape(pos, rot, radius, length, matID));
        }
    }

    // Read visualization data

    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            m_meshFile = d["Visualization"]["Mesh"].GetString();
            m_has_mesh = true;
        }

        if (d["Visualization"].HasMember("Primitives")) {
            assert(d["Visualization"]["Primitives"].IsArray());
            int num_shapes = d["Visualization"]["Primitives"].Size();
            for (int i = 0; i < num_shapes; i++) {
                const Value& shape = d["Visualization"]["Primitives"][i];
                std::string type = shape["Type"].GetString();
                if (type.compare("BOX") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
                    m_vis_boxes.push_back(BoxShape(pos, rot, dims));
                } else if (type.compare("CYLINDER") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    double radius = shape["Radius"].GetDouble();
                    double length = shape["Length"].GetDouble();
                    m_vis_cylinders.push_back(CylinderShape(pos, rot, radius, length));
                }
            }
        }
    }
}

void TrackShoeSinglePin::CreateCylContactMaterial(ChContactMethod contact_method) {
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_cyl_mat_info.mu);
            matNSC->SetRestitution(m_cyl_mat_info.cr);
            m_cyl_material = matNSC;
            break;
        }
        case ChContactMethod::SMC:
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_cyl_mat_info.mu);
            matSMC->SetRestitution(m_cyl_mat_info.cr);
            matSMC->SetYoungModulus(m_cyl_mat_info.Y);
            matSMC->SetPoissonRatio(m_cyl_mat_info.nu);
            matSMC->SetKn(m_cyl_mat_info.kn);
            matSMC->SetGn(m_cyl_mat_info.gn);
            matSMC->SetKt(m_cyl_mat_info.kt);
            matSMC->SetGt(m_cyl_mat_info.gt);
            m_cyl_material = matSMC;
            break;
    }
}

void TrackShoeSinglePin::CreateShoeContactMaterials(ChContactMethod contact_method) {
    for (auto minfo : m_shoe_mat_info) {
        switch (contact_method) {
            case ChContactMethod::NSC: {
                auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
                matNSC->SetFriction(minfo.mu);
                matNSC->SetRestitution(minfo.cr);
                m_shoe_materials.push_back(matNSC);
                break;
            }
            case ChContactMethod::SMC:
                auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
                matSMC->SetFriction(minfo.mu);
                matSMC->SetRestitution(minfo.cr);
                matSMC->SetYoungModulus(minfo.Y);
                matSMC->SetPoissonRatio(minfo.nu);
                matSMC->SetKn(minfo.kn);
                matSMC->SetGn(minfo.gn);
                matSMC->SetKt(minfo.kt);
                matSMC->SetGt(minfo.gt);
                m_shoe_materials.push_back(matSMC);
                break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TrackShoeSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        trimesh_shape->SetStatic(true);
        m_shoe->AddAsset(trimesh_shape);
    } else {
        ChTrackShoeSinglePin::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
