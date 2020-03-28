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

#include "chrono/utils/ChCompositeInertia.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigidChassis::RigidChassis(const std::string& filename) : ChRigidChassis("") {
    Document d = ReadFileJSON(filename);
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

    m_mass = composite.GetMass();
    m_inertia = composite.GetInertia();
    m_COM_loc = composite.GetCOM();

    // Extract driver position
    m_driverCsys.pos = ReadVectorJSON(d["Driver Position"]["Location"]);
    m_driverCsys.rot = ReadQuaternionJSON(d["Driver Position"]["Orientation"]);

    // Read contact information
    if (d.HasMember("Contact")) {
        assert(d["Contact"].HasMember("Materials"));
        assert(d["Contact"].HasMember("Shapes"));

        // Read contact material information (but defer creating and loading materials until LoadCollisionShapes)
        assert(d["Contact"]["Materials"].IsArray());
        int num_mats = d["Contact"]["Materials"].Size();

        for (int i = 0; i < num_mats; i++) {
            MatInfo minfo;
            // Load default values (in case not all are provided in the JSON file)
            minfo.mu = 0.7f;
            minfo.cr = 0.1f;
            minfo.Y = 1e7f;
            minfo.nu = 0.3f;
            minfo.kn = 2e6f;
            minfo.gn = 40.0f;
            minfo.kt = 2e5f;
            minfo.gt = 20.0f;

            const Value& mat = d["Contact"]["Materials"][i];

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

            m_mat_info.push_back(minfo);
        }

        // Read contact shapes
        assert(d["Contact"]["Shapes"].IsArray());
        int num_shapes = d["Contact"]["Shapes"].Size();

        for (int i = 0; i < num_shapes; i++) {
            const Value& shape = d["Contact"]["Shapes"][i];

            std::string type = shape["Type"].GetString();
            int matID = shape["Material Index"].GetInt();
            assert(matID >= 0 && matID < num_mats);

            if (type.compare("SPHERE") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                double radius = shape["Radius"].GetDouble();
                m_coll_spheres.push_back(SphereShape(pos, radius, matID));
            } else if (type.compare("BOX") == 0) {
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
            } else if (type.compare("MESH") == 0) {
                std::string filename = shape["Filename"].GetString();
                m_coll_hulls.push_back(ConvexHullsShape(filename, matID));
            }
        }

        m_has_collision = true;
    }

    // Read chassis visualization
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            m_vis_mesh_file = d["Visualization"]["Mesh"].GetString();
            m_has_mesh = true;
        }
        if (d["Visualization"].HasMember("Primitives")) {
            assert(d["Visualization"]["Primitives"].IsArray());
            int num_shapes = d["Visualization"]["Primitives"].Size();
            for (int i = 0; i < num_shapes; i++) {
                const Value& shape = d["Visualization"]["Primitives"][i];
                std::string type = shape["Type"].GetString();
                if (type.compare("SPHERE") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    double radius = shape["Radius"].GetDouble();
                    m_vis_spheres.push_back(SphereShape(pos, radius));
                } else if (type.compare("BOX") == 0) {
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
            m_has_primitives = true;
        }
    }
}

void RigidChassis::CreateContactMaterials(ChContactMethod contact_method) {
    // Create the contact materials.
    for (auto minfo : m_mat_info) {
        switch (contact_method) {
            case ChContactMethod::NSC: {
                auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
                matNSC->SetFriction(minfo.mu);
                matNSC->SetRestitution(minfo.cr);
                m_materials.push_back(matNSC);
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
                m_materials.push_back(matSMC);
                break;
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
