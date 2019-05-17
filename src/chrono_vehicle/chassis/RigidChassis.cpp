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

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigidChassis::RigidChassis(const std::string& filename) : ChRigidChassis("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

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

        ChMatrix33<> inertia(inertiaXX);
        inertia.SetElement(0, 1, inertiaXY.x());
        inertia.SetElement(0, 2, inertiaXY.y());
        inertia.SetElement(1, 2, inertiaXY.z());
        inertia.SetElement(1, 0, inertiaXY.x());
        inertia.SetElement(2, 0, inertiaXY.y());
        inertia.SetElement(2, 1, inertiaXY.z());

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

        assert(d["Contact"].HasMember("Material"));
        assert(d["Contact"].HasMember("Shapes"));

        // Read contact material data
        const Value& mat = d["Contact"]["Material"];
        float mu = mat["Coefficient of Friction"].GetFloat();
        float cr = mat["Coefficient of Restitution"].GetFloat();

        SetContactFrictionCoefficient(mu);
        SetContactRestitutionCoefficient(cr);

        if (mat.HasMember("Properties")) {
            float ym = mat["Properties"]["Young Modulus"].GetFloat();
            float pr = mat["Properties"]["Poisson Ratio"].GetFloat();
            SetContactMaterialProperties(ym, pr);
        }
        if (mat.HasMember("Coefficients")) {
            float kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
            float gn = mat["Coefficients"]["Normal Damping"].GetFloat();
            float kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
            float gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
            SetContactMaterialCoefficients(kn, gn, kt, gt);
        }

        // Read contact shapes
        assert(d["Contact"]["Shapes"].IsArray());
        int num_shapes = d["Contact"]["Shapes"].Size();

        for (int i = 0; i < num_shapes; i++) {
            const Value& shape = d["Contact"]["Shapes"][i];
            std::string type = shape["Type"].GetString();
            if (type.compare("SPHERE") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                double radius = shape["Radius"].GetDouble();
                m_coll_spheres.push_back(SphereShape(pos, radius));
            } else if (type.compare("BOX") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
                m_coll_boxes.push_back(BoxShape(pos, rot, dims));
            } else if (type.compare("CYLINDER") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                double radius = shape["Radius"].GetDouble();
                double length = shape["Length"].GetDouble();
                m_coll_cylinders.push_back(CylinderShape(pos, rot, radius, length));
            } else if (type.compare("MESH") == 0) {
                m_coll_mesh_names.push_back(shape["Filename"].GetString());
            }
        }

        m_has_collision = true;
    }

    // Read chassis visualization
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            assert(d["Visualization"]["Mesh"].HasMember("Filename"));
            assert(d["Visualization"]["Mesh"].HasMember("Name"));
            m_vis_mesh_file = d["Visualization"]["Mesh"]["Filename"].GetString();
            m_vis_mesh_name = d["Visualization"]["Mesh"]["Name"].GetString();
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

}  // end namespace vehicle
}  // end namespace chrono
