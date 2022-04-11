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

    // Read contact information
    if (d.HasMember("Contact")) {
        assert(d["Contact"].HasMember("Materials"));
        assert(d["Contact"].HasMember("Shapes"));

        // Read contact material information (but defer creating and loading materials until CreateContactMaterials)
        assert(d["Contact"]["Materials"].IsArray());
        int num_mats = d["Contact"]["Materials"].Size();

        for (int i = 0; i < num_mats; i++) {
            MaterialInfo minfo = ReadMaterialInfoJSON(d["Contact"]["Materials"][i]);
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
                m_geometry.m_coll_spheres.push_back(ChVehicleGeometry::SphereShape(pos, radius, matID));
            } else if (type.compare("BOX") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
                m_geometry.m_coll_boxes.push_back(ChVehicleGeometry::BoxShape(pos, rot, dims, matID));
            } else if (type.compare("CYLINDER") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                double radius = shape["Radius"].GetDouble();
                double length = shape["Length"].GetDouble();
                m_geometry.m_coll_cylinders.push_back(
                    ChVehicleGeometry::CylinderShape(pos, rot, radius, length, matID));
            } else if (type.compare("HULL") == 0) {
                std::string filename = shape["Filename"].GetString();
                m_geometry.m_coll_hulls.push_back(ChVehicleGeometry::ConvexHullsShape(filename, matID));
            } else if (type.compare("MESH") == 0) {
                std::string filename = shape["Filename"].GetString();
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                double radius = shape["Contact Radius"].GetDouble();
                m_geometry.m_coll_meshes.push_back(ChVehicleGeometry::TrimeshShape(pos, filename, radius, matID));
            }
        }

        m_geometry.m_has_collision = true;
    }

    // Read chassis visualization
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            m_geometry.m_vis_mesh_file = d["Visualization"]["Mesh"].GetString();
            m_geometry.m_has_mesh = true;
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
                    m_geometry.m_vis_spheres.push_back(ChVehicleGeometry::SphereShape(pos, radius));
                } else if (type.compare("BOX") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
                    m_geometry.m_vis_boxes.push_back(ChVehicleGeometry::BoxShape(pos, rot, dims));
                } else if (type.compare("CYLINDER") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    double radius = shape["Radius"].GetDouble();
                    double length = shape["Length"].GetDouble();
                    m_geometry.m_vis_cylinders.push_back(ChVehicleGeometry::CylinderShape(pos, rot, radius, length));
                }
            }
            m_geometry.m_has_primitives = true;
        }
    }
}

void RigidChassis::CreateContactMaterials(ChContactMethod contact_method) {
    for (auto minfo : m_mat_info) {
        m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
    }
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

    // Read contact information
    if (d.HasMember("Contact")) {
        assert(d["Contact"].HasMember("Materials"));
        assert(d["Contact"].HasMember("Shapes"));

        // Read contact material information (but defer creating and loading materials until CreateContactMaterials)
        assert(d["Contact"]["Materials"].IsArray());
        int num_mats = d["Contact"]["Materials"].Size();

        for (int i = 0; i < num_mats; i++) {
            MaterialInfo minfo = ReadMaterialInfoJSON(d["Contact"]["Materials"][i]);
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
                m_geometry.m_coll_spheres.push_back(ChVehicleGeometry::SphereShape(pos, radius, matID));
            } else if (type.compare("BOX") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
                m_geometry.m_coll_boxes.push_back(ChVehicleGeometry::BoxShape(pos, rot, dims, matID));
            } else if (type.compare("CYLINDER") == 0) {
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                double radius = shape["Radius"].GetDouble();
                double length = shape["Length"].GetDouble();
                m_geometry.m_coll_cylinders.push_back(
                    ChVehicleGeometry::CylinderShape(pos, rot, radius, length, matID));
            } else if (type.compare("HULL") == 0) {
                std::string filename = shape["Filename"].GetString();
                m_geometry.m_coll_hulls.push_back(ChVehicleGeometry::ConvexHullsShape(filename, matID));
            } else if (type.compare("MESH") == 0) {
                std::string filename = shape["Filename"].GetString();
                ChVector<> pos = ReadVectorJSON(shape["Location"]);
                double radius = shape["Contact Radius"].GetDouble();
                m_geometry.m_coll_meshes.push_back(ChVehicleGeometry::TrimeshShape(pos, filename, radius, matID));
            }
        }

        m_geometry.m_has_collision = true;
    }

    // Read chassis visualization
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            m_geometry.m_vis_mesh_file = d["Visualization"]["Mesh"].GetString();
            m_geometry.m_has_mesh = true;
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
                    m_geometry.m_vis_spheres.push_back(ChVehicleGeometry::SphereShape(pos, radius));
                } else if (type.compare("BOX") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    ChVector<> dims = ReadVectorJSON(shape["Dimensions"]);
                    m_geometry.m_vis_boxes.push_back(ChVehicleGeometry::BoxShape(pos, rot, dims));
                } else if (type.compare("CYLINDER") == 0) {
                    ChVector<> pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    double radius = shape["Radius"].GetDouble();
                    double length = shape["Length"].GetDouble();
                    m_geometry.m_vis_cylinders.push_back(ChVehicleGeometry::CylinderShape(pos, rot, radius, length));
                }
            }
            m_geometry.m_has_primitives = true;
        }
    }
}

void RigidChassisRear::CreateContactMaterials(ChContactMethod contact_method) {
    for (auto minfo : m_mat_info) {
        m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
