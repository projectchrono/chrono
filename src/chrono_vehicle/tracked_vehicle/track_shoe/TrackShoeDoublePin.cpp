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
// Double-pin track shoe constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackShoeDoublePin::TrackShoeDoublePin(const std::string& filename)
    : ChTrackShoeDoublePin("", DoublePinTrackShoeType::TWO_CONNECTORS) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackShoeDoublePin::TrackShoeDoublePin(const rapidjson::Document& d)
    : ChTrackShoeDoublePin("", DoublePinTrackShoeType::TWO_CONNECTORS) {
    Create(d);
}

void TrackShoeDoublePin::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read topology of the double-pin track shoe.
    // If not defined, default to the two-connector double-ping track shoe.
    if (d.HasMember("Topology")) {
        std::string topology = d["Topology"].GetString();
        if (topology.compare("One Connector") == 0)
            m_topology = DoublePinTrackShoeType::ONE_CONNECTOR;
    }

    // Read shoe body geometry and mass properties
    assert(d.HasMember("Shoe"));

    m_shoe_length = d["Shoe"]["Length"].GetDouble();
    m_shoe_width = d["Shoe"]["Width"].GetDouble();
    m_shoe_height = d["Shoe"]["Height"].GetDouble();
    m_shoe_mass = d["Shoe"]["Mass"].GetDouble();
    m_shoe_inertia = ReadVectorJSON(d["Shoe"]["Inertia"]);

    // Read location of guide pin center (for detracking control)
    m_pin_center = ReadVectorJSON(d["Guide Pin Center"]);

    // Read connector body geometry and mass properties
    assert(d.HasMember("Connector"));
    m_connector_radius = d["Connector"]["Radius"].GetDouble();
    m_connector_length = d["Connector"]["Length"].GetDouble();
    m_connector_width = d["Connector"]["Width"].GetDouble();
    m_connector_mass = d["Connector"]["Mass"].GetDouble();
    m_connector_inertia = ReadVectorJSON(d["Connector"]["Inertia"]);

    // Read contact geometry data
    assert(d.HasMember("Contact"));
    assert(d["Contact"].HasMember("Connector Material"));
    assert(d["Contact"].HasMember("Shoe Materials"));
    assert(d["Contact"].HasMember("Shoe Shapes"));
    assert(d["Contact"]["Shoe Materials"].IsArray());
    assert(d["Contact"]["Shoe Shapes"].IsArray());

    // Read contact material information (defer creating the materials until CreateContactMaterials)

    m_cyl_mat_info = ReadMaterialInfoJSON(d["Contact"]["Connector Material"]);

    int num_mats = d["Contact"]["Shoe Materials"].Size();
    for (int i = 0; i < num_mats; i++) {
        MaterialInfo minfo = ReadMaterialInfoJSON(d["Contact"]["Shoe Materials"][i]);
        m_shoe_mat_info.push_back(minfo);
    }

    // Read geometric collison data

    int num_shapes = d["Contact"]["Shoe Shapes"].Size();

    for (int i = 0; i < num_shapes; i++) {
        const Value& shape = d["Contact"]["Shoe Shapes"][i];

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
            m_geometry.m_coll_cylinders.push_back(ChVehicleGeometry::CylinderShape(pos, rot, radius, length, matID));
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

    // Read visualization data

    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            m_geometry.m_vis_mesh_file = d["Visualization"]["Mesh"].GetString();
            m_geometry.m_has_mesh = true;
        }

        if (d["Visualization"].HasMember("Primitives")) {
            assert(d["Visualization"]["Primitives"].IsArray());
            num_shapes = d["Visualization"]["Primitives"].Size();
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
    } else {
        // Default to using the collision shapes
        for (auto& box : m_geometry.m_coll_boxes) {
            m_geometry.m_vis_boxes.push_back(box);
        }
        for (auto& cyl : m_geometry.m_coll_cylinders) {
            m_geometry.m_vis_cylinders.push_back(cyl);
        }
        m_geometry.m_has_primitives = true;
    }
}

void TrackShoeDoublePin::CreateContactMaterials(ChContactMethod contact_method) {
    m_shoe_sprk_material = m_cyl_mat_info.CreateMaterial(contact_method);

    for (auto minfo : m_shoe_mat_info) {
        m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
