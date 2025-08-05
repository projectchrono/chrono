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

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

TrackShoeDoublePin::TrackShoeDoublePin(const std::string& filename)
    : ChTrackShoeDoublePin("", DoublePinTrackShoeType::TWO_CONNECTORS) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
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

    // Read contact data
    assert(d.HasMember("Contact"));
    assert(d["Contact"].HasMember("Connector Material"));
    assert(d["Contact"].HasMember("Shoe Materials"));
    assert(d["Contact"].HasMember("Shoe Shapes"));
    assert(d["Contact"]["Shoe Materials"].IsArray());
    assert(d["Contact"]["Shoe Shapes"].IsArray());

    // Read contact material information
    m_shoe_sprk_minfo = ReadMaterialInfoJSON(d["Contact"]["Connector Material"]);

    int num_mats = d["Contact"]["Shoe Materials"].Size();
    for (int i = 0; i < num_mats; i++) {
        ChContactMaterialData minfo = ReadMaterialInfoJSON(d["Contact"]["Shoe Materials"][i]);
        m_geometry.materials.push_back(minfo);
    }
    m_ground_geometry.materials = m_geometry.materials;

    // Read geometric collison data
    int num_shapes = d["Contact"]["Shoe Shapes"].Size();
    assert(num_shapes > 0);

    for (int i = 0; i < num_shapes; i++) {
        const Value& shape = d["Contact"]["Shoe Shapes"][i];

        std::string type = shape["Type"].GetString();
        int matID = shape["Material Index"].GetInt();
        assert(matID >= 0 && matID < num_mats);

        bool ground_geometry = shape.HasMember("Ground Contact") && shape["Ground Contact"].GetBool();

        if (type.compare("SPHERE") == 0) {
            ChVector3d pos = ReadVectorJSON(shape["Location"]);
            double radius = shape["Radius"].GetDouble();
            utils::ChBodyGeometry::SphereShape sphere(pos, radius, matID);
            m_geometry.coll_spheres.push_back(sphere);
            if (ground_geometry)
                m_ground_geometry.coll_spheres.push_back(sphere);
        } else if (type.compare("BOX") == 0) {
            ChVector3d pos = ReadVectorJSON(shape["Location"]);
            ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
            ChVector3d dims = ReadVectorJSON(shape["Dimensions"]);
            utils::ChBodyGeometry::BoxShape box(pos, rot, dims, matID);
            m_geometry.coll_boxes.push_back(box);
            if (ground_geometry)
                m_ground_geometry.coll_boxes.push_back(box);
        } else if (type.compare("CYLINDER") == 0) {
            ChVector3d pos = ReadVectorJSON(shape["Location"]);
            ChVector3d axis = ReadVectorJSON(shape["Axis"]);
            double radius = shape["Radius"].GetDouble();
            double length = shape["Length"].GetDouble();
            utils::ChBodyGeometry::CylinderShape cylinder(pos, axis, radius, length, matID);
            m_geometry.coll_cylinders.push_back(cylinder);
            if (ground_geometry)
                m_ground_geometry.coll_cylinders.push_back(cylinder);
        } else if (type.compare("HULL") == 0) {
            std::string filename = shape["Filename"].GetString();
            utils::ChBodyGeometry::ConvexHullsShape hull(vehicle::GetDataFile(filename), matID);
            m_geometry.coll_hulls.push_back(hull);
            if (ground_geometry)
                m_ground_geometry.coll_hulls.push_back(hull);
        } else if (type.compare("MESH") == 0) {
            std::string filename = shape["Filename"].GetString();
            ChVector3d pos = ReadVectorJSON(shape["Location"]);
            double radius = shape["Contact Radius"].GetDouble();
            utils::ChBodyGeometry::TrimeshShape mesh(pos, QUNIT, vehicle::GetDataFile(filename), 1.0, radius, matID);
            m_geometry.coll_meshes.push_back(mesh);
            if (ground_geometry)
                m_ground_geometry.coll_meshes.push_back(mesh);
        }
    }

    // Read visualization data
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            std::string filename = d["Visualization"]["Mesh"].GetString();
            m_geometry.vis_model_file = vehicle::GetDataFile(filename);
        }

        if (d["Visualization"].HasMember("Primitives")) {
            assert(d["Visualization"]["Primitives"].IsArray());
            num_shapes = d["Visualization"]["Primitives"].Size();
            for (int i = 0; i < num_shapes; i++) {
                const Value& shape = d["Visualization"]["Primitives"][i];
                std::string type = shape["Type"].GetString();
                if (type.compare("SPHERE") == 0) {
                    ChVector3d pos = ReadVectorJSON(shape["Location"]);
                    double radius = shape["Radius"].GetDouble();
                    m_geometry.vis_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius));
                } else if (type.compare("BOX") == 0) {
                    ChVector3d pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    ChVector3d dims = ReadVectorJSON(shape["Dimensions"]);
                    m_geometry.vis_boxes.push_back(utils::ChBodyGeometry::BoxShape(pos, rot, dims));
                } else if (type.compare("CYLINDER") == 0) {
                    ChVector3d pos = ReadVectorJSON(shape["Location"]);
                    ChVector3d axis = ReadVectorJSON(shape["Axis"]);
                    double radius = shape["Radius"].GetDouble();
                    double length = shape["Length"].GetDouble();
                    m_geometry.vis_cylinders.push_back(
                        utils::ChBodyGeometry::CylinderShape(pos, axis, radius, length));
                }
            }
        }
    } else {
        // Default to using the collision shapes
        for (auto& box : m_geometry.coll_boxes) {
            m_geometry.vis_boxes.push_back(box);
        }
        for (auto& cyl : m_geometry.coll_cylinders) {
            m_geometry.vis_cylinders.push_back(cyl);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
