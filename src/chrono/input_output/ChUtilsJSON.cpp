// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Utility functions for parsing JSON files.
//
// =============================================================================

#include <fstream>
#include <utility>

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChForceFunctors.h"

#include "chrono/input_output/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace rapidjson;

namespace chrono {

// -----------------------------------------------------------------------------

void ReadFileJSON(const std::string& filename, Document& d) {
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        std::cerr << "ERROR: Could not open JSON file: " << filename << std::endl;
    } else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            std::cerr << "ERROR: Invalid JSON file: " << filename << std::endl;
        }
    }
}

// -----------------------------------------------------------------------------

ChVector3d ReadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector3d(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

ChQuaternion<> ReadQuaternionJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

ChCoordsys<> ReadCoordinateSystemJSON(const Value& a) {
    assert(a.IsObject());
    assert(a.HasMember("Position"));
    assert(a.HasMember("Rotation"));
    return ChCoordsys<>(ReadVectorJSON(a["Position"]), ReadQuaternionJSON(a["Rotation"]));
}

ChFrame<> ReadFrameJSON(const Value& a) {
    assert(a.HasMember("Position"));
    assert(a.HasMember("Orientation"));
    return ChFrame<>(ReadVectorJSON(a["Position"]), ReadQuaternionJSON(a["Orientation"]));
}

ChColor ReadColorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChColor(a[0u].GetFloat(), a[1u].GetFloat(), a[2u].GetFloat());
}

// -----------------------------------------------------------------------------

ChContactMaterialData ReadMaterialInfoJSON(const rapidjson::Value& mat) {
    ChContactMaterialData minfo;

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

    return minfo;
}

std::shared_ptr<ChJoint::BushingData> ReadBushingDataJSON(const rapidjson::Value& bd) {
    auto bushing_data = chrono_types::make_shared<ChJoint::BushingData>();

    bushing_data->K_lin = bd["Stiffness Linear"].GetDouble();
    bushing_data->D_lin = bd["Damping Linear"].GetDouble();
    bushing_data->K_rot = bd["Stiffness Rotational"].GetDouble();
    bushing_data->D_rot = bd["Damping Rotational"].GetDouble();

    if (bd.HasMember("DOF")) {
        bushing_data->K_lin_dof = bd["DOF"]["Stiffness Linear"].GetDouble();
        bushing_data->D_lin_dof = bd["DOF"]["Damping Linear"].GetDouble();
        bushing_data->K_rot_dof = bd["DOF"]["Stiffness Rotational"].GetDouble();
        bushing_data->D_rot_dof = bd["DOF"]["Damping Rotational"].GetDouble();
    }

    return bushing_data;
}

ChJoint::Type ReadJointTypeJSON(const Value& a) {
    assert(a.IsString());
    std::string type = a.GetString();
    if (type.compare("Lock") == 0) {
        return ChJoint::Type::LOCK;
    } else if (type.compare("Point Line") == 0) {
        return ChJoint::Type::POINTLINE;
    } else if (type.compare("Point Plane") == 0) {
        return ChJoint::Type::POINTPLANE;
    } else if (type.compare("Revolute") == 0) {
        return ChJoint::Type::REVOLUTE;
    } else if (type.compare("Spherical") == 0) {
        return ChJoint::Type::SPHERICAL;
    } else if (type.compare("Universal") == 0) {
        return ChJoint::Type::UNIVERSAL;
    } else {
        // TODO Unknown type, what do we do here?
        return ChJoint::Type::LOCK;
    }
}

// -----------------------------------------------------------------------------

utils::ChBodyGeometry ReadBodyGeometryJSON(const rapidjson::Value& d) {
    utils::ChBodyGeometry geometry;

    // Read contact information
    if (d.HasMember("Contact")) {
        assert(d["Contact"].HasMember("Materials"));
        assert(d["Contact"].HasMember("Shapes"));

        // Read contact material information
        assert(d["Contact"]["Materials"].IsArray());
        int num_mats = d["Contact"]["Materials"].Size();

        for (int i = 0; i < num_mats; i++) {
            ChContactMaterialData minfo = ReadMaterialInfoJSON(d["Contact"]["Materials"][i]);
            geometry.materials.push_back(minfo);
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
                ChVector3d pos = ReadVectorJSON(shape["Location"]);
                double radius = shape["Radius"].GetDouble();
                geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius, matID));
            } else if (type.compare("BOX") == 0) {
                ChVector3d pos = ReadVectorJSON(shape["Location"]);
                ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                ChVector3d dims = ReadVectorJSON(shape["Dimensions"]);
                geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(pos, rot, dims, matID));
            } else if (type.compare("CYLINDER") == 0) {
                ChVector3d pos = ReadVectorJSON(shape["Location"]);
                ChVector3d axis = ReadVectorJSON(shape["Axis"]);
                double radius = shape["Radius"].GetDouble();
                double length = shape["Length"].GetDouble();
                geometry.coll_cylinders.push_back(
                    utils::ChBodyGeometry::CylinderShape(pos, axis, radius, length, matID));
            } else if (type.compare("HULL") == 0) {
                std::string filename = shape["Filename"].GetString();
                geometry.coll_hulls.push_back(
                    utils::ChBodyGeometry::ConvexHullsShape(GetChronoDataFile(filename), matID));
            } else if (type.compare("MESH") == 0) {
                std::string filename = shape["Filename"].GetString();
                ChVector3d pos = ReadVectorJSON(shape["Location"]);
                double radius = shape["Contact Radius"].GetDouble();
                geometry.coll_meshes.push_back(
                    utils::ChBodyGeometry::TrimeshShape(pos, QUNIT, GetChronoDataFile(filename), 1.0, radius, matID));
            }
        }
    }

    // Read visualization
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh")) {
            std::string filename = d["Visualization"]["Mesh"].GetString();
            geometry.vis_model_file = GetChronoDataFile(filename);
        }
        if (d["Visualization"].HasMember("Primitives")) {
            assert(d["Visualization"]["Primitives"].IsArray());
            int num_shapes = d["Visualization"]["Primitives"].Size();
            for (int i = 0; i < num_shapes; i++) {
                const Value& shape = d["Visualization"]["Primitives"][i];
                std::string type = shape["Type"].GetString();
                if (type.compare("SPHERE") == 0) {
                    ChVector3d pos = ReadVectorJSON(shape["Location"]);
                    double radius = shape["Radius"].GetDouble();
                    geometry.vis_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius));
                } else if (type.compare("BOX") == 0) {
                    ChVector3d pos = ReadVectorJSON(shape["Location"]);
                    ChQuaternion<> rot = ReadQuaternionJSON(shape["Orientation"]);
                    ChVector3d dims = ReadVectorJSON(shape["Dimensions"]);
                    geometry.vis_boxes.push_back(utils::ChBodyGeometry::BoxShape(pos, rot, dims));
                } else if (type.compare("CYLINDER") == 0) {
                    ChVector3d pos = ReadVectorJSON(shape["Location"]);
                    ChVector3d axis = ReadVectorJSON(shape["Axis"]);
                    double radius = shape["Radius"].GetDouble();
                    double length = shape["Length"].GetDouble();
                    geometry.vis_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(pos, axis, radius, length));
                }
            }
        }
    }

    return geometry;
}

utils::ChTSDAGeometry ReadTSDAGeometryJSON(const rapidjson::Value& d) {
    utils::ChTSDAGeometry geometry;

    if (d.HasMember("Visualization") && d["Visualization"].IsObject()) {
        auto& vis = d["Visualization"];
        assert(vis.IsObject());
        std::string type = vis["Type"].GetString();
        if (type == "SEGMENT") {
            geometry.vis_segment = chrono_types::make_shared<utils::ChTSDAGeometry::SegmentShape>();
        } else if (type == "SPRING") {
            // the default below are copied from the actual class, and since there
            // are no setters I can't just take the default constructor and override
            // the properties the user specifies (my only alternative would be to
            // construct and read from a prototype instance)
            double radius = 0.05;
            int resolution = 65;
            double turns = 5;
            if (vis.HasMember("Radius") && vis["Radius"].IsNumber()) {
                radius = vis["Radius"].GetDouble();
            }
            if (vis.HasMember("Resolution") && vis["Resolution"].IsInt()) {
                resolution = vis["Resolution"].GetInt();
            }
            if (vis.HasMember("Turns") && vis["Turns"].IsNumber()) {
                turns = vis["Turns"].GetDouble();
            }
            geometry.vis_spring =
                chrono_types::make_shared<utils::ChTSDAGeometry::SpringShape>(radius, resolution, turns);
        }
    }

    return geometry;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctorJSON(const rapidjson::Value& tsda, double& free_length) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        DegressiveDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        MapSpringDamper,
        Unknown
    };

    FunctorType type = FunctorType::Unknown;
    free_length = 0;

    assert(tsda.IsObject());

    if (tsda.HasMember("Spring Coefficient")) {
        if (tsda.HasMember("Damping Coefficient"))
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    } else if (tsda.HasMember("Damping Coefficient")) {
        if (tsda.HasMember("Degressivity Compression") && tsda.HasMember("Degressivity Expansion"))
            type = FunctorType::DegressiveDamper;
        else
            type = FunctorType::LinearDamper;
    }

    if (tsda.HasMember("Spring Curve Data")) {
        if (tsda.HasMember("Damping Curve Data"))
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    } else if (tsda.HasMember("Damping Curve Data")) {
        type = FunctorType::NonlinearDamper;
    }

    if (tsda.HasMember("Map Data"))
        type = FunctorType::MapSpringDamper;

    double preload = 0;
    if (tsda.HasMember("Preload"))
        preload = tsda["Preload"].GetDouble();

    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported TSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            double k = tsda["Spring Coefficient"].GetDouble();

            auto forceCB = chrono_types::make_shared<utils::LinearSpringForce>(k, preload);
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }

        case FunctorType::NonlinearSpring: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            auto forceCB = chrono_types::make_shared<utils::NonlinearSpringForce>(preload);

            assert(tsda["Spring Curve Data"].IsArray() && tsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = tsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["Spring Curve Data"][i][0u].GetDouble();
                double force = tsda["Spring Curve Data"][i][1u].GetDouble();
                forceCB->add_pointK(def, force);
            }
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }

        case FunctorType::LinearDamper: {
            double c = tsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<utils::LinearDamperForce>(c);
        }

        case FunctorType::DegressiveDamper: {
            double c = tsda["Damping Coefficient"].GetDouble();
            double dc = tsda["Degressivity Compression"].GetDouble();
            double de = tsda["Degressivity Expansion"].GetDouble();

            return chrono_types::make_shared<utils::DegressiveDamperForce>(c, dc, de);
        }

        case FunctorType::NonlinearDamper: {
            auto forceCB = chrono_types::make_shared<utils::NonlinearDamperForce>();

            assert(tsda["Damping Curve Data"].IsArray() && tsda["Damping Curve Data"][0u].Size() == 2);
            int num_speeds = tsda["Damping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["Damping Curve Data"][i][0u].GetDouble();
                double force = tsda["Damping Curve Data"][i][1u].GetDouble();
                forceCB->add_pointC(vel, force);
            }

            return forceCB;
        }

        case FunctorType::LinearSpringDamper: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            double k = tsda["Spring Coefficient"].GetDouble();
            double c = tsda["Damping Coefficient"].GetDouble();

            auto forceCB = chrono_types::make_shared<utils::LinearSpringDamperForce>(k, c, preload);
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }

        case FunctorType::NonlinearSpringDamper: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            auto forceCB = chrono_types::make_shared<utils::NonlinearSpringDamperForce>(preload);

            assert(tsda["Spring Curve Data"].IsArray() && tsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = tsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["Spring Curve Data"][i][0u].GetDouble();
                double force = tsda["Spring Curve Data"][i][1u].GetDouble();
                forceCB->add_pointK(def, force);
            }
            int num_speeds = tsda["Damping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["Damping Curve Data"][i][0u].GetDouble();
                double force = tsda["Damping Curve Data"][i][1u].GetDouble();
                forceCB->add_pointC(vel, force);
            }
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }

        case FunctorType::MapSpringDamper: {
            auto forceCB = chrono_types::make_shared<utils::MapSpringDamperForce>(preload);

            assert(tsda.HasMember("Deformation"));
            assert(tsda["Deformation"].IsArray());
            assert(tsda["Map Data"].IsArray() && tsda["Map Data"][0u].Size() == tsda["Deformation"].Size() + 1);
            int num_defs = tsda["Deformation"].Size();
            int num_speeds = tsda["Map Data"].Size();
            std::vector<double> defs(num_defs);
            for (int j = 0; j < num_defs; j++)
                defs[j] = tsda["Deformation"][j].GetDouble();
            forceCB->set_deformations(defs);
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["Map Data"][i][0u].GetDouble();
                std::vector<double> force(num_defs);
                for (int j = 0; j < num_defs; j++)
                    force[j] = tsda["Map Data"][i][j + 1].GetDouble();
                forceCB->add_pointC(vel, force);
            }
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }
    }
}

std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctorJSON(const rapidjson::Value& rsda, double& free_angle) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        Unknown
    };

    FunctorType type = FunctorType::Unknown;
    free_angle = 0;

    if (rsda.HasMember("Spring Coefficient"))
        if (rsda.HasMember("Damping Coefficient"))
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    else if (rsda.HasMember("Damping Coefficient"))
        type = FunctorType::LinearDamper;

    if (rsda.HasMember("Spring Curve Data"))
        if (rsda.HasMember("Damping Curve Data"))
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    else if (rsda.HasMember("Damping Curve Data"))
        type = FunctorType::NonlinearDamper;

    double preload = 0;
    if (rsda.HasMember("Preload"))
        preload = rsda["Preload"].GetDouble();

    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported RSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            double k = rsda["Spring Coefficient"].GetDouble();

            return chrono_types::make_shared<utils::LinearSpringTorque>(k, preload);
        }

        case FunctorType::NonlinearSpring: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            auto torqueCB = chrono_types::make_shared<utils::NonlinearSpringTorque>(preload);

            assert(rsda["Spring Curve Data"].IsArray() && rsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = rsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["Spring Curve Data"][i][0u].GetDouble();
                double force = rsda["Spring Curve Data"][i][1u].GetDouble();
                torqueCB->add_pointK(def, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearDamper: {
            double c = rsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<utils::LinearDamperTorque>(c);
        }

        case FunctorType::NonlinearDamper: {
            auto torqueCB = chrono_types::make_shared<utils::NonlinearDamperTorque>();

            assert(rsda["Damping Curve Data"].IsArray() && rsda["Damping Curve Data"][0u].Size() == 2);
            int num_speeds = rsda["Damping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["Damping Curve Data"][i][0u].GetDouble();
                double force = rsda["Damping Curve Data"][i][1u].GetDouble();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearSpringDamper: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            double k = rsda["Spring Coefficient"].GetDouble();
            double c = rsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<utils::LinearSpringDamperTorque>(k, c, preload);
        }

        case FunctorType::NonlinearSpringDamper: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            auto torqueCB = chrono_types::make_shared<utils::NonlinearSpringDamperTorque>(preload);

            assert(rsda["Spring Curve Data"].IsArray() && rsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = rsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["Spring Curve Data"][i][0u].GetDouble();
                double force = rsda["Spring Curve Data"][i][1u].GetDouble();
                torqueCB->add_pointK(def, force);
            }
            int num_speeds = rsda["Damping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["Damping Curve Data"][i][0u].GetDouble();
                double force = rsda["Damping Curve Data"][i][1u].GetDouble();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }
    }
}

// -----------------------------------------------------------------------------

void ChMapData::Read(const rapidjson::Value& a) {
    assert(a.IsArray());
    m_n = a.Size();
    for (unsigned int i = 0; i < m_n; i++) {
        m_x.push_back(a[i][0u].GetDouble());
        m_y.push_back(a[i][1u].GetDouble());
    }
}

void ChMapData::Set(ChFunctionInterp& map, double x_factor, double y_factor) const {
    for (unsigned int i = 0; i < m_n; i++) {
        map.AddPoint(x_factor * m_x[i], y_factor * m_y[i]);
    }
}

void ChMapData::Set(std::vector<std::pair<double, double>>& vec, double x_factor, double y_factor) const {
    for (unsigned int i = 0; i < m_n; i++) {
        vec.push_back(std::make_pair<double, double>(x_factor * m_x[i], y_factor * m_y[i]));
    }
}

}  // end namespace chrono
