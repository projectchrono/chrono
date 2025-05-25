// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

//// TODO
//// - perform checks during loading of YAML file,
////   or assume the file will be first checked against a fully defined YAML schema?
//// - add definition functions for link-type components using 2 local frames
////   (alternative ChLink initialization)
//// - add support for constraints (e.g., distance)
//// - add support for motors and actuators
//// - what is the best way to deal with collision families?

#include <algorithm>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/utils/ChForceFunctors.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChYamlParser.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace utils {

ChYamlParser::ChYamlParser() : m_verbose(false), m_use_degrees(true), m_initialized(false), m_instance_index(-1) {}
ChYamlParser::ChYamlParser(const std::string& yaml_filename, bool verbose)
    : m_verbose(false), m_use_degrees(true), m_initialized(false), m_instance_index(-1) {
    Load(yaml_filename);
}

ChYamlParser::~ChYamlParser() {}

std::string ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

void ChYamlParser::Load(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    } 

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    if (yaml["angle_degrees"])
        m_use_degrees = yaml["angle_degrees"].as<bool>();

    if (m_verbose) {
        cout << "\nLoading Chrono model specification from: " << yaml_filename << endl;
        cout << "\nangles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
    }

    // Read bodies
    if (yaml["bodies"]) {
        auto bodies = yaml["bodies"];
        ChAssertAlways(bodies.IsSequence());
        if (m_verbose) {
            cout << "\nbodies: " << bodies.size() << endl;
        }
        for (size_t i = 0; i < bodies.size(); i++) {
            auto name = bodies[i]["name"].as<std::string>();

            Body body;
            if (bodies[i]["location"])
                body.pos = ReadVector(bodies[i]["location"]);
            if (bodies[i]["orientation"])
                body.rot = ReadRotation(bodies[i]["orientation"]);
            if (bodies[i]["fixed"])
                body.is_fixed = bodies[i]["fixed"].as<bool>();
            if (bodies[i]["mass"])
                body.mass = bodies[i]["mass"].as<double>();
            if (bodies[i]["com"]) {
                ChVector3d com_pos = VNULL;
                ChQuaterniond com_rot = QUNIT;
                if (bodies[i]["com"]["location"])
                    com_pos = ReadVector(bodies[i]["com"]["location"]);
                if (bodies[i]["com"]["orientation"])
                    com_rot = ReadRotation(bodies[i]["com"]["orientation"]);
                body.com = ChFramed(com_pos, com_rot);
            }
            if (bodies[i]["inertia"]) {
                if (bodies[i]["inertia"]["moments"])
                    body.inertia_moments = ReadVector(bodies[i]["inertia"]["moments"]);
                if (bodies[i]["inertia"]["products"])
                    body.inertia_products = ReadVector(bodies[i]["inertia"]["products"]);
            }
            body.geometry = ReadGeometry(bodies[i]);

            if (m_verbose)
                body.PrintInfo(name);

            m_bodies.insert({name, body});
        }
    }

    // Read joints
    if (yaml["joints"]) {
        auto joints = yaml["joints"];
        ChAssertAlways(joints.IsSequence());
        if (m_verbose) {
            cout << "\njoints: " << joints.size() << endl;
        }
        for (size_t i = 0; i < joints.size(); i++) {
            auto name = joints[i]["name"].as<std::string>();

            Joint joint;
            ChAssertAlways(joints[i]["type"]);
            ChAssertAlways(joints[i]["body1"]);
            ChAssertAlways(joints[i]["body2"]);
            ChAssertAlways(joints[i]["location"]);
            joint.type = ReadJointType(joints[i]["type"]);
            joint.body1 = joints[i]["body1"].as<std::string>();
            joint.body2 = joints[i]["body2"].as<std::string>();

            std::shared_ptr<ChJoint::BushingData> bushing_data = nullptr;
            if (joints[i]["bushing_data"])
                joint.bdata = ReadBushingData(joints[i]["bushing_data"]);

            joint.frame = ReadJointFrame(joints[i]);

            if (m_verbose)
                joint.PrintInfo(name);

            m_joints.insert({name, joint});
        }
    }

    // Read constraints
    if (yaml["constraints"]) {
        //// TODO
    }

    // Read spring damper force elements elements
    if (yaml["spring_dampers"]) {
        auto spring_dampers = yaml["spring_dampers"];
        ChAssertAlways(spring_dampers.IsSequence());
        if (m_verbose) {
            cout << "spring dampers: " << spring_dampers.size() << endl;
        }
        for (size_t i = 0; i < spring_dampers.size(); i++) {
            ChAssertAlways(spring_dampers[i]["type"]);
            auto type = ToUpper(spring_dampers[i]["type"].as<std::string>());
            auto name = spring_dampers[i]["name"].as<std::string>();

            if (type == "TSDA") {
                TSDA tsda;
                ChAssertAlways(spring_dampers[i]["body1"]);
                ChAssertAlways(spring_dampers[i]["body2"]);
                ChAssertAlways(spring_dampers[i]["point1"]);
                ChAssertAlways(spring_dampers[i]["point2"]);
                tsda.body1 = spring_dampers[i]["body1"].as<std::string>();
                tsda.body2 = spring_dampers[i]["body2"].as<std::string>();
                tsda.point1 = ReadVector(spring_dampers[i]["point1"]);
                tsda.point2 = ReadVector(spring_dampers[i]["point2"]);
                tsda.force = ReadTSDAFunctor(spring_dampers[i], tsda.free_length);
                tsda.geometry = ReadTSDAGeometry(spring_dampers[i]);

                m_tsdas.insert({name, tsda});

            } else if (type == "RSDA") {
                RSDA rsda;
                ChAssertAlways(spring_dampers[i]["body1"]);
                ChAssertAlways(spring_dampers[i]["body2"]);
                ChAssertAlways(spring_dampers[i]["axis"]);
                rsda.body1 = spring_dampers[i]["body1"].as<std::string>();
                rsda.body2 = spring_dampers[i]["body2"].as<std::string>();
                rsda.axis = ReadVector(spring_dampers[i]["axis"]);
                rsda.axis.Normalize();
                if (spring_dampers[i]["pos"])
                    rsda.pos = ReadVector(spring_dampers[i]["pos"]);
                rsda.torque = ReadRSDAFunctor(spring_dampers[i], rsda.free_angle);
                if (m_use_degrees)
                    rsda.free_angle *= CH_DEG_TO_RAD;

                m_rsdas.insert({name, rsda});
            }
        }
    }

    m_initialized = true;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChBodyAuxRef> ChYamlParser::FindBody(const std::string& name) const {
    auto b = m_bodies.find(name);
    ChAssertAlways(b != m_bodies.end());
    return b->second.body[m_instance_index];
}

int ChYamlParser::Populate(ChSystem& sys, const ChFramed& model_frame, const std::string& model_prefix) {
    if (!m_initialized) {
        cerr << "Error: no YAML model loaded." << endl;
        throw std::runtime_error("No YAML model loaded");
    }

    m_instance_index++;

    // Create a load container for bushings
    auto container_bushings = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(container_bushings);

    // Create bodies
    for (auto& item : m_bodies) {
        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        body->SetName(model_prefix + item.first);
        body->SetFixed(item.second.is_fixed);
        body->SetMass(item.second.mass);
        body->SetFrameCOMToRef(item.second.com);
        body->SetInertiaXX(item.second.inertia_moments);
        body->SetInertiaXY(item.second.inertia_products);
        body->SetFrameRefToAbs(model_frame * ChFramed(item.second.pos, item.second.rot));
        sys.AddBody(body);
        item.second.body.push_back(body);
    }

    // Create joints (kinematic or bushings)
    for (auto& item : m_joints) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);
        auto joint = chrono_types::make_shared<ChJoint>(item.second.type,                 //
                                                        model_prefix + item.first,        //
                                                        body1,                            //
                                                        body2,                            //
                                                        model_frame * item.second.frame,  //
                                                        item.second.bdata);
        if (joint->IsKinematic())
            sys.AddLink(joint->GetAsLink());
        else
            container_bushings->Add(joint->GetAsBushing());
        item.second.joint.push_back(joint);
    }

    // Create distance constraints
    for (auto& item : m_dists) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);

        auto dist = chrono_types::make_shared<ChLinkDistance>();
        dist->SetName(model_prefix + item.first);
        dist->Initialize(body1, body2, false, model_frame * item.second.point1, model_frame * item.second.point2);
        sys.AddLink(dist);
        item.second.dist.push_back(dist);
    }

    // Create TSDAs
    for (auto& item : m_tsdas) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);
        auto tsda = chrono_types::make_shared<ChLinkTSDA>();
        tsda->SetName(model_prefix + item.first);
        tsda->Initialize(body1, body2, false, model_frame * item.second.point1, model_frame * item.second.point2);
        tsda->SetRestLength(item.second.free_length);
        tsda->RegisterForceFunctor(item.second.force);
        sys.AddLink(tsda);
        item.second.tsda.push_back(tsda);
    }

    // Create RSDAs
    for (auto& item : m_rsdas) {
        auto body1 = FindBody(item.second.body1);
        auto body2 = FindBody(item.second.body2);

        ChMatrix33<> rot;
        rot.SetFromAxisX(item.second.axis);
        ChQuaternion<> quat = rot.GetQuaternion() * QuatFromAngleY(CH_PI_2);

        auto rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName(model_prefix + item.first);
        rsda->Initialize(body1, body2, model_frame * ChFrame<>(item.second.pos, quat));
        rsda->SetRestAngle(item.second.free_angle);
        rsda->RegisterTorqueFunctor(item.second.torque);
        sys.AddLink(rsda);
        item.second.rsda.push_back(rsda);
    }

    // Create body collision models
    for (auto& item : m_bodies) {
        if (item.second.geometry.HasCollision())
            item.second.geometry.CreateCollisionShapes(item.second.body[m_instance_index], 0, sys.GetContactMethod());
    }

    // Create visualization assets
    for (auto& item : m_bodies)
        item.second.geometry.CreateVisualizationAssets(item.second.body[m_instance_index], VisualizationType::MESH);
    for (auto& item : m_tsdas)
        item.second.geometry.CreateVisualizationAssets(item.second.tsda[m_instance_index]);
    for (auto& item : m_dists)
        item.second.dist[m_instance_index]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    return m_instance_index;
}

void ChYamlParser::Depopulate(ChSystem& sys, int instance_index) {
    for (auto& item : m_bodies) {
        ChAssertAlways(item.second.body.size() > instance_index);
        sys.Remove(item.second.body[instance_index]);
        item.second.body.erase(item.second.body.begin() + instance_index);
    }

    for (auto& item : m_joints) {
        ChAssertAlways(item.second.joint.size() > instance_index);
        ChJoint::Remove(item.second.joint[instance_index]);
        item.second.joint.erase(item.second.joint.begin() + instance_index);
    }

    for (auto& item : m_dists) {
        ChAssertAlways(item.second.dist.size() > instance_index);
        sys.Remove(item.second.dist[instance_index]);
        item.second.dist.erase(item.second.dist.begin() + instance_index);
    }

    for (auto& item : m_tsdas) {
        ChAssertAlways(item.second.tsda.size() > instance_index);
        sys.Remove(item.second.tsda[instance_index]);
        item.second.tsda.erase(item.second.tsda.begin() + instance_index);
    }

    for (auto& item : m_rsdas) {
        ChAssertAlways(item.second.rsda.size() > instance_index);
        sys.Remove(item.second.rsda[instance_index]);
        item.second.rsda.erase(item.second.rsda.begin() + instance_index);
    }
}

// -----------------------------------------------------------------------------

ChYamlParser::Body::Body()
    : pos(VNULL),
      rot(QUNIT),
      is_fixed(false),
      mass(1),
      com({VNULL, QUNIT}),
      inertia_moments(ChVector3d(1)),
      inertia_products(ChVector3d(0)) {}

ChYamlParser::Joint::Joint()
    : type(ChJoint::Type::LOCK), body1(""), body2(""), frame({VNULL, QUNIT}), bdata(nullptr) {}

ChYamlParser::TSDA::TSDA()
    : body1(""), body2(""), point1(VNULL), point2(VNULL), free_length(0), force(nullptr) {}

ChYamlParser::RSDA::RSDA()
    : body1(""), body2(""), pos(VNULL), axis(ChVector3d(0, 0, 1)), free_angle(0), torque(nullptr) {}

ChYamlParser::DistanceConstraint::DistanceConstraint()
    : body1(""), body2(""), point1(VNULL), point2(VNULL) {}

void PrintGeometry(const ChBodyGeometry& geometry) {
    bool collision = geometry.HasCollision();
    bool vis_prims = geometry.HasVisualizationPrimitives();
    bool vis_mesh = geometry.HasVisualizationMesh();

    cout << "      collision? " << (collision ? "yes" : "no") << endl;
    cout << "      vis prims? " << (vis_prims ? "yes" : "no") << endl;
    cout << "      vis mesh?  " << (vis_mesh ? "yes" : "no") << endl;

    //// TODO
}

void ChYamlParser::Body::PrintInfo(const std::string& name) {
    cout << "  name:        " << name << endl;
    cout << "    pos:       " << pos << endl;
    cout << "    rot:       " << rot << endl;
    cout << "    fixed?     " << (is_fixed ? "true" : "false") << endl;
    cout << "    mass:      " << mass << endl;
    cout << "    com frame: " << com.GetPos() << " | " << com.GetRot() << endl;
    cout << "    I_xx:      " << inertia_moments << endl;
    cout << "    I_xy:      " << inertia_products << endl;
    cout << "    geometry:  " << endl;
    PrintGeometry(geometry);
}

void ChYamlParser::Joint::PrintInfo(const std::string& name) {
    cout << "  name:           " << name << endl;
    cout << "     type:        " << ChJoint::GetTypeString(type) << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     joint frame: " << frame.GetPos() << " | " << frame.GetRot() << endl;
}

void ChYamlParser::TSDA::PrintInfo(const std::string& name) {
    cout << " name:            " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     point1:      " << point1 << endl;
    cout << "     point2:      " << point2 << endl;
    cout << "     free_length: " << free_length << endl;
}

void ChYamlParser::RSDA::PrintInfo(const std::string& name) {
    cout << " name:            " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     pos:         " << pos << endl;
    cout << "     axis:        " << axis << endl;
    cout << "     free_angle:  " << free_angle << endl;
}

void ChYamlParser::DistanceConstraint::PrintInfo(const std::string& name) {
    cout << " name:            " << name << endl;
    cout << "     body1:       " << body1 << endl;
    cout << "     body2:       " << body2 << endl;
    cout << "     point1:      " << point1 << endl;
    cout << "     point2:      " << point2 << endl;
}

// =============================================================================

void PrintNodeType(const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Null:
            cout << " Null" << endl;
            break;
        case YAML::NodeType::Scalar:
            cout << " Scalar" << endl;
            break;
        case YAML::NodeType::Sequence:
            cout << " Sequence" << endl;
            break;
        case YAML::NodeType::Map:
            cout << " Map" << endl;
            break;
        case YAML::NodeType::Undefined:
            cout << " Undefined" << endl;
            break;
    }
}

ChVector3d ChYamlParser::ReadVector(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChVector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>());
}

ChQuaterniond ChYamlParser::ReadRotation(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3 || a.size() == 4);

    return (a.size() == 3) ? ReadCardanAngles(a) : ReadQuaternion(a);
}

ChQuaterniond ChYamlParser::ReadQuaternion(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 4);
    return ChQuaterniond(a[0].as<double>(), a[1].as<double>(), a[2].as<double>(), a[3].as<double>());
}

ChQuaterniond ChYamlParser::ReadCardanAngles(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    ChVector3d angles = ReadVector(a);

    if (m_use_degrees)
        angles *= CH_DEG_TO_RAD;

    ChQuaterniond q1 = QuatFromAngleZ(angles.x());  // roll
    ChQuaterniond q2 = QuatFromAngleY(angles.y());  // pitch
    ChQuaterniond q3 = QuatFromAngleX(angles.z());  // yaw

    ChQuaterniond q = q1 * q2 * q3;

    return q;
}

ChCoordsysd ChYamlParser::ReadCoordinateSystem(const YAML::Node& a) {
    ChAssertAlways(a["location"]);
    ChAssertAlways(a["orientation"]);
    return ChCoordsysd(ReadVector(a["location"]), ReadRotation(a["orientation"]));
}

ChColor ChYamlParser::ReadColor(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChColor(a[0].as<float>(), a[1].as<float>(), a[2].as<float>());
}

// -----------------------------------------------------------------------------

ChContactMaterialData ChYamlParser::ReadMaterialData(const YAML::Node& mat) {
    ChContactMaterialData minfo;

    if (mat["coefficient_of_friction"])
        minfo.mu = mat["coefficient_of_friction"].as<float>();
    if (mat["coefficient_of_restitution"])
        minfo.cr = mat["coefficient_of_restitution"].as<float>();

    if (mat["properties"]) {
        ChAssertAlways(mat["properties"]["Young_modulus"]);
        ChAssertAlways(mat["properties"]["Poisson_ratio"]);
        minfo.Y = mat["properties"]["Young_modulus"].as<float>();
        minfo.nu = mat["properties"]["Poisson_ratio"].as<float>();
    }

    if (mat["Coefficients"]) {
        ChAssertAlways(mat["coefficients"]["normal_stiffness"]);
        ChAssertAlways(mat["coefficients"]["normal_damping"]);
        ChAssertAlways(mat["coefficients"]["tangential_stiffness"]);
        ChAssertAlways(mat["coefficients"]["tangential_damping"]);
        minfo.kn = mat["coefficients"]["normal_stiffness"].as<float>();
        minfo.gn = mat["coefficients"]["normal_damping"].as<float>();
        minfo.kt = mat["coefficients"]["tangential_stiffness"].as<float>();
        minfo.gt = mat["coefficients"]["tangential_damping"].as<float>();
    }

    return minfo;
}

ChJoint::Type ChYamlParser::ReadJointType(const YAML::Node& a) {
    std::string type = ToUpper(a.as<std::string>());
    if (type.compare("LOCK") == 0) {
        return ChJoint::Type::LOCK;
    } else if (type.compare("POINT LINE") == 0) {
        return ChJoint::Type::POINTLINE;
    } else if (type.compare("POINT PLANE") == 0) {
        return ChJoint::Type::POINTPLANE;
    } else if (type.compare("REVOLUTE") == 0) {
        return ChJoint::Type::REVOLUTE;
    } else if (type.compare("SPHERICAL") == 0) {
        return ChJoint::Type::SPHERICAL;
    } else if (type.compare("PRISMATIC") == 0) {
        return ChJoint::Type::PRISMATIC;
    } else if (type.compare("UNIVERSAL") == 0) {
        return ChJoint::Type::UNIVERSAL;
    } else {
        return ChJoint::Type::LOCK;
    }
}

ChFramed ChYamlParser::ReadJointFrame(const YAML::Node& a) {
    //// TODO - for now, assume joints are specified through a single frame (expressed relative to instance frame)
    ChAssertAlways(a["location"]);
    ChVector3d pos = ReadVector(a["location"]);
    ChQuaterniond rot = QUNIT;

    switch (ReadJointType(a["type"])) {
        case ChJoint::Type::LOCK:
        case ChJoint::Type::SPHERICAL:
            rot = QUNIT;
            break;
        case ChJoint::Type::REVOLUTE:
        case ChJoint::Type::PRISMATIC: {
            ChAssertAlways(a["axis"]);
            auto axis = ReadVector(a["axis"]);
            axis.Normalize();
            ChMatrix33 R;
            R.SetFromAxisZ(axis);
            rot = R.GetQuaternion();
            break;
        }
        case ChJoint::Type::UNIVERSAL: {
            ChAssertAlways(a["axis1"]);
            ChAssertAlways(a["axis2"]);
            auto axis_x = ReadVector(a["axis1"]);
            auto axis_y = ReadVector(a["axis2"]);
            auto axis_z = Vcross(axis_x, axis_y);
            axis_y = Vcross(axis_z, axis_x);
            axis_x.Normalize();
            axis_y.Normalize();
            axis_z.Normalize();
            ChMatrix33d R(axis_x, axis_y, axis_z);
            rot = R.GetQuaternion();
            break;
        }
    }

    //// TODO - POINTLINE, POINTPLANE

    return ChFramed(pos, rot);
}

std::shared_ptr<ChJoint::BushingData> ChYamlParser::ReadBushingData(const YAML::Node& bd) {
    auto bushing_data = chrono_types::make_shared<ChJoint::BushingData>();

    bushing_data->K_lin = bd["stiffness_linear"].as<double>();
    bushing_data->D_lin = bd["damping_linear"].as<double>();
    bushing_data->K_rot = bd["stiffness_rotational"].as<double>();
    bushing_data->D_rot = bd["damping_rotational"].as<double>();

    if (bd["DOF"]) {
        bushing_data->K_lin_dof = bd["DOF"]["stiffness_linear"].as<double>();
        bushing_data->D_lin_dof = bd["DOF"]["damping_linear"].as<double>();
        bushing_data->K_rot_dof = bd["DOF"]["stiffness_rotational"].as<double>();
        bushing_data->D_rot_dof = bd["DOF"]["damping_rotational"].as<double>();
    }

    return bushing_data;
}

// -----------------------------------------------------------------------------

int FindMaterial(const std::string& name, const std::unordered_map<std::string, size_t> materials) {
    auto b = materials.find(name);
    ChAssertAlways(b != materials.end());
    return (int)b->second;
}

ChBodyGeometry ChYamlParser::ReadGeometry(const YAML::Node& d) {
    ChBodyGeometry geometry;

    // Read contact information
    if (d["contact"]) {
        ChAssertAlways(d["contact"]["materials"]);
        ChAssertAlways(d["contact"]["shapes"]);

        // Read contact material information
        ChAssertAlways(d["contact"]["materials"].IsSequence());
        size_t num_mats = d["contact"]["materials"].size();

        std::unordered_map<std::string, size_t> materials;
        for (size_t i = 0; i < num_mats; i++) {
            ChAssertAlways(d["contact"]["materials"][i]["name"]);
            ChContactMaterialData mat_data = ChYamlParser::ReadMaterialData(d["contact"]["materials"][i]);
            geometry.materials.push_back(mat_data);
            materials.insert({d["contact"]["materials"][i]["name"].as<std::string>(), i});
        }

        // Read contact shapes
        ChAssertAlways(d["contact"]["shapes"].IsSequence());
        size_t num_shapes = d["contact"]["shapes"].size();

        for (size_t i = 0; i < num_shapes; i++) {
            const YAML::Node& shape = d["contact"]["shapes"][i];
            ChAssertAlways(shape["type"]);
            ChAssertAlways(shape["material"]);
            std::string type = ToUpper(shape["type"].as<std::string>());
            int matID = FindMaterial(shape["material"].as<std::string>(), materials);

            if (type.compare("SPHERE") == 0) {
                ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                double radius = shape["radius"].as<double>();
                geometry.coll_spheres.push_back(ChBodyGeometry::SphereShape(pos, radius, matID));
            } else if (type.compare("BOX") == 0) {
                ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                ChQuaternion<> rot = ChYamlParser::ReadRotation(shape["orientation"]);
                ChVector3d dims = ChYamlParser::ReadVector(shape["dimensions"]);
                geometry.coll_boxes.push_back(ChBodyGeometry::BoxShape(pos, rot, dims, matID));
            } else if (type.compare("CYLINDER") == 0) {
                ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                ChVector3d axis = ChYamlParser::ReadVector(shape["axis"]);
                double radius = shape["radius"].as<double>();
                double length = shape["length"].as<double>();
                geometry.coll_cylinders.push_back(ChBodyGeometry::CylinderShape(pos, axis, radius, length, matID));
            } else if (type.compare("HULL") == 0) {
                std::string filename = shape["filename"].as<std::string>();
                geometry.coll_hulls.push_back(ChBodyGeometry::ConvexHullsShape(GetChronoDataFile(filename), matID));
            } else if (type.compare("MESH") == 0) {
                std::string filename = shape["filename"].as<std::string>();
                ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                double radius = shape["contact_radius"].as<double>();
                geometry.coll_meshes.push_back(
                    ChBodyGeometry::TrimeshShape(pos, GetChronoDataFile(filename), radius, matID));
            }
        }
    }

    // Read visualization
    if (d["visualization"]) {
        if (d["visualization"]["mesh"]) {
            std::string filename = d["visualization"]["mesh"].as<std::string>();
            geometry.vis_mesh_file = GetChronoDataFile(filename);
        }
        if (d["visualization"]["shapes"]) {
            ChAssertAlways(d["visualization"]["shapes"].IsSequence());
            size_t num_shapes = d["visualization"]["shapes"].size();

            for (size_t i = 0; i < num_shapes; i++) {
                const YAML::Node& shape = d["visualization"]["shapes"][i];
                std::string type = ToUpper(shape["type"].as<std::string>());
                ChColor color({-1, -1, -1});
                if (shape["color"]) {
                    color = ReadColor(shape["color"]);
                }
                if (type.compare("SPHERE") == 0) {
                    ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                    double radius = shape["radius"].as<double>();
                    auto sphere = ChBodyGeometry::SphereShape(pos, radius);
                    sphere.color = color;
                    geometry.vis_spheres.push_back(sphere);
                } else if (type.compare("BOX") == 0) {
                    ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                    ChQuaternion<> rot = ChYamlParser::ReadRotation(shape["orientation"]);
                    ChVector3d dims = ChYamlParser::ReadVector(shape["dimensions"]);
                    auto box = ChBodyGeometry::BoxShape(pos, rot, dims);
                    box.color = color;
                    geometry.vis_boxes.push_back(box);
                } else if (type.compare("CYLINDER") == 0) {
                    ChVector3d pos = ChYamlParser::ReadVector(shape["location"]);
                    ChVector3d axis = ChYamlParser::ReadVector(shape["axis"]);
                    double radius = shape["radius"].as<double>();
                    double length = shape["length"].as<double>();
                    auto cylinder = ChBodyGeometry::CylinderShape(pos, axis, radius, length);
                    cylinder.color = color;
                    geometry.vis_cylinders.push_back(cylinder);
                }
            }
        }
    }

    return geometry;
}

ChTSDAGeometry ChYamlParser::ReadTSDAGeometry(const YAML::Node& d) {
    ChTSDAGeometry geometry;

    if (d["visualization"]) {
        std::string type = ToUpper(d["visualization"]["type"].as<std::string>());
        if (type == "SEGMENT") {
            geometry.vis_segment = chrono_types::make_shared<ChTSDAGeometry::SegmentShape>();
        } else if (type == "SPRING") {
            // the default below are copied from the actual class, and since there
            // are no setters I can't just take the default constructor and override
            // the properties the user specifies (my only alternative would be to
            // construct and read from a prototype instance)
            double radius = 0.05;
            int resolution = 65;
            double turns = 5;
            if (d["visualization"]["radius"])
                radius = d["visualization"]["radius"].as<double>();
            if (d["visualization"]["resolution"])
                resolution = d["visualization"]["resolution"].as<int>();
            if (d["visualization"]["turns"])
                turns = d["visualization"]["turns"].as<double>();
            geometry.vis_spring = chrono_types::make_shared<ChTSDAGeometry::SpringShape>(radius, resolution, turns);
        }
    }

    return geometry;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChLinkTSDA::ForceFunctor> ChYamlParser::ReadTSDAFunctor(const YAML::Node& tsda, double& free_length) {
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

    // Determine type of functor to be created (based on specified keys)
    FunctorType type = FunctorType::Unknown;
    free_length = 0;

    if (tsda["spring_coefficient"]) {
        if (tsda["damping_coefficient"])
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    } else if (tsda["damping_coefficient"]) {
        if (tsda["degressivity_compression"] && tsda["degressivity_expansion"])
            type = FunctorType::DegressiveDamper;
        else
            type = FunctorType::LinearDamper;
    }

    if (tsda["spring_curve_data"]) {
        if (tsda["damping_curve_data"])
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    } else if (tsda["damping_curve_data"]) {
        type = FunctorType::NonlinearDamper;
    }

    if (tsda["map_data"])
        type = FunctorType::MapSpringDamper;

    // Read preload (if specified)
    double preload = 0;
    if (tsda["preload"])
        preload = tsda["preload"].as<double>();

    // Construct functor of appropriate type
    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported TSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            ChAssertAlways(tsda["free_length"]);
            free_length = tsda["free_length"].as<double>();

            double k = tsda["spring_coefficient"].as<double>();

            auto forceCB = chrono_types::make_shared<LinearSpringForce>(k, preload);
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::NonlinearSpring: {
            ChAssertAlways(tsda["free_length"]);
            free_length = tsda["free_length"].as<double>();

            auto forceCB = chrono_types::make_shared<NonlinearSpringForce>(preload);

            ChAssertAlways(tsda["spring_curve_data"].IsSequence() && tsda["spring_curve_data"][0].size() == 2);
            int num_defs = tsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["spring_curve_data"][i][0].as<double>();
                double force = tsda["spring_curve_data"][i][1].as<double>();
                forceCB->add_pointK(def, force);
            }
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::LinearDamper: {
            double c = tsda["damping_coefficient"].as<double>();

            return chrono_types::make_shared<LinearDamperForce>(c);
        }

        case FunctorType::DegressiveDamper: {
            double c = tsda["damping_coefficient"].as<double>();
            double dc = tsda["degressivity_compression"].as<double>();
            double de = tsda["degressivity_expansion"].as<double>();

            return chrono_types::make_shared<DegressiveDamperForce>(c, dc, de);
        }

        case FunctorType::NonlinearDamper: {
            auto forceCB = chrono_types::make_shared<NonlinearDamperForce>();

            ChAssertAlways(tsda["damping_curve_data"].IsSequence() && tsda["damping_curve_data"][0].size() == 2);
            int num_speeds = tsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["damping_curve_data"][i][0].as<double>();
                double force = tsda["damping_curve_data"][i][1].as<double>();
                forceCB->add_pointC(vel, force);
            }

            return forceCB;
        }

        case FunctorType::LinearSpringDamper: {
            ChAssertAlways(tsda["free_length"]);
            free_length = tsda["free_length"].as<double>();

            double k = tsda["spring_coefficient"].as<double>();
            double c = tsda["damping_coefficient"].as<double>();

            auto forceCB = chrono_types::make_shared<LinearSpringDamperForce>(k, c, preload);
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::NonlinearSpringDamper: {
            ChAssertAlways(tsda["free_length"]);
            free_length = tsda["free_length"].as<double>();

            auto forceCB = chrono_types::make_shared<NonlinearSpringDamperForce>(preload);

            ChAssertAlways(tsda["spring_curve_data"].IsSequence() && tsda["spring_curve_data"][0].size() == 2);
            int num_defs = tsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["spring_curve_data"][i][0].as<double>();
                double force = tsda["spring_curve_data"][i][1].as<double>();
                forceCB->add_pointK(def, force);
            }
            int num_speeds = tsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["damping_curve_data"][i][0].as<double>();
                double force = tsda["damping_curve_data"][i][1].as<double>();
                forceCB->add_pointC(vel, force);
            }
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }

        case FunctorType::MapSpringDamper: {
            auto forceCB = chrono_types::make_shared<MapSpringDamperForce>(preload);

            ChAssertAlways(tsda["deformation"]);
            ChAssertAlways(tsda["deformation"].IsSequence());
            ChAssertAlways(tsda["map_data"].IsSequence() &&
                           tsda["map_data"][0].size() == tsda["deformation"].size() + 1);
            int num_defs = tsda["deformation"].size();
            int num_speeds = tsda["map_data"].size();
            std::vector<double> defs(num_defs);
            for (int j = 0; j < num_defs; j++)
                defs[j] = tsda["deformation"][j].as<double>();
            forceCB->set_deformations(defs);
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["map_data"][i][0].as<double>();
                std::vector<double> force(num_defs);
                for (int j = 0; j < num_defs; j++)
                    force[j] = tsda["map_data"][i][j + 1].as<double>();
                forceCB->add_pointC(vel, force);
            }
            if (tsda["minimum_length"] && tsda["maximum_length"])
                forceCB->enable_stops(tsda["minimum_length"].as<double>(), tsda["maximum_length"].as<double>());

            return forceCB;
        }
    }
}

std::shared_ptr<ChLinkRSDA::TorqueFunctor> ChYamlParser::ReadRSDAFunctor(const YAML::Node& rsda, double& free_angle) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        Unknown
    };

    // Determine type of functor to be created (based on specified keys)
    FunctorType type = FunctorType::Unknown;
    free_angle = 0;

    if (rsda["spring_coefficient"])
        if (rsda["damping_coefficient"])
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    else if (rsda["damping_coefficient"])
        type = FunctorType::LinearDamper;

    if (rsda["spring_curve_data"])
        if (rsda["damping_curve_data"])
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    else if (rsda["damping_curve_data"])
        type = FunctorType::NonlinearDamper;

    // Read preload (if specified)
    double preload = 0;
    if (rsda["preload"])
        preload = rsda["preload"].as<double>();

    // Construct functor of appropriate type
    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported RSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            ChAssertAlways(rsda["free_angle"]);
            free_angle = rsda["free_angle"].as<double>();

            double k = rsda["spring_coefficient"].as<double>();

            return chrono_types::make_shared<LinearSpringTorque>(k, preload);
        }

        case FunctorType::NonlinearSpring: {
            ChAssertAlways(rsda["free_angle"]);
            free_angle = rsda["free_angle"].as<double>();

            auto torqueCB = chrono_types::make_shared<NonlinearSpringTorque>(preload);

            ChAssertAlways(rsda["spring_curve_data"].IsSequence() && rsda["spring_curve_data"][0].size() == 2);
            int num_defs = rsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["spring_curve_data"][i][0].as<double>();
                double force = rsda["spring_curve_data"][i][1].as<double>();
                torqueCB->add_pointK(def, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearDamper: {
            double c = rsda["damping_coefficient"].as<double>();

            return chrono_types::make_shared<LinearDamperTorque>(c);
        }

        case FunctorType::NonlinearDamper: {
            auto torqueCB = chrono_types::make_shared<NonlinearDamperTorque>();

            ChAssertAlways(rsda["damping_curve_data"].IsSequence() && rsda["damping_curve_data"][0].size() == 2);
            int num_speeds = rsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["damping_curve_data"][i][0].as<double>();
                double force = rsda["damping_curve_data"][i][1].as<double>();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearSpringDamper: {
            ChAssertAlways(rsda["free_angle"]);
            free_angle = rsda["free_angle"].as<double>();

            double k = rsda["spring_coefficient"].as<double>();
            double c = rsda["damping_coefficient"].as<double>();

            return chrono_types::make_shared<LinearSpringDamperTorque>(k, c, preload);
        }

        case FunctorType::NonlinearSpringDamper: {
            ChAssertAlways(rsda["free_angle"]);
            free_angle = rsda["free_angle"].as<double>();

            auto torqueCB = chrono_types::make_shared<NonlinearSpringDamperTorque>(preload);

            ChAssertAlways(rsda["spring_curve_data"].IsSequence() && rsda["spring_curve_data"][0].size() == 2);
            int num_defs = rsda["spring_curve_data"].size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["spring_curve_data"][i][0].as<double>();
                double force = rsda["spring_curve_data"][i][1].as<double>();
                torqueCB->add_pointK(def, force);
            }
            int num_speeds = rsda["damping_curve_data"].size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["damping_curve_data"][i][0].as<double>();
                double force = rsda["damping_curve_data"][i][1].as<double>();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }
    }
}

}  // namespace utils
}  // namespace chrono
