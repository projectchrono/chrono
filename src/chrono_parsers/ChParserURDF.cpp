// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Parser utility class for URDF input files.
//
// =============================================================================

#include <string>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "chrono_parsers/ChParserURDF.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "chrono_thirdparty/filesystem/path.h"


namespace chrono {
namespace parsers {

using std::cout;
using std::cerr;
using std::endl;

// Threshold for identifying bodies with zero inertia properties.
const double inertia_threshold = 1e-6;

ChParserURDF::ChParserURDF(const std::string& filename) : m_filename(filename), m_vis_collision(false), m_sys(nullptr) {
    // Read input file into XML string
    std::fstream xml_file(filename, std::fstream::in);
    while (xml_file.good()) {
        std::string line;
        std::getline(xml_file, line);
        m_xml_string += (line + "\n");
    }
    xml_file.close();

    // Parse XML string
    m_model = urdf::parseURDF(m_xml_string);
    if (!m_model) {
        cerr << "ERROR: parsing the URDF file " << filename << " failed." << endl;
        return;
    }

    // Cache path to the URDF model file
    m_filepath = filesystem::path(filename).parent_path().str();
}

void ChParserURDF::SetRootInitPose(const ChFrame<>& init_pose) {
    if (m_sys) {
        cerr << "WARNING: SetRootInitPose must be called before PopulateSystem." << endl;
        return;
    }

    m_init_pose = init_pose;
}

void ChParserURDF::SetJointActuationType(const std::string& joint_name, ActuationType actuation_type) {
    if (m_sys) {
        cerr << "WARNING: SetJointActuated must be called before PopulateSystem." << endl;
        return;
    }

    auto joint = m_model->getJoint(joint_name);
    if (!joint) {
        cerr << "WARNING: SetJointActuated: No joint named \"" << joint_name << "\"." << endl;
        return;
    }

    if (joint->type == urdf::Joint::REVOLUTE ||    //
        joint->type == urdf::Joint::CONTINUOUS ||  //
        joint->type == urdf::Joint::PRISMATIC)
        m_actuated_joints[joint_name] = actuation_type;
    else
        cerr << "WARNING: SetJointActuated: Joint \"" << joint_name << "\" cannot be actuated." << endl;
}

void ChParserURDF::SetAllJointsActuationType(ActuationType actuation_type) {
    if (m_sys) {
        cerr << "WARNING: SetAllJointsActuated must be called before PopulateSystem." << endl;
        return;
    }

    for (const auto& joint : m_model->joints_) {
        if (joint.second->type == urdf::Joint::REVOLUTE ||    //
            joint.second->type == urdf::Joint::CONTINUOUS ||  //
            joint.second->type == urdf::Joint::PRISMATIC)
            m_actuated_joints[joint.first] = actuation_type;
    }
}

void ChParserURDF::SetBodyMeshCollisionType(const std::string& body_name, MeshCollisionType collision_type) {
    if (m_sys) {
        cerr << "WARNING: SetBodyMeshCollisionType must be called before PopulateSystem." << endl;
        return;
    }

    auto link = m_model->getLink(body_name);
    if (!link) {
        cerr << "WARNING: SetBodyContactMaterial: No body named \"" << body_name << "\"." << endl;
        return;
    }

    m_coll_type[body_name] = collision_type;
}

void ChParserURDF::SetAllBodiesMeshCollisinoType(MeshCollisionType collision_type) {
    if (m_sys) {
        cerr << "WARNING: SetAllBodiesMeshCollisinoType must be called before PopulateSystem." << endl;
        return;
    }

    for (const auto& link : m_model->links_)
        m_coll_type[link.first] = collision_type;
}

void ChParserURDF::SetDefaultContactMaterial(const ChContactMaterialData& mat_data) {
    if (m_sys) {
        cerr << "WARNING: SetDefaultContactMaterial must be called before PopulateSystem." << endl;
        return;
    }

    m_default_mat_data = mat_data;
}

void ChParserURDF::SetBodyContactMaterial(const std::string& body_name, const ChContactMaterialData& mat_data) {
    if (m_sys) {
        cerr << "WARNING: SetBodyContactMaterial must be called before PopulateSystem." << endl;
        return;
    }

    auto link = m_model->getLink(body_name);
    if (!link) {
        cerr << "WARNING: SetBodyContactMaterial: No body named \"" << body_name << "\"." << endl;
        return;
    }

    m_mat_data[body_name] = mat_data;
}

void ChParserURDF::EnableCollisionVisualization() {
    if (m_sys) {
        cerr << "WARNING: EnableCollisionVisualization must be called before PopulateSystem." << endl;
        return;
    }

    m_vis_collision = true;
}

// -----------------------------------------------------------------------------

void ChParserURDF::PopulateSystem(ChSystem& sys) {
    // Cache the containing Chrono system
    m_sys = &sys;

    // Start at the root body, create the root (if necessary),
    // then traverse all links recursively to populate the Chrono system
    auto root_link = m_model->getRoot();
    ChFrame<> frame = m_init_pose;
    if (root_link->inertial) {
        m_root_body = toChBody(root_link);
        m_root_body->SetFrame_REF_to_abs(frame);
        m_sys->AddBody(m_root_body);
    }
    createChildren(root_link, frame);
}

void ChParserURDF::createChildren(urdf::LinkConstSharedPtr parent, const ChFrame<>& parent_frame) {
    for (auto child = parent->child_links.begin(); child != parent->child_links.end(); ++child) {
        // Get the parent joint of the child link and that joint's transform from the parent.
        // This provides the position of the child w.r.t. its parent.
        auto P2C_frame = toChFrame((*child)->parent_joint->parent_to_joint_origin_transform);

        // Evaluate the position of the child w.r.t. the absolute frame
        auto child_frame = parent_frame * P2C_frame;

        // Create the child Chrono body
        auto body = toChBody(*child);
        if (body) {
            body->SetFrame_REF_to_abs(child_frame);
            m_sys->AddBody(body);
        }

        // Set this as the root body of the model if not already set
        if (!m_root_body)
            m_root_body = body;

        // Create the Chrono link between parent and child
        auto link = toChLink((*child)->parent_joint);
        if (link) {
            m_sys->AddLink(link);
        }

        // Process grandchildren
        createChildren(*child, child_frame);
    }
}

// -----------------------------------------------------------------------------

ChVector<> ChParserURDF::toChVector(const urdf::Vector3& vec) {
    return ChVector<>(vec.x, vec.y, vec.z);
}

ChQuaternion<> ChParserURDF::toChQuaternion(const urdf::Rotation& rot) {
    return ChQuaternion<>(rot.w, rot.x, rot.y, rot.z);
}

ChFrame<> ChParserURDF::toChFrame(const urdf::Pose& pose) {
    return ChFrame<>(toChVector(pose.position), toChQuaternion(pose.rotation));
}

ChColor ChParserURDF::toChColor(const urdf::Color& color) {
    return ChColor(color.r, color.g, color.b);
}

std::shared_ptr<ChVisualShape> ChParserURDF::toChVisualShape(const urdf::GeometrySharedPtr geometry) {
    std::shared_ptr<ChVisualShape> vis_shape;
    switch (geometry->type) {
        case urdf::Geometry::BOX: {
            auto box = std::static_pointer_cast<urdf::Box>(geometry);
            vis_shape = chrono_types::make_shared<ChVisualShapeBox>(box->dim.x, box->dim.y, box->dim.z);
            break;
        }
        case urdf::Geometry::CYLINDER: {
            auto cylinder = std::static_pointer_cast<urdf::Cylinder>(geometry);
            vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cylinder->radius, cylinder->length);
            break;
        }
        case urdf::Geometry::SPHERE: {
            auto sphere = std::static_pointer_cast<urdf::Sphere>(geometry);
            vis_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere->radius);
            break;
        }
        case urdf::Geometry::MESH: {
            auto mesh = std::static_pointer_cast<urdf::Mesh>(geometry);
            auto modelfile_shape = chrono_types::make_shared<ChVisualShapeModelFile>();
            modelfile_shape->SetFilename(m_filepath + "/" + mesh->filename);
            modelfile_shape->SetScale(toChVector(mesh->scale));
            vis_shape = modelfile_shape;
            break;
        }
    }

    return vis_shape;
}

void ChParserURDF::attachVisualization(std::shared_ptr<ChBody> body,
                                       urdf::LinkConstSharedPtr link,
                                       const ChFrame<>& ref_frame) {
    if (m_vis_collision) {
        const auto& collision_array = link->collision_array;
        for (const auto& collision : collision_array) {
            if (collision) {
                auto vis_shape = toChVisualShape(collision->geometry);
                body->AddVisualShape(vis_shape, ref_frame * toChFrame(collision->origin));
            }
        }
    } else {
        const auto& visual_array = link->visual_array;
        for (const auto& visual : visual_array) {
            if (visual) {
                auto vis_shape = toChVisualShape(visual->geometry);
                if (visual->material) {
                    vis_shape->SetColor(toChColor(visual->material->color));
                    if (!visual->material->texture_filename.empty())
                        vis_shape->SetTexture(m_filepath + "/" + visual->material->texture_filename);
                }
                body->AddVisualShape(vis_shape, ref_frame * toChFrame(visual->origin));
            }
        }
    }
}

void ChParserURDF::attachCollision(std::shared_ptr<ChBody> body,
                                   urdf::LinkConstSharedPtr link,
                                   const ChFrame<>& ref_frame) {
    const auto& collision_array = link->collision_array;

    // Create the contact material for all collision shapes associated with this body
    auto link_name = body->GetNameString();
    std::shared_ptr<ChMaterialSurface> contact_material;
    if (m_mat_data.find(link_name) != m_mat_data.end())
        contact_material = m_mat_data.find(link_name)->second.CreateMaterial(m_sys->GetContactMethod());
    else
        contact_material = m_default_mat_data.CreateMaterial(m_sys->GetContactMethod());

    // Create collision shapes
    // Note: a collision model is created for this body when the first collsion shape is added
    for (const auto& collision : collision_array) {
        if (collision) {
            auto frame = ref_frame * toChFrame(collision->origin);

            switch (collision->geometry->type) {
                case urdf::Geometry::BOX: {
                    auto box = std::static_pointer_cast<urdf::Box>(collision->geometry);
                    auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(contact_material, box->dim.x,
                                                                                   box->dim.y, box->dim.z);
                    body->AddCollisionShape(ct_shape, frame);
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    auto cylinder = std::static_pointer_cast<urdf::Cylinder>(collision->geometry);
                    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(
                        contact_material, cylinder->radius, cylinder->length);
                    body->AddCollisionShape(ct_shape, frame);
                    break;
                }
                case urdf::Geometry::SPHERE: {
                    auto sphere = std::static_pointer_cast<urdf::Sphere>(collision->geometry);
                    auto ct_shape = chrono_types::make_shared<ChCollisionShapeSphere>(contact_material, sphere->radius);
                    body->AddCollisionShape(ct_shape, frame);
                    break;
                }
                case urdf::Geometry::MESH: {
                    auto mesh = std::static_pointer_cast<urdf::Mesh>(collision->geometry);
                    auto mesh_filename = m_filepath + "/" + mesh->filename;
                    auto ext = filesystem::path(mesh->filename).extension();

                    std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh;
                    if (ext == "obj" || ext == "OBJ")
                        trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, false);
                    else if (ext == "stl" || ext == "STL")
                        trimesh = geometry::ChTriangleMeshConnected::CreateFromSTLFile(mesh_filename, true);

                    if (!trimesh) {
                        cout << "Warning: Unsupported format for collision mesh file <" << mesh_filename << ">." << endl;
                        cout << "Warning: No collision shape was generated for body <" << link_name << ">.\n" << endl;
                        break;
                    }

                    MeshCollisionType coll_type = m_coll_type.find(link_name) != m_coll_type.end()
                                                      ? m_coll_type.find(link_name)->second
                                                      : MeshCollisionType::TRIANGLE_MESH;
                    switch (coll_type) {
                        case MeshCollisionType::TRIANGLE_MESH: {
                            auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                                contact_material, trimesh, false, false, 0.002);
                            body->AddCollisionShape(ct_shape, frame);
                            break;
                        }
                        case MeshCollisionType::CONVEX_HULL: {
                            auto ct_shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(
                                contact_material, trimesh->getCoordsVertices());
                            body->AddCollisionShape(ct_shape, frame);
                            break;
                        }
                        case MeshCollisionType::NODE_CLOUD: {
                            for (const auto& v : trimesh->getCoordsVertices()) {
                                auto ct_shape =
                                    chrono_types::make_shared<ChCollisionShapeSphere>(contact_material, 0.002);
                                body->AddCollisionShape(ct_shape, ChFrame<>(v, QUNIT));
                            }
                            break;
                        }
                    }
                    break;
                }
            }
            collision->origin;
        }
    }
}

bool Discard(urdf::LinkConstSharedPtr link) {
    const auto& inertial = link->inertial;
    if (!inertial)
        return true;

    if (inertial->mass < inertia_threshold ||  //
        inertial->ixx < inertia_threshold || inertial->iyy < inertia_threshold || inertial->izz < inertia_threshold)
        return true;

    return false;
}

// Create a body (with default collision model type) from the provided URDF link.
// This is called in a base-to-tip traversal, so the parent Chrono body exists.
std::shared_ptr<ChBodyAuxRef> ChParserURDF::toChBody(urdf::LinkConstSharedPtr link) {
    // Discard bodies with zero inertia properties
    if (Discard(link)) {
        cerr << "WARNING: Body " << link->name << " has ZERO inertia." << endl;

        // Error if a discarded body was connected to its parent with anything but a FIXED joint
        if (link->parent_joint->type != urdf::Joint::FIXED) {
            cerr << "ERROR: Body with ZERO inertia not connected through FIXED joint to parent." << endl;
            throw ChException("Body with ZERO inertia not connected through FIXED joint to parent.");
        }

        // Get the parent link and the Chrono parent body
        const auto& parent_link_name = link->parent_joint->parent_link_name;
        const auto& parent_body = m_sys->SearchBody(parent_link_name);

        // Add to the list of discarded bodies and cache the body's parent
        // (this will be used to attach children joints of this body)
        m_discarded.insert(std::make_pair(link->name, parent_link_name));

        // Transfer visualization and collision assets to parent body
        attachVisualization(parent_body, link, toChFrame(link->parent_joint->parent_to_joint_origin_transform));
        attachCollision(parent_body, link, toChFrame(link->parent_joint->parent_to_joint_origin_transform));

        return nullptr;
    }

    // Get inertia properties
    // Note that URDF and Chrono use the same convention regarding sign of products of inertia
    const auto& inertial = link->inertial;
    double mass = inertial->mass;
    auto inertia_moments = ChVector<>(inertial->ixx, inertial->iyy, inertial->izz);
    auto inertia_products = ChVector<>(inertial->ixy, inertial->ixz, inertial->iyz);

    // Create the Chrono body
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetNameString(link->name);
    body->SetFrame_COG_to_REF(toChFrame(inertial->origin));
    body->SetMass(mass);
    body->SetInertiaXX(inertia_moments);
    body->SetInertiaXY(inertia_products);

    // Create and attach visualization and collision assets
    attachVisualization(body, link, ChFrame<>());
    attachCollision(body, link, ChFrame<>());

    return body;
}

std::shared_ptr<ChLink> ChParserURDF::toChLink(urdf::JointSharedPtr& joint) {
    auto joint_name = joint->name;
    auto joint_type = joint->type;

    // Get the names of the parent and child bodies
    auto parent_link_name = joint->parent_link_name;
    auto child_link_name = joint->child_link_name;

    // If the child is a discarded body, do not create this joint
    if (m_discarded.find(child_link_name) != m_discarded.end()) {
        assert(joint_type == urdf::Joint::FIXED);
        return nullptr;
    }

    // If the parent is a discarded body, use the grandparent
    if (m_discarded.find(parent_link_name) != m_discarded.end()) {
        parent_link_name = m_discarded.find(parent_link_name)->second;
    }

    // Find the parent and child Chrono bodies
    const auto& parent = m_sys->SearchBody(parent_link_name);
    const auto& child = m_sys->SearchBody(child_link_name);

    // The parent body may not have been created (e.g., when using a dummy root), but the child must always exist.
    if (!parent)
        return nullptr;
    if (!child) {
        cerr << "ERROR: Body " << child_link_name << " not found." << endl;
        throw ChException("Body not found.");
    }

    // Create 3 mutually orthogonal directions, with d1 being the joint axis.
    // These form a rotation matrix relative to the child frame (in the URDF representation, a body reference frame
    // coincides with the frame of the joint connecting the body to its parent).
    auto joint_axis = toChVector(joint->axis);
    ChVector<> d1, d2, d3;
    joint_axis.DirToDxDyDz(d1, d2, d3);

    // Create motors or passive joints
    ChFrame<> joint_frame = child->GetFrame_REF_to_abs();  // default joint frame == child body frame

    if (m_actuated_joints.find(joint->name) != m_actuated_joints.end()) {
        // Create a motor (with a default zero constant motor function)
        auto actuation_type = m_actuated_joints.find(joint->name)->second;
        auto actuation_fun = chrono_types::make_shared<ChFunction_Const>(0);

        if (joint_type == urdf::Joint::REVOLUTE || joint_type == urdf::Joint::CONTINUOUS) {
            std::shared_ptr<ChLinkMotorRotation> revolute;
            switch (actuation_type) {
                case ChParserURDF::ActuationType::POSITION:
                    revolute = chrono_types::make_shared<ChLinkMotorRotationAngle>();
                    break;
                case ChParserURDF::ActuationType::SPEED:
                    revolute = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
                    break;
                case ChParserURDF::ActuationType::FORCE:
                    revolute = chrono_types::make_shared<ChLinkMotorRotationTorque>();
                    break;
            }
            joint_frame.SetRot(joint_frame.Amatrix * ChMatrix33<>(d2, d3, d1));  // Chrono rot. motor axis along Z
            revolute->Initialize(parent, child, joint_frame);
            revolute->SetMotorFunction(actuation_fun);
            revolute->SetNameString(joint_name);
            return revolute;
        }

        if (joint_type == urdf::Joint::PRISMATIC) {
            std::shared_ptr<ChLinkMotorLinear> prismatic;
            switch (actuation_type) {
                case ChParserURDF::ActuationType::POSITION:
                    prismatic = chrono_types::make_shared<ChLinkMotorLinearPosition>();
                    break;
                case ChParserURDF::ActuationType::SPEED:
                    prismatic = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
                    break;
                case ChParserURDF::ActuationType::FORCE:
                    prismatic = chrono_types::make_shared<ChLinkMotorLinearForce>();
                    break;
            }
            joint_frame.SetRot(joint_frame.Amatrix * ChMatrix33<>(d1, d2, d3));  // Chrono lin. motor axis along X
            prismatic->Initialize(parent, child, joint_frame);
            prismatic->SetMotorFunction(actuation_fun);
            prismatic->SetNameString(joint_name);
            return prismatic;
        }
    } else {
        // Create a passive joint
        if (joint_type == urdf::Joint::REVOLUTE || joint_type == urdf::Joint::CONTINUOUS) {
            auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
            if (joint_type == urdf::Joint::REVOLUTE) {
                revolute->GetLimit_Rz().SetActive(true);
                revolute->GetLimit_Rz().SetMin(joint->limits->lower);
                revolute->GetLimit_Rz().SetMax(joint->limits->upper);
            }
            joint_frame.SetRot(joint_frame.Amatrix * ChMatrix33<>(d2, d3, d1));  // Chrono revolute axis along Z
            revolute->Initialize(parent, child, joint_frame.GetCoord());
            revolute->SetNameString(joint_name);
            return revolute;
        }

        if (joint_type == urdf::Joint::PRISMATIC) {
            auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
            {
                prismatic->GetLimit_Rz().SetActive(true);
                prismatic->GetLimit_Rz().SetMin(joint->limits->lower);
                prismatic->GetLimit_Rz().SetMax(joint->limits->upper);
            }
            joint_frame.SetRot(joint_frame.Amatrix * ChMatrix33<>(d2, d3, d1));  // Chrono prismatic axis along Z
            prismatic->Initialize(parent, child, joint_frame.GetCoord());
            prismatic->SetNameString(joint_name);
            return prismatic;
        }

        if (joint_type == urdf::Joint::FLOATING) {
            auto free = chrono_types::make_shared<ChLinkLockFree>();
            free->Initialize(parent, child, joint_frame.GetCoord());
            free->SetNameString(joint_name);
            return free;
        }

        if (joint_type == urdf::Joint::PLANAR) {
            auto planar = chrono_types::make_shared<ChLinkLockPointPlane>();
            joint_frame.SetRot(joint_frame.Amatrix * ChMatrix33<>(d2, d3, d1));  // Chrono plane normal along Z
            planar->Initialize(parent, child, joint_frame.GetCoord());
            planar->SetNameString(joint_name);
            return planar;
        }

        if (joint_type == urdf::Joint::FIXED) {
            auto fixed = chrono_types::make_shared<ChLinkLockLock>();
            fixed->Initialize(parent, child, joint_frame.GetCoord());
            fixed->SetNameString(joint_name);
            return fixed;
        }
    }

    return nullptr;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChBodyAuxRef> ChParserURDF::GetRootChBody() const {
    if (!m_sys) {
        cerr << "\nWARNING: GetRootChBody: The Chrono model was not yet populated." << endl;
        return nullptr;
    }

    return m_root_body;
}

std::shared_ptr<ChBody> ChParserURDF::GetChBody(const std::string& name) const {
    if (!m_sys) {
        cerr << "\nWARNING: GetChBody: The Chrono model was not yet populated." << endl;
        return nullptr;
    }

    return m_sys->SearchBody(name);
}

std::shared_ptr<ChLinkBase> ChParserURDF::GetChLink(const std::string& name) const {
    if (!m_sys) {
        cerr << "\nWARNING: GetChLink: The Chrono model was not yet populated." << endl;
        return nullptr;
    }

    return m_sys->SearchLink(name);
}

std::shared_ptr<ChLinkMotor> ChParserURDF::GetChMotor(const std::string& name) const {
    if (!m_sys) {
        cerr << "\nWARNING: GetChMotor: The Chrono model was not yet populated." << endl;
        return nullptr;
    }

    if (m_actuated_joints.find(name) == m_actuated_joints.end()) {
        cerr << "\nWARNING: GetChMotor: The joint \"" << name << "\" was not marked as actuated." << endl;
        return nullptr;
    }

    return std::static_pointer_cast<ChLinkMotor>(m_sys->SearchLink(name));
}

void ChParserURDF::SetMotorFunction(const std::string& motor_name, const std::shared_ptr<ChFunction> function) {
    if (!m_sys) {
        cerr << "\nWARNING: SetMotorFunction: The Chrono model was not yet populated." << endl;
        return;
    }

    if (!m_model->getJoint(motor_name)) {
        cerr << "\nWARNING: SetMotorFunction: No joint named \"" << motor_name << "\"." << endl;
        return;
    }

    if (m_actuated_joints.find(motor_name) == m_actuated_joints.end()) {
        cerr << "\nWARNING: SetMotorFunction: The joint \"" << motor_name << "\" was not marked as actuated." << endl;
        return;
    }

    std::static_pointer_cast<ChLinkMotor>(m_sys->SearchLink(motor_name))->SetMotorFunction(function);
}

// -----------------------------------------------------------------------------

// From check_urdf.cpp in the urdfdom distribution
void printBodyTree(urdf::LinkConstSharedPtr link, int level = 0) {
    level += 2;
    int count = 0;
    for (auto child = link->child_links.begin(); child != link->child_links.end(); ++child) {
        if (*child) {
            for (int j = 0; j < level; j++)
                cout << "  ";  // indent
            cout << "child(" << (count++) + 1 << "):  " << (*child)->name << endl;
            // first grandchild
            printBodyTree(*child, level);
        } else {
            for (int j = 0; j < level; j++)
                cout << " ";  // indent
            cout << "root link: " << link->name << " has a null child!" << *child << endl;
        }
    }
}

void ChParserURDF::PrintModelBodyTree() {
    auto root_link = m_model->getRoot();
    cout << "Body tree in <" << m_model->getName() << "> model" << endl;
    cout << "  Root body " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << endl;
    printBodyTree(root_link);
    cout << endl;
}

// -----------------------------------------------------------------------------

void ChParserURDF::PrintModelBodies() {
    cout << "Joint list in <" << m_model->getName() << "> model" << endl;
    std::vector<urdf::LinkSharedPtr> links;
    m_model->getLinks(links);
    for (const auto& link : links) {
        bool collision = !link->collision_array.empty();

        cout << "Link: " << std::left << std::setw(25) << link->name;
        cout << "discarded? " << (Discard(link) ? "Y    " : "     ");
        cout << "collision? " << (collision ? "Y    " : "     ");
        cout << endl;
    }
    cout << endl;
}

// -----------------------------------------------------------------------------

void ChParserURDF::PrintModelJoints() {
    cout << "Joint list in <" << m_model->getName() << "> model" << endl;
    std::vector<urdf::LinkSharedPtr> links;
    m_model->getLinks(links);
    for (const auto& link : links) {
        auto joint = link->parent_joint;
        if (!joint)
            continue;

        cout << (m_actuated_joints.find(joint->name) == m_actuated_joints.end() ? "[P] " : "[A] ");
        switch (joint->type) {
            case urdf::Joint::REVOLUTE:
                cout << "revolute  ";
                break;
            case urdf::Joint::PRISMATIC:
                cout << "prismatic ";
                break;
            case urdf::Joint::PLANAR:
                cout << "planar    ";
                break;
            case urdf::Joint::FLOATING:
                cout << "floating  ";
                break;
            case urdf::Joint::FIXED:
                cout << "fixed     ";
                break;
            case urdf::Joint::CONTINUOUS:
                cout << "continous ";
                break;
        }
        cout << "Joint: " << std::left << std::setw(25) << joint->name;
        cout << "Link: " << std::left << std::setw(25) << link->name;
        cout << "Parent:" << std::left << std::setw(25) << joint->parent_link_name << endl;
    }
    cout << endl;
}

// -----------------------------------------------------------------------------

void ChParserURDF::CustomProcess(const std::string& key, std::shared_ptr<CustomProcessor> callback) {
    tinyxml2::XMLDocument xml_doc;
    xml_doc.Parse(m_xml_string.c_str());
    if (xml_doc.Error()) {
        std::cerr << xml_doc.ErrorStr() << std::endl;
        xml_doc.Clear();
        return;
    }

    tinyxml2::XMLElement* robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml) {
        std::cerr << "Could not find the 'robot' element in the xml file" << std::endl;
        return;
    }

    for (tinyxml2::XMLElement* xml_element = robot_xml->FirstChildElement(key.c_str());  //
         xml_element;                                                                    //
         xml_element = xml_element->NextSiblingElement(key.c_str())                      //
    ) {
        callback->Process(*xml_element, *m_sys);
    }
}

}  // end namespace parsers
}  // end namespace chrono
