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

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChModelFileShape.h"

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

ChParserURDF::ChParserURDF(const std::string& filename) : m_filename(filename), m_sys(nullptr) {
    // Read input file into XML string
    std::string xml_string;
    std::fstream xml_file(filename, std::fstream::in);
    while (xml_file.good()) {
        std::string line;
        std::getline(xml_file, line);
        xml_string += (line + "\n");
    }
    xml_file.close();

    // Parse XML string
    m_model = urdf::parseURDF(xml_string);
    if (!m_model) {
        cerr << "ERROR: parsing the URDF file " << filename << " failed." << endl;
        return;
    }

    // Cache path to the URDF model file
    m_filepath = filesystem::path(filename).parent_path().str();
}

void ChParserURDF::SetRootInitPose(const ChFrame<>& init_pose) {
    m_init_pose = init_pose;
}

void ChParserURDF::SetJointActuated(const std::string& joint_name, ActuationType actuation_type) {
    auto joint = m_model->getJoint(joint_name);
    if (!joint) {
        cerr << "WARNING: SetJointActuated: No joint named \"" << joint_name << "\"." << endl;
        return;
    }

    if (joint->type == urdf::Joint::REVOLUTE ||    //
        joint->type == urdf::Joint::CONTINUOUS ||  //
        joint->type == urdf::Joint::PRISMATIC)
        m_actuated_joints.insert(std::make_pair(joint_name, actuation_type));
    else
        cerr << "WARNING: SetJointActuated: Joint \"" << joint_name << "\" cannot be actuated." << endl;
}

void ChParserURDF::SetAllJointsActuated(ActuationType actuation_type) {
    for (auto joint : m_model->joints_) {
        if (joint.second->type == urdf::Joint::REVOLUTE ||    //
            joint.second->type == urdf::Joint::CONTINUOUS ||  //
            joint.second->type == urdf::Joint::PRISMATIC)
            m_actuated_joints.insert(std::make_pair(joint.first, actuation_type));
    }
}

void ChParserURDF::SetDefaultContactMaterial(const ChContactMaterialData& mat_data) {
    m_default_mat_data = mat_data;
}

void ChParserURDF::SetBodyContactMaterial(const std::string& body_name, const ChContactMaterialData& mat_data) {
    auto link = m_model->getLink(body_name);
    if (!link)
        return;

    m_mat_data.insert(std::make_pair(body_name, mat_data));
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
            vis_shape = chrono_types::make_shared<ChBoxShape>(box->dim.x, box->dim.y, box->dim.z);
            break;
        }
        case urdf::Geometry::CYLINDER: {
            auto cylinder = std::static_pointer_cast<urdf::Cylinder>(geometry);
            vis_shape = chrono_types::make_shared<ChCylinderShape>(cylinder->radius, cylinder->length);
            break;
        }
        case urdf::Geometry::SPHERE: {
            auto sphere = std::static_pointer_cast<urdf::Sphere>(geometry);
            vis_shape = chrono_types::make_shared<ChSphereShape>(sphere->radius);
            break;
        }
        case urdf::Geometry::MESH: {
            auto mesh = std::static_pointer_cast<urdf::Mesh>(geometry);
            auto modelfile_shape = chrono_types::make_shared<ChModelFileShape>();
            modelfile_shape->SetFilename(m_filepath + "/" + mesh->filename);
            vis_shape = modelfile_shape;
            break;
        }
    }

    return vis_shape;
}

void ChParserURDF::attachVisualization(std::shared_ptr<ChBody> body,
                                       const std::vector<urdf::VisualSharedPtr>& visual_array,
                                       const ChFrame<>& ref_frame) {
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

void ChParserURDF::attachCollision(std::shared_ptr<ChBody> body,
                                   const std::vector<urdf::CollisionSharedPtr>& collision_array,
                                   const ChFrame<>& ref_frame) {
    // Create the contact material for all collision shapes associated with this body
    auto link_name = body->GetNameString();
    std::shared_ptr<ChMaterialSurface> contact_material;
    if (m_mat_data.find(link_name) != m_mat_data.end())
        contact_material = m_mat_data.find(link_name)->second.CreateMaterial(m_sys->GetContactMethod());
    else
        contact_material = m_default_mat_data.CreateMaterial(m_sys->GetContactMethod());

    // Create collision shapes
    auto collision_model = body->GetCollisionModel();
    collision_model->ClearModel();
    for (const auto& collision : collision_array) {
        if (collision) {
            auto frame = ref_frame * toChFrame(collision->origin);

            switch (collision->geometry->type) {
                case urdf::Geometry::BOX: {
                    auto box = std::static_pointer_cast<urdf::Box>(collision->geometry);
                    collision_model->AddBox(contact_material,                    //
                                            box->dim.x, box->dim.y, box->dim.z,  //
                                            frame.GetPos(), frame.GetA());
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    auto cylinder = std::static_pointer_cast<urdf::Cylinder>(collision->geometry);
                    collision_model->AddCylinder(contact_material,                    //
                                                 cylinder->radius, cylinder->length,  //
                                                 frame.GetPos(), frame.GetA());
                    break;
                }
                case urdf::Geometry::SPHERE: {
                    auto sphere = std::static_pointer_cast<urdf::Sphere>(collision->geometry);
                    collision_model->AddSphere(contact_material,  //
                                               sphere->radius,    //
                                               frame.GetPos());
                    break;
                }
                case urdf::Geometry::MESH: {
                    auto mesh = std::static_pointer_cast<urdf::Mesh>(collision->geometry);
                    //// TODO: we can only support OBJ files!
                    ////       use assimp?
                    break;
                }
            }
            collision->origin;
        }
    }
    collision_model->BuildModel();
}

// Create a body (with default collision model type) from the provided URDF link
std::shared_ptr<ChBodyAuxRef> ChParserURDF::toChBody(urdf::LinkConstSharedPtr link) {
    // Get inertia properties
    // Note that URDF and Chrono use the same convention regarding sign of products of inertia
    const auto& inertial = link->inertial;
    double mass = inertial->mass;
    auto inertia_moments = ChVector<>(inertial->ixx, inertial->iyy, inertial->izz);
    auto inertia_products = ChVector<>(inertial->ixy, inertial->ixz, inertial->iyz);

    // Discard bodies with zero inertia properties
    if (mass < inertia_threshold ||                 //
        inertia_moments.x() < inertia_threshold ||  //
        inertia_moments.y() < inertia_threshold ||  //
        inertia_moments.z() < inertia_threshold) {
        cerr << "WARNING: Body " << link->name << " has ZERO inertia." << endl;

        // Error if a discarded body was connected to its parent with anything but a FIXED joint
        if (link->parent_joint->type != urdf::Joint::FIXED) {
            cerr << "ERROR: Body with ZERO inertia not connected through FIXED joint to parent." << endl;
            throw ChException("Body with ZERO inertia not connected through FIXED joint to parent.");
        }

        // Get the parent link and the Chrono parent body
        const auto& parent_link_name = link->parent_joint->parent_link_name;
        const auto& parent_link = m_model->getLink(parent_link_name);
        const auto& parent_body = m_sys->SearchBody(parent_link_name);

        // Add to the list of discarded bodies and cache the body's parent
        // (this will be used to attach children joints of this body)
        m_discarded.insert(std::make_pair(link->name, parent_link_name));

        // Transfer visualization and collision assets to parent body
        attachVisualization(parent_body, link->visual_array,
                            toChFrame(link->parent_joint->parent_to_joint_origin_transform));
        attachCollision(parent_body, link->collision_array,
                        toChFrame(link->parent_joint->parent_to_joint_origin_transform));

        return nullptr;
    }

    // Create the Chrono body
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetNameString(link->name);
    body->SetFrame_COG_to_REF(toChFrame(inertial->origin));
    body->SetMass(mass);
    body->SetInertiaXX(inertia_moments);
    body->SetInertiaXY(inertia_products);

    // Create and attach visualization assets
    attachVisualization(body, link->visual_array, ChFrame<>());

    // Create and attach collision shapes
    attachCollision(body, link->collision_array, ChFrame<>());

    return body;
}

std::shared_ptr<ChLink> ChParserURDF::toChLink(urdf::JointSharedPtr& joint) {
    auto joint_name = joint->name;
    auto joint_type = joint->type;

    // Find the parent and child bodies
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

    const auto& parent = m_sys->SearchBody(parent_link_name);
    const auto& child = m_sys->SearchBody(child_link_name);

    // The parent body may not have been created (e.g., when using a dummy root),
    // but the child must always exist.
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
    ChFrame<> joint_frame = *child;  // default joint frame == child body frame

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

std::shared_ptr<ChBodyAuxRef> ChParserURDF::GetRootBody() const {
    if (!m_sys)
        cerr << "WARNING: GetRootBody: The Chrono model was not yet populated." << endl;

    return m_root_body;
}

void ChParserURDF::SetMotorFunction(const std::string& motor_name, const std::shared_ptr<ChFunction> function) {
    if (!m_sys) {
        cerr << "WARNING: SetMotorFunction: The Chrono model was not yet populated." << endl;
        return;
    }

    if (!m_model->getJoint(motor_name)) {
        cerr << "WARNING: SetMotorFunction: No joint named \"" << motor_name << "\"." << endl;
        return;
    }

    if (m_actuated_joints.find(motor_name) == m_actuated_joints.end()) {
        cerr << "WARNING: SetMotorFunction: The joint \"" << motor_name << "\" was not marked as actuated." << endl;
        return;
    }

    auto motor =
        std::find_if(std::begin(m_sys->Get_linklist()), std::end(m_sys->Get_linklist()),
                     [motor_name](std::shared_ptr<ChLinkBase> link) { return link->GetNameString() == motor_name; });

    std::static_pointer_cast<ChLinkMotor>(*motor)->SetMotorFunction(function);
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

void ChParserURDF::PrintModelBodies() {
    auto root_link = m_model->getRoot();
    cout << "Body list in <" << m_model->getName() << "> model" << endl;
    cout << "  Root body " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << endl;
    printBodyTree(root_link);
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
        cout << "Joint: " << std::left << std::setw(30) << joint->name;
        cout << "Link: " << std::left << std::setw(20) << link->name;
        cout << "Parent:" << std::left << std::setw(20) << joint->parent_link_name << std::endl;
    }
    cout << endl;
}

// -----------------------------------------------------------------------------

}  // end namespace parsers
}  // end namespace chrono
