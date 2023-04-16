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
#include <fstream>
#include <iostream>
#include <algorithm>

#include "chrono_parsers/ChParserURDF.h"

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChModelFileShape.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace parsers {

using std::cout;
using std::cerr;
using std::endl;

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
            //// TODO deal with mismatching Chrono cylinder definition
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

// Create a body (with default collision model type) from the provided URDF link
std::shared_ptr<ChBodyAuxRef> ChParserURDF::toChBody(urdf::LinkConstSharedPtr link) {
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetNameString(link->name);

    // Set inertia properties
    const auto& inertial = link->inertial;
    body->SetFrame_COG_to_REF(toChFrame(inertial->origin));
    body->SetMass(inertial->mass);
    //// TODO:
    //// - check if inertial has inertias w.r.t. COG or REF
    //// - check convention for signs of products of inertia
    body->SetInertiaXX(ChVector<>(inertial->ixx, inertial->iyy, inertial->izz));
    body->SetInertiaXY(ChVector<>(inertial->ixy, inertial->ixz, inertial->iyz));

    // Set visualization assets
    for (const auto& visual : link->visual_array) {
        if (visual) {
            auto vis_shape = toChVisualShape(visual->geometry);
            if (visual->material) {
                vis_shape->SetColor(toChColor(visual->material->color));
                vis_shape->SetTexture(m_filepath + "/" + visual->material->texture_filename);
            }
            body->AddVisualShape(vis_shape, toChFrame(visual->origin));
        }
    }

    // Set collision
    for (const auto& collision : link->collision_array) {
        if (collision) {
            switch (collision->geometry->type) {
                case urdf::Geometry::BOX:
                    break;
                case urdf::Geometry::CYLINDER:
                    break;
                case urdf::Geometry::SPHERE:
                    break;
                case urdf::Geometry::MESH:
                    break;
            }
            collision->origin;
        }
    }

    return body;
}

std::shared_ptr<ChLink> ChParserURDF::toChLink(urdf::JointSharedPtr& joint) {
    auto link = chrono_types::make_shared<ChLink>();
    link->SetNameString(joint->name);

    // Find the parent and child bodies
    const auto& parent_link_name = joint->parent_link_name;
    const auto& child_link_name = joint->child_link_name;
    auto parent = std::find_if(
        std::begin(m_sys->Get_bodylist()), std::end(m_sys->Get_bodylist()),
        [parent_link_name](std::shared_ptr<ChBody> body) { return body->GetNameString() == parent_link_name; });
    auto child = std::find_if(
        std::begin(m_sys->Get_bodylist()), std::end(m_sys->Get_bodylist()),
        [child_link_name](std::shared_ptr<ChBody> body) { return body->GetNameString() == child_link_name; });

    // The parent body may not have been created (e.g., when using a dummy root)
    if (parent == m_sys->Get_bodylist().end())
        return nullptr;
    // The child body should always exist
    assert(child != m_sys->Get_bodylist().end());

    //// TODO
    auto axis = toChVector(joint->axis);
    switch (joint->type) {
        case urdf::Joint::REVOLUTE: {
            auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
            ////revolute->Initialize(parent, child, );
            link = revolute;
            break;
        }
        case urdf::Joint::PRISMATIC:{
            auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
            ////prismatic->Initialize(parent, child, );
            link = prismatic;
            break;
        }
        case urdf::Joint::FLOATING: {
            break;
        }
        case urdf::Joint::PLANAR: {
            break;
        }
        case urdf::Joint::FIXED: {
            auto fixed = chrono_types::make_shared<ChLinkLockLock>();
            ////fixed->Initialize(parent, child, );
            link = fixed;
            break;
        }
    }

    return link;
}

void ChParserURDF::populateSystem(urdf::LinkConstSharedPtr parent, const ChFrame<>& parent_frame) {
    for (auto child = parent->child_links.begin(); child != parent->child_links.end(); ++child) {
        // Get the parent joint of the child link and that joint's transform from the parent.
        // This provides the position of the child w.r.t. its parent.
        auto P2C_frame = toChFrame((*child)->parent_joint->parent_to_joint_origin_transform);

        // Evaluate the position of the child w.r.t. the absolute frame
        auto child_frame = parent_frame * P2C_frame;

        // Create the child Chrono body
        auto body = toChBody(*child);
        body->SetFrame_REF_to_abs(child_frame);
        m_sys->AddBody(body);

        // Create the Chrono link between parent and child
        auto link = toChLink((*child)->parent_joint);
        if (link) {
            m_sys->AddLink(link);
        }

        // Process grandchildren
        populateSystem(*child, child_frame);
    }
}

void ChParserURDF::Parse(ChSystem& sys, const std::string& filename) {
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

    // Cache the containing Chrono system
    m_sys = &sys;

    // Start at the root body, create the root (if necessary),
    // then traverse all links recursively to populate the Chrono system
    auto root_link = m_model->getRoot();
    ChFrame<> frame;
    if (root_link->inertial) {
        auto body = toChBody(root_link);
        body->SetFrame_REF_to_abs(frame);
        sys.AddBody(body);
    }
    populateSystem(root_link, frame);
}

// -----------------------------------------------------------------------------

// From check_urdf.cpp in the urdfdom distribution
void printTree(urdf::LinkConstSharedPtr link, int level = 0) {
    level += 2;
    int count = 0;
    for (auto child = link->child_links.begin(); child != link->child_links.end(); ++child) {
        if (*child) {
            for (int j = 0; j < level; j++)
                cout << "  ";  // indent
            cout << "child(" << (count++) + 1 << "):  " << (*child)->name << endl;
            // first grandchild
            printTree(*child, level);
        } else {
            for (int j = 0; j < level; j++)
                cout << " ";  // indent
            cout << "root link: " << link->name << " has a null child!" << *child << endl;
        }
    }
}

void ChParserURDF::PrintModelTree() {
    if (!m_model)
        return;

    auto root_link = m_model->getRoot();
    cout << "Name: " << m_model->getName() << endl;
    cout << "Root: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << endl;
    printTree(root_link);
}

// -----------------------------------------------------------------------------

}  // end namespace parsers
}  // end namespace chrono
