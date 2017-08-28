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
// Authors: Conlain Kelly
// =============================================================================
//
// Parser utility class for OpenSim input files.
//
// This uses several frame transforms for the forward kinematics pass. A
// transform is denoted by `X_1_2`, where 1 is the initial fram and 2 is the
// final frame. The frames are denoted g: global, P: parent, F: joint on parent,
// M: joint on body, B: body. Thus, X_P_F is the transform from parent to joint.
//
// =============================================================================

#include "chrono/utils/ChParserOpenSim.h"
#include "chrono_thirdparty/rapidxml/rapidxml_print.hpp"
#include "chrono_thirdparty/rapidxml/rapidxml_utils.hpp"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChCubicSpline.h"

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/utils/ChUtilsCreators.h"

#include <utility>

namespace chrono {
namespace utils {

using namespace rapidxml;
using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Constructor for the OpenSim parser.
// Initializes lambda table.
// -----------------------------------------------------------------------------

ChParserOpenSim::ChParserOpenSim()
    : m_collide(false),
      m_family_1(1),
      m_family_2(2),
      m_visType(VisType::NONE),
      m_verbose(false),
      m_friction(0.6f),
      m_restitution(0.4f),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_kn(2e5),
      m_kt(2e5),
      m_gn(40),
      m_gt(20) {
    initFunctionTable();
}

// -----------------------------------------------------------------------------
// Enable collision and specify collision families.
// -----------------------------------------------------------------------------

void ChParserOpenSim::EnableCollision(int family_1, int family_2) {
    assert(family_1 != family_2);
    assert(family_1 >= 0 && family_1 <= 15);
    assert(family_2 >= 0 && family_2 <= 15);

    m_family_1 = family_1;
    m_family_2 = family_2;
    m_collide = true;
}

// -----------------------------------------------------------------------------
// Parse an OpenSim file into an existing system.
// -----------------------------------------------------------------------------

void ChParserOpenSim::Parse(ChSystem& system, const std::string& filename) {
    rapidxml::file<char> file(filename.c_str());

    xml_document<> doc;
    doc.parse<0>(file.data());

    // Hold list of bodies
    xml_node<>* bodySet = doc.first_node()->first_node("Model")->first_node("BodySet")->first_node("objects");

    // Get gravity from model and set it in system
    auto elems = strToSTLVector<double>(doc.first_node()->first_node("Model")->first_node("gravity")->value());
    system.Set_G_acc(ChVector<>(elems[0], elems[1], elems[2]));

    // Traverse the list of bodies and parse the information for each one
    xml_node<>* bodyNode = bodySet->first_node();
    while (bodyNode != NULL) {
        parseBody(bodyNode, system);
        bodyNode = bodyNode->next_sibling();
    }

    initShapes(bodyNode, system);
}

// -----------------------------------------------------------------------------
// Makes a new system and then parses into it
// -----------------------------------------------------------------------------

ChSystem* ChParserOpenSim::Parse(const std::string& filename, ChMaterialSurface::ContactMethod contact_method) {
    ChSystem* sys = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                               : static_cast<ChSystem*>(new ChSystemSMC);

    Parse(*sys, filename);

    return sys;
}

// -----------------------------------------------------------------------------
// Create and add a new body.
// Parse its various properties from its XML child nodes.
// -----------------------------------------------------------------------------

bool ChParserOpenSim::parseBody(xml_node<>* bodyNode, ChSystem& system) {
    // Make a new body and name it for later
    if (m_verbose) {
        cout << "New body " << bodyNode->first_attribute("name")->value() << endl;
    }
    // Create a new body, consistent with the type of the containing system
    auto newBody = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
    newBody->SetName(bodyNode->first_attribute("name")->value());
    system.AddBody(newBody);

    // If body collision is enabled, set the contact material properties
    if (m_collide) {
        switch (newBody->GetContactMethod()) {
            case ChMaterialSurface::NSC:
                newBody->GetMaterialSurfaceNSC()->SetFriction(m_friction);
                newBody->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
                break;
            case ChMaterialSurface::SMC:
                newBody->GetMaterialSurfaceSMC()->SetFriction(m_friction);
                newBody->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
                newBody->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
                newBody->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
                newBody->GetMaterialSurfaceSMC()->SetKn(m_kn);
                newBody->GetMaterialSurfaceSMC()->SetGn(m_gn);
                newBody->GetMaterialSurfaceSMC()->SetKt(m_kt);
                newBody->GetMaterialSurfaceSMC()->SetGt(m_gt);
                break;
        }
    }

    newBody->SetCollide(m_collide);

    // Traverse the list of fields and parse the information for each one
    xml_node<>* fieldNode = bodyNode->first_node();
    while (fieldNode != NULL) {
        function_table[fieldNode->name()](fieldNode, newBody);
        fieldNode = fieldNode->next_sibling();
    }

    m_bodyList.push_back(newBody);

    return true;
}

// -----------------------------------------------------------------------------
// Set up the lambda table for body parsing.
// This is where the magic happens -- it sets up a map of strings to lambdas
// that do the heavy lifting of osim -> chrono parsing.
// Each entry in the function_table is a child of the "Body" XML node.
// -----------------------------------------------------------------------------

void ChParserOpenSim::initFunctionTable() {
    function_table["mass"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        if (std::stod(fieldNode->value()) == 0) {
            // Ground-like body, massless => fixed
            newBody->SetBodyFixed(true);
            newBody->SetCollide(false);
            newBody->SetPos(ChVector<>(0, 0, 0));
            // Mark as ground body
            newBody->AddAsset(std::make_shared<ChColorAsset>(0.0f, 0.0f, 0.0f));
        } else {
            // Give new body mass
            newBody->SetMass(std::stod(fieldNode->value()));
            // Add to bodies that are collision
        }
    };

    function_table["mass_center"] = [this](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set COM in reference frame
        auto elems = strToSTLVector<double>(fieldNode->value());
        // Opensim doesn't really use a rotated COM to REF frame, so unit quaternion
        newBody->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(elems[0], elems[1], elems[2]), ChQuaternion<>(1, 0, 0, 0)));
        // Only add vis balls if we're using primitives
    };

    function_table["inertia_xx"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xx inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };

    function_table["inertia_yy"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set yy inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };

    function_table["inertia_zz"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set zz inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };

    function_table["inertia_xy"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xy inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };

    function_table["inertia_xz"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };

    function_table["inertia_yz"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set yz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };

    function_table["Joint"] = [this](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // If there are no joints, this is hopefully the ground (or another global parent??)
        if (fieldNode->first_node() == NULL) {
            if (m_verbose)
                cout << "No joints for this body " << endl;
            return;
        }

        // Containing system
        auto system = newBody->GetSystem();

        // Deduce child body from joint orientation
        xml_node<>* jointNode = fieldNode->first_node();

        // Make a joint here
        if (m_verbose)
            cout << "Making a " << jointNode->name() << " with " << jointNode->first_node("parent_body")->value()
                 << endl;

        // Get other body for joint
        auto parent = system->SearchBody(jointNode->first_node("parent_body")->value());

        if (parent != nullptr) {
            if (m_verbose)
                cout << "other body found!" << endl;
        } else {
            // really bad, either file is invalid or we messed up
            if (m_verbose)
                cout << "Parent not found!!!! Exiting." << endl;
            exit(1);
        }

        // Global to parent transform
        auto X_G_P = parent->GetFrame_REF_to_abs();

        // Get joint's position in parent frame
        auto elems = ChParserOpenSim::strToSTLVector<double>(jointNode->first_node("location_in_parent")->value());
        ChVector<> jointPosInParent = ChVector<>(elems[0], elems[1], elems[2]);

        // Get joint's orientation in parent frame
        elems = ChParserOpenSim::strToSTLVector<double>(jointNode->first_node("orientation_in_parent")->value());
        ChQuaternion<> jointOrientationInParent = Q_from_AngX(elems[0]) * Q_from_AngY(elems[1]) * Q_from_AngZ(elems[2]);

        // Parent to joint in parent transform
        ChFrame<> X_P_F(jointPosInParent, jointOrientationInParent);

        // Offset location and orientation caused by joint initial configuration, default is identity
        ChFrame<> X_F_M(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));
        // Get offsets, depending on joint type
        if (std::string(jointNode->name()) == std::string("PinJoint")) {
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            // Just a rotation about Z
            double thetaZ = std::stod(coordinates->first_node("default_value")->value());
            X_F_M = Q_from_AngZ(thetaZ) * X_F_M;
        } else if ((std::string(jointNode->name()) == std::string("WeldJoint"))) {
            // Do absolutely nothing, they're stuck together
        } else if ((std::string(jointNode->name()) == std::string("UniversalJoint"))) {
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            // Body-fixed X,Y rotations
            double thetaX = std::stod(coordinates->first_node("default_value")->value());
            coordinates = coordinates->next_sibling();
            double thetaY = std::stod(coordinates->first_node("default_value")->value());
            X_F_M = Q_from_AngX(thetaX) * Q_from_AngY(thetaY) * X_F_M;
        } else if ((std::string(jointNode->name()) == std::string("BallJoint"))) {
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            // Body-fixed X,Y,Z rotations
            double thetaX = std::stod(coordinates->first_node("default_value")->value());
            coordinates = coordinates->next_sibling();
            double thetaY = std::stod(coordinates->first_node("default_value")->value());
            coordinates = coordinates->next_sibling();
            double thetaZ = std::stod(coordinates->first_node("default_value")->value());
            X_F_M = Q_from_AngX(thetaX) * Q_from_AngY(thetaY) * Q_from_AngZ(thetaZ) * X_F_M;
        } else if ((std::string(jointNode->name()) == std::string("CustomJoint"))) {
            // Make a map of generalized coordinates' default values so we can get initial position
            std::map<std::string, double> coordVals;
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            // Take in coordinate set
            while (coordinates != nullptr) {
                // Map coordinate name to its default value
                coordVals[std::string(coordinates->first_attribute("name")->value())] =
                    std::stod(coordinates->first_node("default_value")->value());
                coordinates = coordinates->next_sibling();
            }
            xml_node<>* transforms = jointNode->first_node("SpatialTransform")->first_node("TransformAxis");
            // First 3 are rotation, next 3 are translation
            int transformIndex = 0;
            while (transforms != nullptr) {
                // Type of OpenSim function
                std::string functionType(transforms->first_node("function")->first_node()->name());

                // Get coordinates involved
                auto coordNames =
                    ChParserOpenSim::strToSTLVector<std::string>(transforms->first_node("coordinates")->value());
                // There is no point in going on if the transform is empty
                if ((coordNames.size() == 0) && (functionType != std::string("Constant"))) {
                    // Still need to increment
                    transformIndex++;
                    transforms = transforms->next_sibling();
                    continue;
                }

                // Value to transform about/along axis, set later
                double transformValue;

                // Get axis to transform about/along
                auto axisElems = ChParserOpenSim::strToSTLVector<double>(transforms->first_node("axis")->value());
                ChVector<> axis(axisElems[0], axisElems[1], axisElems[2]);

                if (functionType == std::string("LinearFunction")) {
                    // ax + b style linear mapping
                    auto elems = ChParserOpenSim::strToSTLVector<double>(
                        transforms->first_node("function")->first_node()->first_node("coefficients")->value());
                    transformValue = elems[0] * coordVals[coordNames.at(0)] + elems[1];
                } else if (functionType == std::string("Constant")) {
                    // Just a constant
                    transformValue =
                        std::stod(transforms->first_node("function")->first_node()->first_node("value")->value());

                    // } else if (functionType == std::string("SimmSpline")) {
                } else if (functionType == std::string("NaturalCubicSpline") ||
                           functionType == std::string("SimmSpline")) {
                    // In opensim, both types of spline are treated as SimmSplines, which we approximate with
                    // Natural Cubic Splines
                    auto vectX = ChParserOpenSim::strToSTLVector<double>(
                        transforms->first_node("function")->first_node()->first_node("x")->value());
                    auto vectY = ChParserOpenSim::strToSTLVector<double>(
                        transforms->first_node("function")->first_node()->first_node("y")->value());
                    // If not, the following logic will fail
                    assert(vectX.size() == vectY.size());

                    // Make a Natural Cubic Spline
                    ChCubicSpline transformFunc(vectX, vectY);
                    transformFunc.SetLeftBC(ChCubicSpline::BCType::SECOND_BC, 0);
                    transformFunc.SetRightBC(ChCubicSpline::BCType::SECOND_BC, 0);
                    // These are the first and second derivatives which we do not need
                    double a, b;
                    // Drop spline's value into the transformValue
                    transformFunc.Evaluate(coordVals[coordNames.at(0)], transformValue, a, b);

                } else {
                    if (m_verbose)
                        cout << "Unknown transform type: " << functionType
                             << ", skipping initial position calculations." << endl;
                    continue;
                }

                if (transformIndex < 3) {
                    // Rotation
                    X_F_M = Q_from_AngAxis(transformValue, axis) * X_F_M;
                } else {
                    // Translation
                    X_F_M = (transformValue * axis) * X_F_M;
                }

                // Iterate through linked list
                transforms = transforms->next_sibling();
                transformIndex++;
            }
        }

        // Get joint's position in child frame
        elems = ChParserOpenSim::strToSTLVector<double>(jointNode->first_node("location")->value());
        ChVector<> jointPosInChild = ChVector<>(elems[0], elems[1], elems[2]);

        // Get joint's orientation in child frame
        elems = ChParserOpenSim::strToSTLVector<double>(jointNode->first_node("orientation")->value());
        ChQuaternion<> jointOrientationInChild = Q_from_AngX(elems[0]) * Q_from_AngY(elems[1]) * Q_from_AngZ(elems[2]);

        // Joint in child to child
        ChFrame<> X_B_M(jointPosInChild, jointOrientationInChild);

        // Multiply transforms through
        auto X_G_B = X_G_P * X_P_F * X_F_M * X_B_M.GetInverse();

        // Set body frame, not necessarily centroidal
        newBody->SetFrame_REF_to_abs(X_G_B);

        // Used for following output
        ChFrame<> jointFrame = X_G_P * X_P_F;

        assert(std::abs(newBody->GetRot().Length() - 1) < 1e-10);

        if (m_verbose) {
            cout << "Parent is at global " << parent->GetPos().x() << "," << parent->GetPos().y() << ","
                 << parent->GetPos().z() << "|" << parent->GetRot().e0() << "," << parent->GetRot().e1() << ","
                 << parent->GetRot().e2() << "," << parent->GetRot().e3() << endl;
            cout << "Joint is at parent " << jointPosInParent.x() << "," << jointPosInParent.y() << ","
                 << jointPosInParent.z() << "|" << jointOrientationInParent.e0() << "," << jointOrientationInParent.e1()
                 << "," << jointOrientationInParent.e2() << "," << jointOrientationInParent.e3() << endl;
            cout << "Joint is at global " << jointFrame.GetPos().x() << "," << jointFrame.GetPos().y() << ","
                 << jointFrame.GetPos().z() << "|" << jointFrame.GetRot().e0() << "," << jointFrame.GetRot().e1() << ","
                 << jointFrame.GetRot().e2() << "," << jointFrame.GetRot().e3() << endl;
            cout << "Joint is at child " << jointPosInChild.x() << "," << jointPosInChild.y() << ","
                 << jointPosInChild.z() << "|" << jointOrientationInChild.e0() << "," << jointOrientationInChild.e1()
                 << "," << jointOrientationInChild.e2() << "," << jointOrientationInChild.e3() << endl;
            cout << "Putting body " << newBody->GetName() << " at " << newBody->GetFrame_REF_to_abs().GetPos().x()
                 << "," << newBody->GetFrame_REF_to_abs().GetPos().y() << ","
                 << newBody->GetFrame_REF_to_abs().GetPos().z() << endl;
            cout << "Orientation is " << newBody->GetFrame_REF_to_abs().GetRot().e0() << ","
                 << newBody->GetFrame_REF_to_abs().GetRot().e1() << "," << newBody->GetFrame_REF_to_abs().GetRot().e2()
                 << "," << newBody->GetFrame_REF_to_abs().GetRot().e3() << endl;
        }

        // Due to the weird inheritance of the ChLink family, this has a lot of code duplication

        // Make a joint, depending on what it actually is
        if (std::string(jointNode->name()) == std::string("PinJoint")) {
            auto joint = std::make_shared<ChLinkLockRevolute>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            system->AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("WeldJoint"))) {
            auto joint = std::make_shared<ChLinkLockLock>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            system->AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("UniversalJoint"))) {
            // Do some universal magic here
            auto joint = std::make_shared<ChLinkUniversal>();
            joint->Initialize(parent, newBody, jointFrame);
            joint->SetNameString(jointNode->first_attribute("name")->value());
            system->AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("BallJoint"))) {
            // Add a spherical joint
            auto joint = std::make_shared<ChLinkLockSpherical>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            system->AddLink(joint);
            m_jointList.push_back(joint);
        } else {
            // Unknown joint type.  Replace with a spherical
            cout << "Unknown Joint type " << jointNode->name() << " between " << parent->GetName() << " and "
                 << newBody->GetName() << " -- making spherical standin." << endl;
            auto joint = std::make_shared<ChLinkLockSpherical>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(std::string(jointNode->first_attribute("name")->value()) + "_standin");
            system->AddLink(joint);
            m_jointList.push_back(joint);
        }

    };

    function_table["VisibleObject"] = [this](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // Add visualization assets to body, they must be in data/opensim
        if (m_visType == VisType::MESH) {
            auto geometry = fieldNode->first_node("GeometrySet")->first_node("objects")->first_node();
            while (geometry != nullptr) {
                std::string meshFilename(geometry->first_node("geometry_file")->value());
                auto bodyMesh = std::make_shared<ChObjShapeFile>();
                bodyMesh->SetFilename(GetChronoDataFile("opensim/" + meshFilename));
                newBody->AddAsset(bodyMesh);
                geometry = geometry->next_sibling();
            }
        }
    };

    function_table["WrapObjectSet"] = [](xml_node<>* fieldNode, std::shared_ptr<ChBodyAuxRef> newBody) {
        // I don't think we need this
    };
}

// -----------------------------------------------------------------------------
// Initialize collision shapes and attach visualization assets.
// -----------------------------------------------------------------------------

// Holds info for a single collision cylinder
struct collision_cylinder_specs {
    double rad;
    double hlen;
    ChVector<> pos;
    ChQuaternion<> rot;
};

// Holds all cylinder info and family info for a single body
struct body_collision_struct {
    ChBodyAuxRef* body;
    std::vector<collision_cylinder_specs> cylinders;
    int family;
    int family_mask_nocollide;
    int level;  // Depth in tree
};

// Used for vis shapes
static std::map<std::string, body_collision_struct> body_collision_info;

void ChParserOpenSim::initShapes(rapidxml::xml_node<>* node, ChSystem& system) {
    if (m_verbose)
        cout << "Creating collision and visualization shapes " << endl;

    // Keep the maximum depth so that we can color appropriately
    int max_depth_level = 0;

    // Go through the motions of constructing collision models, but don't do it until we have all of the necessary info
    for (auto link : m_jointList) {
        auto linkCoords = link->GetLinkAbsoluteCoords();
        // These need to be ChBodyAuxRef, but hopefully that's all that we'll have
        auto parent = dynamic_cast<ChBodyAuxRef*>(link->GetBody1());
        auto child = dynamic_cast<ChBodyAuxRef*>(link->GetBody2());

        // Add pointers to struct if they don't exist, struct entry should be default-constructed
        if (body_collision_info.find(parent->GetName()) == body_collision_info.end()) {
            body_collision_info[parent->GetName()].body = parent;
        }
        if (body_collision_info.find(child->GetName()) == body_collision_info.end()) {
            body_collision_info[child->GetName()].body = child;
        }
        body_collision_struct& parent_col_info = body_collision_info[parent->GetName()];
        body_collision_struct& child_col_info = body_collision_info[child->GetName()];

        // If a body is fixed, treat it as level 0
        if (parent->GetBodyFixed()) {
            parent_col_info.level = 0;
        }
        // Child is one more level than parent
        int child_level = parent_col_info.level + 1;
        // Keep a max depth so we know how to color it
        if (child_level > max_depth_level) {
            max_depth_level = child_level;
        }
        child_col_info.level = child_level;

        ChVector<> p1, p2;
        // Make cylinder from parent to outbound joint
        // First end is at parent COM
        p1 = parent->GetFrame_COG_to_REF().GetPos();
        // Second end is at joint location
        p2 = parent->GetFrame_REF_to_abs().TransformPointParentToLocal(linkCoords.pos);

        // Don't make a connection between overlapping bodies
        if ((p2 - p1).Length() > 1e-5) {
            ChMatrix33<> rot;
            rot.Set_A_Xdir(p1 - p2);
            // Center of cylinder is halfway between points
            collision_cylinder_specs new_cyl;
            new_cyl.rad = .075;
            new_cyl.hlen = (p1 - p2).Length() / 2;
            new_cyl.pos = (p1 - p2) / 2;
            new_cyl.rot = rot.Get_A_quaternion() * Q_from_AngZ(-CH_C_PI / 2);
            body_collision_info[parent->GetName()].cylinders.push_back(new_cyl);
        }

        // First end is at joint location
        p1 = child->GetFrame_REF_to_abs().TransformPointParentToLocal(linkCoords.pos);
        // Second end is at child COM
        p2 = child->GetFrame_COG_to_REF().GetPos();

        // Don't make a connection between overlapping bodies
        if ((p2 - p1).Length() > 1e-5) {
            ChMatrix33<> rot;
            rot.Set_A_Xdir(p2 - p1);
            // Center of cylinder is halfway between points
            collision_cylinder_specs new_cyl;
            new_cyl.rad = .075;
            new_cyl.hlen = (p2 - p1).Length() / 2;
            new_cyl.pos = (p1 - p2) / 2;
            new_cyl.rot = rot.Get_A_quaternion() * Q_from_AngZ(-CH_C_PI / 2);
            body_collision_info[child->GetName()].cylinders.push_back(new_cyl);
        }

        // Make cylinder from child to inbound joint
        if (m_verbose)
            cout << parent->GetName() << " family is " << body_collision_info[parent->GetName()].family << endl;

        if ((parent->GetCollide() == false) || (body_collision_info[parent->GetName()].family == m_family_2)) {
            if (m_verbose)
                cout << "Setting " << child->GetName() << " to family " << m_family_1 << endl;
            body_collision_info[child->GetName()].family = m_family_1;
            body_collision_info[child->GetName()].family_mask_nocollide = m_family_2;
        } else {
            if (m_verbose)
                cout << "Setting " << child->GetName() << " to family " << m_family_2 << endl;
            body_collision_info[child->GetName()].family = m_family_2;
            body_collision_info[child->GetName()].family_mask_nocollide = m_family_1;
        }
    }

    // Loop through the list of bodies in the model and create visualization and collision shapes
    for (auto body_info_pair : body_collision_info) {
        auto body_info = body_info_pair.second;

        // Create primitive visualization assets
        if (m_visType == VisType::PRIMITIVES) {
            // Assign a color based on the body's level in the tree hierarchy
            float colorVal = (1.0f * body_info.level) / max_depth_level;
            body_info.body->AddAsset(std::make_shared<ChColorAsset>(colorVal, 1.0f - colorVal, 0.0f));

            // Create a sphere at the body COM
            auto sphere = std::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = 0.1;
            sphere->Pos = body_info.body->GetFrame_COG_to_REF().GetPos();
            body_info.body->AddAsset(sphere);

            // Create visualization cylinders
            for (auto cyl_info : body_info.cylinders) {
                auto cylinder = std::make_shared<ChCylinderShape>();
                cylinder->GetCylinderGeometry().rad = cyl_info.rad;
                cylinder->GetCylinderGeometry().p1 = ChVector<>(0, cyl_info.hlen, 0);
                cylinder->GetCylinderGeometry().p2 = ChVector<>(0, -cyl_info.hlen, 0);
                cylinder->Pos = cyl_info.pos;
                cylinder->Rot = cyl_info.rot;
                body_info.body->AddAsset(cylinder);
            }
        }

        // Set collision shapes
        if (body_info.body->GetCollide()) {
            body_info.body->GetCollisionModel()->ClearModel();

            for (auto cyl_info : body_info.cylinders) {
                utils::AddCylinderGeometry(body_info.body, cyl_info.rad, cyl_info.hlen, cyl_info.pos, cyl_info.rot,
                                           false);
            }

            body_info.body->GetCollisionModel()->SetFamily(body_info.family);
            body_info.body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(body_info.family_mask_nocollide);

            body_info.body->GetCollisionModel()->BuildModel();
        }
    }
}

}  // end namespace utils
}  // end namespace chrono
