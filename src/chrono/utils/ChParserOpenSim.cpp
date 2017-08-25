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

// Initializes lambda table
ChParserOpenSim::ChParserOpenSim() {
    initFunctionTable();
}
// Used for coloring bodies
static int body_idx = 0;

// Holds info for a single collision cylinder
struct collision_cylinder_specs {
    double rad;
    double hlen;
    ChVector<> pos;
    ChQuaternion<> rot;
};

// Holds all cylinder info and family info for a single body
struct body_collision_struct {
    ChBody* body;
    std::vector<collision_cylinder_specs> cylinders;
    int family;
    int family_mask_nocollide;
};

// So we can keep adding to the same body
static std::map<std::string, body_collision_struct> body_collision_info;

// Parses an opensim file into an existing system
void ChParserOpenSim::parse(ChSystem& p_system,
                            const std::string& filename,
                            ChParserOpenSim::VisType vis,
                            bool collision) {
    bodies_collide = collision;
    m_visType = vis;
    rapidxml::file<char> file(filename.c_str());

    xml_document<> doc;
    doc.parse<0>(file.data());

    // Hold list of bodies
    xml_node<>* bodySet = doc.first_node()->first_node("Model")->first_node("BodySet")->first_node("objects");

    // Get gravity from model and set it in system
    auto elems = strToSTLVector<double>(doc.first_node()->first_node("Model")->first_node("gravity")->value());
    p_system.Set_G_acc(ChVector<>(elems[0], elems[1], elems[2]));

    // Holds list of fields for body
    xml_node<>* bodyNode = bodySet->first_node();

    // Loop through each body
    while (bodyNode != NULL) {
        parseBody(bodyNode, p_system);
        bodyNode = bodyNode->next_sibling();
    }
    // initVisualizations(bodyNode, p_system);
    initShapes(bodyNode, p_system);
}

// Makes a new system and then parses into it
ChSystem* ChParserOpenSim::parse(const std::string& filename,
                                 ChMaterialSurface::ContactMethod contact_method,
                                 ChParserOpenSim::VisType vis,
                                 bool collision) {
    ChSystem* sys = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                               : static_cast<ChSystem*>(new ChSystemSMC);

    parse(*sys, filename, vis, collision);

    return sys;
}

// -------------------------------------------------------------------------------------------

// Creates ChBody and parses its various properties from its XML child nodes
bool ChParserOpenSim::parseBody(xml_node<>* bodyNode, ChSystem& my_system) {
    // Make a new body and name it for later
    std::cout << "New body " << bodyNode->first_attribute("name")->value() << std::endl;
    auto newBody = std::make_shared<ChBodyAuxRef>();

    my_system.AddBody(newBody);
    if (bodies_collide) {
        newBody->SetCollide(true);
        // Material properties
        // float Y = 2e6f;
        // float mu = 0.4f;
        // float cr = 0.4f;

        auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
        // ballMat->SetYoungModulus(Y);
        // ballMat->SetFriction(mu);
        // ballMat->SetRestitution(cr);
        // ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

        newBody->SetMaterialSurface(ballMat);
    } else {
        newBody->SetCollide(false);
    }

    // Used for lookup in the joint creation process
    newBody->SetName(bodyNode->first_attribute("name")->value());
    // Used for coloring
    newBody->SetIdentifier(body_idx++);

    // First node in linked list of fields
    xml_node<>* fieldNode = bodyNode->first_node();

    // Parse the body, field-by-field
    while (fieldNode != NULL) {
        // Parse in body information
        function_table[fieldNode->name()](fieldNode, my_system, newBody);

        fieldNode = fieldNode->next_sibling();
    }
    return true;
}

// This is where the magic happens -- it sets up a map of strings to lambdas
// that do the heavy lifting of osim -> chrono parsing
// Each entry in the function_table is a child of the "Body" XML node
void ChParserOpenSim::initFunctionTable() {
    function_table["mass"] = [](xml_node<>* fieldNode, ChSystem& my_system, std::shared_ptr<ChBodyAuxRef> newBody) {
        if (std::stod(fieldNode->value()) == 0) {
            // Ground-like body, massless => fixed
            newBody->SetBodyFixed(true);
            newBody->SetCollide(false);
            newBody->SetPos(ChVector<>(0, 0, 0));
            // Ground has special color to identify it
            newBody->AddAsset(std::make_shared<ChColorAsset>(0, 0, 0));
        } else {
            // Give new body mass
            newBody->SetMass(std::stod(fieldNode->value()));
        }
    };
    function_table["mass_center"] = [this](xml_node<>* fieldNode, const ChSystem& my_system,
                                           std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set COM in reference frame
        auto elems = strToSTLVector<double>(fieldNode->value());
        // Opensim doesn't really use a rotated COM to REF frame, so unit quaternion
        newBody->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(elems[0], elems[1], elems[2]), ChQuaternion<>(1, 0, 0, 0)));
        // Only add vis balls if we're using primitives
        if ((m_visType == PRIMITIVES)) {
            auto sphere = std::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = 0.1;
            // Put vis asset at COM
            sphere->Pos = ChVector<>(elems[0], elems[1], elems[2]);
            newBody->AddAsset(sphere);
        }
    };
    function_table["inertia_xx"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xx inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_yy"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set yy inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_zz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set zz inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_xy"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xy inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["inertia_xz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["inertia_yz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set yz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["Joint"] = [this](xml_node<>* fieldNode, ChSystem& my_system,
                                     std::shared_ptr<ChBodyAuxRef> newBody) {
        // If there are no joints, this is hopefully the ground (or another global parent??)
        if (fieldNode->first_node() == NULL) {
            std::cout << "No joints for this body " << std::endl;
            return;
        }

        // Deduce child body from joint orientation
        xml_node<>* jointNode = fieldNode->first_node();
        // Make a joint here
        std::cout << "Making a " << jointNode->name() << " with " << jointNode->first_node("parent_body")->value()
                  << std::endl;

        // Get other body for joint
        auto parent = my_system.SearchBody(jointNode->first_node("parent_body")->value());

        if (parent != nullptr) {
            std::cout << "other body found!" << std::endl;
        } else {
            std::cout << "Parent not found!!!! Exiting." << std::endl;
            // really bad, either file is invalid or we messed up
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
                    std::cout << "Unknown transform type: " << functionType
                              << ", skipping initial position calculations." << std::endl;
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

        std::cout << "Parent is at global " << parent->GetPos().x() << "," << parent->GetPos().y() << ","
                  << parent->GetPos().z() << "|" << parent->GetRot().e0() << "," << parent->GetRot().e1() << ","
                  << parent->GetRot().e2() << "," << parent->GetRot().e3() << std::endl;
        std::cout << "Joint is at parent " << jointPosInParent.x() << "," << jointPosInParent.y() << ","
                  << jointPosInParent.z() << "|" << jointOrientationInParent.e0() << ","
                  << jointOrientationInParent.e1() << "," << jointOrientationInParent.e2() << ","
                  << jointOrientationInParent.e3() << std::endl;
        std::cout << "Joint is at global " << jointFrame.GetPos().x() << "," << jointFrame.GetPos().y() << ","
                  << jointFrame.GetPos().z() << "|" << jointFrame.GetRot().e0() << "," << jointFrame.GetRot().e1()
                  << "," << jointFrame.GetRot().e2() << "," << jointFrame.GetRot().e3() << std::endl;
        std::cout << "Joint is at child " << jointPosInChild.x() << "," << jointPosInChild.y() << ","
                  << jointPosInChild.z() << "|" << jointOrientationInChild.e0() << "," << jointOrientationInChild.e1()
                  << "," << jointOrientationInChild.e2() << "," << jointOrientationInChild.e3() << std::endl;
        std::cout << "Putting body " << newBody->GetName() << " at " << newBody->GetFrame_REF_to_abs().GetPos().x()
                  << "," << newBody->GetFrame_REF_to_abs().GetPos().y() << ","
                  << newBody->GetFrame_REF_to_abs().GetPos().z() << std::endl;
        std::cout << "Orientation is " << newBody->GetFrame_REF_to_abs().GetRot().e0() << ","
                  << newBody->GetFrame_REF_to_abs().GetRot().e1() << "," << newBody->GetFrame_REF_to_abs().GetRot().e2()
                  << "," << newBody->GetFrame_REF_to_abs().GetRot().e3() << std::endl;

        // Due to the weird inheritance of the ChLink family, this has a lot of code duplication

        // Make a joint, depending on what it actually is
        if (std::string(jointNode->name()) == std::string("PinJoint")) {
            auto joint = std::make_shared<ChLinkLockRevolute>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("WeldJoint"))) {
            auto joint = std::make_shared<ChLinkLockLock>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("UniversalJoint"))) {
            // Do some universal magic here
            auto joint = std::make_shared<ChLinkUniversal>();
            joint->Initialize(parent, newBody, jointFrame);
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("BallJoint"))) {
            // Add a spherical joint
            auto joint = std::make_shared<ChLinkLockSpherical>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);
        } else {
            // Cry
            std::cout << "Unknown Joint type " << jointNode->name() << " between " << parent->GetName() << " and "
                      << newBody->GetName() << " -- making spherical standin." << std::endl;
            auto joint = std::make_shared<ChLinkLockSpherical>();
            joint->Initialize(parent, newBody, jointFrame.GetCoord());
            joint->SetNameString(std::string(jointNode->first_attribute("name")->value()) + "_standin");
            my_system.AddLink(joint);
            m_jointList.push_back(joint);
        }

    };

    function_table["VisibleObject"] = [this](xml_node<>* fieldNode, const ChSystem& my_system,
                                             std::shared_ptr<ChBodyAuxRef> newBody) {
        // Add visualization assets to body, they must be in data/opensim
        if (m_visType == VisType::MESH) {
            auto geometry = fieldNode->first_node("GeometrySet")->first_node("objects")->first_node();
            while (geometry != nullptr) {
                std::string meshFilename(geometry->first_node("geometry_file")->value());
                auto bodyMesh = std::make_shared<ChObjShapeFile>();
                bodyMesh->SetFilename(GetChronoDataFile("../../data/opensim/" + meshFilename));
                newBody->AddAsset(bodyMesh);
                geometry = geometry->next_sibling();
            }
        }
    };
    function_table["WrapObjectSet"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                         std::shared_ptr<ChBodyAuxRef> newBody) {
        // I don't think we need this
    };
}

// Initialize collision shapes and attaches vis objects if m_visType is PRIMITIVES
void ChParserOpenSim::initShapes(rapidxml::xml_node<>* node, ChSystem& p_system) {
    std::cout << "Assembling Collision Shapes " << std::endl;
    bool vis_primitives = (m_visType == VisType::PRIMITIVES);
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

        // Each child body gets a unique color
        if (m_visType == VisType::PRIMITIVES) {
            double colorVal = (1.0 * child->GetIdentifier()) / p_system.Get_bodylist()->size();
            child->AddAsset(std::make_shared<ChColorAsset>(colorVal, 1 - colorVal, 0));
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
        std::cout << parent->GetName() << " family is " << body_collision_info[parent->GetName()].family << std::endl;
        if ((parent->GetCollide() == false) || (body_collision_info[parent->GetName()].family == 2)) {
            std::cout << "Setting "<< child->GetName()<<" to 1" << std::endl;
            body_collision_info[child->GetName()].family = 1;
            body_collision_info[child->GetName()].family_mask_nocollide = 2;
        } else {
            std::cout << "Setting "<< child->GetName()<<" to 2" << std::endl;
            body_collision_info[child->GetName()].family = 2;
            body_collision_info[child->GetName()].family_mask_nocollide = 1;
        }
    }
    // Loops through created set of bodies and actually set up their models
    for (auto body_info_pair : body_collision_info) {
        auto body_info = body_info_pair.second;
        body_info.body->GetCollisionModel()->ClearModel();

        for (auto cyl_info : body_info.cylinders) {
            utils::AddCylinderGeometry(body_info.body, cyl_info.rad, cyl_info.hlen, cyl_info.pos, cyl_info.rot,
                                       vis_primitives);
        }
        // Set up the model
        body_info.body->GetCollisionModel()->SetFamily(body_info.family);
        body_info.body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(body_info.family_mask_nocollide);
        body_info.body->GetCollisionModel()->BuildModel();
    }
}

}  // end namespace utils
}  // end namespace chrono
