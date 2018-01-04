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

#include "chrono/utils/ChParserAdams.h"

#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/physics/ChLinkRackpinion.h"
#include "chrono/utils/ChUtilsCreators.h"

#include <fstream>

#include <utility>

namespace chrono {
namespace utils {

using std::cout;
using std::endl;

struct adams_part {
    bool fixed;
    double mass;               // if -1; it is fixed to ground
    std::string cm_marker_id;  // In case none exists
    double loc[3];
    double rot[3];
    double inertia[6];
};
struct adams_joint {
    std::string type;
    std::string marker_I;
    std::string marker_J;
};
struct adams_marker {
    std::string part_id;
    double loc[3];
    double rot[3];
};

// Maps the part id to an adams part
std::map<std::string, adams_part> parts_map;
// Maps the joint id to an adams joint
std::map<std::string, adams_joint> joints_map;
// Maps the marker id to an adams marker
std::map<std::string, adams_marker> markers_map;
// List of adams graphics and their corresponding reference markers
std::vector<std::vector<std::pair<int, std::string>>> graphics_token_list;

// Easy check for file syntax errors
void tokenParseError(int expected, std::pair<int, std::string>& got) {
    cout << "Unexpected token occured, token should have been: " << expected << ", got " << got.first << ","
         << got.second << endl;
}

// Read a set of tokens into a struct representing a part in adams
// These will later be read into ChBodyAuxRefs
void parseADMPart(std::string ID, std::vector<std::pair<int, std::string>>& tokens, ChSystem& sys) {
    adams_part& part = parts_map[ID];  // uses default contructor
    auto iter = tokens.begin();
    while (iter != tokens.end()) {
        // cout << "token is " << iter->first << ", " << iter->second << endl;
        if (iter->first == LABEL) {
            if (iter->second == std::string("GROUND")) {
                part.fixed = true;
            } else if (iter->second == std::string("CM")) {
                iter++;  // skip that equals sign
                part.fixed = false;
                part.cm_marker_id = iter->second;
            } else if (iter->second == std::string("MASS")) {
                iter++;  // Skip over the equals sign
                part.mass = std::stod(iter->second);
            } else if (iter->second == std::string("IP")) {
                iter++;  // Skip over the equals sign
                for (int i = 0; i < 6 && (iter != tokens.end()); i++) {
                    if (i < 3 || (iter->first == VALUE)) {
                        part.inertia[i] = std::stod(iter->second);
                        iter++;
                    } else if (i < 3) {
                        // Something went wrong in the diagonal inertias
                        tokenParseError(VALUE, *iter);
                        break;
                    } else {
                        // diagonal components don't exist
                        break;
                    }
                }
                // No more inertias to read, but we just read too far
                iter--;
            }
        } else {
            tokenParseError(LABEL, *iter);
        }
        iter++;
    }
}
void parseADMJoint(std::string ID, std::vector<std::pair<int, std::string>>& tokens, ChSystem& sys) {
    adams_joint& joint = joints_map[ID];  // uses default contructor
    auto iter = tokens.begin();
    while (iter != tokens.end()) {
        // cout << "token is " << iter->first << ", " << iter->second << endl;

        if (iter->first == LABEL) {
            if (iter->second == std::string("I")) {
                iter++;  // Get digit
                joint.marker_I = iter->second;
            } else if (iter->second == std::string("J")) {
                iter++;  // Get digit
                joint.marker_J = iter->second;
            } else {
                joint.type = iter->second;
                // cout << "type is " << joint.type << endl;
            }
        }

        iter++;
    }
}

void parseADMMarker(std::string ID, std::vector<std::pair<int, std::string>>& tokens, ChSystem& sys) {
    adams_marker& marker = markers_map[ID];  // uses default contructor
    auto iter = tokens.begin();
    while (iter != tokens.end()) {
        // cout << "token is " << iter->first << ", " << iter->second << endl;
        if (iter->first == LABEL) {
            if (iter->second == std::string("PART")) {
                iter++;
                marker.part_id = iter->second;
            }
            if (iter->second == std::string("QP")) {
                // cout << "reading loc " << endl;

                for (int i = 0; i < 3; i++) {
                    iter++;
                    // cout << "token is " << iter->first << ", " << iter->second << endl;

                    marker.loc[i] = std::stod(iter->second);
                }
            }
            if (iter->second == std::string("REULER")) {
                // cout << "reading rot " << endl;
                for (int i = 0; i < 3; i++) {
                    iter++;
                    // cout << "token is " << iter->first << ", " << iter->second << endl;
                    double val = std::stod(iter->second);

                    if (iter->second.back() == 'D') {
                        // This is a decimal and we need to convert to radians
                        val *= CH_C_DEG_TO_RAD;
                    }
                    // cout << "val is " << val << endl;
                    marker.rot[i] = val;
                }
            }
        }

        iter++;
    }
}

ChQuaternion<> Q_from_313_angles(double q1, double q2, double q3) {
    return ChQuaternion<>(Q_from_AngZ(q1) * Q_from_AngX(q2) * Q_from_AngZ(q3));
}

// -----------------------------------------------------------------------------
// Parse an OpenSim file into an existing system.
// -----------------------------------------------------------------------------

void ChParserAdams::Parse(ChSystem& sys, const std::string& filename) {
    // Has to be in lex file
    tokenize(filename);

    auto iter = m_tokens.begin();
    while (iter != m_tokens.end()) {
        auto token = iter->second;
        switch (iter->first) {
            case DELIMITER: {
                // std::cout << "DELIMITER(" << token << ") ";
                iter++;  // Get next field, should be a type
                int type = iter->first;
                if (type == END) {
                    // cout << "END" << endl;
                    break;  // We hit the end, go no further
                }
                // cout << "Type is :" << iter->first << ", value is :" << iter->second << endl;
                iter++;  // Get next field, should be ID (except for ACCGRAV)

                std::string ID;
                // ACCGRAV has no ID
                if (type != ACCGRAV) {
                    if (iter->first != VALUE) {
                        tokenParseError(VALUE, *iter);
                        return;
                    }
                    // cout << "Getting ID " << endl;
                    ID = iter->second;
                    iter++;
                }
                std::vector<std::pair<int, std::string>> tokens_to_parse;
                // Go until next primary token
                while (iter->first != DELIMITER) {
                    tokens_to_parse.push_back(*iter);
                    iter++;
                }
                iter--;  // We just overshot, gotta go back TODO this is trash

                switch (type) {
                    case PART:
                        // std::cout << "PART(" << ID << ") ";
                        parseADMPart(ID, tokens_to_parse, sys);
                        // cout << endl;
                        break;
                    case JOINT:
                        // std::cout << "JOINT(" << ID << ") ";
                        parseADMJoint(ID, tokens_to_parse, sys);
                        // cout << endl;
                        break;
                    case MARKER:
                        // std::cout << "MARKER(" << ID << ") ";
                        parseADMMarker(ID, tokens_to_parse, sys);
                        // cout << endl;
                        break;
                    case ACCGRAV: {
                        auto grav_it = tokens_to_parse.begin();
                        // std::cout << "ACCGRAV(" << token << ") ";
                        double grav[3];
                        std::string grav_strs[3] = {"IGRAV", "JGRAV", "KGRAV"};
                        for (int i = 0; i < 3; i++) {
                            // Check to see if it matches gravity
                            if (grav_it->second == grav_strs[i]) {
                                grav_it++;  // Get number
                                grav[i] = std::stod(grav_it->second);
                                grav_it++;  // Get next grav
                                // cout << "setting " << grav_strs[i] << " to:" << grav[i] << endl;
                            }
                        }
                        sys.Set_G_acc(ChVector<>(grav[0], grav[1], grav[2]));
                        // cout << endl;
                        break;
                    }
                    case GRAPHICS:
                        // Fall-through for now
                        // std::cout << "GRAPHICS(" << ID << ") ";
                        graphics_token_list.push_back(std::vector<std::pair<int, std::string>>(tokens_to_parse));
                        // cout << endl;
                        break;

                    case REQUEST:
                        // We need to catch these but we don't do anything with them
                        break;
                }

                // cout << endl;
                break;
            }
                // case ADAMS:
                //     std::cout << "ADAMS(" << token << ") ";
                //     cout << endl;
                //     break;
                // case UNITS:
                //     std::cout << "UNITS(" << token << ") ";
                //     cout << endl;
                //     break;
                // case OUTPUT:
                //     std::cout << "OUTPUT(" << token << ") ";
                //     cout << endl;
                //     break;
                // case LABEL:
                //     std::cout << "LABEL(" << token << ") ";
                //     cout << endl;
                //     break;
                // case VALUE:
                //     std::cout << "VALUE(" << token << ") ";
                //     cout << endl;
                //     break;
                // case EQUALS:
                //     std::cout << "EQUALS ";
                //     break;
        }
        iter++;
    }
    // Make the bodies from the cached info
    for (auto part_pair : parts_map) {
        adams_part part = part_pair.second;
        auto newBody = std::make_shared<ChBodyAuxRef>();
        // cout << "fixed is " << part.fixed << endl;
        newBody->SetBodyFixed(part.fixed);
        newBody->SetMass(part.mass);
        if (part.cm_marker_id.length() != 0) {
            // CM marker exists
            adams_marker marker_struct = markers_map[part.cm_marker_id];
            // Multiply through Euler angles
            ChQuaternion<> CM_rot = Q_from_313_angles(marker_struct.rot[0], marker_struct.rot[1], marker_struct.rot[2]);
            ChVector<> CM_loc(marker_struct.loc[0], marker_struct.loc[1], marker_struct.loc[2]);
            // Make new marker attached to body, without any initial motion
            ChFrame<> CM_frame(CM_loc, CM_rot);
            newBody->SetFrame_COG_to_REF(CM_frame);
            // cout << "setting COM at " << CM_loc.x() << "," << CM_loc.y() << "," << CM_loc.z() << "|" << CM_rot.e0()
            //      << "," << CM_rot.e1() << "," << CM_rot.e2() << "," << CM_rot.e3() << endl;
        }
        // set up information cached from adams
        newBody->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(part.loc[0], part.loc[1], part.loc[2]),
                                               Q_from_313_angles(part.rot[0], part.rot[1], part.rot[2])));
        // newBody->SetRot(Q_from_313_angles(part.rot[0], part.rot[1], part.rot[2]));
        // cout << "Inertia is " << part.inertia[0] << "," << part.inertia[1] << "," << part.inertia[2] << ","
        // << part.inertia[3] << "," << part.inertia[4] << "," << part.inertia[5] << endl;
        newBody->SetInertiaXX(ChVector<>(part.inertia[0], part.inertia[1], part.inertia[2]));
        newBody->SetInertiaXY(ChVector<>(part.inertia[3], part.inertia[4], part.inertia[5]));
        newBody->SetCollide(false);
        // Hacky way to allow lookups for markers later
        newBody->SetNameString(part_pair.first);
        // cout << "body " << part_pair.first << " is at " << part.loc[0] << "," << part.loc[1] << "," << part.loc[2]
        //      << "|" << newBody->GetRot().e0() << "," << newBody->GetRot().e1() << "," << newBody->GetRot().e2() <<
        //      ","
        //      << newBody->GetRot().e3() << endl;
        // cout << "COM is at " << newBody->GetFrame_COG_to_REF().GetPos().x() << ","
        //      << newBody->GetFrame_COG_to_REF().GetPos().y() << "," << newBody->GetFrame_COG_to_REF().GetPos().z()
        //      << endl;
        sys.AddBody(newBody);

        // auto sphere = std::make_shared<ChSphereShape>();
        // sphere->GetSphereGeometry().rad = 0.05;
        // sphere->Pos = newBody->GetFrame_REF_to_abs().GetPos();
        // newBody->AddAsset(sphere);
    }
    // Make any markers that don't exist yet
    for (auto marker_pair : markers_map) {
        adams_marker marker = marker_pair.second;
        // find the parent body in the system
        auto parentBody = std::dynamic_pointer_cast<ChBodyAuxRef>(sys.SearchBody(marker.part_id.c_str()));
        assert(parentBody);
        // cout << "body is " << marker.part_id << endl;
        ChVector<> loc(marker.loc[0], marker.loc[1], marker.loc[2]);
        auto parentFrame = parentBody->GetFrame_REF_to_COG();
        ChQuaternion<> rot = Q_from_313_angles(marker.rot[0], marker.rot[1], marker.rot[2]);
        // Convert to aux frame instead of COG frame
        ChCoordsys<> markerCoord = (parentFrame * ChFrame<>(loc, rot)).GetCoord();
        // Make new marker attached to body, without any initial motion
        auto ch_marker = std::make_shared<ChMarker>(marker_pair.first, parentBody.get(), markerCoord, ChCoordsys<>(),
                                                    ChCoordsys<>());
        parentBody->AddMarker(ch_marker);
        // cout << "marker " << marker_pair.first << " is at " << loc.x() << "," << loc.y() << "," << loc.z() << endl;
        auto absloc = ch_marker->GetAbsCoord().pos;
        // cout << "marker " << marker_pair.first << " is at abs " << absloc.x() << "," << absloc.y() << "," <<
        // absloc.z()
        // << endl;
    }
    for (auto joint_pair : joints_map) {
        adams_joint joint = joint_pair.second;
        // Search by c-string representations of marker IDs
        // cout << "I is " << joint.marker_I << endl;
        // cout << "J is " << joint.marker_J << endl;
        auto marker_I = sys.SearchMarker(joint.marker_I.c_str());
        auto marker_J = sys.SearchMarker(joint.marker_J.c_str());
        assert(marker_I);
        assert(marker_J);
        /*
            Cylindrical
            Fixed
            Hooke
            Planar
            Rack-and-pinion
            Revolute
            Screw
            Spherical
            Translational
            Universal
        */
        if (joint.type == std::string("REVOLUTE")) {
            // Make revolute
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // cout << "adding revolute joint " << joint_pair.first << endl;
            auto ch_joint = std::make_shared<ChLinkLockRevolute>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()),
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord());
            ch_joint->SetNameString(joint_pair.first);
            sys.AddLink(ch_joint);
        } else if (joint.type == std::string("SPHERICAL")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // cout << "adding spherical joint " << joint_pair.first << endl;
            auto ch_joint = std::make_shared<ChLinkLockSpherical>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()),
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord());
            ch_joint->SetNameString(joint_pair.first);
            sys.AddLink(ch_joint);

        } else if (joint.type == std::string("HOOKE")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // cout << "adding revolute joint " << joint_pair.first << endl;
            auto ch_joint = std::make_shared<ChLinkUniversal>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)));
            ch_joint->SetNameString(joint_pair.first);
            sys.AddLink(ch_joint);
        } else if (joint.type == std::string("TRANSLATIONAL")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // cout << "adding prismatic joint " << joint_pair.first << endl;
            auto ch_joint = std::make_shared<ChLinkLockPrismatic>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord(),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)).GetCoord());
            ch_joint->SetNameString(joint_pair.first);
            sys.AddLink(ch_joint);

        } else if (joint.type == std::string("CYLINDRICAL")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // cout << "adding cylindrical joint " << joint_pair.first << endl;
            auto ch_joint = std::make_shared<ChLinkLockCylindrical>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord(),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)).GetCoord());
            ch_joint->SetNameString(joint_pair.first);
            sys.AddLink(ch_joint);
        } else if (joint.type == std::string("RACKPIN")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // cout << "adding rackpin joint " << joint_pair.first << endl;
            auto ch_joint = std::make_shared<ChLinkRackpinion>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)));
            ch_joint->SetNameString(joint_pair.first);
            sys.AddLink(ch_joint);
        } else {
            // cout << "unknown joint type " << joint.type << "!" << endl;
        }
    }
    for (auto tokens : graphics_token_list) {
        auto iter = tokens.begin();
        //
        if (iter->second == std::string("CYLINDER")) {
            auto cylinder = std::make_shared<ChCylinderShape>();
            iter++;  // Get next field
            std::shared_ptr<ChMarker> cm_marker;
            ChBodyAuxRef* parentBody;
            while (iter != tokens.end()) {
                // cout << "token is " << iter->first << ", " << iter->second << endl;
                if (iter->second == std::string("CM")) {
                    iter++;  // Get marker ID
                    cm_marker = sys.SearchMarker(iter->second.c_str());
                    cout << " adding cylinder to marker " << iter->second << endl;
                    assert(cm_marker != nullptr);
                    parentBody = dynamic_cast<ChBodyAuxRef*>(cm_marker->GetBody());
                    assert(parentBody != nullptr);
                    cylinder->GetCylinderGeometry().p1 = (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                                             .TransformPointLocalToParent(ChVector<>(0, 0, 0));
                    // cout << "setting p1 to " << cylinder->GetCylinderGeometry().p1.x() << ", "
                    //      << cylinder->GetCylinderGeometry().p1.y() << ", " << cylinder->GetCylinderGeometry().p1.z()
                    //      << endl;
                    parentBody->AddAsset(cylinder);
                } else if (iter->second == std::string("RADIUS")) {
                    iter++;  // get radius
                    cylinder->GetCylinderGeometry().rad = std::stod(iter->second);
                } else if (iter->second == std::string("LENGTH")) {
                    iter++;  // get length
                    double length = std::stod(iter->second);
                    // cout << "length is " << length << endl;
                    cylinder->GetCylinderGeometry().p2 = (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                                             .TransformPointLocalToParent(ChVector<>(0, 0, length));
                    // cout << "setting p2 to " << cylinder->GetCylinderGeometry().p2.x() << ", "
                    //      << cylinder->GetCylinderGeometry().p2.y() << ", " << cylinder->GetCylinderGeometry().p2.z()
                    //      << endl;

                } else {
                    tokenParseError(LABEL, *iter);
                }
                iter++;  // next loop
            }
        } else if (iter->second == std::string("ELLIPSOID")) {
            auto ellipsoid = std::make_shared<ChEllipsoidShape>();
            iter++;  // Get next field
            std::shared_ptr<ChMarker> cm_marker;
            ChBodyAuxRef* parentBody;
            while (iter != tokens.end()) {
                // cout << "token is " << iter->first << ", " << iter->second << endl;
                if (iter->second == std::string("CM")) {
                    iter++;  // Get marker ID
                    cm_marker = sys.SearchMarker(iter->second.c_str());
                    // cout << " adding ellipsoid to marker " << iter->second << endl;
                    assert(cm_marker != nullptr);
                    parentBody = dynamic_cast<ChBodyAuxRef*>(cm_marker->GetBody());
                    assert(parentBody != nullptr);
                    // Put ellipsoid at marker, orientation might be wrong
                    // adams says the orientation is marker-specific but I'm guessing Chrono says it's global or
                    // body-specific
                    ellipsoid->GetEllipsoidGeometry().center = (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                                                   .TransformPointLocalToParent(ChVector<>(0, 0, 0));
                    parentBody->AddAsset(ellipsoid);
                } else if (iter->second == std::string("XSCALE") || iter->second == std::string("XS")) {
                    iter++;  // get length
                    double scale = std::stod(iter->second);
                    // maybe this is right?
                    ellipsoid->GetEllipsoidGeometry().rad.x() = scale;
                } else if (iter->second == std::string("YSCALE") || iter->second == std::string("YS")) {
                    iter++;  // get length
                    double scale = std::stod(iter->second);
                    // maybe this is right?
                    ellipsoid->GetEllipsoidGeometry().rad.y() = scale;
                } else if (iter->second == std::string("ZSCALE") || iter->second == std::string("ZS")) {
                    iter++;  // get length
                    double scale = std::stod(iter->second);
                    // maybe this is right?
                    ellipsoid->GetEllipsoidGeometry().rad.z() = scale;

                } else {
                    tokenParseError(LABEL, *iter);
                }
                iter++;  // next loop
            }
        } else if (iter->second == std::string("BOX")) {
            auto box = std::make_shared<ChBoxShape>();
            iter++;  // Get next field
            std::shared_ptr<ChMarker> cm_marker;
            ChBodyAuxRef* parentBody;
            while (iter != tokens.end()) {
                // cout << "token is " << iter->first << ", " << iter->second << endl;
                if (iter->second == std::string("CORNER")) {
                    iter++;  // Get marker ID
                    cm_marker = sys.SearchMarker(iter->second.c_str());
                    // cout << " adding box to marker " << iter->second << endl;
                    assert(cm_marker != nullptr);
                    parentBody = dynamic_cast<ChBodyAuxRef*>(cm_marker->GetBody());
                    assert(parentBody != nullptr);
                    // Put box at marker
                    box->GetBoxGeometry().Pos = (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                                    .TransformPointLocalToParent(ChVector<>(0, 0, 0));

                    box->GetBoxGeometry().Rot = (parentBody->GetFrame_COG_to_REF() * (*cm_marker)).Amatrix;
                    parentBody->AddAsset(box);
                } else if (iter->second == std::string("X")) {
                    iter++;  // get length
                    double scale = std::stod(iter->second);
                    // maybe this is right?
                    box->GetBoxGeometry().Size.x() = scale / 2;
                } else if (iter->second == std::string("Y")) {
                    iter++;  // get length
                    double scale = std::stod(iter->second);
                    // maybe this is right?
                    box->GetBoxGeometry().Size.y() = scale / 2;
                } else if (iter->second == std::string("Z")) {
                    iter++;  // get length
                    double scale = std::stod(iter->second);
                    // maybe this is right?
                    box->GetBoxGeometry().Size.z() = scale / 2;

                } else {
                    tokenParseError(LABEL, *iter);
                }
                iter++;  // next loop
            }
            ChVector<> oldPos = box->GetBoxGeometry().Pos;
            // cout << "Old pos is " << oldPos.x() << "," << oldPos.y() << "," << oldPos.z() << endl;
            auto oldRot = box->GetBoxGeometry().Rot.Get_A_quaternion();
            ChVector<> size = box->GetBoxGeometry().Size;
            ChVector<> newPos = oldPos + oldRot.Rotate(ChVector<>(size.x(), size.y(), size.z()));
            // cout << "new pos is " << newPos.x() << "," << newPos.y() << "," << newPos.z() << endl;

            box->GetBoxGeometry().Pos = newPos;
        }
    }
}
// -----------------------------------------------------------------------------
// Makes a new system and then parses into it
// -----------------------------------------------------------------------------

ChSystem* ChParserAdams::Parse(const std::string& filename, ChMaterialSurface::ContactMethod contact_method) {
    ChSystem* sys = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                               : static_cast<ChSystem*>(new ChSystemSMC);

    Parse(*sys, filename);

    return sys;
}

}  // end namespace utils
}  // end namespace chrono
