// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2017 projectchrono.org
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
// Parser utility class for ADAMS input files.
//
// This code does strange things with ChBodyAuxRef and frame transforms. This is
// because the ChMarker code uses a body's COM as its reference frame, ignoring
// any non-centroidal bodies. Since ADAMS uses many non-centroidal bodies,
// ChBodyAuxRef is used. However, there is an incompatibility between ChMarkers
// and ChBodyAuxRefs, forcing this code to make sometimes confusing transforms.
//
// =============================================================================

#include "chrono/utils/ChParserAdams.h"

#include "chrono/core/ChFrame.h"

#include "chrono/physics/ChLinkRackpinion.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChModelFileShape.h"
#include "chrono/assets/ChSphereShape.h"

#include "chrono/utils/ChUtilsCreators.h"

#include <fstream>

#include <utility>

namespace chrono {
namespace utils {

void ChParserAdams::Report::Print() const {
    std::cout << "Parsed " << bodies.size() << " bodies:\n";
    for (auto const& body : bodies) {
        std::cout << "   name: \"" << body.first << "\"" << std::endl;
    }

    std::cout << "Parsed " << joints.size() << " joints:\n";
    for (auto const& joint : joints) {
        std::cout << "   name: \"" << joint.first << "\", type: \"" << joint.second.type << "\"" << std::endl;
    }
}

std::shared_ptr<ChBodyAuxRef> ChParserAdams::Report::GetBody(const std::string& name) const {
    std::shared_ptr<ChBodyAuxRef> body;
    auto body_info = bodies.find(name);
    if (body_info != bodies.end()) {
        body = body_info->second;
    }
    return body;
}

std::shared_ptr<ChLink> ChParserAdams::Report::GetJoint(const std::string& name) const {
    std::shared_ptr<ChLink> joint;
    auto joint_info = joints.find(name);
    if (joint_info != joints.end()) {
        joint = joint_info->second.joint;
    }
    return joint;
}

struct adams_part_struct {
    bool fixed;                // Fixed to ground
    double mass;               // Part mass
    std::string cm_marker_id;  // COM marker
    double loc[3];             // Location of part in global frame
    double rot[3];             // Orientation of part in global frame
    double inertia[6];         // Moments of inertia
};
struct adams_joint_struct {
    std::string type;      // REVOLUTE, TRANSLATIONAL, etc.
    std::string marker_I;  // First constrained marker
    std::string marker_J;  // Second constrained marker
};
struct adams_marker_struct {
    std::string part_id;  // Attached part
    double loc[3];        // Location relative to part
    double rot[3];        // Orientation relative to part
};

// Maps the part id to an adams part
std::map<std::string, adams_part_struct> parts_map;
// Maps the joint id to an adams joint
std::map<std::string, adams_joint_struct> joints_map;
// Maps the marker id to an adams marker
std::map<std::string, adams_marker_struct> markers_map;
// List of adams graphics and their corresponding reference markers
std::vector<std::vector<std::pair<int, std::string>>> graphics_token_list;

// Easy check for file syntax errors
void tokenParseError(int expected, std::pair<int, std::string>& got) {
    std::cout << "Unexpected token occured, token should have been: " << expected << ", got " << got.first << ","
              << got.second << std::endl;
}

// Read a set of tokens into a struct representing a part in adams
// These will later be read into ChBodyAuxRefs
void parseADMPart(std::string ID, std::vector<std::pair<int, std::string>>& tokens, ChSystem& sys) {
    adams_part_struct& part = parts_map[ID];  // uses default contructor
    auto iter = tokens.begin();
    while (iter != tokens.end()) {
        // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;
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
    adams_joint_struct& joint = joints_map[ID];  // uses default contructor
    auto iter = tokens.begin();
    while (iter != tokens.end()) {
        // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;

        if (iter->first == LABEL) {
            if (iter->second == std::string("I")) {
                iter++;  // Get digit
                joint.marker_I = iter->second;
            } else if (iter->second == std::string("J")) {
                iter++;  // Get digit
                joint.marker_J = iter->second;
            } else {
                joint.type = iter->second;
                // std::cout << "type is " << joint.type <<std::endl;
            }
        }

        iter++;
    }
}

void parseADMMarker(std::string ID, std::vector<std::pair<int, std::string>>& tokens, ChSystem& sys) {
    adams_marker_struct& marker = markers_map[ID];  // uses default contructor
    auto iter = tokens.begin();
    while (iter != tokens.end()) {
        // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;
        if (iter->first == LABEL) {
            if (iter->second == std::string("PART")) {
                iter++;
                marker.part_id = iter->second;
            }
            if (iter->second == std::string("QP")) {
                // std::cout << "reading loc " <<std::endl;

                for (int i = 0; i < 3; i++) {
                    iter++;
                    // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;

                    marker.loc[i] = std::stod(iter->second);
                }
            }
            if (iter->second == std::string("REULER")) {
                // std::cout << "reading rot " <<std::endl;
                for (int i = 0; i < 3; i++) {
                    iter++;
                    // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;
                    double val = std::stod(iter->second);

                    if (iter->second.back() == 'D') {
                        // This is a decimal and we need to convert to radians
                        val *= CH_C_DEG_TO_RAD;
                    }
                    // std::cout << "val is " << val <<std::endl;
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
// Parse an ADM file into an existing system.
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
                    // std::cout << "END" <<std::endl;
                    break;  // We hit the end, go no further
                }
                // std::cout << "Type is :" << iter->first << ", value is :" << iter->second <<std::endl;
                iter++;  // Get next field, should be ID (except for ACCGRAV)

                std::string ID;
                // ACCGRAV has no ID
                if (type != ACCGRAV) {
                    if (iter->first != VALUE) {
                        tokenParseError(VALUE, *iter);
                        return;
                    }
                    // std::cout << "Getting ID " <<std::endl;
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
                        // std::cout <<std::endl;
                        break;
                    case JOINT:
                        // std::cout << "JOINT(" << ID << ") ";
                        parseADMJoint(ID, tokens_to_parse, sys);
                        // std::cout <<std::endl;
                        break;
                    case MARKER:
                        // std::cout << "MARKER(" << ID << ") ";
                        parseADMMarker(ID, tokens_to_parse, sys);
                        // std::cout <<std::endl;
                        break;
                    case ACCGRAV: {
                        auto grav_it = tokens_to_parse.begin();
                        // std::cout << "ACCGRAV(" << token << ") ";
                        ChVector<> grav;
                        std::string grav_strs[3] = {"IGRAV", "JGRAV", "KGRAV"};
                        for (int i = 0; i < 3; i++) {
                            // Check to see if it matches gravity
                            if (grav_it->second == grav_strs[i]) {
                                grav_it++;  // Get number
                                grav[i] = std::stod(grav_it->second);
                                grav_it++;  // Get next grav
                                // std::cout << "setting " << grav_strs[i] << " to:" << grav[i] <<std::endl;
                            }
                        }
                        sys.Set_G_acc(grav);
                        // std::cout <<std::endl;
                        break;
                    }
                    case GRAPHICS:
                        // Fall-through for now
                        // std::cout << "GRAPHICS(" << ID << ") ";
                        graphics_token_list.push_back(std::vector<std::pair<int, std::string>>(tokens_to_parse));
                        // std::cout <<std::endl;
                        break;

                    case REQUEST:
                        // We need to catch these but we don't do anything with them
                        break;
                }

                // std::cout <<std::endl;
                break;
            }
                // case ADAMS:
                //     std::cout << "ADAMS(" << token << ") ";
                //    std::cout <<std::endl;
                //     break;
                // case UNITS:
                //     std::cout << "UNITS(" << token << ") ";
                //    std::cout <<std::endl;
                //     break;
                // case OUTPUT:
                //     std::cout << "OUTPUT(" << token << ") ";
                //    std::cout <<std::endl;
                //     break;
                // case LABEL:
                //     std::cout << "LABEL(" << token << ") ";
                //    std::cout <<std::endl;
                //     break;
                // case VALUE:
                //     std::cout << "VALUE(" << token << ") ";
                //    std::cout <<std::endl;
                //     break;
                // case EQUALS:
                //     std::cout << "EQUALS ";
                //     break;
        }
        iter++;
    }
    // Make the bodies from the cached info
    for (auto part_pair : parts_map) {
        adams_part_struct part = part_pair.second;
        auto newBody = chrono_types::make_shared<ChBodyAuxRef>();
        // std::cout << "fixed is " << part.fixed <<std::endl;
        newBody->SetBodyFixed(part.fixed);
        newBody->SetMass(part.mass);
        if (part.cm_marker_id.length() != 0) {
            // CM marker exists
            adams_marker_struct marker_struct = markers_map[part.cm_marker_id];
            // Multiply through Euler angles
            ChQuaternion<> CM_rot = Q_from_313_angles(marker_struct.rot[0], marker_struct.rot[1], marker_struct.rot[2]);
            ChVector<> CM_loc(marker_struct.loc[0], marker_struct.loc[1], marker_struct.loc[2]);
            // Make new marker attached to body, without any initial motion
            ChFrame<> CM_frame(CM_loc, CM_rot);
            newBody->SetFrame_COG_to_REF(CM_frame);
            // std::cout << "setting COM at " << CM_loc.x() << "," << CM_loc.y() << "," << CM_loc.z() << "|" <<
            // CM_rot.e0()
            //      << "," << CM_rot.e1() << "," << CM_rot.e2() << "," << CM_rot.e3() <<std::endl;
        }
        // set up information cached from adams
        newBody->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(part.loc[0], part.loc[1], part.loc[2]),
                                               Q_from_313_angles(part.rot[0], part.rot[1], part.rot[2])));
        // newBody->SetRot(Q_from_313_angles(part.rot[0], part.rot[1], part.rot[2]));
        // std::cout << "Inertia is " << part.inertia[0] << "," << part.inertia[1] << "," << part.inertia[2] << ","
        // << part.inertia[3] << "," << part.inertia[4] << "," << part.inertia[5] <<std::endl;
        newBody->SetInertiaXX(ChVector<>(part.inertia[0], part.inertia[1], part.inertia[2]));
        newBody->SetInertiaXY(ChVector<>(part.inertia[3], part.inertia[4], part.inertia[5]));
        newBody->SetCollide(false);
        // Hacky way to allow lookups for markers later
        newBody->SetNameString(part_pair.first);

        // Add to report
        m_report.bodies.insert(std::make_pair(part_pair.first, newBody));

        // // std::cout << "body " << part_pair.first << " is at " << part.loc[0] << "," << part.loc[1] << "," <<
        //  part.loc[2]
        //       << "|" << newBody->GetRot().e0() << "," << newBody->GetRot().e1() << "," << newBody->GetRot().e2() <<
        //       ","
        //       << newBody->GetRot().e3() <<std::endl;
        //  std::cout << "COM is at " << newBody->GetFrame_COG_to_REF().GetPos().x() << ","
        //       << newBody->GetFrame_COG_to_REF().GetPos().y() << "," << newBody->GetFrame_COG_to_REF().GetPos().z()
        //       <<std::endl;

        sys.AddBody(newBody);

        // auto sphere = chrono_types::make_shared<ChSphereShape>(0.05);
        // newBody->AddVisualShape(sphere, );
    }
    // Make any markers that don't exist yet
    for (auto marker_pair : markers_map) {
        adams_marker_struct marker = marker_pair.second;
        // find the parent body in the system
        auto parentBody = std::dynamic_pointer_cast<ChBodyAuxRef>(sys.SearchBody(marker.part_id.c_str()));
        assert(parentBody);
        // std::cout << "body is " << marker.part_id <<std::endl;
        ChVector<> loc(marker.loc[0], marker.loc[1], marker.loc[2]);
        auto parentFrame = parentBody->GetFrame_REF_to_COG();
        ChQuaternion<> rot = Q_from_313_angles(marker.rot[0], marker.rot[1], marker.rot[2]);
        // Convert to aux frame instead of COG frame
        ChCoordsys<> markerCoord = (parentFrame * ChFrame<>(loc, rot)).GetCoord();
        // Make new marker attached to body, without any initial motion
        auto ch_marker = chrono_types::make_shared<ChMarker>(marker_pair.first, parentBody.get(), markerCoord, ChCoordsys<>(),
                                                    ChCoordsys<>());
        parentBody->AddMarker(ch_marker);
        // std::cout << "marker " << marker_pair.first << " is at " << loc.x() << "," << loc.y() << "," << loc.z()
        // <<std::endl;
        // auto absloc = ch_marker->GetAbsCoord().pos;
        // std::cout << "marker " << marker_pair.first << " is at abs " << absloc.x() << "," << absloc.y() << "," <<
        // absloc.z()
        // <<std::endl;
    }
    for (auto joint_pair : joints_map) {
        std::shared_ptr<ChLink> new_joint;
        // Catch unrecognized joint
        bool joint_not_parsed = false;
        std::string joint_type;
        adams_joint_struct joint = joint_pair.second;
        // Search by c-string representations of marker IDs
        // std::cout << "I is " << joint.marker_I <<std::endl;
        // std::cout << "J is " << joint.marker_J <<std::endl;
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

            // std::cout << "adding revolute joint " << joint_pair.first <<std::endl;
            auto ch_joint = chrono_types::make_shared<ChLinkLockRevolute>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()),
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord());
            new_joint = ch_joint;
        } else if (joint.type == std::string("SPHERICAL")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // std::cout << "adding spherical joint " << joint_pair.first <<std::endl;
            auto ch_joint = chrono_types::make_shared<ChLinkLockSpherical>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()),
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord());
            new_joint = ch_joint;
        } else if (joint.type == std::string("HOOKE")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // std::cout << "adding revolute joint " << joint_pair.first <<std::endl;
            auto ch_joint = chrono_types::make_shared<ChLinkUniversal>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)));
            new_joint = ch_joint;
        } else if (joint.type == std::string("TRANSLATIONAL")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // std::cout << "adding prismatic joint " << joint_pair.first <<std::endl;
            auto ch_joint = chrono_types::make_shared<ChLinkLockPrismatic>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord(),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)).GetCoord());
            new_joint = ch_joint;
        } else if (joint.type == std::string("CYLINDRICAL")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // std::cout << "adding cylindrical joint " << joint_pair.first <<std::endl;
            auto ch_joint = chrono_types::make_shared<ChLinkLockCylindrical>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)).GetCoord(),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)).GetCoord());
            new_joint = ch_joint;
        } else if (joint.type == std::string("RACKPIN")) {
            // Make spherical
            auto body_I = dynamic_cast<ChBodyAuxRef*>(marker_I->GetBody());
            auto body_J = dynamic_cast<ChBodyAuxRef*>(marker_J->GetBody());

            assert(body_I);
            assert(body_J);

            // std::cout << "adding rackpin joint " << joint_pair.first <<std::endl;
            auto ch_joint = chrono_types::make_shared<ChLinkRackpinion>();
            ch_joint->Initialize(sys.SearchBody(body_I->GetName()), sys.SearchBody(body_J->GetName()), true,
                                 (body_I->GetFrame_REF_to_abs() * (*marker_I)),
                                 (body_J->GetFrame_REF_to_abs() * (*marker_J)));
            new_joint = ch_joint;
        } else {
            joint_not_parsed = true;
            std::cerr << "unknown joint type " << joint.type << "!" << std::endl;
        }
        if (!joint_not_parsed) {
            new_joint->SetNameString(joint_pair.first);
            sys.AddLink(new_joint);
            m_report.joints.insert(std::make_pair(joint_pair.first, Report::JointInfo{joint.type, new_joint}));
        }
    }
    // Load visualizations, if enabled
    if (m_visType == ChParserAdams::VisType::LOADED) {
        for (auto tokens : graphics_token_list) {
            iter = tokens.begin();
            //
            if (iter->second == std::string("CYLINDER")) {
                auto cylinder = chrono_types::make_shared<ChCylinderShape>();
                iter++;  // Get next field
                std::shared_ptr<ChMarker> cm_marker;
                ChBodyAuxRef* parentBody = nullptr;
                while (iter != tokens.end()) {
                    // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;
                    if (iter->second == std::string("CM")) {
                        iter++;  // Get marker ID
                        cm_marker = sys.SearchMarker(iter->second.c_str());
                        // std::cout << " adding cylinder to marker " << iter->second << std::endl;
                        assert(cm_marker != nullptr);
                        parentBody = dynamic_cast<ChBodyAuxRef*>(cm_marker->GetBody());
                        assert(parentBody != nullptr);
                        auto cyl_pos = (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                           .TransformPointLocalToParent(ChVector<>(0, 0, 0));
                        auto cyl_rot = (parentBody->GetFrame_COG_to_REF() * (*cm_marker)).Amatrix;
                        parentBody->AddVisualShape(cylinder, ChFrame<>(cyl_pos, cyl_rot));
                    } else if (iter->second == std::string("RADIUS")) {
                        iter++;  // get radius
                        cylinder->GetGeometry().r = std::stod(iter->second);
                    } else if (iter->second == std::string("LENGTH")) {
                        assert(parentBody);
                        iter++;  // get length
                        cylinder->GetGeometry().h = std::stod(iter->second);
                    } else {
                        tokenParseError(LABEL, *iter);
                    }
                    iter++;  // next loop
                }
            } else if (iter->second == std::string("ELLIPSOID")) {
                auto ellipsoid = chrono_types::make_shared<ChEllipsoidShape>();
                iter++;  // Get next field
                std::shared_ptr<ChMarker> cm_marker;
                ChBodyAuxRef* parentBody;
                while (iter != tokens.end()) {
                    // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;
                    if (iter->second == std::string("CM")) {
                        iter++;  // Get marker ID
                        cm_marker = sys.SearchMarker(iter->second.c_str());
                        // std::cout << " adding ellipsoid to marker " << iter->second <<std::endl;
                        assert(cm_marker != nullptr);
                        parentBody = dynamic_cast<ChBodyAuxRef*>(cm_marker->GetBody());
                        assert(parentBody != nullptr);
                        // Put ellipsoid at marker, orientation might be wrong
                        // adams says the orientation is marker-specific but I'm guessing Chrono says it's global or
                        // body-specific
                        auto ell_pos =
                            (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                .TransformPointLocalToParent(ChVector<>(0, 0, 0));
                        parentBody->AddVisualShape(ellipsoid, ChFrame<>(ell_pos, QUNIT));
                    } else if (iter->second == std::string("XSCALE") || iter->second == std::string("XS")) {
                        iter++;  // get length
                        double scale = std::stod(iter->second);
                        // maybe this is right?
                        ellipsoid->GetGeometry().rad.x() = scale;
                    } else if (iter->second == std::string("YSCALE") || iter->second == std::string("YS")) {
                        iter++;  // get length
                        double scale = std::stod(iter->second);
                        // maybe this is right?
                        ellipsoid->GetGeometry().rad.y() = scale;
                    } else if (iter->second == std::string("ZSCALE") || iter->second == std::string("ZS")) {
                        iter++;  // get length
                        double scale = std::stod(iter->second);
                        // maybe this is right?
                        ellipsoid->GetGeometry().rad.z() = scale;

                    } else {
                        tokenParseError(LABEL, *iter);
                    }
                    iter++;  // next loop
                }
            } else if (iter->second == std::string("BOX")) {
                auto box = chrono_types::make_shared<ChBoxShape>();
                iter++;  // Get next field
                std::shared_ptr<ChMarker> cm_marker;
                ChBodyAuxRef* parentBody;
                while (iter != tokens.end()) {
                    // std::cout << "token is " << iter->first << ", " << iter->second <<std::endl;
                    if (iter->second == std::string("CORNER")) {
                        iter++;  // Get marker ID
                        cm_marker = sys.SearchMarker(iter->second.c_str());
                        // std::cout << " adding box to marker " << iter->second <<std::endl;
                        assert(cm_marker != nullptr);
                        parentBody = dynamic_cast<ChBodyAuxRef*>(cm_marker->GetBody());
                        assert(parentBody != nullptr);
                        // Put box at marker
                        auto box_pos = (parentBody->GetFrame_COG_to_REF() * (*cm_marker))
                                                        .TransformPointLocalToParent(ChVector<>(0, 0, 0));

                        auto box_rot = (parentBody->GetFrame_COG_to_REF() * (*cm_marker)).Amatrix;
                        parentBody->AddVisualShape(box, ChFrame<>(box_pos, box_rot));
                    } else if (iter->second == std::string("X")) {
                        iter++;  // get length
                        double scale = std::stod(iter->second);
                        // maybe this is right?
                        box->GetGeometry().hlen.x() = scale / 2;
                    } else if (iter->second == std::string("Y")) {
                        iter++;  // get length
                        double scale = std::stod(iter->second);
                        // maybe this is right?
                        box->GetGeometry().hlen.y() = scale / 2;
                    } else if (iter->second == std::string("Z")) {
                        iter++;  // get length
                        double scale = std::stod(iter->second);
                        // maybe this is right?
                        box->GetGeometry().hlen.z() = scale / 2;

                    } else {
                        tokenParseError(LABEL, *iter);
                    }
                    iter++;  // next loop
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Makes a new system and then parses into it
// -----------------------------------------------------------------------------

ChSystem* ChParserAdams::Parse(const std::string& filename, ChContactMethod contact_method) {
    ChSystem* sys = (contact_method == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                             : static_cast<ChSystem*>(new ChSystemSMC);

    Parse(*sys, filename);

    return sys;
}

}  // end namespace utils
}  // end namespace chrono
