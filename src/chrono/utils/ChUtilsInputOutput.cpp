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
// =============================================================================

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapeRoundedCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// WriteBodies
//
// Write to a CSV file pody position, orientation, and (optionally) linear and
// angular velocity. Optionally, only active bodies are processed.
// -----------------------------------------------------------------------------
void WriteBodies(ChSystem* system,
                 const std::string& filename,
                 bool active_only,
                 bool dump_vel,
                 const std::string& delim) {
    ChWriterCSV csv(delim);

    for (auto body : system->GetBodies()) {
        if (active_only && !body->IsActive())
            continue;
        csv << body->GetPos() << body->GetRot();
        if (dump_vel)
            csv << body->GetPosDt() << body->GetAngVelLocal();
        csv << std::endl;
    }

    csv.WriteToFile(filename);
}

// -----------------------------------------------------------------------------
// WriteCheckpoint
//
// Create a CSV file with a checkpoint ...
//
// -----------------------------------------------------------------------------
bool WriteCheckpoint(ChSystem* system, const std::string& filename) {
    // Create the CSV stream.
    ChWriterCSV csv(" ");
    std::string tab("    ");

    // Write contact method type (0: NSC, 1: SMC)
    int ctype = (system->GetContactMethod() == ChContactMethod::NSC) ? 0 : 1;
    csv << ctype << std::endl;

    for (auto body : system->GetBodies()) {
        // Write body identifier, the body fixed flag, and the collide flag
        csv << body->GetTag() << body->IsFixed() << body->IsCollisionEnabled() << tab;

        // Write collision family information.
        csv << body->GetCollisionModel()->GetFamilyGroup() << body->GetCollisionModel()->GetFamilyMask() << tab;

        // Write body mass and inertia
        csv << body->GetMass() << body->GetInertiaXX() << tab;

        // Write body position, orientation, and their time derivatives
        csv << body->GetPos() << body->GetRot() << tab;
        csv << body->GetPosDt() << body->GetRotDt() << tab;

        csv << std::endl;

        // Write number of collision shapes
        unsigned int n_shapes = body->GetCollisionModel()->GetNumShapes();
        csv << n_shapes << std::endl;

        // Loop over each shape and write its data on a separate line.
        // If we encounter an unsupported type, return false.
        for (const auto& s : body->GetCollisionModel()->GetShapeInstances()) {
            const auto& shape = s.first;
            const auto& frame = s.second;

            // Write relative position and rotation
            csv << frame.GetPos() << frame.GetRot() << tab;

            // Write shape material information
            if (ctype == 0) {
                auto mat = std::static_pointer_cast<ChContactMaterialNSC>(shape->GetMaterial());
                csv << mat->static_friction << mat->sliding_friction << mat->rolling_friction << mat->spinning_friction;
                csv << mat->restitution << mat->cohesion << mat->dampingf;
                csv << mat->compliance << mat->complianceT << mat->complianceRoll << mat->complianceSpin;
                csv << tab;
            } else {
                auto mat = std::static_pointer_cast<ChContactMaterialSMC>(shape->GetMaterial());
                csv << mat->young_modulus << mat->poisson_ratio;
                csv << mat->static_friction << mat->sliding_friction;
                csv << mat->restitution << mat->constant_adhesion << mat->adhesionMultDMT;
                csv << mat->kn << mat->gn << mat->kt << mat->gt;
                csv << tab;
            }

            // Write shape type and characteristic dimensions
            std::vector<double> dims;
            switch (shape->GetType()) {
                case ChCollisionShape::Type::SPHERE: {
                    auto sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
                    dims = {sphere->GetRadius()};
                    break;
                }
                case ChCollisionShape::Type::BOX: {
                    auto box = std::static_pointer_cast<ChCollisionShapeBox>(shape);
                    const auto& hdims = box->GetLengths();
                    dims = {hdims.x(), hdims.y(), hdims.z()};
                    break;
                }
                case ChCollisionShape::Type::ELLIPSOID: {
                    auto ellipsoid = std::static_pointer_cast<ChCollisionShapeEllipsoid>(shape);
                    const auto& r = ellipsoid->GetAxes();
                    dims = {r.x(), r.y(), r.z()};
                    break;
                }
                case ChCollisionShape::Type::CYLINDER: {
                    auto cylinder = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
                    auto height = cylinder->GetHeight();
                    auto radius = cylinder->GetRadius();
                    dims = {radius, radius, height};
                    break;
                }
                case ChCollisionShape::Type::CYLSHELL: {
                    auto cylshell = std::static_pointer_cast<ChCollisionShapeCylindricalShell>(shape);
                    auto height = cylshell->GetHeight();
                    auto radius = cylshell->GetRadius();
                    dims = {radius, radius, height};
                    break;
                }
                case ChCollisionShape::Type::CONE: {
                    auto cone = std::static_pointer_cast<ChCollisionShapeCone>(shape);
                    auto height = cone->GetHeight();
                    auto radius = cone->GetRadius();
                    dims = {radius, radius, height};
                    break;
                }
                case ChCollisionShape::Type::CAPSULE: {
                    auto capsule = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
                    auto height = capsule->GetHeight();
                    auto radius = capsule->GetRadius();
                    dims = {radius, radius, height};
                    break;
                }
                case ChCollisionShape::Type::ROUNDEDBOX: {
                    auto box = std::static_pointer_cast<ChCollisionShapeRoundedBox>(shape);
                    const auto& hdims = box->GetLengths();
                    auto sradius = box->GetSRadius();
                    dims = {hdims.x(), hdims.y(), hdims.z(), sradius};
                    break;
                }
                case ChCollisionShape::Type::ROUNDEDCYL: {
                    auto cylinder = std::static_pointer_cast<ChCollisionShapeRoundedCylinder>(shape);
                    auto height = cylinder->GetHeight();
                    auto radius = cylinder->GetRadius();
                    auto sradius = cylinder->GetSRadius();
                    dims = {radius, radius, height, sradius};
                    break;
                }
                default:
                    std::cout << "utils::WriteCheckpoint ERROR: unknown or not supported collision shape\n";
                    return false;
            }

            csv << shape->GetType() << dims;

            csv << std::endl;
        }
    }

    csv.WriteToFile(filename);

    return true;
}

// -----------------------------------------------------------------------------
// ReadCheckpoint
//
// Read a CSV file with checkpoint data and create the bodies.
//
// -----------------------------------------------------------------------------
void ReadCheckpoint(ChSystem* system, const std::string& filename) {
    // Open input file stream
    std::ifstream ifile(filename);
    std::string line;

    // Read the contact method type
    std::getline(ifile, line);
    std::istringstream iss0(line);
    int ctype;
    iss0 >> ctype;
    auto contact_method = (ctype == 0) ? ChContactMethod::NSC : ChContactMethod::SMC;

    // Check consistency with the current system
    if (contact_method != system->GetContactMethod()) {
        std::cout << "utils::ReadCheckpoint ERROR: checkpoint data file inconsistent with the Chrono system\n";
        std::cout << "    Contact method in data file: " << (ctype == 0 ? "NSC" : "SMC") << "\n";
        return;
    }

    while (std::getline(ifile, line)) {
        std::istringstream iss1(line);

        // Read body Id and flags
        int btag, bfixed, bcollide;
        short family_group, family_mask;
        iss1 >> btag >> bfixed >> bcollide >> family_group >> family_mask;

        // Read body mass and inertia
        double mass;
        ChVector3d inertiaXX;
        iss1 >> mass >> inertiaXX.x() >> inertiaXX.y() >> inertiaXX.z();

        // Read body position, orientation, and their time derivatives
        ChVector3d bpos, bpos_dt;
        ChQuaternion<> brot, brot_dt;
        iss1 >> bpos.x() >> bpos.y() >> bpos.z();
        iss1 >> brot.e0() >> brot.e1() >> brot.e2() >> brot.e3();
        iss1 >> bpos_dt.x() >> bpos_dt.y() >> bpos_dt.z();
        iss1 >> brot_dt.e0() >> brot_dt.e1() >> brot_dt.e2() >> brot_dt.e3();

        // Create a body of the appropriate type
        auto body = chrono_types::make_shared<ChBody>();
        system->AddBody(body);

        // Set body properties and state
        body->SetPos(bpos);
        body->SetRot(brot);
        body->SetPosDt(bpos_dt);
        body->SetRotDt(brot_dt);

        body->SetTag(btag);
        body->SetFixed(bfixed != 0);
        body->EnableCollision(bcollide != 0);

        body->SetMass(mass);
        body->SetInertiaXX(inertiaXX);

        // Get next line in the file (number of collision shapes)
        std::getline(ifile, line);
        std::istringstream iss3(line);

        int n_shapes;
        iss3 >> n_shapes;

        // In a loop, read information about each shape and add geometry to the body
        for (int j = 0; j < n_shapes; j++) {
            std::getline(ifile, line);
            std::istringstream iss(line);

            // Get shape relative position and rotation
            ChVector3d spos;
            ChQuaternion<> srot;
            iss >> spos.x() >> spos.y() >> spos.z() >> srot.e0() >> srot.e1() >> srot.e2() >> srot.e3();

            // Get material information and create the material
            std::shared_ptr<ChContactMaterial> mat;
            if (ctype == 0) {
                auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
                iss >> matNSC->static_friction >> matNSC->sliding_friction >> matNSC->rolling_friction >>
                    matNSC->spinning_friction;
                iss >> matNSC->restitution >> matNSC->cohesion >> matNSC->dampingf;
                iss >> matNSC->compliance >> matNSC->complianceT >> matNSC->complianceRoll >> matNSC->complianceSpin;
                mat = matNSC;
            } else {
                auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
                iss >> matSMC->young_modulus >> matSMC->poisson_ratio;
                iss >> matSMC->static_friction >> matSMC->sliding_friction;
                iss >> matSMC->restitution >> matSMC->constant_adhesion >> matSMC->adhesionMultDMT;
                iss >> matSMC->kn >> matSMC->gn >> matSMC->kt >> matSMC->gt;
                mat = matSMC;
            }

            // Get shape type and geometry data and create the shape (both visualization and contact).
            int stype;
            iss >> stype;

            switch (ChCollisionShape::Type(stype)) {
                case ChCollisionShape::Type::SPHERE: {
                    double radius;
                    iss >> radius;
                    AddSphereGeometry(body.get(), mat, radius, spos, srot);
                } break;
                case ChCollisionShape::Type::ELLIPSOID: {
                    ChVector3d size;
                    iss >> size.x() >> size.y() >> size.z();
                    AddEllipsoidGeometry(body.get(), mat, size, spos, srot);
                } break;
                case ChCollisionShape::Type::BOX: {
                    ChVector3d size;
                    iss >> size.x() >> size.y() >> size.z();
                    AddBoxGeometry(body.get(), mat, size, spos, srot);
                } break;
                case ChCollisionShape::Type::CAPSULE: {
                    double radius, height;
                    iss >> radius >> height;
                    AddCapsuleGeometry(body.get(), mat, radius, height, spos, srot);
                } break;
                case ChCollisionShape::Type::CYLINDER: {
                    double radius, height;
                    iss >> radius >> height;
                    AddCylinderGeometry(body.get(), mat, radius, height, spos, srot);
                } break;
                case ChCollisionShape::Type::CONE: {
                    double radius, height;
                    iss >> radius >> height;
                    AddConeGeometry(body.get(), mat, radius, height, spos, srot);
                } break;
                case ChCollisionShape::Type::ROUNDEDBOX: {
                    ChVector3d size;
                    double srad;
                    iss >> size.x() >> size.y() >> size.z() >> srad;
                    AddRoundedBoxGeometry(body.get(), mat, size, srad, spos, srot);
                } break;
                case ChCollisionShape::Type::ROUNDEDCYL: {
                    double radius, height, srad;
                    iss >> radius >> height >> srad;
                    AddRoundedCylinderGeometry(body.get(), mat, radius, height, srad, spos, srot);
                } break;
                default:
                    break;
            }
        }

        // Set the collision family group and the collision family mask.
        body->GetCollisionModel()->SetFamilyGroup(family_group);
        body->GetCollisionModel()->SetFamilyMask(family_mask);
    }
}

// -----------------------------------------------------------------------------
// Write CSV output file with current camera information
// -----------------------------------------------------------------------------
void WriteCamera(const std::string& filename,
                 const ChVector3d& cam_location,
                 const ChVector3d& cam_target,
                 const ChVector3d& camera_upvec,
                 const std::string& delim) {
    ChWriterCSV csv(delim);
    csv << cam_location << std::endl;
    csv << cam_target << std::endl;
    csv << camera_upvec << std::endl;
    csv.WriteToFile(filename);
}

// -----------------------------------------------------------------------------
// Write CSV output file for PovRay.
// A line with information about a visualization asset contains:
//    bodyId, bodyActive, x, y, z, e0, e1, e2, e3, shapeType, [shape Data]
// A line with information about a link contains:
//    linkType, [linkData]
//
// NOTE: we do not account for any transform specified for the ChGeometry of
// a visual asset (except for cylinders where that is implicit)!
// -----------------------------------------------------------------------------
enum POVRayShapeType {
    SPHERE = 0,
    ELLIPSOID = 1,
    BOX = 2,
    CYLINDER = 3,
    CONVEXHULL = 4,
    TRIANGLEMESH = 5,
    BARREL = 6,
    CAPSULE = 7,
    CONE = 8,
    ROUNDEDBOX = 9,
    ROUNDEDCYL = 10,
    BEZIER = 11
};

enum POVRayLinkType {
    REVOLUTE = 0,
    SPHERICAL = 1,
    PRISMATIC = 2,
    UNIVERSAL = 3,
    DISTANCE = 4,
    ENGINE = 5,
    SPRING = 6,
    TSDA = 7,
    CYLINDRICAL = 8,
    REV_SPH = 9
};

enum POVRayLineType { SEGMENT = 0, COIL = 1 };

void WriteVisualizationAssets(ChSystem* system, const std::string& filename, bool body_info, const std::string& delim) {
    WriteVisualizationAssets(
        system,                                        //
        filename,                                      //
        [](const ChBody& b) -> bool { return true; },  //
        body_info,                                     //
        delim);
}

void WriteVisualizationAssets(ChSystem* system,
                              const std::string& filename,
                              std::function<bool(const ChBody&)> selector,
                              bool body_info,
                              const std::string& delim) {
    ChWriterCSV csv(delim);

    // If requested, Loop over all bodies and write out their position and
    // orientation.  Otherwise, body count is left at 0.
    int b_count = 0;

    if (body_info) {
        for (auto body : system->GetBodies()) {
            if (!selector(*body))
                continue;

            const ChVector3d& body_pos = body->GetFrameRefToAbs().GetPos();
            const ChQuaternion<>& body_rot = body->GetFrameRefToAbs().GetRot();

            csv << body->GetIdentifier() << body->IsActive() << body_pos << body_rot << std::endl;

            b_count++;
        }
    }

    // Loop over all bodies and over all their assets.
    int a_count = 0;
    for (auto body : system->GetBodies()) {
        if (!selector(*body))
            continue;

        if (!body->GetVisualModel())
            continue;

        // Loop over visual shapes -- write information for supported types.
        for (auto& shape_instance : body->GetVisualModel()->GetShapeInstances()) {
            auto& shape = shape_instance.first;
            auto X_GS = body->GetFrameRefToAbs() * shape_instance.second;
            auto& pos = X_GS.GetPos();
            auto& rot = X_GS.GetRot();

            bool supported = true;
            std::stringstream gss;

            if (auto sphere = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
                gss << SPHERE << delim << sphere->GetRadius();
                a_count++;
            } else if (auto ellipsoid = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
                const ChVector3d& size = ellipsoid->GetSemiaxes();
                gss << ELLIPSOID << delim << size.x() << delim << size.y() << delim << size.z();
                a_count++;
            } else if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
                const ChVector3d& hlen = box->GetHalflengths();
                gss << BOX << delim << hlen.x() << delim << hlen.y() << delim << hlen.z();
                a_count++;
            } else if (auto capsule = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
                gss << CAPSULE << delim << capsule->GetRadius() << delim << capsule->GetHeight();
                a_count++;
            } else if (auto cylinder = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
                gss << CYLINDER << delim << cylinder->GetRadius() << delim << cylinder->GetHeight();
                a_count++;
            } else if (auto cone = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
                gss << CONE << delim << cone->GetRadius() << delim << cone->GetHeight();
                a_count++;
            } else if (auto rbox = std::dynamic_pointer_cast<ChVisualShapeRoundedBox>(shape)) {
                const ChVector3d& hlen = rbox->GetHalflengths();
                double srad = rbox->GetSphereRadius();
                gss << ROUNDEDBOX << delim << hlen.x() << delim << hlen.y() << delim << hlen.z() << delim << srad;
                a_count++;
            } else if (auto rcyl = std::dynamic_pointer_cast<ChVisualShapeRoundedCylinder>(shape)) {
                double rad = rcyl->GetRadius();
                double height = rcyl->GetHeight();
                double srad = rcyl->GetSphereRadius();
                gss << ROUNDEDCYL << delim << rad << delim << height << delim << srad;
                a_count++;
            } else if (auto mesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
                gss << TRIANGLEMESH << delim << "\"" << mesh->GetName() << "\"";
                a_count++;
            } else if (auto line = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
                std::shared_ptr<ChLine> geom = line->GetLineGeometry();
                if (auto bezier = std::dynamic_pointer_cast<ChLineBezier>(geom)) {
                    gss << BEZIER << delim << "\"" << line->GetName() << "\"";
                    a_count++;
                } else {
                    supported = false;
                }
            } else {
                supported = false;
            }

            if (supported) {
                ChColor col = ChVisualMaterial::Default()->GetDiffuseColor();
                if (shape->GetNumMaterials() > 0)
                    col = shape->GetMaterial(0)->GetDiffuseColor();
                csv << body->GetIdentifier() << body->IsActive() << pos << rot << col << gss.str() << std::endl;
            }
        }
    }

    // Loop over all links.  Write information on selected types of links.
    int l_count = 0;
    for (auto ilink : system->GetLinks()) {
        if (auto linkR = std::dynamic_pointer_cast<ChLinkLockRevolute>(ilink)) {
            chrono::ChFrame<> frA_abs = *(linkR->GetMarker1()) >> *(linkR->GetBody1());
            chrono::ChFrame<> frB_abs = *(linkR->GetMarker2()) >> *(linkR->GetBody2());

            csv << REVOLUTE << frA_abs.GetPos() << frA_abs.GetRotMat().GetAxisZ() << std::endl;
            l_count++;
        } else if (auto linkS = std::dynamic_pointer_cast<ChLinkLockSpherical>(ilink)) {
            chrono::ChFrame<> frA_abs = *(linkS->GetMarker1()) >> *(linkS->GetBody1());
            chrono::ChFrame<> frB_abs = *(linkS->GetMarker2()) >> *(linkS->GetBody2());

            csv << SPHERICAL << frA_abs.GetPos() << std::endl;
            l_count++;
        } else if (auto linkP = std::dynamic_pointer_cast<ChLinkLockPrismatic>(ilink)) {
            chrono::ChFrame<> frA_abs = *(linkP->GetMarker1()) >> *(linkP->GetBody1());
            chrono::ChFrame<> frB_abs = *(linkP->GetMarker2()) >> *(linkP->GetBody2());

            csv << PRISMATIC << frA_abs.GetPos() << frA_abs.GetRotMat().GetAxisZ() << std::endl;
            l_count++;
        } else if (auto linkC = std::dynamic_pointer_cast<ChLinkLockCylindrical>(ilink)) {
            chrono::ChFrame<> frA_abs = *(linkC->GetMarker1()) >> *(linkC->GetBody1());
            chrono::ChFrame<> frB_abs = *(linkC->GetMarker2()) >> *(linkC->GetBody2());

            csv << CYLINDRICAL << frA_abs.GetPos() << frA_abs.GetRotMat().GetAxisZ() << std::endl;
            l_count++;
        } else if (auto linkU = std::dynamic_pointer_cast<ChLinkUniversal>(ilink)) {
            chrono::ChFrame<> frA_abs = linkU->GetFrame1Abs();
            chrono::ChFrame<> frB_abs = linkU->GetFrame2Abs();

            csv << UNIVERSAL << frA_abs.GetPos() << frA_abs.GetRotMat().GetAxisX() << frB_abs.GetRotMat().GetAxisY()
                << std::endl;
            l_count++;
        } else if (auto linkT = std::dynamic_pointer_cast<ChLinkTSDA>(ilink)) {
            csv << TSDA << linkT->GetPoint1Abs() << linkT->GetPoint2Abs() << std::endl;
            l_count++;
        } else if (auto linkD = std::dynamic_pointer_cast<ChLinkDistance>(ilink)) {
            csv << DISTANCE << linkD->GetEndPoint1Abs() << linkD->GetEndPoint2Abs() << std::endl;
            l_count++;
        } else if (auto linkRS = std::dynamic_pointer_cast<ChLinkRevoluteSpherical>(ilink)) {
            csv << REV_SPH << linkRS->GetPoint1Abs() << linkRS->GetPoint2Abs() << std::endl;
            l_count++;
        }
    }

    // Loop over links and write assets associated with spring-dampers.
    int la_count = 0;
    for (auto ilink : system->GetLinks()) {
        auto link = std::dynamic_pointer_cast<ChLinkTSDA>(ilink);
        if (!link)
            continue;
        if (!link->GetVisualModel())
            continue;
        for (auto& shape_instance : link->GetVisualModel()->GetShapeInstances()) {
            auto& shape = shape_instance.first;
            if (std::dynamic_pointer_cast<ChVisualShapeSegment>(shape)) {
                csv << SEGMENT << link->GetPoint1Abs() << link->GetPoint2Abs() << std::endl;
                la_count++;
            } else if (std::dynamic_pointer_cast<ChVisualShapeSpring>(shape)) {
                csv << COIL << link->GetPoint1Abs() << link->GetPoint2Abs() << std::endl;
                la_count++;
            }
        }
    }

    // Write the output file, including a first line with number of bodies, visual
    // assets, links, and TSDA assets.
    std::stringstream header;
    header << b_count << delim << a_count << delim << l_count << delim << la_count << delim << std::endl;

    csv.WriteToFile(filename, header.str());
}

// -----------------------------------------------------------------------------
// WriteMeshPovray
//
// Write the triangular mesh from the specified OBJ file as a macro in a PovRay
// include file.
// -----------------------------------------------------------------------------
void WriteMeshPovray(ChTriangleMeshConnected& trimesh,
                     const std::string& mesh_name,
                     const std::string& out_dir,
                     const ChColor& col,
                     const ChVector3d& pos,
                     const ChQuaternion<>& rot,
                     bool smoothed) {
    // Transform vertices.
    for (unsigned int i = 0; i < trimesh.m_vertices.size(); i++)
        trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);

    // Transform normals
    if (smoothed) {
        for (unsigned int i = 0; i < trimesh.m_normals.size(); i++)
            trimesh.m_normals[i] = rot.Rotate(trimesh.m_normals[i]);
    }

    // Open output file.
    std::string pov_filename = out_dir + "/" + mesh_name + ".inc";
    std::ofstream ofile(pov_filename);

    ofile << "#declare " << mesh_name << "_mesh = mesh2 {" << std::endl;

    // Write vertices.
    ofile << "vertex_vectors {" << std::endl;
    ofile << trimesh.m_vertices.size();
    for (unsigned int i = 0; i < trimesh.m_vertices.size(); i++) {
        ChVector3d v = trimesh.m_vertices[i];
        ofile << ",\n<" << v.x() << ", " << v.z() << ", " << v.y() << ">";
    }
    ofile << "\n}" << std::endl;

    // Write normals.
    if (smoothed) {
        ofile << "normal_vectors {" << std::endl;
        ofile << trimesh.m_normals.size();
        for (unsigned int i = 0; i < trimesh.m_normals.size(); i++) {
            ChVector3d n = trimesh.m_normals[i];
            ofile << ",\n<" << n.x() << ", " << n.z() << ", " << n.y() << ">";
        }
        ofile << "\n}" << std::endl;
    }

    // Write face connectivity.
    ofile << "face_indices {" << std::endl;
    ofile << trimesh.m_face_v_indices.size();
    for (int i = 0; i < trimesh.m_face_v_indices.size(); i++) {
        ChVector3i face = trimesh.m_face_v_indices[i];
        ofile << ",\n<" << face.x() << ", " << face.y() << ", " << face.z() << ">";
    }
    ofile << "\n}" << std::endl;

    ofile << "\n}" << std::endl;

    // Write the object
    ofile << "#declare " << mesh_name << " = object {" << std::endl;

    ofile << "   " << mesh_name << "_mesh" << std::endl;
    ofile << "   texture {" << std::endl;
    ofile << "      pigment {color rgb<" << col.R << ", " << col.G << ", " << col.B << ">}" << std::endl;
    ofile << "      finish  {phong 0.2  diffuse 0.6}" << std::endl;
    ofile << "    }" << std::endl;
    ofile << "}" << std::endl;

    // Close the output file.
    ofile.close();
}

bool WriteMeshPovray(const std::string& obj_filename,
                     const std::string& mesh_name,
                     const std::string& out_dir,
                     const ChColor& col,
                     const ChVector3d& pos,
                     const ChQuaternion<>& rot) {
    // Read trimesh from OBJ file
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, false, false);
    if (!trimesh)
        return false;

    // Generate output
    WriteMeshPovray(*trimesh, mesh_name, out_dir, col, pos, rot);

    return true;
}

// -----------------------------------------------------------------------------
// WriteCurvePovray
//
// Write the specified Bezier curve as a macro in a PovRay include file.
// -----------------------------------------------------------------------------
void WriteCurvePovray(const ChBezierCurve& curve,
                      const std::string& curve_name,
                      const std::string& out_dir,
                      double radius,
                      const ChColor& col) {
    int nP = 20;
    double dt = 1.0 / nP;
    size_t nS = curve.GetNumPoints() - 1;

    // Open output file.
    std::string pov_filename = out_dir + "/" + curve_name + ".inc";
    std::ofstream ofile(pov_filename);

    ofile << "#declare " << curve_name << " = object {" << std::endl;
    ofile << "  sphere_sweep {" << std::endl;
    ofile << "    linear_spline " << nP * nS + 1 << "," << std::endl;

    ChVector3d v = curve.Eval(0, 0.0);
    ofile << "        <" << v.x() << ", " << v.z() << ", " << v.x() << "> ," << radius << std::endl;

    for (int iS = 0; iS < nS; iS++) {
        for (int iP = 1; iP <= nP; iP++) {
            v = curve.Eval(iS, iP * dt);
            ofile << "        <" << v.x() << ", " << v.z() << ", " << v.y() << "> ," << radius << std::endl;
        }
    }

    ofile << "    texture {" << std::endl;
    ofile << "      pigment {color rgb<" << col.R << ", " << col.G << ", " << col.B << ">}" << std::endl;
    ofile << "      finish  {phong 0.2  diffuse 0.6}" << std::endl;
    ofile << "     }" << std::endl;

    ofile << "  }" << std::endl;
    ofile << "}" << std::endl;

    // Close the output file.
    ofile.close();
}

}  // namespace utils
}  // namespace chrono
