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
// Base class for a tracked vehicle sprocket. A sprocket is responsible for
// contact processing with the track shoes of the containing track assembly.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChColor.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChSprocket::ChSprocket(const std::string& name) : ChPart(name), m_lateral_contact(true) {}

ChSprocket::~ChSprocket() {
    auto sys = m_gear->GetSystem();
    if (sys) {
        sys->Remove(m_gear);
        sys->Remove(m_axle);
        sys->Remove(m_axle_to_spindle);
        sys->Remove(m_revolute);

        sys->UnregisterCustomCollisionCallback(m_callback);
    }
}

// -----------------------------------------------------------------------------
void ChSprocket::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location, ChTrackAssembly* track) {
    m_parent = chassis;
    m_rel_loc = location;

    // The sprocket reference frame is aligned with that of the chassis and centered at the specified location.
    ChVector<> loc = chassis->GetBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(location);
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();
    ChQuaternion<> y2z = Q_from_AngX(CH_C_PI_2);
    ChMatrix33<> rot_y2z(y2z);

    // Create and initialize the gear body (same orientation as the chassis).
    m_gear = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_gear->SetNameString(m_name + "_gear");
    m_gear->SetIdentifier(BodyID::SPROCKET_BODY);
    m_gear->SetPos(loc);
    m_gear->SetRot(chassisRot);
    m_gear->SetMass(GetGearMass());
    m_gear->SetInertiaXX(GetGearInertia());
    chassis->GetSystem()->AddBody(m_gear);

    // Create and initialize the revolute joint between chassis and gear.
    ChCoordsys<> rev_csys(loc, chassisRot * y2z);
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis->GetBody(), m_gear, rev_csys);
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the axle shaft and its connection to the gear. Note that the
    // gear rotates about the Y axis.
    m_axle = chrono_types::make_shared<ChShaft>();
    m_axle->SetNameString(m_name + "_axle");
    m_axle->SetInertia(GetAxleInertia());
    chassis->GetSystem()->AddShaft(m_axle);

    m_axle_to_spindle = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle->SetNameString(m_name + "_axle_to_spindle");
    m_axle_to_spindle->Initialize(m_axle, m_gear, ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle);

    // Enable contact for the gear body and set contact material properties.
    m_gear->SetCollide(true);
    CreateContactMaterial(chassis->GetSystem()->GetContactMethod());

    // Set user-defined custom collision callback class for sprocket-shoes contact.
    m_callback = GetCollisionCallback(track);
    chassis->GetSystem()->RegisterCustomCollisionCallback(m_callback);
}

void ChSprocket::InitializeInertiaProperties() {
    m_mass = GetGearMass();
    m_inertia = ChMatrix33<>(0);
    m_inertia.diagonal() = GetGearInertia().eigen();
    m_com = ChFrame<>();
}

void ChSprocket::UpdateInertiaProperties() {
    m_xform = m_gear->GetFrame_REF_to_abs();
}

// -----------------------------------------------------------------------------
void ChSprocket::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto sep = GetSeparation();
    auto profile = GetProfile();

    ChQuaternion<> y2z = Q_from_AngX(CH_C_PI_2);
    ChMatrix33<> rot_y2z(y2z);

    auto asset_1 = chrono_types::make_shared<ChLineShape>();
    asset_1->SetLineGeometry(profile);
    asset_1->Pos = ChVector<>(0, sep / 2, 0);
    asset_1->Rot = rot_y2z;
    asset_1->SetColor(ChColor(1, 0, 0));
    m_gear->AddAsset(asset_1);

    auto asset_2 = chrono_types::make_shared<ChLineShape>();
    asset_2->SetLineGeometry(profile);
    asset_2->Pos = ChVector<>(0, -sep / 2, 0);
    asset_2->Rot = rot_y2z;
    asset_2->SetColor(ChColor(1, 0, 0));
    m_gear->AddAsset(asset_2);
}

void ChSprocket::RemoveVisualizationAssets() {
    m_gear->GetAssets().clear();
}

// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChTriangleMeshConnected> ChSprocket::CreateVisualizationMesh(double radius,
                                                                                       double width,
                                                                                       double delta,
                                                                                       ChVector<float> color) const {
    auto sep = GetSeparation();
    auto profile = GetProfile();

    // Evaluate points on gear profile and on ring.
    // Generate equidistant points on profile, then transform to x-z plane.
    // Calculate normals in radial direction.
    std::vector<ChVector<>> ppoints; // points on profile
    std::vector<ChVector<>> rpoints; // points on ring
    std::vector<ChVector<>> pnormals; // normals on profile
    std::vector<ChVector<>> rnormals; // normals on ring
    ChVector<> p;
    for (auto il = 0; il < profile->GetSubLinesCount(); il++) {
        auto line = profile->GetSubLineN(il);
        auto n = static_cast<int>(std::ceil(line->Length(2) / delta));
        for (auto ip = 0; ip < n; ip++) {            
            line->Evaluate(p, (1.0 * ip)/n);
            ppoints.push_back(ChVector<>(p.x(), 0, p.y())); // Point on profile
            p *= radius / p.Length();
            rpoints.push_back(ChVector<>(p.x(), 0, p.y())); // Point on ring
            p.Normalize();
            pnormals.push_back(ChVector<>(+p.x(), 0, +p.y())); // Normal on profile (approximate)
            rnormals.push_back(ChVector<>(-p.x(), 0, -p.y())); // Normal on ring
        }
    }

    // Create trimesh
    auto mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    std::vector<ChVector<>>& vertices = mesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = mesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = mesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = mesh->getIndicesNormals();
    ////std::vector<ChVector<>>& uv_coords = mesh->getCoordsUV();
    std::vector<ChVector<float>>& colors = mesh->getCoordsColors();

    // Calculate number of vertices, normals, and faces. Resize mesh arrays.
    auto npoints = ppoints.size();
    auto n_verts = 8 * npoints;
    auto n_normals = 4 + 2 * npoints;
    auto n_faces = 16 * npoints;

    vertices.resize(n_verts);
    ////uv_coords.resize(n_verts);
    colors.resize(n_verts, color);
    normals.resize(n_normals);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    // Create mesh vertices (4 layers, shifted in y direction)
    double offset[4] = {
        +0.5 * sep + 0.5 * width,  // Outer gear, outer face
        +0.5 * sep - 0.5 * width,  // Outer gear, inner face
        -0.5 * sep + 0.5 * width,  // Inner gear, outer face
        -0.5 * sep - 0.5 * width   // Inner gear, inner face
    };
    for (size_t i = 0; i < 4; i++) {
        size_t ivstart = i * (2 * npoints);
        for (size_t ip = 0; ip < npoints; ip++) {
            vertices[ivstart + ip] = ppoints[ip] + ChVector<>(0, offset[i], 0);
            vertices[ivstart + ip + npoints] = rpoints[ip] + ChVector<>(0, offset[i], 0);
        }
    }

    // Create mesh normals
    normals[0] = ChVector<>(0, +1, 0);
    normals[1] = ChVector<>(0, -1, 0);
    normals[2] = ChVector<>(0, +1, 0);
    normals[3] = ChVector<>(0, -1, 0);
    for (size_t ip = 0; ip < npoints; ip++) {
        normals[4 + ip] = pnormals[ip];
        normals[4 + npoints + ip] = rnormals[ip];
    }

    // Create mesh triangular faces, two at a time (4 gear planes)
    int np = static_cast<int>(npoints);
    int it = 0;

    // Create faces on the 4 gear planes
    for (int i = 0; i < 4; i++) {
        size_t ivstart = i * (2 * npoints);
        for (size_t ip = 0; ip < npoints-1; ip++) {
            int iv = static_cast<int>(ivstart + ip);
            idx_vertices[it] = ChVector<int>(iv, iv + 1, iv + np);
            idx_normals[it] = ChVector<int>(i, i, i);
            ++it;
            idx_vertices[it] = ChVector<int>(iv + 1, iv + np + 1, iv + np);
            idx_normals[it] = ChVector<int>(i, i, i);
            ++it;
        }
        int iv = static_cast<int>(ivstart);
        idx_vertices[it] = ChVector<int>(iv + np - 1, iv, iv + 2 * np - 1);
        idx_normals[it] = ChVector<int>(i, i, i);
        ++it;
        idx_vertices[it] = ChVector<int>(iv, iv + np, iv + 2 * np - 1);
        idx_normals[it] = ChVector<int>(i, i, i);
        ++it;
    }

    // Create faces on the gear profile
    for (int i = 0; i < 2; i++) {
        size_t ivstart = i * (4 * npoints);
        for (size_t ip = 0; ip < npoints - 1; ip++) {
            int iv = static_cast<int>(ivstart + ip);
            int in = static_cast<int>(4 + ip);
            idx_vertices[it] = ChVector<int>(iv, iv + 1, iv + 2 * np);
            idx_normals[it] = ChVector<int>(in, in, in);
            ++it;
            idx_vertices[it] = ChVector<int>(iv + 1, iv + 2 * np + 1, iv + 2 * np);
            idx_normals[it] = ChVector<int>(in, in, in);
            ++it;
        }
        int iv = static_cast<int>(ivstart);
        int in = static_cast<int>(4 + npoints - 1);
        idx_vertices[it] = ChVector<int>(iv + np - 1, iv, iv + 3 * np - 1);
        idx_normals[it] = ChVector<int>(in, in, in);
        ++it;
        idx_vertices[it] = ChVector<int>(iv, iv + 2 * np, iv + 3 * np - 1);
        idx_normals[it] = ChVector<int>(in, in, in);
        ++it;
    }

    // Create the faces on the inner ring
    for (int i = 0; i < 2; i++) {
        size_t ivstart = i * (4 * npoints) + npoints;
        for (size_t ip = 0; ip < npoints - 1; ip++) {
            int iv = static_cast<int>(ivstart + ip);
            int in = static_cast<int>(4 + npoints + ip);
            idx_vertices[it] = ChVector<int>(iv, iv + 1, iv + 2 * np);
            idx_normals[it] = ChVector<int>(in, in, in);
            ++it;
            idx_vertices[it] = ChVector<int>(iv + 1, iv + 2 * np + 1, iv + 2 * np);
            idx_normals[it] = ChVector<int>(in, in, in);
            ++it;
        }
        int iv = static_cast<int>(ivstart);
        int in = static_cast<int>(4 + 2 * npoints - 1);
        idx_vertices[it] = ChVector<int>(iv + np - 1, iv, iv + 3 * np - 1);
        idx_normals[it] = ChVector<int>(in, in, in);
        ++it;
        idx_vertices[it] = ChVector<int>(iv, iv + 2 * np, iv + 3 * np - 1);
        idx_normals[it] = ChVector<int>(in, in, in);
        ++it;
    }

    return mesh;
}

// -----------------------------------------------------------------------------
void ChSprocket::ApplyAxleTorque(double torque) {
    // Make sure this is added to any already applied torque. Change sign of provided torque (given the configuration of
    // the ChShaft) so that a positive torque corresponds to "forward" motion.
    m_axle->SetAppliedTorque(m_axle->GetAppliedTorque() - torque);
}

// -----------------------------------------------------------------------------
void ChSprocket::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    GetLog() << "  Sprocket-chassis revolute\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";
}

// -----------------------------------------------------------------------------
void ChSprocket::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_gear);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChSprocket::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_gear);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
