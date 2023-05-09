// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for continuous band track shoes using rigid treads.
// Derived classes specify actual template definitions, using different models
// for the track web.
//
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Utility function to calculate the center of a circle of given radius which passes through two given points.
static ChVector2<> CalcCircleCenter(const ChVector2<>& A, const ChVector2<>& B, double r, double direction) {
    // midpoint
    ChVector2<> C = (A + B) / 2;
    // distance between A and B
    double l = (B - A).Length();
    // distance between C and O
    double d = std::sqrt(r * r - l * l / 4);
    // slope of line AB
    double mAB = (B.y() - A.y()) / (B.x() - A.x());
    // slope of line CO (perpendicular to AB)
    double mCO = -1 / mAB;
    // x offset from C
    double x_offset = d / std::sqrt(1 + mCO * mCO);
    // y offset from C
    double y_offset = mCO * x_offset;
    // circle center
    ChVector2<> O(C.x() + direction * x_offset, C.y() + direction * y_offset);

    ////GetLog() << "\n";
    ////GetLog() << "radius: " << r << "\n";
    ////GetLog() << A.x() << "  " << A.y() << "\n";
    ////GetLog() << B.x() << "  " << B.y() << "\n";
    ////GetLog() << O.x() << "  " << O.y() << "\n";
    ////GetLog() << "Check: " << (A - O).Length() - r << "  " << (B - O).Length() - r << "\n\n";

    return O;
}

// -----------------------------------------------------------------------------
ChTrackShoeBand::ChTrackShoeBand(const std::string& name) : ChTrackShoe(name) {}

void ChTrackShoeBand::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                 const ChVector<>& location,
                                 const ChQuaternion<>& rotation) {
    ChTrackShoe::Initialize(chassis, location, rotation);

    // Cache the postive (+x) tooth arc position and arc starting and ending angles
    ChVector2<> tooth_base_p(GetToothBaseLength() / 2, GetWebThickness() / 2);
    ChVector2<> tooth_tip_p(GetToothTipLength() / 2, GetToothHeight() + GetWebThickness() / 2);
    m_center_p = CalcCircleCenter(tooth_base_p, tooth_tip_p, GetToothArcRadius(), -1);
    m_center_p_arc_start = std::atan2(tooth_base_p.y() - m_center_p.y(), tooth_base_p.x() - m_center_p.x());
    m_center_p_arc_start = m_center_p_arc_start < 0 ? m_center_p_arc_start + CH_C_2PI : m_center_p_arc_start;
    m_center_p_arc_end = std::atan2(tooth_tip_p.y() - m_center_p.y(), tooth_tip_p.x() - m_center_p.x());
    m_center_p_arc_end = m_center_p_arc_end < 0 ? m_center_p_arc_end + CH_C_2PI : m_center_p_arc_end;
    if (m_center_p_arc_start > m_center_p_arc_end) {
        double temp = m_center_p_arc_start;
        m_center_p_arc_start = m_center_p_arc_end;
        m_center_p_arc_end = temp;
    }

    // Cache the negative (-x) tooth arc position and arc starting and ending angles
    ChVector2<> tooth_base_m(-GetToothBaseLength() / 2, GetWebThickness() / 2);
    ChVector2<> tooth_tip_m(-GetToothTipLength() / 2, GetToothHeight() + GetWebThickness() / 2);
    m_center_m = CalcCircleCenter(tooth_base_m, tooth_tip_m, GetToothArcRadius(), +1);
    m_center_m_arc_start = std::atan2(tooth_base_m.y() - m_center_m.y(), tooth_base_m.x() - m_center_m.x());
    m_center_m_arc_start = m_center_m_arc_start < 0 ? m_center_m_arc_start + CH_C_2PI : m_center_m_arc_start;
    m_center_m_arc_end = std::atan2(tooth_tip_m.y() - m_center_m.y(), tooth_tip_m.x() - m_center_m.x());
    m_center_m_arc_end = m_center_m_arc_end < 0 ? m_center_m_arc_end + CH_C_2PI : m_center_m_arc_end;
    if (m_center_m_arc_start > m_center_m_arc_end) {
        double temp = m_center_m_arc_start;
        m_center_m_arc_start = m_center_m_arc_end;
        m_center_m_arc_end = temp;
    }

    // Express the tread body location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;

    // Create the tread body
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_tread");
    m_shoe->SetIdentifier(BodyID::SHOE_BODY);
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetTreadMass());
    m_shoe->SetInertiaXX(GetTreadInertia());
    m_shoe->SetCollide(true);
    chassis->GetSystem()->AddBody(m_shoe);

    // Add contact geometry for the tread body and create material for the teeth (sprocket contact)
    auto contact_method = chassis->GetSystem()->GetContactMethod();
    AddShoeContact(contact_method);
    m_tooth_material = m_tooth_matinfo.CreateMaterial(contact_method);
}

double ChTrackShoeBand::GetPitch() const {
    return GetToothBaseLength() + GetWebLength();
}

ChVector<> ChTrackShoeBand::GetLateralContactPoint() const {
    return ChVector<>(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + GetGuideBoxDimensions().z() / 2);
}

// -----------------------------------------------------------------------------
void ChTrackShoeBand::AddShoeContact(ChContactMethod contact_method) {
    // Create contact materials
    auto pad_material = m_pad_matinfo.CreateMaterial(contact_method);
    auto body_material = m_body_matinfo.CreateMaterial(contact_method);
    auto guide_material = m_guide_matinfo.CreateMaterial(contact_method);

    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    // Guide pin
    ChVector<> g_dims = GetGuideBoxDimensions();
    ChVector<> g_loc(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + g_dims.z() / 2);
    m_shoe->GetCollisionModel()->AddBox(guide_material, g_dims.x(), g_dims.y(), g_dims.z(), g_loc);

    // Main box
    ChVector<> b_dims(GetToothBaseLength(), GetBeltWidth(), GetWebThickness());
    ChVector<> b_loc(0, 0, 0);
    m_shoe->GetCollisionModel()->AddBox(body_material, b_dims.x(), b_dims.y(), b_dims.z(), b_loc);

    // Pad box
    ChVector<> t_dims(GetTreadLength(), GetBeltWidth(), GetTreadThickness());
    ChVector<> t_loc(0, 0, (-GetWebThickness() - GetTreadThickness()) / 2);
    m_shoe->GetCollisionModel()->AddBox(pad_material, t_dims.x(), t_dims.y(), t_dims.z(), t_loc);

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
ChColor ChTrackShoeBand::GetColor(size_t index) {
    if (index == 0)
        return ChColor(0.7f, 0.4f, 0.4f);
    else if (index % 2 == 0)
        return ChColor(0.4f, 0.7f, 0.4f);
    else
        return ChColor(0.4f, 0.4f, 0.7f);
}

void ChTrackShoeBand::AddShoeVisualization() {
    // Guide pin
    ChVector<> g_loc(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + GetGuideBoxDimensions().z() / 2);
    auto box_pin = chrono_types::make_shared<ChBoxShape>(GetGuideBoxDimensions());
    m_shoe->AddVisualShape(box_pin, ChFrame<>(g_loc));

    // Main box
    ChVector<> b_loc(0, 0, 0);
    auto box_main = chrono_types::make_shared<ChBoxShape>(GetToothBaseLength(), GetBeltWidth(), GetWebThickness());
    m_shoe->AddVisualShape(box_main, ChFrame<>(b_loc));

    // Pad box
    ChVector<> t_loc(0, 0, (-GetWebThickness() - GetTreadThickness()) / 2);
    auto box_tread = chrono_types::make_shared<ChBoxShape>(GetTreadLength(), GetBeltWidth(), GetTreadThickness());
    m_shoe->AddVisualShape(box_tread, ChFrame<>(t_loc));

    // Connection to first web segment
    double radius = GetWebThickness() / 4;
    ChVehicleGeometry::AddVisualizationCylinder(
        m_shoe,                                                                     //
        ChVector<>(GetToothBaseLength() / 2, -GetBeltWidth() / 2 - 2 * radius, 0),  //
        ChVector<>(GetToothBaseLength() / 2, +GetBeltWidth() / 2 + 2 * radius, 0),  //
        radius);

    // Create tooth meshes
    m_shoe->AddVisualShape(ToothMesh(GetBeltWidth() / 2 - GetToothWidth() / 2));
    m_shoe->AddVisualShape(ToothMesh(-GetBeltWidth() / 2 + GetToothWidth() / 2));
}

// -----------------------------------------------------------------------------
// Utilties for writing/exporting the tooth visualization mesh
// -----------------------------------------------------------------------------
void ChTrackShoeBand::WriteTreadVisualizationMesh(const std::string& out_dir) {
    auto mesh_shape1 = ToothMesh(GetBeltWidth() / 2 - GetToothWidth() / 2);
    auto mesh_shape2 = ToothMesh(-GetBeltWidth() / 2 + GetToothWidth() / 2);
    std::vector<geometry::ChTriangleMeshConnected> meshes = {*mesh_shape1->GetMesh(), *mesh_shape2->GetMesh()};
    std::string filename = out_dir + "/" + GetTreadVisualizationMeshName() + ".obj";
    geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
}

void ChTrackShoeBand::ExportTreadVisualizationMeshPovray(const std::string& out_dir) {
    auto mesh_shape1 = ToothMesh(GetBeltWidth() / 2 - GetToothWidth() / 2);
    auto mesh_shape2 = ToothMesh(-GetBeltWidth() / 2 + GetToothWidth() / 2);
    std::vector<geometry::ChTriangleMeshConnected> meshes = {*mesh_shape1->GetMesh(), *mesh_shape2->GetMesh()};
    auto trimesh = geometry::ChTriangleMeshConnected::Merge(meshes);
    utils::WriteMeshPovray(trimesh, GetTreadVisualizationMeshName(), out_dir, ChColor(1, 1, 1));
}

// -----------------------------------------------------------------------------
// Utilities for creating tooth visualization mesh
// -----------------------------------------------------------------------------
int ChTrackShoeBand::ProfilePoints(std::vector<ChVector2<>>& points, std::vector<ChVector2<>>& normals) {
    int np = 4;
    double step = 1.0 / (np - 1);

    // Start from point on left tooth base (Am).
    ChVector2<> Am(-GetToothBaseLength() / 2, GetWebThickness() / 2);
    ChVector2<> Bm(-GetToothTipLength() / 2, GetToothHeight() + GetWebThickness() / 2);
    for (int i = 0; i < np; i++) {
        ChVector2<> pc(Am.x() + (i * step) * (Bm.x() - Am.x()), Am.y() + (i * step) * (Bm.y() - Am.y()));
        ChVector2<> nrm = (pc - m_center_m).GetNormalized();
        ChVector2<> pt = m_center_m + nrm * GetToothArcRadius();
        points.push_back(pt);
        normals.push_back(nrm);
    }

    // Mid-point on tooth tip.
    points.push_back(ChVector2<>(0, GetToothHeight() + GetWebThickness() / 2));
    normals.push_back(ChVector2<>(0, 1));

    // Continue from point on right tooth tip (Bp).
    ChVector2<> Ap(GetToothBaseLength() / 2, GetWebThickness() / 2);
    ChVector2<> Bp(GetToothTipLength() / 2, GetToothHeight() + GetWebThickness() / 2);
    for (int i = 0; i < np; i++) {
        ChVector2<> pc(Bp.x() + (i * step) * (Ap.x() - Bp.x()), Bp.y() + (i * step) * (Ap.y() - Bp.y()));
        ChVector2<> nrm = (pc - m_center_p).GetNormalized();
        ChVector2<> pt = m_center_p + nrm * GetToothArcRadius();
        points.push_back(pt);
        normals.push_back(nrm);
    }

    ////GetLog() << "\n\n";
    ////for (auto p : points)
    ////    GetLog() << p.x() << "  " << p.y() << "\n";

    return (int)points.size();
}

std::shared_ptr<ChTriangleMeshShape> ChTrackShoeBand::ToothMesh(double y) {
    // Obtain profile points.
    std::vector<ChVector2<>> points2;
    std::vector<ChVector2<>> normals2;
    int np = ProfilePoints(points2, normals2);

    // Create the triangular mesh.
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();

    // Number of vertices:
    //   - 1 for the middle of the tooth base on +y side
    //   - np for the +y tooth side
    //   - 1 for the middle of the tooth base on -y side
    //   - np for the -y tooth side
    size_t num_vertices = 2 * (np + 1);
    vertices.resize(num_vertices);

    // Number of normals:
    //   - 1 for the +y face
    //   - 1 for the -y face
    //   - np for the tooth surface
    size_t num_normals = 2 + np;
    normals.resize(num_normals);

    // Number of faces:
    //    - np-1 for the +y side
    //    - np-1 for the -y side
    //    - 2 * (np-1) for the tooth surface
    size_t num_faces = 4 * (np - 1);
    idx_vertices.resize(num_faces);
    idx_normals.resize(num_faces);

    // Load vertices.
    double yp = y + GetToothWidth() / 2;
    double ym = y - GetToothWidth() / 2;

    size_t iv = 0;
    vertices[iv++] = ChVector<>(0, yp, GetWebThickness() / 2);
    for (size_t i = 0; i < np; i++)
        vertices[iv++] = ChVector<>(points2[i].x(), yp, points2[i].y());
    vertices[iv++] = ChVector<>(0, ym, GetWebThickness() / 2);
    for (size_t i = 0; i < np; i++)
        vertices[iv++] = ChVector<>(points2[i].x(), ym, points2[i].y());

    // Load normals.
    size_t in = 0;
    normals[in++] = ChVector<>(0, +1, 0);
    normals[in++] = ChVector<>(0, -1, 0);
    for (size_t i = 0; i < np; i++)
        normals[in++] = ChVector<>(normals2[i].x(), 0, normals2[i].y());

    // Load triangles on +y side.
    size_t it = 0;
    for (int i = 0; i < np - 1; i++) {
        idx_vertices[it] = ChVector<int>(0, i + 1, i + 2);
        idx_normals[it] = ChVector<int>(0, 0, 0);
        it++;
    }

    // Load triangles on -y side.
    for (int i = 0; i < np - 1; i++) {
        idx_vertices[it] = ChVector<int>(0, i + 1, i + 2) + (np + 1);
        idx_normals[it] = ChVector<int>(1, 1, 1);
        it++;
    }

    // Load triangles on tooth surface.
    for (int i = 0; i < np - 1; i++) {
        idx_vertices[it] = ChVector<int>(i + 1, i + 1 + (np + 1), i + 2 + (np + 1));
        idx_normals[it] = ChVector<int>(i + 2, i + 2, i + 3);
        it++;
        idx_vertices[it] = ChVector<int>(i + 1, i + 2 + (np + 1), i + 2);
        idx_normals[it] = ChVector<int>(i + 2, i + 3, i + 3);
        it++;
    }

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(GetTreadVisualizationMeshName());
    return trimesh_shape;
}

}  // end namespace vehicle
}  // end namespace chrono
