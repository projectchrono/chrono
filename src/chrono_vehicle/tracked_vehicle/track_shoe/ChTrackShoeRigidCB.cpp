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
// Base class for a continuous band rigid-link track shoe (template definition).
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadContainer.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Utility function to calculate the center of a circle of given radius which
// passes through two given points.
ChVector2<> CalcCircleCenter(const ChVector2<>& A, const ChVector2<>& B, double r, double direction) {
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

    ////std::cout << std::endl;
    ////std::cout << "radius: " << r << std::endl;
    ////std::cout << A.x() << "  " << A.y() << std::endl;
    ////std::cout << B.x() << "  " << B.y() << std::endl;
    ////std::cout << O.x() << "  " << O.y() << std::endl;
    ////std::cout << "Check: " << (A - O).Length() - r << "  " << (B - O).Length() - r << std::endl;
    ////std::cout << std::endl;

    return O;
}

ChTrackShoeRigidCB::ChTrackShoeRigidCB(const std::string& name) : ChTrackShoe(name) {}

void ChTrackShoeRigidCB::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                    const ChVector<>& location,
                                    const ChQuaternion<>& rotation) {
    // Cache values calculated from template parameters.
    m_seg_length = GetWebLength() / GetNumWebSegments();
    m_seg_mass = GetWebMass() / GetNumWebSegments();
    m_seg_inertia = GetWebInertia();  //// TODO - properly distribute web inertia

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
    ChVector<> xdir = rot.GetXaxis();

    // Create the tread body
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_tread");
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetTreadMass());
    m_shoe->SetInertiaXX(GetTreadInertia());
    chassis->GetSystem()->AddBody(m_shoe);

    // Add contact geometry.
    m_shoe->SetCollide(true);

    switch (m_shoe->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_shoe->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_shoe->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_shoe->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_shoe->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_shoe->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_shoe->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_shoe->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_shoe->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_shoe->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_shoe->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    AddShoeContact();

    // Create the required number of web segment bodies
    ChVector<> seg_loc = loc + (0.5 * GetToothBaseLength()) * xdir;
    for (int is = 0; is < GetNumWebSegments(); is++) {
        m_web_segments.push_back(std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody()));
        m_web_segments[is]->SetNameString(m_name + "_web_" + std::to_string(is));
        m_web_segments[is]->SetPos(seg_loc + ((2 * is + 1) * m_seg_length / 2) * xdir);
        m_web_segments[is]->SetRot(rot);
        m_web_segments[is]->SetMass(m_seg_mass);
        m_web_segments[is]->SetInertiaXX(m_seg_inertia);
        chassis->GetSystem()->AddBody(m_web_segments[is]);

        // Add contact geometry.
        m_web_segments[is]->SetCollide(true);

        switch (m_web_segments[is]->GetContactMethod()) {
            case ChMaterialSurface::NSC:
                m_web_segments[is]->GetMaterialSurfaceNSC()->SetFriction(m_friction);
                m_web_segments[is]->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
                break;
            case ChMaterialSurface::SMC:
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetFriction(m_friction);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetKn(m_kn);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetGn(m_gn);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetKt(m_kt);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetGt(m_gt);
                break;
        }

        AddWebContact(m_web_segments[is]);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                    const std::vector<ChCoordsys<>>& component_pos) {
    // Check the number of provided locations and orientations.
    assert(component_pos.size() == GetNumWebSegments() + 1);

    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis->TransformPointLocalToParent(component_pos[0].pos));
    m_shoe->SetRot(chassis->GetRot() * component_pos[0].rot);

    for (int is = 0; is < GetNumWebSegments(); is++) {
        m_web_segments[is]->SetPos(chassis->TransformPointLocalToParent(component_pos[is + 1].pos));
        m_web_segments[is]->SetRot(chassis->GetRot() * component_pos[is + 1].rot);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeRigidCB::GetMass() const {
    return GetTreadMass() + GetWebMass();
}

double ChTrackShoeRigidCB::GetPitch() const {
    return GetToothBaseLength() + GetWebLength();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    // Guide pin
    ChVector<> g_hdims = GetGuideBoxDimensions() / 2;
    ChVector<> g_loc(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + g_hdims.z());
    m_shoe->GetCollisionModel()->AddBox(g_hdims.x(), g_hdims.y(), g_hdims.z(), g_loc);

    // Main box
    ChVector<> b_hdims(GetToothBaseLength() / 2, GetBeltWidth() / 2, GetWebThickness() / 2);
    ChVector<> b_loc(0, 0, 0);
    m_shoe->GetCollisionModel()->AddBox(b_hdims.x(), b_hdims.y(), b_hdims.z(), b_loc);

    // Tread box
    ChVector<> t_hdims(GetTreadLength() / 2, GetBeltWidth() / 2, GetTreadThickness() / 2);
    ChVector<> t_loc(0, 0, (-GetWebThickness() - GetTreadThickness()) / 2);
    m_shoe->GetCollisionModel()->AddBox(t_hdims.x(), t_hdims.y(), t_hdims.z(), t_loc);

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddWebContact(std::shared_ptr<ChBody> segment) {
    segment->GetCollisionModel()->ClearModel();

    segment->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    segment->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    segment->GetCollisionModel()->AddBox(m_seg_length / 2, GetBeltWidth() / 2, GetWebThickness() / 2);

    segment->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
    for (auto segment : m_web_segments)
        AddWebVisualization(segment);
}

void ChTrackShoeRigidCB::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
    for (auto segment : m_web_segments) {
        segment->GetAssets().clear();
    }
}

ChColor GetColor(size_t index) {
    if (index == 0)
        return ChColor(0.7f, 0.4f, 0.4f);
    else if (index % 2 == 0)
        return ChColor(0.4f, 0.7f, 0.4f);
    else
        return ChColor(0.4f, 0.4f, 0.7f);
}

void ChTrackShoeRigidCB::AddShoeVisualization() {
    m_shoe->AddAsset(std::make_shared<ChColorAsset>(GetColor(m_index)));

    // Guide pin
    ChVector<> g_hdims = GetGuideBoxDimensions() / 2;
    ChVector<> g_loc(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + g_hdims.z());
    auto box_pin = std::make_shared<ChBoxShape>();
    box_pin->GetBoxGeometry().Size = g_hdims;
    box_pin->GetBoxGeometry().Pos = g_loc;
    m_shoe->AddAsset(box_pin);

    // Main box
    ChVector<> b_hdims(GetToothBaseLength() / 2, GetBeltWidth() / 2, GetWebThickness() / 2);
    ChVector<> b_loc(0, 0, 0);
    auto box_main = std::make_shared<ChBoxShape>();
    box_main->GetBoxGeometry().Size = b_hdims;
    box_main->GetBoxGeometry().Pos = b_loc;
    m_shoe->AddAsset(box_main);

    // Tread box
    ChVector<> t_hdims(GetTreadLength() / 2, GetBeltWidth() / 2, GetTreadThickness() / 2);
    ChVector<> t_loc(0, 0, (-GetWebThickness() - GetTreadThickness()) / 2);
    auto box_tread = std::make_shared<ChBoxShape>();
    box_tread->GetBoxGeometry().Size = t_hdims;
    box_tread->GetBoxGeometry().Pos = t_loc;
    m_shoe->AddAsset(box_tread);

    // Connection to first web segment
    double radius = GetWebThickness() / 4;
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(GetToothBaseLength() / 2, -GetBeltWidth() / 2 - 2 * radius, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(GetToothBaseLength() / 2, +GetBeltWidth() / 2 + 2 * radius, 0);
    m_shoe->AddAsset(cyl);

    // Create tooth meshes
    m_shoe->AddAsset(ToothMesh(GetBeltWidth() / 2 - GetToothWidth() / 2));
    m_shoe->AddAsset(ToothMesh(-GetBeltWidth() / 2 + GetToothWidth() / 2));
}

void ChTrackShoeRigidCB::AddWebVisualization(std::shared_ptr<ChBody> segment) {
    segment->AddAsset(std::make_shared<ChColorAsset>(GetColor(m_index)));

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(m_seg_length, GetBeltWidth(), GetWebThickness()));
    segment->AddAsset(box);

    auto cyl = std::make_shared<ChCylinderShape>();
    double radius = GetWebThickness() / 4;
    cyl->GetCylinderGeometry().rad = radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(m_seg_length / 2, -GetBeltWidth() / 2 - 2 * radius, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(m_seg_length / 2, +GetBeltWidth() / 2 + 2 * radius, 0);
    segment->AddAsset(cyl);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChSystem* system = m_shoe->GetSystem();
    ChVector<> loc;
    ChQuaternion<> rot;

#if FALSE  // Use busing elements to connect the belt segments, otherwise use revolute joints
    // Bushings are inherited from ChLoad, so they require a 'load container'

    auto my_loadcontainer = std::make_shared<ChLoadContainer>();
    system->Add(my_loadcontainer);

    ChMatrixNM<double, 6, 6> K_matrix;
    ChMatrixNM<double, 6, 6> R_matrix;

    //Sample Stiffness and Damping matrix values for testing purposes
    for (unsigned int ii = 0; ii < 3; ii++) {
        K_matrix(ii, ii) = 20000.0;
        R_matrix(ii, ii) = 0.05 * K_matrix(ii, ii);
    }
    for (unsigned int ii = 3; ii < 6; ii++) {
        K_matrix(ii, ii) = 1000.0;
        R_matrix(ii, ii) = 0.05 * K_matrix(ii, ii);
    }
    K_matrix(4, 4) = 0;
    R_matrix(4, 4) = 0;

    // Connect tread body to the first web segment.
    loc = m_shoe->TransformPointLocalToParent(ChVector<>(GetToothBaseLength() / 2, 0, 0));
    rot = m_shoe->GetRot();
    auto my_loadbushingg0 = std::make_shared<ChLoadBodyBodyBushingGeneric>(
        m_shoe,               // body A
        m_web_segments[0],    // body B
        ChFrame<>(loc, rot),  // initial frame of bushing in abs space
        K_matrix,             // the 6x6 (translation+rotation) K matrix in local frame
        R_matrix              // the 6x6 (translation+rotation) R matrix in local frame
        );
    my_loadbushingg0->SetApplicationFrameA(ChFrame<>(ChVector<>(GetToothBaseLength() / 2, 0, 0)));
    my_loadbushingg0->SetApplicationFrameB(ChFrame<>(ChVector<>(-m_seg_length / 2, 0, 0)));
    my_loadcontainer->Add(my_loadbushingg0);

    // Connect the web segments to each other.
    for (size_t is = 0; is < GetNumWebSegments() - 1; is++) {
        loc = m_web_segments[is]->TransformPointLocalToParent(ChVector<>(m_seg_length / 2, 0, 0));
        rot = m_web_segments[is]->GetRot();
        auto my_loadbushingg = std::make_shared<ChLoadBodyBodyBushingGeneric>(
            m_web_segments[is],      // body A
            m_web_segments[is + 1],  // body B
            ChFrame<>(loc, rot),     // initial frame of bushing in abs space
            K_matrix,                // the 6x6 (translation+rotation) K matrix in local frame
            R_matrix                 // the 6x6 (translation+rotation) R matrix in local frame
            );
        my_loadbushingg->SetApplicationFrameA(ChFrame<>(ChVector<>(m_seg_length / 2, 0, 0)));
        my_loadbushingg->SetApplicationFrameB(ChFrame<>(ChVector<>(-m_seg_length / 2, 0, 0)));
        my_loadcontainer->Add(my_loadbushingg);
    }

    // Connect the last web segment to the tread body from the next track shoe.
    int is = GetNumWebSegments() - 1;
    loc = m_web_segments[is]->TransformPointLocalToParent(ChVector<>(m_seg_length / 2, 0, 0));
    rot = m_web_segments[is]->GetRot();
    auto my_loadbushingg1 = std::make_shared<ChLoadBodyBodyBushingGeneric>(
        m_web_segments[is],   // body A
        next->GetShoeBody(),  // body B
        ChFrame<>(loc, rot),  // initial frame of bushing in abs space
        K_matrix,             // the 6x6 (translation+rotation) K matrix in local frame
        R_matrix              // the 6x6 (translation+rotation) R matrix in local frame
        );
    my_loadbushingg1->SetApplicationFrameA(ChFrame<>(ChVector<>(m_seg_length / 2, 0, 0)));
    my_loadbushingg1->SetApplicationFrameB(ChFrame<>(ChVector<>(-GetToothBaseLength() / 2, 0, 0)));
    my_loadcontainer->Add(my_loadbushingg1);

#else
    // Connect tread body to the first web segment.
    loc = m_shoe->TransformPointLocalToParent(ChVector<>(GetToothBaseLength() / 2, 0, 0));
    rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);
    auto revolute0 = std::make_shared<ChLinkLockRevolute>();
    system->AddLink(revolute0);
    revolute0->SetNameString(m_name + "_revolute_0");
    revolute0->Initialize(m_shoe, m_web_segments[0], ChCoordsys<>(loc, rot));

    // Connect the web segments to each other.
    for (size_t is = 0; is < GetNumWebSegments() - 1; is++) {
        loc = m_web_segments[is]->TransformPointLocalToParent(ChVector<>(m_seg_length / 2, 0, 0));
        rot = m_web_segments[is]->GetRot() * Q_from_AngX(CH_C_PI_2);
        auto revolute = std::make_shared<ChLinkLockRevolute>();
        system->AddLink(revolute);
        revolute->SetNameString(m_name + "_revolute_" + std::to_string(is + 1));
        revolute->Initialize(m_web_segments[is], m_web_segments[is + 1], ChCoordsys<>(loc, rot));
    }

    // Connect the last web segment to the tread body from the next track shoe.
    int is = GetNumWebSegments() - 1;
    loc = m_web_segments[is]->TransformPointLocalToParent(ChVector<>(m_seg_length / 2, 0, 0));
    rot = m_web_segments[is]->GetRot() * Q_from_AngX(CH_C_PI_2);
    auto revolute1 = std::make_shared<ChLinkLockRevolute>();
    system->AddLink(revolute1);
    revolute1->SetNameString(m_name + "_revolute");
    revolute1->Initialize(m_web_segments[is], next->GetShoeBody(), ChCoordsys<>(loc, rot));

#endif
}

// -----------------------------------------------------------------------------
// Utilities for creating tooth mesh
// -----------------------------------------------------------------------------
size_t ChTrackShoeRigidCB::ProfilePoints(std::vector<ChVector2<>>& points, std::vector<ChVector2<>>& normals) {
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

    ////std::cout << std::endl << std::endl;
    ////for (auto p : points)
    ////    std::cout << p.x() << "  " << p.y() << std::endl;

    return points.size();
}

std::shared_ptr<ChTriangleMeshShape> ChTrackShoeRigidCB::ToothMesh(double y) {
    // Obtain profile points.
    std::vector<ChVector2<>> points2;
    std::vector<ChVector2<>> normals2;
    size_t np = ProfilePoints(points2, normals2);

    // Create the triangular mesh.
    geometry::ChTriangleMeshConnected trimesh;
    std::vector<ChVector<>>& vertices = trimesh.getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh.getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = trimesh.getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh.getIndicesNormals();

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
    for (size_t i = 0; i < np - 1; i++) {
        idx_vertices[it] = ChVector<int>(0, i + 1, i + 2);
        idx_normals[it] = ChVector<int>(0, 0, 0);
        it++;
    }

    // Load triangles on -y side.
    for (size_t i = 0; i < np - 1; i++) {
        idx_vertices[it] = ChVector<int>(0, i + 1, i + 2) + (np + 1);
        idx_normals[it] = ChVector<int>(1, 1, 1);
        it++;
    }

    // Load triangles on tooth surface.
    for (size_t i = 0; i < np - 1; i++) {
        idx_vertices[it] = ChVector<int>(i + 1, i + 1 + (np + 1), i + 2 + (np + 1));
        idx_normals[it] = ChVector<int>(i + 2, i + 2, i + 3);
        it++;
        idx_vertices[it] = ChVector<int>(i + 1, i + 2 + (np + 1), i + 2);
        idx_normals[it] = ChVector<int>(i + 2, i + 3, i + 3);
        it++;
    }

    auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);

    return trimesh_shape;
}

}  // end namespace vehicle
}  // end namespace chrono
