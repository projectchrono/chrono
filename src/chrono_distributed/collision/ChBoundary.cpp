// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <cmath>

#include "chrono_distributed/collision/ChBoundary.h"

namespace chrono {

ChBoundary::ChBoundary(std::shared_ptr<ChBody> body, std::shared_ptr<ChMaterialSurfaceSMC> material)
    : m_body(body), m_material(material), m_crt_count(0) {
    assert(body->GetSystem());
    assert(body->GetBodyFixed());
    std::shared_ptr<ChBoundary> shared_this(this, [](ChBoundary*) {});
    body->GetSystem()->RegisterCustomCollisionCallback(shared_this);
}

void ChBoundary::AddPlane(const ChFrame<>& frame, const ChVector2<>& lengths) {
    ChFrame<> frame_abs;
    m_body->ChFrame<>::TransformLocalToParent(frame, frame_abs);
    m_planes.push_back(Plane(frame, frame_abs, lengths));
}

void ChBoundary::Update() {
    for (auto& plane : m_planes) {
        ChFrame<> frame_abs;
        m_body->ChFrame<>::TransformLocalToParent(plane.m_frame_loc, frame_abs);
        plane.m_frame = frame_abs;
        plane.m_normal = frame_abs.GetA().Get_A_Zaxis();
    }
}

void ChBoundary::UpdatePlane(size_t id, const ChFrame<>& frame) {
    assert(id >= 0 && id < m_planes.size());
    ChFrame<> frame_abs;
    m_body->ChFrame<>::TransformLocalToParent(frame, frame_abs);
    m_planes[id].m_frame_loc = frame;
    m_planes[id].m_frame = frame_abs;
    m_planes[id].m_normal = frame_abs.GetA().Get_A_Zaxis();

    if (m_planes[id].m_vis_box) {
        ChVector<> normal = m_planes[id].m_frame_loc.GetA().Get_A_Zaxis();
    }
}

void ChBoundary::UpdatePlane(size_t id, const ChVector2<>& lengths) {
    assert(id >= 0 && id < m_planes.size());
    m_planes[id].m_hlen = lengths * 0.5;

    if (m_planes[id].m_vis_box) {
        double hthick = m_planes[id].m_vis_box->GetGeometry().hlen.z();
        ChVector<> hlen(m_planes[id].m_hlen.x(), m_planes[id].m_hlen.y(), hthick);
        m_planes[id].m_vis_box->GetGeometry().hlen = hlen;
    }
}

void ChBoundary::AddVisualization(size_t id, double thickness) {
    double hthick = thickness / 2;
    ChVector<> hlen(m_planes[id].m_hlen.x(), m_planes[id].m_hlen.y(), hthick);
    ChVector<> normal = m_planes[id].m_frame_loc.GetA().Get_A_Zaxis();
    auto box = chrono_types::make_shared<ChBoxShape>(hlen * 2);
    m_body->AddVisualShape(
        box, ChFrame<>(m_planes[id].m_frame_loc.GetPos() - normal * hthick, m_planes[id].m_frame_loc.GetRot()));
    m_planes[id].m_vis_box = box;
}

void ChBoundary::AddVisualization(double thickness) {
    for (size_t id = 0; id < m_planes.size(); id++)
        AddVisualization(id, thickness);
}

ChBoundary::Plane::Plane(const ChFrame<>& frame_loc, const ChFrame<>& frame, const ChVector2<>& lengths)
    : m_frame_loc(frame_loc), m_frame(frame), m_hlen(lengths * 0.5), m_normal(frame.GetA().Get_A_Zaxis()) {}

void ChBoundary::OnCustomCollision(ChSystem* system) {
    auto sys = static_cast<ChSystemMulticore*>(system);

    m_crt_count = 0;

    // Loop over all bodies in the system
    //// TODO: maybe better to loop over all collision shapes in the system?
    for (auto body : *sys->data_manager->body_list) {
        if (body.get() == m_body.get())
            continue;
        if (!sys->data_manager->host_data.active_rigid[body->GetId()])
            continue;

        auto model = std::static_pointer_cast<collision::ChCollisionModelChrono>(body->GetCollisionModel());

        //// TODO: Broadphase rejection based on model AABB?

        for (auto s : model->GetShapes()) {
            auto shape = std::static_pointer_cast<collision::ChCollisionShapeChrono>(s);
            switch (shape->GetType()) {
                case collision::ChCollisionShape::Type::SPHERE : {
                    ChVector<> center_loc = ChVector<>(shape->A.x, shape->A.y, shape->A.z);
                    ChVector<> center_abs = body->TransformPointLocalToParent(center_loc);
                    CheckSphere(model.get(), shape->GetMaterial(), center_abs, shape->B.x);
                    break;
                }
                case collision::ChCollisionShape::Type::BOX: {
                    ChFrame<> frame_loc(ChVector<>(shape->A.x, shape->A.y, shape->A.z),
                                        ChQuaternion<>(shape->R.w, shape->R.x, shape->R.y, shape->R.z));
                    ChFrame<> frame_abs;
                    body->ChFrame<>::TransformLocalToParent(frame_loc, frame_abs);
                    CheckBox(model.get(), shape->GetMaterial(), frame_abs, ChVector<>(shape->B.x, shape->B.y, shape->B.z));
                    break;
                }
                default:
                    // not supported
                    break;
            }
        }
    }

    ////if (m_crt_count > 0)
    ////    std::cout << "Added " << m_crt_count << " collisions" << std::endl;
}

void ChBoundary::CheckSphere(collision::ChCollisionModel* model,
                             std::shared_ptr<ChMaterialSurface> material,
                             const ChVector<>& center,
                             double radius) {
    for (auto& plane : m_planes) {
        CheckSpherePlane(model, material, center, radius, plane);
    }
}

void ChBoundary::CheckBox(collision::ChCollisionModel* model,
                          std::shared_ptr<ChMaterialSurface> material,
                          const ChFrame<>& frame,
                          const ChVector<>& size) {
    for (auto& plane : m_planes) {
        CheckBoxPlane(model, material, frame, size, plane);
    }
}

void ChBoundary::CheckSpherePlane(collision::ChCollisionModel* model,
                                  std::shared_ptr<ChMaterialSurface> material,
                                  const ChVector<>& center,
                                  double radius,
                                  const Plane& plane) {
    // Express sphere center in plane coordinate system
    auto P = plane.m_frame.TransformPointParentToLocal(center);

    // Height of sphere above plane.
    double depth = P.z() - radius;

    if (depth >= 0)
        return;

    if (std::abs(P.x()) > plane.m_hlen.x() || std::abs(P.y()) > plane.m_hlen.y())
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_body->GetCollisionModel().get();
    contact.modelB = model;
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = plane.m_normal;
    contact.vpA = center - P.z() * plane.m_normal;
    contact.vpB = center - radius * plane.m_normal;
    contact.distance = depth;
    contact.eff_radius = radius;

    auto sys = static_cast<ChSystemMulticore*>(m_body->GetSystem());
    sys->GetContactContainer()->AddContact(contact, m_material, material);

    m_crt_count++;
}

void ChBoundary::CheckBoxPlane(collision::ChCollisionModel* model,
                               std::shared_ptr<ChMaterialSurface> material,
                               const ChFrame<>& frame,
                               const ChVector<>& size,
                               const Plane& plane) {
    //// TODO: implement missing utilities & verify algorithm.

    /*
    // Express plane frame in box coordinate system
    ChFrame<> X;
    frame.TransformParentToLocal(plane.m_frame, X);

    // Plane normal and tangent vectors (expressed in box frame)
    ChVector<> N = X.GetA().Get_A_Zaxis();
    ChVector<> U = X.GetA().Get_A_Xaxis();
    ChVector<> V = X.GetA().Get_A_Yaxis();

    // Working in the frame of this box, find the distance from the plane to the closest corner of this box.
    // If the distance is greater than zero, there is no contact.
    ChVector<> near_loc;  //// = getClosestCorner(N);
    double h = Vdot(near_loc - X.GetPos(), N);

    if (h >= 0)
        return;

    // Find closest box feature in the direction of the plane normal.
    // This can be a corner, an edge, or a face, resulting in 1, 2, or 4 contacts, respectively.
    uint code;  //// = getClosestFeature(N);
    uint num_axes = (code & 1) + ((code >> 1) & 1) + ((code >> 2) & 1);
    uint num_corners = 1 << (3 - num_axes);
    ChVector<> corners[4];

    if (num_axes == 1)
        getFaceCorners(near_loc, code, corners);
    else if (num_axes == 2)
        getEdgeCorners(near_loc, code, corners);
    else
        corners[0] = near_loc;

    // For each corner to be considered, see if there actually is contact.
    for (uint i = 0; i < num_corners; i++) {
        ChVector<> vec = corners[i] - X.GetPos();
        double depth = Vdot(vec, N);

        if (depth < 0 && std::abs(Vdot(vec, U)) < plane.m_hlen.x() && std::abs(Vdot(vec, V)) < plane.m_hlen.y()) {
            collision::ChCollisionInfo contact;
            contact.modelA = m_body->GetCollisionModel().get();
            contact.modelB = model.get();
            contact.shapeA = nullptr;
            contact.shapeB = nullptr;
            contact.vN = plane.m_normal;
            contact.vpB = frame * corners[i];
            contact.vpA = contact.vpB - depth * plane.m_normal;
            contact.distance = depth;
        }
    }
    */
}

}  // end namespace chrono