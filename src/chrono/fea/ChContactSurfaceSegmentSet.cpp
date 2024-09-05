// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <set>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChElementBeamEuler.h"

#include "chrono/fea/ChContactSurfaceSegmentSet.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

ChContactSegmentXYZ::ChContactSegmentXYZ() : m_owns_node({true, true}), m_container(nullptr) {}

ChContactSegmentXYZ::ChContactSegmentXYZ(const std::array<std::shared_ptr<ChNodeFEAxyz>, 2>& nodes,
                                         ChContactSurface* container)
    : m_nodes(nodes), m_owns_node({true, true}), m_container(container) {}

void ChContactSegmentXYZ::ContactableGetStateBlockPosLevel(ChState& x) {
    x.segment(0, 3) = m_nodes[0]->pos.eigen();
    x.segment(3, 3) = m_nodes[1]->pos.eigen();
}

void ChContactSegmentXYZ::ContactableGetStateBlockVelLevel(ChStateDelta& w) {
    w.segment(0, 3) = m_nodes[0]->pos_dt.eigen();
    w.segment(3, 3) = m_nodes[1]->pos_dt.eigen();
}

void ChContactSegmentXYZ::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    m_nodes[0]->NodeIntStateIncrement(0, x_new, x, 0, dw);
    m_nodes[1]->NodeIntStateIncrement(3, x_new, x, 3, dw);
}

ChVector3d ChContactSegmentXYZ::GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) {
    // Note: the segment nodes are relative the global frame and therefore the given point is already expressed in the
    // global frame
    double s2 = ComputeUfromP(loc_point);
    double s1 = 1 - s2;

    ChVector3d A1(state_x.segment(0, 3));
    ChVector3d A2(state_x.segment(3, 3));

    return s1 * A1 + s2 * A2;
}

ChVector3d ChContactSegmentXYZ::GetContactPointSpeed(const ChVector3d& loc_point,
                                                     const ChState& state_x,
                                                     const ChStateDelta& state_w) {
    // Note: the segment nodes are relative the global frame and therefore the given point is already expressed in the
    // global frame
    double s2 = ComputeUfromP(loc_point);
    double s1 = 1 - s2;

    ChVector3d A1_dt(state_w.segment(0, 3));
    ChVector3d A2_dt(state_w.segment(3, 3));

    return s1 * A1_dt + s2 * A2_dt;
}

ChVector3d ChContactSegmentXYZ::GetContactPointSpeed(const ChVector3d& abs_point) {
    double s2 = ComputeUfromP(abs_point);
    double s1 = 1 - s2;

    return s1 * m_nodes[0]->GetPosDt() + s2 * m_nodes[1]->GetPosDt();
}

void ChContactSegmentXYZ::ContactForceLoadResidual_F(const ChVector3d& F,
                                                     const ChVector3d& T,
                                                     const ChVector3d& abs_point,
                                                     ChVectorDynamic<>& R) {
    double s2 = ComputeUfromP(abs_point);
    double s1 = 1 - s2;

    R.segment(m_nodes[0]->NodeGetOffsetVelLevel(), 3) += F.eigen() * s1;
    R.segment(m_nodes[1]->NodeGetOffsetVelLevel(), 3) += F.eigen() * s2;
}

void ChContactSegmentXYZ::ContactComputeQ(const ChVector3d& F,
                                          const ChVector3d& T,
                                          const ChVector3d& point,
                                          const ChState& state_x,
                                          ChVectorDynamic<>& Q,
                                          int offset) {
    ChVector3d A1(state_x.segment(0, 3));
    ChVector3d A2(state_x.segment(3, 3));

    bool in_segment = false;
    double s2 = -1;
    /*double dist =*/utils::PointLineDistance(point, A1, A2, s2, in_segment);
    double s1 = 1 - s2;

    Q.segment(offset + 0, 3) = F.eigen() * s1;
    Q.segment(offset + 3, 3) = F.eigen() * s2;
}

void ChContactSegmentXYZ::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                                        ChMatrix33<>& contact_plane,
                                                        type_constraint_tuple& jacobian_tuple_N,
                                                        type_constraint_tuple& jacobian_tuple_U,
                                                        type_constraint_tuple& jacobian_tuple_V,
                                                        bool second) {
    double s2 = ComputeUfromP(abs_point);
    double s1 = 1 - s2;

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    jacobian_tuple_N.Get_Cq_1().segment(0, 3) = s1 * Jx1.row(0);
    jacobian_tuple_U.Get_Cq_1().segment(0, 3) = s1 * Jx1.row(1);
    jacobian_tuple_V.Get_Cq_1().segment(0, 3) = s1 * Jx1.row(2);

    jacobian_tuple_N.Get_Cq_2().segment(0, 3) = s2 * Jx1.row(0);
    jacobian_tuple_U.Get_Cq_2().segment(0, 3) = s2 * Jx1.row(1);
    jacobian_tuple_V.Get_Cq_2().segment(0, 3) = s2 * Jx1.row(2);
}

ChPhysicsItem* ChContactSegmentXYZ::GetPhysicsItem() {
    if (!m_container)
        return nullptr;

    return m_container->GetPhysicsItem();
}

double ChContactSegmentXYZ::ComputeUfromP(const ChVector3d& P) {
    bool in_segment = false;
    double u = -1;
    /*double dist =*/utils::PointLineDistance(P, m_nodes[0]->GetPos(), m_nodes[1]->GetPos(), u, in_segment);
    return u;
}

// -----------------------------------------------------------------------------

ChContactSegmentXYZRot::ChContactSegmentXYZRot() : m_owns_node({true, true}), m_container(nullptr) {}

ChContactSegmentXYZRot::ChContactSegmentXYZRot(const std::array<std::shared_ptr<ChNodeFEAxyzrot>, 2>& nodes,
                                               ChContactSurface* container)
    : m_nodes(nodes), m_owns_node({true, true}), m_container(container) {}

void ChContactSegmentXYZRot::ContactableGetStateBlockPosLevel(ChState& x) {
    x.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    x.segment(3, 4) = m_nodes[0]->GetRot().eigen();

    x.segment(7, 3) = m_nodes[1]->GetPos().eigen();
    x.segment(10, 4) = m_nodes[1]->GetRot().eigen();
}

void ChContactSegmentXYZRot::ContactableGetStateBlockVelLevel(ChStateDelta& w) {
    w.segment(0, 3) = m_nodes[0]->GetPosDt().eigen();
    w.segment(3, 3) = m_nodes[0]->GetAngVelLocal().eigen();

    w.segment(6, 3) = m_nodes[1]->GetPosDt().eigen();
    w.segment(9, 3) = m_nodes[1]->GetAngVelLocal().eigen();
}

void ChContactSegmentXYZRot::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    m_nodes[0]->NodeIntStateIncrement(0, x_new, x, 0, dw);
    m_nodes[1]->NodeIntStateIncrement(7, x_new, x, 6, dw);
}

ChVector3d ChContactSegmentXYZRot::GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) {
    // Note: the segment nodes are relative the global frame and therefore the given point is already expressed in the
    // global frame
    double s2 = ComputeUfromP(loc_point);
    double s1 = 1 - s2;

    ChVector3d A1(state_x.segment(0, 3));
    ChVector3d A2(state_x.segment(3, 3));

    return s1 * A1 + s2 * A2;
}

ChVector3d ChContactSegmentXYZRot::GetContactPointSpeed(const ChVector3d& loc_point,
                                                        const ChState& state_x,
                                                        const ChStateDelta& state_w) {
    // Note: the segment nodes are relative the global frame and therefore the given point is already expressed in the
    // global frame
    double s2 = ComputeUfromP(loc_point);
    double s1 = 1 - s2;

    ChVector3d A1_dt(state_w.segment(0, 3));
    ChVector3d A2_dt(state_w.segment(3, 3));

    return s1 * A1_dt + s2 * A2_dt;
}

ChVector3d ChContactSegmentXYZRot::GetContactPointSpeed(const ChVector3d& abs_point) {
    double s2 = ComputeUfromP(abs_point);
    double s1 = 1 - s2;

    return s1 * m_nodes[0]->GetPosDt() + s2 * m_nodes[1]->GetPosDt();
}

void ChContactSegmentXYZRot::ContactForceLoadResidual_F(const ChVector3d& F,
                                                        const ChVector3d& T,
                                                        const ChVector3d& abs_point,
                                                        ChVectorDynamic<>& R) {
    double s2 = ComputeUfromP(abs_point);
    double s1 = 1 - s2;

    R.segment(m_nodes[0]->NodeGetOffsetVelLevel(), 3) += F.eigen() * s1;
    R.segment(m_nodes[1]->NodeGetOffsetVelLevel(), 3) += F.eigen() * s2;

    if (!T.IsNull()) {
        R.segment(m_nodes[0]->NodeGetOffsetVelLevel() + 3, 3) +=
            (m_nodes[0]->TransformDirectionParentToLocal(T * s1)).eigen();
        R.segment(m_nodes[1]->NodeGetOffsetVelLevel() + 3, 3) +=
            (m_nodes[1]->TransformDirectionParentToLocal(T * s2)).eigen();
    }
}

void ChContactSegmentXYZRot::ContactComputeQ(const ChVector3d& F,
                                             const ChVector3d& T,
                                             const ChVector3d& point,
                                             const ChState& state_x,
                                             ChVectorDynamic<>& Q,
                                             int offset) {
    ChVector3d A1(state_x.segment(0, 3));
    ChVector3d A2(state_x.segment(3, 3));

    bool in_segment = false;
    double s2 = -1;
    /*double dist =*/utils::PointLineDistance(point, A1, A2, s2, in_segment);
    double s1 = 1 - s2;

    Q.segment(offset + 0, 3) = F.eigen() * s1;
    Q.segment(offset + 3, 3) = F.eigen() * s2;

    if (!T.IsNull()) {
        ChCoordsys<> C1(state_x.segment(0, 7));
        ChCoordsys<> C2(state_x.segment(7, 7));
        Q.segment(offset + 3, 3) = (C1.TransformDirectionParentToLocal(T * s1)).eigen();
        Q.segment(offset + 9, 3) = (C2.TransformDirectionParentToLocal(T * s2)).eigen();
    }
}

void ChContactSegmentXYZRot::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                                           ChMatrix33<>& contact_plane,
                                                           type_constraint_tuple& jacobian_tuple_N,
                                                           type_constraint_tuple& jacobian_tuple_U,
                                                           type_constraint_tuple& jacobian_tuple_V,
                                                           bool second) {
    double s2 = ComputeUfromP(abs_point);
    double s1 = 1 - s2;

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    jacobian_tuple_N.Get_Cq_1().segment(0, 3) = s1 * Jx1.row(0);
    jacobian_tuple_U.Get_Cq_1().segment(0, 3) = s1 * Jx1.row(1);
    jacobian_tuple_V.Get_Cq_1().segment(0, 3) = s1 * Jx1.row(2);

    jacobian_tuple_N.Get_Cq_2().segment(0, 3) = s2 * Jx1.row(0);
    jacobian_tuple_U.Get_Cq_2().segment(0, 3) = s2 * Jx1.row(1);
    jacobian_tuple_V.Get_Cq_2().segment(0, 3) = s2 * Jx1.row(2);
}

ChPhysicsItem* ChContactSegmentXYZRot::GetPhysicsItem() {
    if (!m_container)
        return nullptr;

    return m_container->GetPhysicsItem();
}

double ChContactSegmentXYZRot::ComputeUfromP(const ChVector3d& P) {
    bool in_segment = false;
    double u = -1;
    /*double dist =*/utils::PointLineDistance(P, m_nodes[0]->GetPos(), m_nodes[1]->GetPos(), u, in_segment);
    return u;
}

// -----------------------------------------------------------------------------

ChContactSurfaceSegmentSet::ChContactSurfaceSegmentSet(std::shared_ptr<ChContactMaterial> material, ChMesh* mesh)
    : ChContactSurface(material, mesh) {}

void ChContactSurfaceSegmentSet::AddSegment(std::shared_ptr<ChNodeFEAxyz> node1,  // segment node1
                                            std::shared_ptr<ChNodeFEAxyz> node2,  // segment node2
                                            bool owns_node1,                      // collision segment owns node1
                                            bool owns_node2,                      // collision segment owns node2
                                            double sphere_swept                   // thickness
) {
    assert(node1);
    assert(node2);

    auto segment = chrono_types::make_shared<fea::ChContactSegmentXYZ>();
    segment->SetNodes({{node1, node2}});
    segment->SetNodeOwnership({owns_node1, owns_node2});
    segment->SetContactSurface(this);

    auto segment_shape = chrono_types::make_shared<ChCollisionShapeSegment>(
        m_material, &node1->GetPos(), &node2->GetPos(), owns_node1, owns_node2, sphere_swept);
    segment->AddCollisionShape(segment_shape);

    m_segments.push_back(segment);
}

void ChContactSurfaceSegmentSet::AddSegment(std::shared_ptr<ChNodeFEAxyzrot> node1,  // segment node1
                                            std::shared_ptr<ChNodeFEAxyzrot> node2,  // segment node2
                                            bool owns_node1,                         // collision segment owns node1
                                            bool owns_node2,                         // collision segment owns node2
                                            double sphere_swept                      // thickness
) {
    assert(node1);
    assert(node2);

    auto segment = chrono_types::make_shared<fea::ChContactSegmentXYZRot>();
    segment->SetNodes({{node1, node2}});
    segment->SetNodeOwnership({owns_node1, owns_node2});
    segment->SetContactSurface(this);

    auto segment_shape = chrono_types::make_shared<ChCollisionShapeSegment>(
        m_material, &node1->GetPos(), &node2->GetPos(), owns_node1, owns_node2, sphere_swept);
    segment->AddCollisionShape(segment_shape);

    m_segments_rot.push_back(segment);
}

void ChContactSurfaceSegmentSet::AddAllSegments(double sphere_swept) {
    if (!m_physics_item)
        return;

    auto mesh = dynamic_cast<ChMesh*>(m_physics_item);
    if (!mesh)
        return;

    // Traverse all elements in the provided mesh
    // - extract the 1-D elements elements that use XYZ nodes
    // - extract the 1-D elements that use XYZRot nodes

    std::vector<std::array<std::shared_ptr<ChNodeFEAxyz>, 2>> seg_ptrs;
    std::vector<std::array<std::shared_ptr<ChNodeFEAxyzrot>, 2>> seg_rot_ptrs;

    for (const auto& element : mesh->GetElements()) {
        if (auto cable_el = std::dynamic_pointer_cast<fea::ChElementCableANCF>(element)) {
            seg_ptrs.push_back({{cable_el->GetNodeA(), cable_el->GetNodeB()}});
        } else if (auto beam3243_el = std::dynamic_pointer_cast<fea::ChElementBeamANCF_3243>(element)) {
            seg_ptrs.push_back({{beam3243_el->GetNodeA(), beam3243_el->GetNodeB()}});
        } else if (auto beam3333_el = std::dynamic_pointer_cast<fea::ChElementBeamANCF_3333>(element)) {
            seg_ptrs.push_back({{beam3333_el->GetNodeA(), beam3333_el->GetNodeC()}});
            seg_ptrs.push_back({{beam3333_el->GetNodeC(), beam3333_el->GetNodeB()}});
        } else if (auto beam_el = std::dynamic_pointer_cast<fea::ChElementBeamEuler>(element)) {
            seg_rot_ptrs.push_back({{beam_el->GetNodeA(), beam_el->GetNodeB()}});
        }
    }

    // Loop through all 1D elements found, set node ownership, and create a contact segment

    std::set<fea::ChNodeFEAxyz*> assigned;
    for (const auto& s : seg_ptrs) {
        ChVector2<bool> owns_node = {false, false};
        if (assigned.count(s[0].get()) == 0) {
            assigned.insert(s[0].get());
            owns_node[0] = true;
        }
        if (assigned.count(s[1].get()) == 0) {
            assigned.insert(s[1].get());
            owns_node[1] = true;
        }

        AddSegment(s[0], s[1], owns_node[0], owns_node[1], sphere_swept);
    }

    std::set<fea::ChNodeFEAxyzrot*> assigned_rot;
    for (const auto& s : seg_rot_ptrs) {
        ChVector2<bool> owns_node = {false, false};
        if (assigned_rot.count(s[0].get()) == 0) {
            assigned_rot.insert(s[0].get());
            owns_node[0] = true;
        }
        if (assigned_rot.count(s[1].get()) == 0) {
            assigned_rot.insert(s[1].get());
            owns_node[1] = true;
        }

        AddSegment(s[0], s[1], owns_node[0], owns_node[1], sphere_swept);
    }
}

void ChContactSurfaceSegmentSet::SyncCollisionModels() const {
    for (auto& seg : m_segments) {
        seg->GetCollisionModel()->SyncPosition();
    }
    for (auto& seg : m_segments_rot) {
        seg->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceSegmentSet::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    SyncCollisionModels();
    for (const auto& seg : m_segments) {
        coll_sys->Add(seg->GetCollisionModel());
    }
    for (const auto& seg : m_segments_rot) {
        coll_sys->Add(seg->GetCollisionModel());
    }
}

void ChContactSurfaceSegmentSet::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    for (const auto& seg : m_segments) {
        coll_sys->Remove(seg->GetCollisionModel());
    }
    for (const auto& seg : m_segments_rot) {
        coll_sys->Remove(seg->GetCollisionModel());
    }
}

}  // end namespace fea
}  // end namespace chrono
