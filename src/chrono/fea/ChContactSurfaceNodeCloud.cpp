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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChMesh.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
//  ChContactNodeXYZ

ChContactNodeXYZ::ChContactNodeXYZ(ChNodeFEAxyz* node, ChContactSurface* contact_surface) {
    m_node = node;
    m_container = contact_surface;

    // Load contactable variables list
    m_contactable_variables.push_back(&m_node->Variables());
}

void ChContactNodeXYZ::ContactForceLoadResidual_F(const ChVector3d& F,
                                                  const ChVector3d& T,
                                                  const ChVector3d& abs_point,
                                                  ChVectorDynamic<>& R) {
    R.segment(m_node->NodeGetOffsetVelLevel(), 3) += F.eigen();
}

void ChContactNodeXYZ::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                                     ChMatrix33<>& contact_plane,
                                                     ChConstraintTuple* jacobian_tuple_N,
                                                     ChConstraintTuple* jacobian_tuple_U,
                                                     ChConstraintTuple* jacobian_tuple_V,
                                                     bool second) {
    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    auto tuple_N = static_cast<ChConstraintTuple_1vars<3>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<3>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<3>*>(jacobian_tuple_V);

    tuple_N->Cq1().segment(0, 3) = Jx1.row(0);
    tuple_U->Cq1().segment(0, 3) = Jx1.row(1);
    tuple_V->Cq1().segment(0, 3) = Jx1.row(2);
}

ChPhysicsItem* ChContactNodeXYZ::GetPhysicsItem() {
    return m_container->GetPhysicsItem();
}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZRot

ChContactNodeXYZRot::ChContactNodeXYZRot(ChNodeFEAxyzrot* node, ChContactSurface* contact_surface) {
    m_node = node;
    m_container = contact_surface;

    // Load contactable variables list
    m_contactable_variables.push_back(&m_node->Variables());
}

void ChContactNodeXYZRot::ContactForceLoadResidual_F(const ChVector3d& F,
                                                     const ChVector3d& T,
                                                     const ChVector3d& abs_point,
                                                     ChVectorDynamic<>& R) {
    R.segment(m_node->NodeGetOffsetVelLevel(), 3) += F.eigen();
}

void ChContactNodeXYZRot::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                                        ChMatrix33<>& contact_plane,
                                                        ChConstraintTuple* jacobian_tuple_N,
                                                        ChConstraintTuple* jacobian_tuple_U,
                                                        ChConstraintTuple* jacobian_tuple_V,
                                                        bool second) {
    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    auto tuple_N = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_V);

    tuple_N->Cq1().segment(0, 3) = Jx1.row(0);
    tuple_U->Cq1().segment(0, 3) = Jx1.row(1);
    tuple_V->Cq1().segment(0, 3) = Jx1.row(2);
}

ChPhysicsItem* ChContactNodeXYZRot::GetPhysicsItem() {
    return m_container->GetPhysicsItem();
}

// -----------------------------------------------------------------------------
//  ChContactSurfaceNodeCloud

ChContactSurfaceNodeCloud::ChContactSurfaceNodeCloud(std::shared_ptr<ChContactMaterial> material, ChMesh* mesh)
    : ChContactSurface(material, mesh) {}

void ChContactSurfaceNodeCloud::AddNode(std::shared_ptr<ChNodeFEAxyz> node, const double point_radius) {
    if (!node)
        return;

    auto contact_node = chrono_types::make_shared<ChContactNodeXYZ>(node.get(), this);
    auto point_shape = chrono_types::make_shared<ChCollisionShapePoint>(m_material, VNULL, point_radius);
    contact_node->AddCollisionShape(point_shape);

    if (!m_self_collide) {
        contact_node->GetCollisionModel()->SetFamily(m_collision_family);
        contact_node->GetCollisionModel()->DisallowCollisionsWith(m_collision_family);
    }

    m_nodes.push_back(contact_node);
}

void ChContactSurfaceNodeCloud::AddNode(std::shared_ptr<ChNodeFEAxyzrot> node, const double point_radius) {
    if (!node)
        return;

    auto contact_node = chrono_types::make_shared<ChContactNodeXYZRot>(node.get(), this);
    auto point_shape = chrono_types::make_shared<ChCollisionShapePoint>(m_material, VNULL, point_radius);
    contact_node->AddCollisionShape(point_shape);

    if (!m_self_collide) {
        contact_node->GetCollisionModel()->SetFamily(m_collision_family);
        contact_node->GetCollisionModel()->DisallowCollisionsWith(m_collision_family);
    }

    m_nodes_rot.push_back(contact_node);
}

// Add all nodes of the mesh to this collision cloud
void ChContactSurfaceNodeCloud::AddAllNodes(const ChMesh& mesh, double point_radius) {
    for (unsigned int i = 0; i < mesh.GetNumNodes(); ++i)
        if (auto nodeFEA = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh.GetNode(i)))
            this->AddNode(nodeFEA, point_radius);
        else if (auto nodeFEArot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh.GetNode(i)))
            this->AddNode(nodeFEArot, point_radius);
}

// Add nodes of the mesh, belonging to the node_set, to this collision cloud
void ChContactSurfaceNodeCloud::AddNodesFromNodeSet(const std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set,
                                                    const double point_radius) {
    for (unsigned int i = 0; i < node_set.size(); ++i)
        if (auto nodeFEA = std::dynamic_pointer_cast<ChNodeFEAxyz>(node_set[i]))
            this->AddNode(nodeFEA, point_radius);
        else if (auto nodeFEArot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node_set[i]))
            this->AddNode(nodeFEArot, point_radius);
}

void ChContactSurfaceNodeCloud::SyncCollisionModels() const {
    for (auto& node : m_nodes) {
        node->GetCollisionModel()->SyncPosition();
    }
    for (auto& node : m_nodes_rot) {
        node->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceNodeCloud::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    SyncCollisionModels();
    for (const auto& node : m_nodes) {
        coll_sys->Add(node->GetCollisionModel());
    }
    for (const auto& node : m_nodes_rot) {
        coll_sys->Add(node->GetCollisionModel());
    }
}

void ChContactSurfaceNodeCloud::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    for (const auto& node : m_nodes) {
        coll_sys->Remove(node->GetCollisionModel());
    }
    for (const auto& node : m_nodes_rot) {
        coll_sys->Remove(node->GetCollisionModel());
    }
}

ChAABB ChContactSurfaceNodeCloud::GetAABB() const {
    ChAABB aabb;
    for (const auto& node : m_nodes) {
        aabb += node->GetPos();
    }
    for (const auto& node : m_nodes_rot) {
        aabb += node->GetPos();
    }
    return aabb;
}

}  // end namespace fea
}  // end namespace chrono
