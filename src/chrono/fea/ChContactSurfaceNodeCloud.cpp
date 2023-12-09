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

#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChMesh.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
//  ChContactNodeXYZ

ChContactNodeXYZ::ChContactNodeXYZ(ChNodeFEAxyz* node, ChContactSurface* contact_surface) {
    m_node = node;
    m_container = contact_surface;
}

void ChContactNodeXYZ::ContactForceLoadResidual_F(const ChVector<>& F,
                                                  const ChVector<>& T,
                                                  const ChVector<>& abs_point,
                                                  ChVectorDynamic<>& R) {
    R.segment(m_node->NodeGetOffsetW(), 3) += F.eigen();
}

void ChContactNodeXYZ::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                                     ChMatrix33<>& contact_plane,
                                                     type_constraint_tuple& jacobian_tuple_N,
                                                     type_constraint_tuple& jacobian_tuple_U,
                                                     type_constraint_tuple& jacobian_tuple_V,
                                                     bool second) {
    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    jacobian_tuple_N.Get_Cq().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq().segment(0, 3) = Jx1.row(2);
}

ChPhysicsItem* ChContactNodeXYZ::GetPhysicsItem() {
    return m_container->GetPhysicsItem();
}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZsphere

ChContactNodeXYZsphere::ChContactNodeXYZsphere(ChNodeFEAxyz* node, ChContactSurface* contact_surface)
    : ChContactNodeXYZ(node, contact_surface) {}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZROT

ChContactNodeXYZROT::ChContactNodeXYZROT(ChNodeFEAxyzrot* node, ChContactSurface* contact_surface) {
    m_node = node;
    m_container = contact_surface;
}

void ChContactNodeXYZROT::ContactForceLoadResidual_F(const ChVector<>& F,
                                                     const ChVector<>& T,
                                                     const ChVector<>& abs_point,
                                                     ChVectorDynamic<>& R) {
    R.segment(m_node->NodeGetOffsetW(), 3) += F.eigen();
}

void ChContactNodeXYZROT::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                                        ChMatrix33<>& contact_plane,
                                                        type_constraint_tuple& jacobian_tuple_N,
                                                        type_constraint_tuple& jacobian_tuple_U,
                                                        type_constraint_tuple& jacobian_tuple_V,
                                                        bool second) {
    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    jacobian_tuple_N.Get_Cq().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq().segment(0, 3) = Jx1.row(2);
}

ChPhysicsItem* ChContactNodeXYZROT::GetPhysicsItem() {
    return m_container->GetPhysicsItem();
}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZROTsphere

ChContactNodeXYZROTsphere::ChContactNodeXYZROTsphere(ChNodeFEAxyzrot* node, ChContactSurface* contact_surface)
    : ChContactNodeXYZROT(node, contact_surface) {}

// -----------------------------------------------------------------------------
//  ChContactSurfaceNodeCloud

ChContactSurfaceNodeCloud::ChContactSurfaceNodeCloud(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh)
    : ChContactSurface(material, mesh) {}

void ChContactSurfaceNodeCloud::AddNode(std::shared_ptr<ChNodeFEAxyz> node, const double point_radius) {
    if (!node)
        return;

    auto contact_node = chrono_types::make_shared<ChContactNodeXYZsphere>(node.get(), this);
    auto point_shape = chrono_types::make_shared<ChCollisionShapePoint>(m_material, VNULL, point_radius);
    contact_node->AddCollisionShape(point_shape);

    m_nodes.push_back(contact_node);
}

void ChContactSurfaceNodeCloud::AddNode(std::shared_ptr<ChNodeFEAxyzrot> node, const double point_radius) {
    if (!node)
        return;

    auto contact_node = chrono_types::make_shared<ChContactNodeXYZROTsphere>(node.get(), this);
    auto point_shape = chrono_types::make_shared<ChCollisionShapePoint>(m_material, VNULL, point_radius);
    contact_node->AddCollisionShape(point_shape);

    m_nodes_rot.push_back(contact_node);
}

/// Add all nodes of the mesh to this collision cloud
void ChContactSurfaceNodeCloud::AddAllNodes(const double point_radius) {
    if (!m_physics_item)
        return;
    auto mesh = dynamic_cast<ChMesh*>(m_physics_item);
    if (!mesh)
        return;

    for (unsigned int i = 0; i < mesh->GetNnodes(); ++i)
        if (auto nodeFEA = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i)))
            this->AddNode(nodeFEA, point_radius);
        else if (auto nodeFEArot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(i)))
            this->AddNode(nodeFEArot, point_radius);
}

/// Add nodes of the mesh, belonging to the node_set, to this collision cloud
void ChContactSurfaceNodeCloud::AddNodesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set,
                                                    const double point_radius) {
    if (!m_physics_item)
        return;
    auto mesh = dynamic_cast<ChMesh*>(m_physics_item);
    if (!mesh)
        return;

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

}  // end namespace fea
}  // end namespace chrono
