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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChMesh.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
//  ChContactNodeXYZ

void ChContactNodeXYZ::ContactForceLoadResidual_F(const ChVector<>& F,
                                                  const ChVector<>& abs_point,
                                                  ChVectorDynamic<>& R) {
    R.segment(this->mnode->NodeGetOffsetW(), 3) += F.eigen();
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
    return (ChPhysicsItem*)container->GetMesh();
}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZsphere

ChContactNodeXYZsphere::ChContactNodeXYZsphere(ChNodeFEAxyz* anode, ChContactSurface* acontainer)
    : ChContactNodeXYZ(anode, acontainer) {
    this->collision_model = new collision::ChCollisionModelBullet;
    this->collision_model->SetContactable(this);
}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZROT

void ChContactNodeXYZROT::ContactForceLoadResidual_F(const ChVector<>& F,
                                                     const ChVector<>& abs_point,
                                                     ChVectorDynamic<>& R) {
    R.segment(this->mnode->NodeGetOffsetW(), 3) += F.eigen();
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
    return (ChPhysicsItem*)container->GetMesh();
}

// -----------------------------------------------------------------------------
//  ChContactNodeXYZROTsphere

ChContactNodeXYZROTsphere::ChContactNodeXYZROTsphere(ChNodeFEAxyzrot* anode, ChContactSurface* acontainer)
    : ChContactNodeXYZROT(anode, acontainer) {
    this->collision_model = new collision::ChCollisionModelBullet;
    this->collision_model->SetContactable(this);
}

// -----------------------------------------------------------------------------
//  ChContactSurfaceNodeCloud

ChContactSurfaceNodeCloud::ChContactSurfaceNodeCloud(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh)
    : ChContactSurface(material, mesh) {}

void ChContactSurfaceNodeCloud::AddNode(std::shared_ptr<ChNodeFEAxyz> mnode, const double point_radius) {
    if (!mnode)
        return;

    auto newp = chrono_types::make_shared<ChContactNodeXYZsphere>(mnode.get(), this);

    newp->GetCollisionModel()->AddPoint(m_material, point_radius);
    newp->GetCollisionModel()->BuildModel();  // will also add to system, if collision is on.

    this->vnodes.push_back(newp);
}

void ChContactSurfaceNodeCloud::AddNode(std::shared_ptr<ChNodeFEAxyzrot> mnode, const double point_radius) {
    if (!mnode)
        return;

    auto newp = chrono_types::make_shared<ChContactNodeXYZROTsphere>(mnode.get(), this);

    newp->GetCollisionModel()->AddPoint(m_material, point_radius);
    newp->GetCollisionModel()->BuildModel();  // will also add to system, if collision is on.

    this->vnodes_rot.push_back(newp);
}

/// Add all nodes of the mesh to this collision cloud
void ChContactSurfaceNodeCloud::AddAllNodes(const double point_radius) {
    if (!m_mesh)
        return;
    for (unsigned int i = 0; i < m_mesh->GetNnodes(); ++i)
        if (auto mnodeFEA = std::dynamic_pointer_cast<ChNodeFEAxyz>(m_mesh->GetNode(i)))
            this->AddNode(mnodeFEA, point_radius);
        else if (auto mnodeFEArot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(i)))
            this->AddNode(mnodeFEArot, point_radius);
}

/// Add nodes of the mesh, belonging to the node_set, to this collision cloud
void ChContactSurfaceNodeCloud::AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set,
                                                    const double point_radius) {
    if (!m_mesh)
        return;
    for (unsigned int i = 0; i < node_set.size(); ++i)
        if (auto mnodeFEA = std::dynamic_pointer_cast<ChNodeFEAxyz>(node_set[i]))
            this->AddNode(mnodeFEA, point_radius);
        else if (auto mnodeFEArot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node_set[i]))
            this->AddNode(mnodeFEArot, point_radius);
}

void ChContactSurfaceNodeCloud::SurfaceSyncCollisionModels() {
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        this->vnodes[j]->GetCollisionModel()->SyncPosition();
    }
    for (unsigned int j = 0; j < vnodes_rot.size(); j++) {
        this->vnodes_rot[j]->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceNodeCloud::SurfaceAddCollisionModelsToSystem(ChSystem* msys) {
    assert(msys);
    SurfaceSyncCollisionModels();
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vnodes[j]->GetCollisionModel());
    }
    for (unsigned int j = 0; j < vnodes_rot.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vnodes_rot[j]->GetCollisionModel());
    }
}

void ChContactSurfaceNodeCloud::SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) {
    assert(msys);
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vnodes[j]->GetCollisionModel());
    }
    for (unsigned int j = 0; j < vnodes_rot.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vnodes_rot[j]->GetCollisionModel());
    }
}

}  // end namespace fea
}  // end namespace chrono
