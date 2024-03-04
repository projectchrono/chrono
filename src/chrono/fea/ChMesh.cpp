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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>

#include "chrono/core/ChMath.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChObject.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

ChMesh::ChMesh(const ChMesh& other) : ChIndexedNodes(other) {
    vnodes = other.vnodes;
    velements = other.velements;

    n_dofs = other.n_dofs;
    n_dofs_w = other.n_dofs_w;

    vcontactsurfaces = other.vcontactsurfaces;
    vmeshsurfaces = other.vmeshsurfaces;

    automatic_gravity_load = other.automatic_gravity_load;
    num_points_gravity = other.num_points_gravity;

    ncalls_internal_forces = 0;
    ncalls_KRMload = 0;
}

void ChMesh::SetupInitial() {
    n_dofs = 0;
    n_dofs_w = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        if (!vnodes[i]->IsFixed()) {
            vnodes[i]->SetupInitial(GetSystem());

            // count the degrees of freedom
            n_dofs += vnodes[i]->GetNdofX_active();
            n_dofs_w += vnodes[i]->GetNdofW_active();
        }
    }

    for (unsigned int i = 0; i < velements.size(); i++) {
        // precompute matrices, such as the [Kl] local stiffness of each element, if needed, etc.
        velements[i]->SetupInitial(GetSystem());
    }
}

void ChMesh::Relax() {
    for (unsigned int i = 0; i < vnodes.size(); i++) {
        // "relaxes" the structure by setting all X0 = 0, and null speeds
        vnodes[i]->Relax();
    }
}

void ChMesh::SetNoSpeedNoAcceleration() {
    for (unsigned int i = 0; i < vnodes.size(); i++) {
        // set null speeds, null accelerations
        vnodes[i]->SetNoSpeedNoAcceleration();
    }
}

void ChMesh::AddNode(std::shared_ptr<ChNodeFEAbase> m_node) {
    m_node->SetIndex(static_cast<unsigned int>(vnodes.size()) + 1);
    vnodes.push_back(m_node);

    // If the mesh is already added to a system, mark the system uninitialized and out-of-date
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
}

void ChMesh::AddElement(std::shared_ptr<ChElementBase> m_elem) {
    velements.push_back(m_elem);

    // If the mesh is already added to a system, mark the system uninitialized and out-of-date
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
}

void ChMesh::ClearElements() {
    velements.clear();
    vcontactsurfaces.clear();

    // If the mesh is already added to a system, mark the system out-of-date
    if (system) {
        system->is_updated = false;
    }
}

void ChMesh::ClearNodes() {
    velements.clear();
    vnodes.clear();
    vcontactsurfaces.clear();

    // If the mesh is already added to a system, mark the system out-of-date
    if (system) {
        system->is_updated = false;
    }
}

void ChMesh::AddContactSurface(std::shared_ptr<ChContactSurface> m_surf) {
    m_surf->SetPhysicsItem(this);
    vcontactsurfaces.push_back(m_surf);
}

void ChMesh::ClearContactSurfaces() {
    vcontactsurfaces.clear();
}

void ChMesh::AddMeshSurface(std::shared_ptr<ChMeshSurface> m_surf) {
    m_surf->SetMesh(this);
    vmeshsurfaces.push_back(m_surf);
}

/// This recomputes the number of DOFs, constraints, as well as state offsets of contained items
void ChMesh::Setup() {
    n_dofs = 0;
    n_dofs_w = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        // Set node offsets in state vectors (based on the offsets of the containing mesh)
        vnodes[i]->NodeSetOffset_x(GetOffset_x() + n_dofs);
        vnodes[i]->NodeSetOffset_w(GetOffset_w() + n_dofs_w);

        // Count the actual degrees of freedom (consider only nodes that are not fixed)
        if (!vnodes[i]->IsFixed()) {
            n_dofs += vnodes[i]->GetNdofX_active();
            n_dofs_w += vnodes[i]->GetNdofW_active();
        }
    }
}

// Updates all time-dependant variables, if any...
// Ex: maybe the elasticity can increase in time, etc.
void ChMesh::Update(double m_time, bool update_assets) {
    // Parent class update
    ChIndexedNodes::Update(m_time, update_assets);

    for (unsigned int i = 0; i < velements.size(); i++) {
        //    - update auxiliary stuff, ex. update element's rotation matrices if corotational..
        velements[i]->Update();
    }
}

void ChMesh::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    for (const auto& surf : vcontactsurfaces)
        surf->AddCollisionModelsToSystem(coll_sys);
}

void ChMesh::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    for (const auto& surf : vcontactsurfaces)
        surf->RemoveCollisionModelsFromSystem(coll_sys);
}

void ChMesh::SyncCollisionModels() {
    for (const auto& surf : vcontactsurfaces)
        surf->SyncCollisionModels();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChMesh::IntStateGather(const unsigned int off_x,
                            ChState& x,
                            const unsigned int off_v,
                            ChStateDelta& v,
                            double& T) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
            local_off_x += vnodes[j]->GetNdofX_active();
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }

    T = GetChTime();
}

void ChMesh::IntStateScatter(const unsigned int off_x,
                             const ChState& x,
                             const unsigned int off_v,
                             const ChStateDelta& v,
                             const double T,
                             bool full_update) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
            local_off_x += vnodes[j]->GetNdofX_active();
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }

    Update(T, full_update);
}

void ChMesh::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int local_off_a = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntStateGatherAcceleration(off_a + local_off_a, a);
            local_off_a += vnodes[j]->GetNdofW_active();
        }
    }
}

void ChMesh::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int local_off_a = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntStateScatterAcceleration(off_a + local_off_a, a);
            local_off_a += vnodes[j]->GetNdofW_active();
        }
    }
}

void ChMesh::IntStateIncrement(const unsigned int off_x,
                               ChState& x_new,
                               const ChState& x,
                               const unsigned int off_v,
                               const ChStateDelta& Dv) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
            local_off_x += vnodes[j]->GetNdofX_active();
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }
    for (unsigned int ie = 0; ie < velements.size(); ie++) {
        velements[ie]->EleDoIntegration();
    }
}

void ChMesh::IntStateGetIncrement(const unsigned int off_x,
                                  const ChState& x_new,
                                  const ChState& x,
                                  const unsigned int off_v,
                                  ChStateDelta& Dv) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
            local_off_x += vnodes[j]->GetNdofX_active();
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }
}

void ChMesh::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // nodes applied forces
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntLoadResidual_F(off + local_off_v, R, c);
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }

    int nthreads = GetSystem()->nthreads_chrono;

    // elements internal forces
    timer_internal_forces.start();
    //***PARALLEL FOR***, must use omp atomic to avoid race condition in writing to R
#pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int ie = 0; ie < velements.size(); ie++) {
        velements[ie]->EleIntLoadResidual_F(R, c);
    }
    timer_internal_forces.stop();
    ncalls_internal_forces++;

    // elements gravity forces
    if (automatic_gravity_load) {
        //***PARALLEL FOR***, must use omp atomic to avoid race condition in writing to R
#pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
        for (int ie = 0; ie < velements.size(); ie++) {
            velements[ie]->EleIntLoadResidual_F_gravity(R, GetSystem()->Get_G_acc(), c);
        }
    }

    // nodes gravity forces
    local_off_v = 0;
    if (automatic_gravity_load && system) {
        // #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
        //***PARALLEL FOR***, (no need here to use omp atomic to avoid race condition in writing to R)
        for (int in = 0; in < vnodes.size(); in++) {
            if (!vnodes[in]->IsFixed()) {
                if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyz>(vnodes[in])) {
                    ChVector<> fg = c * mnode->GetMass() * system->Get_G_acc();
                    R.segment(off + local_off_v, 3) += fg.eigen();
                }
                // ChNodeFEAxyzrot is not inherited from ChNodeFEAxyz, so must deal with it too
                if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(vnodes[in])) {
                    ChVector<> fg = c * mnode->GetMass() * system->Get_G_acc();
                    R.segment(off + local_off_v, 3) += fg.eigen();
                }
                local_off_v += vnodes[in]->GetNdofW_active();
            }
        }
    }
}

void ChMesh::ComputeMassProperties(double& mass,           // ChMesh object mass
                                   ChVector<>& com,        // ChMesh center of gravity
                                   ChMatrix33<>& inertia)  // ChMesh inertia tensor
{
    mass = 0;
    com = ChVector<>(0);
    inertia = ChMatrix33<>(1);

    ChVector<> mmass_weighted_radius(0);
    double mJxx=0;
    double mJyy=0;
    double mJzz=0;
    double mJxy=0;
    double mJyz=0;
    double mJxz=0;

    // Initialize all nodal total masses to zero
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        vnodes[j]->m_TotalMass = 0.0;
    }
    // Loop over all elements and calculate contribution to nodal mass
    for (unsigned int ie = 0; ie < velements.size(); ie++) {
        velements[ie]->ComputeNodalMass();
    }

    // Loop over all the nodes of the mesh to obtain total object mass.
    // GetMass() will get the atomic mass attached at the node;
    // m_TotalMass is the accumulative equivalent mass lumped at the node from its associated elements.
    for (auto& node : vnodes) {
        if (auto xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node)) {
            double mm = xyz->GetMass() + xyz->m_TotalMass;
            mass += mm;
            mmass_weighted_radius += mm * xyz->GetPos();
            mJxx += mm * (xyz->GetPos().y() * xyz->GetPos().y() + xyz->GetPos().z() * xyz->GetPos().z());
            mJyy += mm * (xyz->GetPos().x() * xyz->GetPos().x() + xyz->GetPos().z() * xyz->GetPos().z());
            mJzz += mm * (xyz->GetPos().x() * xyz->GetPos().x() + xyz->GetPos().y() * xyz->GetPos().y());
            mJxy += mm * xyz->GetPos().x() * xyz->GetPos().y();
            mJyz += mm * xyz->GetPos().y() * xyz->GetPos().z();
            mJxz += mm * xyz->GetPos().x() * xyz->GetPos().z();
        }
        if (auto xyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node)) {
            double mm = xyzrot->GetMass() + xyzrot->m_TotalMass;
            mass += mm;
            mmass_weighted_radius += mm * xyzrot->GetPos();
            mJxx += mm * (xyzrot->GetPos().y() * xyzrot->GetPos().y() + xyzrot->GetPos().z() * xyzrot->GetPos().z());
            mJyy += mm * (xyzrot->GetPos().x() * xyzrot->GetPos().x() + xyzrot->GetPos().z() * xyzrot->GetPos().z());
            mJzz += mm * (xyzrot->GetPos().x() * xyzrot->GetPos().x() + xyzrot->GetPos().y() * xyzrot->GetPos().y());
            mJxy += mm * xyzrot->GetPos().x() * xyzrot->GetPos().y();
            mJyz += mm * xyzrot->GetPos().y() * xyzrot->GetPos().z();
            mJxz += mm * xyzrot->GetPos().x() * xyzrot->GetPos().z();
        }
    }

    if (mass)
        com = mmass_weighted_radius / mass;

    // Using the lumped mass approximation to calculate the inertia tensor.
    // Note: the inertia of the cross sections of the associated elements at the nodes 
    // are neglected here for the sake of the conciseness of code implementation.
    inertia(0, 0) = mJxx;
    inertia(0, 1) = -mJxy;
    inertia(0, 2) = -mJxz;
    inertia(1, 0) = -mJxy;
    inertia(1, 1) = mJyy;
    inertia(1, 2) = -mJyz;
    inertia(2, 0) = -mJxz;
    inertia(2, 1) = -mJyz;
    inertia(2, 2) = mJzz;
}

void ChMesh::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                const ChVectorDynamic<>& w,  ///< the w vector
                                const double c               ///< a scaling factor
) {
    // nodal masses
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntLoadResidual_Mv(off + local_off_v, R, w, c);
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }

    // internal masses
    for (unsigned int ie = 0; ie < velements.size(); ie++) {
        velements[ie]->EleIntLoadResidual_Mv(R, w, c);
    }
}

void ChMesh::IntLoadLumpedMass_Md(const unsigned int off,
                                  ChVectorDynamic<>& Md,
                                  double& err,
                                  const double c
) {
    // nodal masses
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntLoadLumpedMass_Md(off + local_off_v, Md, err, c);
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }

    // internal masses
    for (unsigned int ie = 0; ie < velements.size(); ie++) {
        velements[ie]->EleIntLoadLumpedMass_Md(Md, err, c);
    }
}

void ChMesh::IntToDescriptor(const unsigned int off_v,
                             const ChStateDelta& v,
                             const ChVectorDynamic<>& R,
                             const unsigned int off_L,
                             const ChVectorDynamic<>& L,
                             const ChVectorDynamic<>& Qc) {
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntToDescriptor(off_v + local_off_v, v, R);
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }
}

void ChMesh::IntFromDescriptor(const unsigned int off_v,
                               ChStateDelta& v,
                               const unsigned int off_L,
                               ChVectorDynamic<>& L) {
    unsigned int local_off_v = 0;
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        if (!vnodes[j]->IsFixed()) {
            vnodes[j]->NodeIntFromDescriptor(off_v + local_off_v, v);
            local_off_v += vnodes[j]->GetNdofW_active();
        }
    }
}

//// SOLVER FUNCTIONS

void ChMesh::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    for (unsigned int ie = 0; ie < velements.size(); ie++)
        velements[ie]->InjectKRMmatrices(mdescriptor);
}

void ChMesh::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    int nthreads = GetSystem()->nthreads_chrono;

    timer_KRMload.start();
#pragma omp parallel for num_threads(nthreads)
    for (int ie = 0; ie < velements.size(); ie++)
        velements[ie]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    timer_KRMload.stop();
    ncalls_KRMload++;
}

void ChMesh::VariablesFbReset() {
    for (unsigned int ie = 0; ie < vnodes.size(); ie++)
        vnodes[ie]->VariablesFbReset();
}

void ChMesh::VariablesFbLoadForces(double factor) {
    // applied nodal forces
    for (unsigned int in = 0; in < vnodes.size(); in++)
        vnodes[in]->VariablesFbLoadForces(factor);

    // internal forces
    for (unsigned int ie = 0; ie < velements.size(); ie++)
        velements[ie]->VariablesFbLoadInternalForces(factor);
}

void ChMesh::VariablesQbLoadSpeed() {
    for (unsigned int ie = 0; ie < vnodes.size(); ie++)
        vnodes[ie]->VariablesQbLoadSpeed();
}

void ChMesh::VariablesFbIncrementMq() {
    // nodal masses
    for (unsigned int ie = 0; ie < vnodes.size(); ie++)
        vnodes[ie]->VariablesFbIncrementMq();

    // internal masses
    for (unsigned int ie = 0; ie < velements.size(); ie++)
        velements[ie]->VariablesFbIncrementMq();
}

void ChMesh::VariablesQbSetSpeed(double step) {
    for (unsigned int ie = 0; ie < vnodes.size(); ie++)
        vnodes[ie]->VariablesQbSetSpeed(step);
}

void ChMesh::VariablesQbIncrementPosition(double step) {
    for (unsigned int ie = 0; ie < vnodes.size(); ie++)
        vnodes[ie]->VariablesQbIncrementPosition(step);
}

void ChMesh::InjectVariables(ChSystemDescriptor& mdescriptor) {
    for (unsigned int ie = 0; ie < vnodes.size(); ie++)
        vnodes[ie]->InjectVariables(mdescriptor);
}

}  // end namespace fea
}  // end namespace chrono
