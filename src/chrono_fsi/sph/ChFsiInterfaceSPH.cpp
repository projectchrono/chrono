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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Base class for processing the interface between Chrono and fsi modules
// =============================================================================

#include "chrono_fsi/sph/ChFsiInterfaceSPH.h"
#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/sph/utils/ChUtilsTypeConvert.h"

namespace chrono {
namespace fsi {

using namespace sph;

ChFsiInterfaceSPH::ChFsiInterfaceSPH(FsiDataManager& data_mgr, bool verbose)
    : ChFsiInterface(verbose), m_data_mgr(data_mgr) {}

ChFsiInterfaceSPH::~ChFsiInterfaceSPH() {}

// ---------- Chrono rigid body Specifics --------------------------------------

void ChFsiInterfaceSPH::ApplyBodyForces() {
    size_t num_bodies = m_fsi_bodies.size();

    thrust::host_vector<Real3> forcesH = m_data_mgr.rigid_FSI_ForcesD;
    thrust::host_vector<Real3> torquesH = m_data_mgr.rigid_FSI_TorquesD;

    for (size_t i = 0; i < num_bodies; i++) {
        std::shared_ptr<ChBody> body = m_fsi_bodies[i].body;

        m_fsi_bodies[i].fsi_force = ToChVector(forcesH[i]);
        m_fsi_bodies[i].fsi_torque = ToChVector(torquesH[i]);

        body->EmptyAccumulators();
        body->AccumulateForce(m_fsi_bodies[i].fsi_force, body->GetPos(), false);
        body->AccumulateTorque(m_fsi_bodies[i].fsi_torque, false);
    }
}

void ChFsiInterfaceSPH::LoadBodyStates() {
    size_t num_bodies = m_fsi_bodies.size();

    for (size_t i = 0; i < num_bodies; i++) {
        std::shared_ptr<ChBody> body = m_fsi_bodies[i].body;

        m_data_mgr.fsiBodyState_H->pos[i] = ToReal3(body->GetPos());
        m_data_mgr.fsiBodyState_H->lin_vel[i] = ToReal4(body->GetPosDt(), body->GetMass());
        m_data_mgr.fsiBodyState_H->lin_acc[i] = ToReal3(body->GetPosDt2());
        m_data_mgr.fsiBodyState_H->rot[i] = ToReal4(body->GetRot());
        m_data_mgr.fsiBodyState_H->ang_vel[i] = ToReal3(body->GetAngVelLocal());
        m_data_mgr.fsiBodyState_H->ang_acc[i] = ToReal3(body->GetAngAccLocal());
    }
    
    m_data_mgr.fsiBodyState_D->CopyFromH(*m_data_mgr.fsiBodyState_H);
}

// ---------- Chrono FEA Specifics ---------------------------------------------

void ChFsiInterfaceSPH::ApplyMeshForces() {
    {
        // Transfer to host
        thrust::host_vector<Real3> forces_H = m_data_mgr.flex1D_FSIforces_D;

        // Apply to FEA nodes
        int counter = 0;
        for (const auto& fsi_mesh : m_fsi_meshes1D) {
            int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (int i = 0; i < num_nodes; i++) {
                const auto& node = fsi_mesh.ind2ptr_map.at(i);
                node->SetForce(ToChVector(forces_H[counter]));
                counter++;
            }
        }
    }

    {
        // Transfer to host
        thrust::host_vector<Real3> forces_H = m_data_mgr.flex2D_FSIforces_D;

        // Apply to FEA nodes
        int counter = 0;
        for (const auto& fsi_mesh : m_fsi_meshes2D) {
            int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (int i = 0; i < num_nodes; i++) {
                const auto& node = fsi_mesh.ind2ptr_map.at(i);
                node->SetForce(ToChVector(forces_H[counter]));
                counter++;
            }
        }
    }
}

void ChFsiInterfaceSPH::LoadMeshStates() {
    {
        // Load from FEA nodes on host
        int counter = 0;
        for (const auto& fsi_mesh : m_fsi_meshes1D) {
            int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (int i = 0; i < num_nodes; i++) {
                const auto& node = fsi_mesh.ind2ptr_map.at(i);
                m_data_mgr.fsiMesh1DState_H->pos_fsi_fea_H[counter] = ToReal3(node->GetPos());
                m_data_mgr.fsiMesh1DState_H->vel_fsi_fea_H[counter] = ToReal3(node->GetPosDt());
                m_data_mgr.fsiMesh1DState_H->acc_fsi_fea_H[counter] = ToReal3(node->GetPosDt2());
                counter++;
            }
        }

        // Transfer to device
        m_data_mgr.fsiMesh1DState_D->CopyFromH(*m_data_mgr.fsiMesh1DState_H);
    }

    {
        // Load from FEA nodes on host
        int counter = 0;
        for (const auto& fsi_mesh : m_fsi_meshes2D) {
            int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (int i = 0; i < num_nodes; i++) {
                const auto& node = fsi_mesh.ind2ptr_map.at(i);
                m_data_mgr.fsiMesh2DState_H->pos_fsi_fea_H[counter] = ToReal3(node->GetPos());
                m_data_mgr.fsiMesh2DState_H->vel_fsi_fea_H[counter] = ToReal3(node->GetPosDt());
                m_data_mgr.fsiMesh2DState_H->acc_fsi_fea_H[counter] = ToReal3(node->GetPosDt2());
                counter++;
            }
        }

        // Transfer to device
        m_data_mgr.fsiMesh2DState_D->CopyFromH(*m_data_mgr.fsiMesh2DState_H);
    }
}

}  // end namespace fsi
}  // end namespace chrono
