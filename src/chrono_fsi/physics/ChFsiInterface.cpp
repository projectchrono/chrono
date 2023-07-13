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

#include "chrono_fsi/physics/ChFsiInterface.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fsi {

ChFsiInterface::ChFsiInterface(ChSystemFsi_impl& fsi, std::shared_ptr<SimParams> params)
    : ChFsiBase(params, nullptr), m_sysFSI(fsi), m_verbose(true) {}

ChFsiInterface::~ChFsiInterface() {}

//-----------------------Chrono rigid body Specifics----------------------------------

void ChFsiInterface::ApplyBodyForce_Fsi2Chrono() {
    size_t numRigids = m_fsi_bodies.size();

    thrust::host_vector<Real3> forcesH = m_sysFSI.fsiData->rigid_FSI_ForcesD;
    thrust::host_vector<Real3> torquesH = m_sysFSI.fsiData->rigid_FSI_TorquesD;

    for (size_t i = 0; i < numRigids; i++) {
        ChVector<> mforce = utils::ToChVector(forcesH[i]);
        ChVector<> mtorque = utils::ToChVector(torquesH[i]);

        std::shared_ptr<ChBody> body = m_fsi_bodies[i];

        // note: when this FSI body goes back to Chrono system, the gravity
        // will be automaticly added. Here only accumulate force from fluid
        body->Empty_forces_accumulators();
        body->Accumulate_force(mforce, body->GetPos(), false);
        body->Accumulate_torque(mtorque, false);
    }
}

void ChFsiInterface::LoadBodyState_Chrono2Fsi(std::shared_ptr<FsiBodyStateD> fsiBodyStateD) {
    size_t num_fsiBodies_Rigids = m_fsi_bodies.size();
    for (size_t i = 0; i < num_fsiBodies_Rigids; i++) {
        std::shared_ptr<ChBody> bodyPtr = m_fsi_bodies[i];
        m_sysFSI.fsiBodyStateH->pos[i] = utils::ToReal3(bodyPtr->GetPos());
        m_sysFSI.fsiBodyStateH->lin_vel[i] = utils::ToReal4(bodyPtr->GetPos_dt(), bodyPtr->GetMass());
        m_sysFSI.fsiBodyStateH->lin_acc[i] = utils::ToReal3(bodyPtr->GetPos_dtdt());
        m_sysFSI.fsiBodyStateH->rot[i] = utils::ToReal4(bodyPtr->GetRot());
        m_sysFSI.fsiBodyStateH->ang_vel[i] = utils::ToReal3(bodyPtr->GetWvel_loc());
        m_sysFSI.fsiBodyStateH->ang_acc[i] = utils::ToReal3(bodyPtr->GetWacc_loc());
    }
    fsiBodyStateD->CopyFromH(*m_sysFSI.fsiBodyStateH);
}

//-----------------------Chrono FEA Specifics-----------------------------------------

void ChFsiInterface::ApplyMesh1DForce_Fsi2Chrono() {
    // Transfer to host
    thrust::host_vector<Real3> forces_H = m_sysFSI.fsiData->flex1D_FSIforces_D;

    // Apply to FEA nodes
    int counter = 0;
    for (const auto& fsi_mesh : m_fsi_meshes1D) {
        int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
        for (int i = 0; i < num_nodes; i++) {
            const auto& node = fsi_mesh.ind2ptr_map.at(i);
            node->SetForce(utils::ToChVector(forces_H[counter]));
            counter++;
        }
    }
}

void ChFsiInterface::ApplyMesh2DForce_Fsi2Chrono() {
    // Transfer to host
    thrust::host_vector<Real3> forces_H = m_sysFSI.fsiData->flex2D_FSIforces_D;

    // Apply to FEA nodes
    int counter = 0;
    for (const auto& fsi_mesh : m_fsi_meshes2D) {
        int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
        for (int i = 0; i < num_nodes; i++) {
            const auto& node = fsi_mesh.ind2ptr_map.at(i);
            node->SetForce(utils::ToChVector(forces_H[counter]));
            counter++;
        }
    }
}

void ChFsiInterface::LoadMesh1DState_Chrono2Fsi(std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    // Load from FEA nodes on host
    int counter = 0;
    for (const auto& fsi_mesh : m_fsi_meshes1D) {
        int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
        for (int i = 0; i < num_nodes; i++) {
            const auto& node = fsi_mesh.ind2ptr_map.at(i);
            m_sysFSI.fsiMesh1DState_H->pos_fsi_fea_H[counter] = utils::ToReal3(node->GetPos());
            m_sysFSI.fsiMesh1DState_H->vel_fsi_fea_H[counter] = utils::ToReal3(node->GetPos_dt());
            m_sysFSI.fsiMesh1DState_H->acc_fsi_fea_H[counter] = utils::ToReal3(node->GetPos_dtdt());
            counter++;
        }
    }

    // Transfer to device
    fsiMesh1DState_D->CopyFromH(*m_sysFSI.fsiMesh1DState_H);
}

void ChFsiInterface::LoadMesh2DState_Chrono2Fsi(std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    // Load from FEA nodes on host
    int counter = 0;
    for (const auto& fsi_mesh : m_fsi_meshes2D) {
        int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
        for (int i = 0; i < num_nodes; i++) {
            const auto& node = fsi_mesh.ind2ptr_map.at(i);
            m_sysFSI.fsiMesh2DState_H->pos_fsi_fea_H[counter] = utils::ToReal3(node->GetPos());
            m_sysFSI.fsiMesh2DState_H->vel_fsi_fea_H[counter] = utils::ToReal3(node->GetPos_dt());
            m_sysFSI.fsiMesh2DState_H->acc_fsi_fea_H[counter] = utils::ToReal3(node->GetPos_dtdt());
            counter++;
        }
    }

    // Transfer to device
    fsiMesh2DState_D->CopyFromH(*m_sysFSI.fsiMesh2DState_H);
}

}  // end namespace fsi
}  // end namespace chrono
