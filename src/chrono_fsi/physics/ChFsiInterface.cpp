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
    : m_sysFSI(fsi), m_paramsH(params), m_verbose(true) {}

ChFsiInterface::~ChFsiInterface() {}

//-----------------------Chrono rigid body Specifics----------------------------------

void ChFsiInterface::Add_Rigid_ForceTorques_To_ChSystem() {
    size_t numRigids = m_fsi_bodies.size();

    thrust::host_vector<Real3> forcesH = m_sysFSI.fsiGeneralData->rigid_FSI_ForcesD;
    thrust::host_vector<Real3> torquesH = m_sysFSI.fsiGeneralData->rigid_FSI_TorquesD;

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

void ChFsiInterface::Copy_FsiBodies_ChSystem_to_FsiSystem(std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    size_t num_fsiBodies_Rigids = m_fsi_bodies.size();
    for (size_t i = 0; i < num_fsiBodies_Rigids; i++) {
        std::shared_ptr<ChBody> bodyPtr = m_fsi_bodies[i];
        m_sysFSI.fsiBodiesH->posRigid_fsiBodies_H[i] = utils::ToReal3(bodyPtr->GetPos());
        m_sysFSI.fsiBodiesH->velMassRigid_fsiBodies_H[i] =
            utils::ToReal4(bodyPtr->GetPos_dt(), bodyPtr->GetMass());
        m_sysFSI.fsiBodiesH->accRigid_fsiBodies_H[i] = utils::ToReal3(bodyPtr->GetPos_dtdt());
        m_sysFSI.fsiBodiesH->q_fsiBodies_H[i] = utils::ToReal4(bodyPtr->GetRot());
        m_sysFSI.fsiBodiesH->omegaVelLRF_fsiBodies_H[i] = utils::ToReal3(bodyPtr->GetWvel_loc());
        m_sysFSI.fsiBodiesH->omegaAccLRF_fsiBodies_H[i] = utils::ToReal3(bodyPtr->GetWacc_loc());
    }
    fsiBodiesD->CopyFromH(*m_sysFSI.fsiBodiesH);
}

//-----------------------Chrono FEA Specifics-----------------------------------------

void ChFsiInterface::Add_Flex_Forces_To_ChSystem() {
    size_t num_nodes = m_fsi_nodes.size();

    thrust::host_vector<Real3> forcesH = m_sysFSI.fsiGeneralData->Flex_FSI_ForcesD;

    for (size_t i = 0; i < num_nodes; i++) {
        ChVector<> force = utils::ToChVector(forcesH[i]);
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(m_fsi_mesh->GetNode((unsigned int)i));
        node->SetForce(force);
    }
}

void ChFsiInterface::Copy_FsiNodes_ChSystem_to_FsiSystem(std::shared_ptr<FsiMeshDataD> FsiMeshD) {
    size_t num_nodes = m_fsi_nodes.size();

    for (size_t i = 0; i < num_nodes; i++) {
        const auto& node = m_fsi_nodes[i];
        m_sysFSI.fsiMeshH->pos_fsi_fea_H[i] = utils::ToReal3(node->GetPos());
        m_sysFSI.fsiMeshH->vel_fsi_fea_H[i] = utils::ToReal3(node->GetPos_dt());
        m_sysFSI.fsiMeshH->acc_fsi_fea_H[i] = utils::ToReal3(node->GetPos_dtdt());
        m_sysFSI.fsiMeshH->dir_fsi_fea_H[i] = utils::ToReal3(node->GetD());
    }
    FsiMeshD->CopyFromH(*m_sysFSI.fsiMeshH);
}

void ChFsiInterface::ResizeChronoCablesData(const std::vector<std::vector<int>>& CableElementsNodesSTDVector) {
    size_t numCables = 0;
    for (size_t i = 0; i < m_fsi_mesh->GetNelements(); i++) {
        if (std::dynamic_pointer_cast<fea::ChElementCableANCF>(m_fsi_mesh->GetElement((unsigned int)i)))
            numCables++;
    }
    if (m_verbose) {
        printf("numCables in ResizeChronoCablesData  %zd\n", numCables);
        printf("CableElementsNodesSTDVector.size() in ResizeChronoCablesData  %zd\n",
               CableElementsNodesSTDVector.size());
    }

    if (CableElementsNodesSTDVector.size() != numCables) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from ChFsiInterface::ResizeChronoCableData "
            "!\n");
    }

    // CableElementsNodesH is the elements connectivity
    // Important: in CableElementsNodesH[i][j] j index starts from 1 not zero
    // This is because of how the GMF files are read in Chrono
    m_sysFSI.fsiGeneralData->CableElementsNodesH.resize(numCables);
    for (size_t i = 0; i < numCables; i++) {
        m_sysFSI.fsiGeneralData->CableElementsNodesH[i].x = CableElementsNodesSTDVector[i][0];
        m_sysFSI.fsiGeneralData->CableElementsNodesH[i].y = CableElementsNodesSTDVector[i][1];
    }
}
 
void ChFsiInterface::ResizeChronoShellsData(const std::vector<std::vector<int>>& ShellElementsNodesSTDVector) {
    size_t numShells = 0;
    for (unsigned int i = 0; i < m_fsi_mesh->GetNelements(); i++) {
        if (std::dynamic_pointer_cast<fea::ChElementShellANCF_3423>(m_fsi_mesh->GetElement(i)))
            numShells++;
    }

    if (m_verbose) {
        printf("numShells in ResizeChronoShellsData  %zd\n", numShells);
        printf("ShellElementsNodesSTDVector.size() in ResizeChronoShellsData  %zd\n",
               ShellElementsNodesSTDVector.size());
    }

    if (ShellElementsNodesSTDVector.size() != numShells) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from ChFsiInterface::ResizeChronoShellsData "
            "!\n");
    }

    // ShellElementsNodesH is the elements connectivity
    m_sysFSI.fsiGeneralData->ShellElementsNodesH.resize(numShells);
    for (size_t i = 0; i < numShells; i++) {
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].x = ShellElementsNodesSTDVector[i][0];
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].y = ShellElementsNodesSTDVector[i][1];
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].z = ShellElementsNodesSTDVector[i][2];
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].w = ShellElementsNodesSTDVector[i][3];
    }
}

}  // end namespace fsi
}  // end namespace chrono
