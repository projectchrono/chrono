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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
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

ChFsiInterface::ChFsiInterface(ChSystem& mbs,
                               ChSystemFsi_impl& fsi,
                               std::shared_ptr<SimParams> params,
                               std::shared_ptr<fea::ChMesh>& mesh,
                               std::vector<std::shared_ptr<ChBody>>& bodies,
                               std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& nodes,
                               std::vector<std::shared_ptr<fea::ChElementCableANCF>>& cables,
                               std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& shells)
    : m_sysMBS(mbs),
      m_sysFSI(fsi),
      m_paramsH(params),
      m_fsi_mesh(mesh),
      m_fsi_bodies(bodies),
      m_fsi_nodes(nodes),
      m_fsi_cables(cables),
      m_fsi_shells(shells),
      m_verbose(true) {
    size_t numBodies = m_sysMBS.Get_bodylist().size();
    m_rigid_backup = chrono_types::make_shared<ChronoBodiesDataH>(numBodies);
    m_flex_backup = chrono_types::make_shared<ChronoMeshDataH>(0);
    size_t numNodes = 0;

    if (m_sysMBS.Get_otherphysicslist().size())
        numNodes = std::dynamic_pointer_cast<fea::ChMesh>(m_sysMBS.Get_otherphysicslist().at(0))->GetNnodes();
    m_flex_backup = chrono_types::make_shared<ChronoMeshDataH>(numNodes);
}

ChFsiInterface::~ChFsiInterface() {}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//-----------------------Chrono rigid body Specifics----------------------------------
//------------------------------------------------------------------------------------
void ChFsiInterface::Add_Rigid_ForceTorques_To_ChSystem() {
    size_t numRigids = m_fsi_bodies.size();

    for (size_t i = 0; i < numRigids; i++) {
        ChVector<> mforce = utils::ToChVector(
            ChUtilsDevice::FetchElement(m_sysFSI.fsiGeneralData->rigid_FSI_ForcesD, i));
        ChVector<> mtorque = utils::ToChVector(
            ChUtilsDevice::FetchElement(m_sysFSI.fsiGeneralData->rigid_FSI_TorquesD, i));

        std::shared_ptr<ChBody> body = m_fsi_bodies[i];

        // note: when this FSI body goes back to Chrono system, the gravity
        // will be automaticly added. Here only accumulate force from fluid
        body->Empty_forces_accumulators();
        body->Accumulate_force(mforce, body->GetPos(), false);
        body->Accumulate_torque(mtorque, false);
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_External_To_ChSystem() {
    size_t numBodies = m_sysMBS.Get_bodylist().size();
    size_t numNodes = m_fsi_mesh->GetNnodes();

    if (m_rigid_backup->pos_ChSystemH.size() != numBodies || 
        m_flex_backup->posFlex_ChSystemH_H.size() != numNodes) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from Copy_External_To_ChSystem "
            "!\n");
    }

    // Copy data between ChSystem and FSI system for rigid bodies
    for (size_t i = 0; i < numBodies; i++) {
        auto mBody = m_sysMBS.Get_bodylist().at(i);
        mBody->SetPos(utils::ToChVector(m_rigid_backup->pos_ChSystemH[i]));
        mBody->SetPos_dt(utils::ToChVector(m_rigid_backup->vel_ChSystemH[i]));
        mBody->SetPos_dtdt(utils::ToChVector(m_rigid_backup->acc_ChSystemH[i]));

        mBody->SetRot(utils::ToChQuaternion(m_rigid_backup->quat_ChSystemH[i]));
        mBody->SetWvel_par(utils::ToChVector(m_rigid_backup->omegaVelGRF_ChSystemH[i]));
        mBody->SetWacc_par(utils::ToChVector(m_rigid_backup->omegaAccGRF_ChSystemH[i]));
    }

    // Copy data between ChSystem and FSI system for flexible bodies
    m_flex_backup->resize(numNodes);
    for (size_t i = 0; i < numNodes; i++) {
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(m_fsi_mesh->GetNode((unsigned int)i));
        node->SetPos(utils::ToChVector(m_flex_backup->posFlex_ChSystemH_H[i]));
        node->SetPos_dt(utils::ToChVector(m_flex_backup->velFlex_ChSystemH_H[i]));
        node->SetPos_dtdt(utils::ToChVector(m_flex_backup->accFlex_ChSystemH_H[i]));
        node->SetD(utils::ToChVector(m_flex_backup->dirFlex_ChSystemH_H[i]));
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_ChSystem_to_External() {
    size_t numBodies = m_sysMBS.Get_bodylist().size();
    size_t numNodes = m_fsi_mesh->GetNnodes();

    if (m_rigid_backup->pos_ChSystemH.size() != numBodies || 
        m_flex_backup->posFlex_ChSystemH_H.size() != numNodes) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from Copy_ChSystem_to_External "
            "!\n");
    }

    // Copy data between ChSystem and FSI system for rigid bodies
    m_rigid_backup->resize(numBodies);
    for (size_t i = 0; i < numBodies; i++) {
        auto mBody = m_sysMBS.Get_bodylist().at(i);
        m_rigid_backup->pos_ChSystemH[i] = utils::ToReal3(mBody->GetPos());
        m_rigid_backup->vel_ChSystemH[i] = utils::ToReal3(mBody->GetPos_dt());
        m_rigid_backup->acc_ChSystemH[i] = utils::ToReal3(mBody->GetPos_dtdt());

        m_rigid_backup->quat_ChSystemH[i] = utils::ToReal4(mBody->GetRot());
        m_rigid_backup->omegaVelGRF_ChSystemH[i] = utils::ToReal3(mBody->GetWvel_par());
        m_rigid_backup->omegaAccGRF_ChSystemH[i] = utils::ToReal3(mBody->GetWacc_par());
    }

    // Copy data between ChSystem and FSI system for flexible bodies
    m_flex_backup->resize(numNodes);
    for (size_t i = 0; i < numNodes; i++) {
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(m_fsi_mesh->GetNode((unsigned int)i));
        m_flex_backup->posFlex_ChSystemH_H[i] = utils::ToReal3(node->GetPos());
        m_flex_backup->velFlex_ChSystemH_H[i] = utils::ToReal3(node->GetPos_dt());
        m_flex_backup->accFlex_ChSystemH_H[i] = utils::ToReal3(node->GetPos_dtdt());
        m_flex_backup->dirFlex_ChSystemH_H[i] = utils::ToReal3(node->GetD());
    }
}
//------------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoBodiesData() {
    size_t numBodies = m_sysMBS.Get_bodylist().size();
    m_rigid_backup->resize(numBodies);
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//-----------------------Chrono FEA Specifics-----------------------------------------
//------------------------------------------------------------------------------------
void ChFsiInterface::Add_Flex_Forces_To_ChSystem() {
    size_t numNodes = m_fsi_nodes.size();

    for (size_t i = 0; i < numNodes; i++) {
        ChVector<> mforce = utils::ToChVector(
            ChUtilsDevice::FetchElement(m_sysFSI.fsiGeneralData->Flex_FSI_ForcesD, i));
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(m_fsi_mesh->GetNode((unsigned int)i));
        node->SetForce(mforce);
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_FsiNodes_ChSystem_to_FsiSystem(std::shared_ptr<FsiMeshDataD> FsiMeshD) {
    size_t num_fsiNodes_Felx = m_fsi_nodes.size();

    for (size_t i = 0; i < num_fsiNodes_Felx; i++) {
        auto NodePtr = m_fsi_nodes[i];
        m_sysFSI.fsiMeshH->pos_fsi_fea_H[i] = utils::ToReal3(NodePtr->GetPos());
        m_sysFSI.fsiMeshH->vel_fsi_fea_H[i] = utils::ToReal3(NodePtr->GetPos_dt());
        m_sysFSI.fsiMeshH->acc_fsi_fea_H[i] = utils::ToReal3(NodePtr->GetPos_dtdt());
        m_sysFSI.fsiMeshH->dir_fsi_fea_H[i] = utils::ToReal3(NodePtr->GetD());
    }
    FsiMeshD->CopyFromH(*m_sysFSI.fsiMeshH);
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoFEANodesData() {
    size_t numNodes = m_fsi_mesh->GetNnodes();
    m_flex_backup->resize(numNodes);
}
//------------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------------
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
    // Important: in ShellElementsNodesH[i][j] j index starts from 1 not zero
    // This is because of how the GMF files are read in Chrono
    m_sysFSI.fsiGeneralData->ShellElementsNodesH.resize(numShells);
    for (size_t i = 0; i < numShells; i++) {
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].x = ShellElementsNodesSTDVector[i][0];
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].y = ShellElementsNodesSTDVector[i][1];
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].z = ShellElementsNodesSTDVector[i][2];
        m_sysFSI.fsiGeneralData->ShellElementsNodesH[i].w = ShellElementsNodesSTDVector[i][3];
    }

    //  (*ShellElementsNodesD).resize(numShells);
    //  thrust::copy(ShellelementsNodes_H.begin(), ShellelementsNodes_H.end(), (*m_fea_shell_nodes).begin());
}

}  // end namespace fsi
}  // end namespace chrono
