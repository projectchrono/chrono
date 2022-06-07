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
// Author: Milad Rakhsha, Arman Pazouki
// =============================================================================
//
// Base class for processing the interface between Chrono and fsi modules
// =============================================================================

#include "chrono_fsi/ChFsiInterface.h"
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
    : sysMBS(mbs),
      sysFSI(fsi),
      paramsH(params),
      fsi_mesh(mesh),
      fsiBodies(bodies),
      fsiNodes(nodes),
      fsiCables(cables),
      fsiShells(shells) {
    size_t numBodies = sysMBS.Get_bodylist().size();
    chronoRigidBackup = chrono_types::make_shared<ChronoBodiesDataH>(numBodies);
    chronoFlexMeshBackup = chrono_types::make_shared<ChronoMeshDataH>(0);
    int numNodes = 0;

    if (sysMBS.Get_otherphysicslist().size())
        numNodes = std::dynamic_pointer_cast<fea::ChMesh>(sysMBS.Get_otherphysicslist().at(0))->GetNnodes();
    chronoFlexMeshBackup = chrono_types::make_shared<ChronoMeshDataH>(numNodes);
}

ChFsiInterface::~ChFsiInterface() {}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//-----------------------Chrono rigid body Specifics----------------------------------
//------------------------------------------------------------------------------------
void ChFsiInterface::Add_Rigid_ForceTorques_To_ChSystem() {
    size_t numRigids = fsiBodies.size();
    std::string delim = ",";
    char filename[4096];
    ChVector<> totalForce(0);
    ChVector<> totalTorque(0);

    for (size_t i = 0; i < numRigids; i++) {
        ChVector<> mforce = ChUtilsTypeConvert::Real3ToChVector(
            ChUtilsDevice::FetchElement(sysFSI.fsiGeneralData->rigid_FSI_ForcesD, i));
        ChVector<> mtorque = ChUtilsTypeConvert::Real3ToChVector(
            ChUtilsDevice::FetchElement(sysFSI.fsiGeneralData->rigid_FSI_TorquesD, i));

        totalForce += mforce;
        totalTorque + mtorque;
        std::shared_ptr<ChBody> body = fsiBodies[i];

        // note: when this FSI body goes back to Chrono system, the gravity
        // will be automaticly added. Here only accumulate force from fluid
        body->Empty_forces_accumulators();
        body->Accumulate_force(mforce, body->GetPos(), false);
        body->Accumulate_torque(mtorque, false);

        // output FSI information into csv files for each body
        // this can be disabled by setting paramsH->output_fsi
        // to false for better IO performance
        if (paramsH->output_fsi){
            ChVector<> pos = body->GetPos();
            ChVector<> vel = body->GetPos_dt();
            ChQuaternion<> rot = body->GetRot();

            sprintf(filename, "%s/FSI_body%zd.csv", paramsH->demo_dir, i);
            std::ofstream file;
            if (sysMBS.GetChTime() > 0)
                file.open(filename, std::fstream::app);
            else {
                file.open(filename);
                file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                     << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << delim << "Fx" << delim
                     << "Fy" << delim << "Fz" << delim << "Tx" << delim << "Ty" << delim << "Tz" << std::endl;
            }

            file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                 << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim
                 << vel.y() << delim << vel.z() << delim << mforce.x() << delim << mforce.y() << delim << mforce.z()
                 << delim << mtorque.x() << delim << mtorque.y() << delim << mtorque.z() << std::endl;
            file.close();
        }
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_External_To_ChSystem() {
    size_t numBodies = sysMBS.Get_bodylist().size();
    if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from Copy_External_To_ChSystem "
            "!\n");
    }

    for (size_t i = 0; i < numBodies; i++) {
        auto mBody = sysMBS.Get_bodylist().at(i);
        mBody->SetPos(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->pos_ChSystemH[i]));
        mBody->SetPos_dt(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->vel_ChSystemH[i]));
        mBody->SetPos_dtdt(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->acc_ChSystemH[i]));

        mBody->SetRot(ChUtilsTypeConvert::Real4ToChQuaternion(chronoRigidBackup->quat_ChSystemH[i]));
        mBody->SetWvel_par(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->omegaVelGRF_ChSystemH[i]));
        ChVector<> acc = ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->omegaAccGRF_ChSystemH[i]);
        mBody->SetWacc_par(acc);
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_ChSystem_to_External() {
    size_t numBodies = sysMBS.Get_bodylist().size();
    auto bodyList = sysMBS.Get_bodylist();

    if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from Copy_ChSystem_to_External "
            "!\n");
    }

    chronoRigidBackup->resize(numBodies);
    for (size_t i = 0; i < numBodies; i++) {
        auto mBody = sysMBS.Get_bodylist().at(i);
        chronoRigidBackup->pos_ChSystemH[i] = ChUtilsTypeConvert::ChVectorToReal3(mBody->GetPos());
        chronoRigidBackup->vel_ChSystemH[i] = ChUtilsTypeConvert::ChVectorToReal3(mBody->GetPos_dt());
        chronoRigidBackup->acc_ChSystemH[i] = ChUtilsTypeConvert::ChVectorToReal3(mBody->GetPos_dtdt());

        chronoRigidBackup->quat_ChSystemH[i] = ChUtilsTypeConvert::ChQuaternionToReal4(mBody->GetRot());
        chronoRigidBackup->omegaVelGRF_ChSystemH[i] = ChUtilsTypeConvert::ChVectorToReal3(mBody->GetWvel_par());
        chronoRigidBackup->omegaAccGRF_ChSystemH[i] = ChUtilsTypeConvert::ChVectorToReal3(mBody->GetWacc_par());
    }

    int numNodes = fsi_mesh->GetNnodes();
    for (size_t i = 0; i < numNodes; i++) {
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(fsi_mesh->GetNode((unsigned int)i));
        chronoFlexMeshBackup->posFlex_ChSystemH_H[i] = ChUtilsTypeConvert::ChVectorToReal3(node->GetPos());
        chronoFlexMeshBackup->velFlex_ChSystemH_H[i] = ChUtilsTypeConvert::ChVectorToReal3(node->GetPos_dt());
        chronoFlexMeshBackup->accFlex_ChSystemH_H[i] = ChUtilsTypeConvert::ChVectorToReal3(node->GetPos_dtdt());
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_fsiBodies_ChSystem_to_FluidSystem(std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    size_t num_fsiBodies_Rigids = fsiBodies.size();
    for (size_t i = 0; i < num_fsiBodies_Rigids; i++) {
        std::shared_ptr<ChBody> bodyPtr = fsiBodies[i];
        sysFSI.fsiBodiesH->posRigid_fsiBodies_H[i] = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetPos());
        sysFSI.fsiBodiesH->velMassRigid_fsiBodies_H[i] =
            ChUtilsTypeConvert::ChVectorToReal4(bodyPtr->GetPos_dt(), bodyPtr->GetMass());
        sysFSI.fsiBodiesH->accRigid_fsiBodies_H[i] = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetPos_dtdt());
        sysFSI.fsiBodiesH->q_fsiBodies_H[i] = ChUtilsTypeConvert::ChQuaternionToReal4(bodyPtr->GetRot());
        sysFSI.fsiBodiesH->omegaVelLRF_fsiBodies_H[i] = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetWvel_loc());
        sysFSI.fsiBodiesH->omegaAccLRF_fsiBodies_H[i] = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetWacc_loc());
    }
    fsiBodiesD->CopyFromH(*sysFSI.fsiBodiesH);
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoBodiesData() {
    size_t numBodies = sysMBS.Get_bodylist().size();
    chronoRigidBackup->resize(numBodies);
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//-----------------------Chrono FEA Specifics-----------------------------------------
//------------------------------------------------------------------------------------
void ChFsiInterface::Add_Flex_Forces_To_ChSystem() {
    std::string delim = ",";
    size_t numNodes = fsiNodes.size();
    ChVector<> total_force(0, 0, 0);
    for (size_t i = 0; i < numNodes; i++) {
        ChVector<> mforce = ChUtilsTypeConvert::Real3ToChVector(
            ChUtilsDevice::FetchElement(sysFSI.fsiGeneralData->Flex_FSI_ForcesD, i));
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(fsi_mesh->GetNode((unsigned int)i));
        node->SetForce(mforce);

        // output FSI information into csv files for each node
        // this can be disabled by setting paramsH->output_fsi
        // to false for better IO performance
        if (paramsH->output_fsi){
            ChVector<> pos = node->GetPos();
            ChVector<> vel = node->GetPos_dt();
            char filename[4096];
            sprintf(filename, "%s/FSI_node%zd.csv", paramsH->demo_dir, i);
            std::ofstream file;
            if (sysMBS.GetChTime() > 0)
                file.open(filename, std::fstream::app);
            else {
                file.open(filename);
                file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "Vx" << delim << "Vy" << delim
                     << "Vz" << delim << "Fx" << delim << "Fy" << delim << "Fz" << std::endl;
            }

            file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                 << vel.x() << delim << vel.y() << delim << vel.z() << delim << mforce.x() << delim << mforce.y() << delim
                 << mforce.z() << std::endl;
            file.close();
        }
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_fsiNodes_ChSystem_to_FluidSystem(std::shared_ptr<FsiMeshDataD> FsiMeshD) {
    size_t num_fsiNodes_Felx = fsiNodes.size();

    for (size_t i = 0; i < num_fsiNodes_Felx; i++) {
        auto NodePtr = fsiNodes[i];
        sysFSI.fsiMeshH->pos_fsi_fea_H[i] = ChUtilsTypeConvert::ChVectorToReal3(NodePtr->GetPos());
        sysFSI.fsiMeshH->vel_fsi_fea_H[i] = ChUtilsTypeConvert::ChVectorToReal3(NodePtr->GetPos_dt());
        sysFSI.fsiMeshH->acc_fsi_fea_H[i] = ChUtilsTypeConvert::ChVectorToReal3(NodePtr->GetPos_dtdt());
    }
    FsiMeshD->CopyFromH(*sysFSI.fsiMeshH);
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoNodesData() {
    int numNodes = 0;
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (sysMBS.Get_otherphysicslist().size()) {
        printf("fsi_mesh.size in ResizeChronNodesData  %d\n", numNodes);
    }
    numNodes = fsi_mesh->GetNnodes();
    printf("numNodes in ResizeChronNodesData  %d\n", numNodes);
    chronoFlexMeshBackup->resize(numNodes);
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoFEANodesData() {
    int numNodes = 0;
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (sysMBS.Get_otherphysicslist().size()) {
        my_mesh = std::dynamic_pointer_cast<fea::ChMesh>(sysMBS.Get_otherphysicslist().at(0));
    }
    numNodes = fsi_mesh->GetNnodes();
    printf("fsi_mesh.size in ResizeChronoFEANodesData  %d\n", numNodes);
    printf("numNodes in ResizeChronoFEANodeData  %d\n", numNodes);

    chronoFlexMeshBackup->resize(numNodes);
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoCablesData(std::vector<std::vector<int>> CableElementsNodesSTDVector) {
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (sysMBS.Get_otherphysicslist().size()) {
        my_mesh = std::dynamic_pointer_cast<fea::ChMesh>(sysMBS.Get_otherphysicslist().at(0));
    }

    size_t numCables = 0;
    for (size_t i = 0; i < fsi_mesh->GetNelements(); i++) {
        if (std::dynamic_pointer_cast<fea::ChElementCableANCF>(fsi_mesh->GetElement((unsigned int)i)))
            numCables++;
    }
    printf("fsi_mesh.size.shell in ResizeChronoCablesData  %zd\n", numCables);
    printf("numCables in ResizeChronoCablesData  %zd\n", numCables);
    printf("CableElementsNodesSTDVector.size() in ResizeChronoCablesData  %zd\n", CableElementsNodesSTDVector.size());

    if (CableElementsNodesSTDVector.size() != numCables) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from ChFsiInterface::ResizeChronoCableData "
            "!\n");
    }

    // CableElementsNodesH is the elements connectivity
    // Important: in CableElementsNodesH[i][j] j index starts from 1 not zero
    // This is because of how the GMF files are read in Chrono
    sysFSI.fsiGeneralData->CableElementsNodesH.resize(numCables);
    for (size_t i = 0; i < numCables; i++) {
        sysFSI.fsiGeneralData->CableElementsNodesH[i].x = CableElementsNodesSTDVector[i][0];
        sysFSI.fsiGeneralData->CableElementsNodesH[i].y = CableElementsNodesSTDVector[i][1];
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoShellsData(std::vector<std::vector<int>> ShellElementsNodesSTDVector) {
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (sysMBS.Get_otherphysicslist().size()) {
        my_mesh = std::dynamic_pointer_cast<fea::ChMesh>(sysMBS.Get_otherphysicslist().at(0));
    }

    int numShells = 0;
    for (unsigned int i = 0; i < fsi_mesh->GetNelements(); i++) {
        if (std::dynamic_pointer_cast<fea::ChElementShellANCF_3423>(fsi_mesh->GetElement(i)))
            numShells++;
    }

    printf("numShells in ResizeChronoShellsData  %d\n", numShells);
    printf("ShellElementsNodesSTDVector.size() in ResizeChronoShellsData  %zd\n", ShellElementsNodesSTDVector.size());

    if (ShellElementsNodesSTDVector.size() != numShells) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from ChFsiInterface::ResizeChronoShellsData "
            "!\n");
    }

    // ShellElementsNodesH is the elements connectivity
    // Important: in ShellElementsNodesH[i][j] j index starts from 1 not zero
    // This is because of how the GMF files are read in Chrono
    sysFSI.fsiGeneralData->ShellElementsNodesH.resize(numShells);
    for (size_t i = 0; i < numShells; i++) {
        sysFSI.fsiGeneralData->ShellElementsNodesH[i].x = ShellElementsNodesSTDVector[i][0];
        sysFSI.fsiGeneralData->ShellElementsNodesH[i].y = ShellElementsNodesSTDVector[i][1];
        sysFSI.fsiGeneralData->ShellElementsNodesH[i].z = ShellElementsNodesSTDVector[i][2];
        sysFSI.fsiGeneralData->ShellElementsNodesH[i].w = ShellElementsNodesSTDVector[i][3];
    }

    //  (*ShellelementsNodes).resize(numShells);
    //  thrust::copy(ShellelementsNodes_H.begin(), ShellelementsNodes_H.end(), (*ShellElementsNodes).begin());
}

}  // end namespace fsi
}  // end namespace chrono
