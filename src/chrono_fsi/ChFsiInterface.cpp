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
// Base class for processing the interface between chrono and fsi modules
// =============================================================================

#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fsi {
//------------------------------------------------------------------------------------
ChFsiInterface::ChFsiInterface(chrono::ChSystem& other_mphysicalSystem,
                               std::shared_ptr<fea::ChMesh> other_fsiMesh,
                               std::shared_ptr<SimParams> other_paramsH,
                               std::shared_ptr<FsiBodiesDataH> other_fsiBodiesH,
                               std::shared_ptr<FsiMeshDataH> other_fsiMeshH,
                               std::vector<std::shared_ptr<chrono::ChBody>>& other_fsiBodeis,
                               std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& other_fsiNodes,
                               std::vector<std::shared_ptr<fea::ChElementCableANCF>>& other_fsiCables,
                               std::vector<std::shared_ptr<fea::ChElementShellANCF>>& other_fsiShells,
                               thrust::host_vector<int2>& other_CableElementsNodesH,
                               thrust::device_vector<int2>& other_CableElementsNodes,
                               thrust::host_vector<int4>& other_ShellElementsNodesH,
                               thrust::device_vector<int4>& other_ShellElementsNodes,
                               thrust::device_vector<Real3>& other_rigid_FSI_ForcesD,
                               thrust::device_vector<Real3>& other_rigid_FSI_TorquesD,
                               thrust::device_vector<Real3>& other_Flex_FSI_ForcesD)
    : mphysicalSystem(other_mphysicalSystem),
      fsi_mesh(other_fsiMesh),
      paramsH(other_paramsH),
      fsiBodiesH(other_fsiBodiesH),
      fsiMeshH(other_fsiMeshH),
      fsiBodeis(other_fsiBodeis),
      fsiNodes(other_fsiNodes),
      fsiCables(other_fsiCables),
      fsiShells(other_fsiShells),
      CableElementsNodesH(other_CableElementsNodesH),
      CableElementsNodes(other_CableElementsNodes),
      ShellElementsNodesH(other_ShellElementsNodesH),
      ShellElementsNodes(other_ShellElementsNodes),
      rigid_FSI_ForcesD(other_rigid_FSI_ForcesD),
      rigid_FSI_TorquesD(other_rigid_FSI_TorquesD),
      Flex_FSI_ForcesD(other_Flex_FSI_ForcesD) {
    size_t numBodies = mphysicalSystem.Get_bodylist().size();
    chronoRigidBackup = chrono_types::make_shared<ChronoBodiesDataH>(numBodies);
    chronoFlexMeshBackup = chrono_types::make_shared<ChronoMeshDataH>(0);
    int numShells = 0;
    int numCables = 0;
    int numNodes = 0;
    if (mphysicalSystem.Get_otherphysicslist().size())
        numShells = std::dynamic_pointer_cast<fea::ChMesh>(mphysicalSystem.Get_otherphysicslist().at(0))->GetNelements();
    if (mphysicalSystem.Get_otherphysicslist().size())
        numNodes = std::dynamic_pointer_cast<fea::ChMesh>(mphysicalSystem.Get_otherphysicslist().at(0))->GetNnodes();
    chronoFlexMeshBackup = chrono_types::make_shared<ChronoMeshDataH>(numNodes);
}
//------------------------------------------------------------------------------------
ChFsiInterface::~ChFsiInterface() {}
//------------------------------------------------------------------------------------

void ChFsiInterface::Add_Rigid_ForceTorques_To_ChSystem() {
    size_t numRigids = fsiBodeis.size();
    std::string delim = ",";
    char filename[4096];
    ChVector<> totalForce(0);
    ChVector<> totalTorque(0);

    for (size_t i = 0; i < numRigids; i++) {
        chrono::ChVector<> mforce = ChUtilsTypeConvert::Real3ToChVector(ChUtilsDevice::FetchElement(rigid_FSI_ForcesD, i));
        chrono::ChVector<> mtorque = ChUtilsTypeConvert::Real3ToChVector(ChUtilsDevice::FetchElement(rigid_FSI_TorquesD, i));

        totalForce += mforce;
        totalTorque + mtorque;
        std::shared_ptr<chrono::ChBody> body = fsiBodeis[i];
        ChVector<> pos = body->GetPos();
        ChVector<> vel = body->GetPos_dt();
        ChQuaternion<> rot = body->GetRot();

        sprintf(filename, "%s/FS_body%zd.csv", paramsH->demo_dir, i);
        std::ofstream file;
        if (mphysicalSystem.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << delim << "Fx" << delim
                 << "Fy" << delim << "Fz" << delim << "Tx" << delim << "Ty" << delim << "Tz" << std::endl;
        }

        if (1) // set as 1 if output to file
            file << mphysicalSystem.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                 << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim
                 << vel.y() << delim << vel.z() << delim << mforce.x() << delim << mforce.y() << delim << mforce.z()
                 << delim << mtorque.x() << delim << mtorque.y() << delim << mtorque.z() << std::endl;

        if (0) // set as 1 if output to screen
            std::cout << mphysicalSystem.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z()
                      << delim << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim
                      << vel.x() << delim << vel.y() << delim << vel.z() << delim << mforce.x() << delim << mforce.y()
                      << delim << mforce.z() << delim << mtorque.x() << delim << mtorque.y() << delim << mtorque.z()
                      << std::endl;

        // note: when this FSI body goes back to Chrono system, the gravity
        // will be automaticly added. Here only accumulate force from fluid
        body->Empty_forces_accumulators(); 
        body->Accumulate_force(mforce, body->GetPos(), false);
        body->Accumulate_torque(mtorque, false);

        file.close();
    }
}

//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_External_To_ChSystem() {
    size_t numBodies = mphysicalSystem.Get_bodylist().size();
    if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from Copy_External_To_ChSystem "
            "!\n");
    }
    
    for (size_t i = 0; i < numBodies; i++) {
        auto mBody = mphysicalSystem.Get_bodylist().at(i);
        mBody->SetPos(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->pos_ChSystemH[i]));
        mBody->SetPos_dt(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->vel_ChSystemH[i]));
        mBody->SetPos_dtdt(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->acc_ChSystemH[i]));

        mBody->SetRot(ChUtilsTypeConvert::Real4ToChQuaternion(chronoRigidBackup->quat_ChSystemH[i]));
        mBody->SetWvel_par(ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->omegaVelGRF_ChSystemH[i]));
        chrono::ChVector<> acc = ChUtilsTypeConvert::Real3ToChVector(chronoRigidBackup->omegaAccGRF_ChSystemH[i]);
        mBody->SetWacc_par(acc);
    }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_ChSystem_to_External() {
    size_t numBodies = mphysicalSystem.Get_bodylist().size();
    auto bodyList = mphysicalSystem.Get_bodylist();

    if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from Copy_ChSystem_to_External "
            "!\n");
    }

    chronoRigidBackup->resize(numBodies);
    for (size_t i = 0; i < numBodies; i++) {
        auto mBody = mphysicalSystem.Get_bodylist().at(i);
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
    size_t num_fsiBodies_Rigids = fsiBodeis.size();
    for (size_t i = 0; i < num_fsiBodies_Rigids; i++) {
        std::shared_ptr<ChBody> bodyPtr = fsiBodeis[i];
        fsiBodiesH->posRigid_fsiBodies_H[i]     = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetPos());
        fsiBodiesH->velMassRigid_fsiBodies_H[i] = ChUtilsTypeConvert::ChVectorRToReal4(bodyPtr->GetPos_dt(), bodyPtr->GetMass());
        fsiBodiesH->accRigid_fsiBodies_H[i]     = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetPos_dtdt());
        fsiBodiesH->q_fsiBodies_H[i]            = ChUtilsTypeConvert::ChQuaternionToReal4(bodyPtr->GetRot());
        fsiBodiesH->omegaVelLRF_fsiBodies_H[i]  = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetWvel_loc());
        fsiBodiesH->omegaAccLRF_fsiBodies_H[i]  = ChUtilsTypeConvert::ChVectorToReal3(bodyPtr->GetWacc_loc());
    }
    fsiBodiesD->CopyFromH(*fsiBodiesH);
}

//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoBodiesData() {
    size_t numBodies = mphysicalSystem.Get_bodylist().size();
    chronoRigidBackup->resize(numBodies);
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//-----------------------Chrono FEA Specifics------------------------------------------
//------------------------------------------------------------------------------------
void ChFsiInterface::Add_Flex_Forces_To_ChSystem() {
    int numShells = 0;
    int numNodes_ChSystem = 0;
    std::string delim = ",";

    size_t numNodes = fsiNodes.size();
    chrono::ChVector<> total_force(0, 0, 0);
    for (size_t i = 0; i < numNodes; i++) {
        chrono::ChVector<> mforce =
            ChUtilsTypeConvert::Real3ToChVector(ChUtilsDevice::FetchElement(Flex_FSI_ForcesD, i));
        //        if (mforce.Length() != 0.0)
        //            printf("mforce= (%.3e,%.3e,%.3e)\n", mforce.x(), mforce.y(), mforce.z());
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(fsi_mesh->GetNode((unsigned int)i));

        ChVector<> pos = node->GetPos();
        ChVector<> vel = node->GetPos_dt();
        char filename[4096];
        sprintf(filename, "%s/FS_node%zd.csv", paramsH->demo_dir, i);
        std::ofstream file;
        if (mphysicalSystem.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "Vx" << delim << "Vy" << delim
                 << "Vz" << delim << "Fx" << delim << "Fy" << delim << "Fz" << std::endl;
        }

        file << mphysicalSystem.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
             << vel.x() << delim << vel.y() << delim << vel.z() << delim << mforce.x() << delim << mforce.y() << delim
             << mforce.z() << std::endl;
        file.close();

        //    ChVector<> OldForce = node->GetForce();
        node->SetForce(mforce);
    }

    //    printf("Total Force from the fluid to the solid = (%.3e,%.3e,%.3e)\n", total_force.x(), total_force.y(),
    //           total_force.z());
}
//------------------------------------------------------------------------------------

void ChFsiInterface::Copy_fsiNodes_ChSystem_to_FluidSystem(std::shared_ptr<FsiMeshDataD> FsiMeshD) {
    size_t num_fsiNodes_Felx = fsiNodes.size();

    for (size_t i = 0; i < num_fsiNodes_Felx; i++) {
        auto NodePtr = fsiNodes[i];
        fsiMeshH->pos_fsi_fea_H[i] = ChUtilsTypeConvert::ChVectorToReal3(NodePtr->GetPos());
        fsiMeshH->vel_fsi_fea_H[i] = ChUtilsTypeConvert::ChVectorToReal3(NodePtr->GetPos_dt());
        fsiMeshH->acc_fsi_fea_H[i] = ChUtilsTypeConvert::ChVectorToReal3(NodePtr->GetPos_dtdt());
    }
    FsiMeshD->CopyFromH(*fsiMeshH);
}
//------------------------------------------------------------------------------------

void ChFsiInterface::ResizeChronoNodesData() {
    int numNodes = 0;
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (mphysicalSystem.Get_otherphysicslist().size()) {
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
    if (mphysicalSystem.Get_otherphysicslist().size()) {
        my_mesh = std::dynamic_pointer_cast<fea::ChMesh>(mphysicalSystem.Get_otherphysicslist().at(0));
    }
    numNodes = fsi_mesh->GetNnodes();
    printf("fsi_mesh.size in ResizeChronoFEANodesData  %d\n", numNodes);
    printf("numNodes in ResizeChronoFEANodeData  %d\n", numNodes);

    chronoFlexMeshBackup->resize(numNodes);
}
//------------------------------------------------------------------------------------

void ChFsiInterface::ResizeChronoCablesData(std::vector<std::vector<int>> CableElementsNodesSTDVector,
                                            thrust::host_vector<int2>& CableElementsNodesH) {
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (mphysicalSystem.Get_otherphysicslist().size()) {
        my_mesh = std::dynamic_pointer_cast<fea::ChMesh>(mphysicalSystem.Get_otherphysicslist().at(0));
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

    //  ShellElementsNodesH is the elements connectivity
    // Important: in CableElementsNodesH[i][j] j index starts from 1 not zero
    // This is because of how the GMF files are read in chrono
    CableElementsNodesH.resize(numCables);
    for (size_t i = 0; i < numCables; i++) {
        CableElementsNodesH[i].x = CableElementsNodesSTDVector[i][0];
        CableElementsNodesH[i].y = CableElementsNodesSTDVector[i][1];
    }
}
//------------------------------------------------------------------------------------

void ChFsiInterface::ResizeChronoShellsData(std::vector<std::vector<int>> ShellElementsNodesSTDVector,
                                            thrust::host_vector<int4>& ShellElementsNodesH) {
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    if (mphysicalSystem.Get_otherphysicslist().size()) {
        my_mesh = std::dynamic_pointer_cast<fea::ChMesh>(mphysicalSystem.Get_otherphysicslist().at(0));
    }

    int numShells = 0;
    for (unsigned int i = 0; i < fsi_mesh->GetNelements(); i++) {
        if (std::dynamic_pointer_cast<fea::ChElementShellANCF>(fsi_mesh->GetElement(i)))
            numShells++;
    }

    printf("\n\n numShells in ResizeChronoShellsData  %d\n", numShells);
    printf("ShellElementsNodesSTDVector.size() in ResizeChronoShellsData  %zd\n", ShellElementsNodesSTDVector.size());

    if (ShellElementsNodesSTDVector.size() != numShells) {
        throw std::runtime_error(
            "Size of the external data does not match the "
            "ChSystem; thrown from ChFsiInterface::ResizeChronoShellsData "
            "!\n");
    }

    // ShellElementsNodesH is the elements connectivity
    // Important: in ShellElementsNodesH[i][j] j index starts from 1 not zero
    // This is because of how the GMF files are read in chrono
    ShellElementsNodesH.resize(numShells);
    for (size_t i = 0; i < numShells; i++) {
        ShellElementsNodesH[i].x = ShellElementsNodesSTDVector[i][0];
        ShellElementsNodesH[i].y = ShellElementsNodesSTDVector[i][1];
        ShellElementsNodesH[i].z = ShellElementsNodesSTDVector[i][2];
        ShellElementsNodesH[i].w = ShellElementsNodesSTDVector[i][3];
    }

    //  (*ShellelementsNodes).resize(numShells);
    //  thrust::copy(ShellelementsNodes_H.begin(), ShellelementsNodes_H.end(), (*ShellElementsNodes).begin());
}

}  // end namespace fsi
}  // end namespace chrono
