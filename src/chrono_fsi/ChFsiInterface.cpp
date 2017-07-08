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
// Author: Arman Pazouki
// =============================================================================
//
// Base class for processing the interface between chrono and fsi modules
// =============================================================================

#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/ChFsiTypeConvert.h"

namespace chrono {
namespace fsi {
//------------------------------------------------------------------------------------
ChFsiInterface::ChFsiInterface(
    FsiBodiesDataH *other_fsiBodiesH, chrono::ChSystem *other_mphysicalSystem,
    std::vector<std::shared_ptr<chrono::ChBody>> *other_fsiBodeisPtr,
    thrust::device_vector<Real3> *other_rigid_FSI_ForcesD,
    thrust::device_vector<Real3> *other_rigid_FSI_TorquesD)
    : fsiBodiesH(other_fsiBodiesH), mphysicalSystem(other_mphysicalSystem),
      fsiBodeisPtr(other_fsiBodeisPtr),
      rigid_FSI_ForcesD(other_rigid_FSI_ForcesD),
      rigid_FSI_TorquesD(other_rigid_FSI_TorquesD) {
  int numBodies = mphysicalSystem->Get_bodylist()->size();
  chronoRigidBackup = new ChronoBodiesDataH(numBodies);

  printf("** size chronoRigidBackup %d \n ",
         chronoRigidBackup->pos_ChSystemH.size());
}
//------------------------------------------------------------------------------------
ChFsiInterface::~ChFsiInterface() {
  // TODO
}
//------------------------------------------------------------------------------------
// FSI_Bodies_Index_H[i] is the the index of the i_th sph represented rigid body
// in ChSystem
void ChFsiInterface::Add_Rigid_ForceTorques_To_ChSystem() {
  int numRigids = fsiBodeisPtr->size();
  //#pragma omp parallel for // Arman: you can bring it back later, when you
  //have a lot of bodies
  for (int i = 0; i < numRigids; i++) {
    auto bodyPtr = (*fsiBodeisPtr)[i];

    //		// --------------------------------
    //		// Add forces to bodies: Version 1
    //		// --------------------------------
    //
    //		bodyPtr->Empty_forces_accumulators();
    //		Real3 mforce = (*rigid_FSI_ForcesD)[i];
    //
    //		printf("\n\n\n\n\n\n\n rigid forces %e %e %e \n", mforce.x,
    //mforce.y,
    //				mforce.z);
    //		std::cout << "body name: " << bodyPtr->GetName() <<
    //"\n\n\n\n\n";
    //		bodyPtr->Empty_forces_accumulators();
    //
    //		bodyPtr->Accumulate_force(ChFsiTypeConvert::Real3ToChVector(mforce),
    //				bodyPtr->GetPos(), false);
    //
    //		Real3 mtorque = (*rigid_FSI_TorquesD)[i];
    //		bodyPtr->Accumulate_torque(ChFsiTypeConvert::Real3ToChVector(mtorque),
    //false);

    // --------------------------------
    // Add forces to bodies: Version 2
    // --------------------------------

    //	string forceTag("hydrodynamics_force");
    char forceTag[] = "fsi_force";
    char torqueTag[] = "fsi_torque";
    auto hydroForce = bodyPtr->SearchForce(forceTag);
    auto hydroTorque = bodyPtr->SearchForce(torqueTag);

    if (!hydroForce) {
      hydroForce = std::make_shared<chrono::ChForce>();
      hydroTorque = std::make_shared<chrono::ChForce>();

      hydroForce->SetMode(ChForce::FORCE);
      hydroTorque->SetMode(ChForce::TORQUE);

      hydroForce->SetName(forceTag);
      hydroTorque->SetName(torqueTag);

      bodyPtr->AddForce(hydroForce);
      bodyPtr->AddForce(hydroTorque);
    }

    chrono::ChVector<> mforce =
        ChFsiTypeConvert::Real3ToChVector((*rigid_FSI_ForcesD)[i]);
    chrono::ChVector<> mtorque =
        ChFsiTypeConvert::Real3ToChVector((*rigid_FSI_TorquesD)[i]);

    hydroForce->SetVpoint(bodyPtr->GetPos());
    hydroForce->SetMforce(mforce.Length());
    mforce.Normalize();
    hydroForce->SetDir(mforce);

    hydroTorque->SetMforce(mtorque.Length());
    mtorque.Normalize();
    hydroTorque->SetDir(mtorque);
  }
}
//------------------------------------------------------------------------------------
// FSI_Bodies_Index_H[i] is the the index of the i_th sph represented rigid body
// in ChSystem
void ChFsiInterface::Copy_External_To_ChSystem() {
  int numBodies = mphysicalSystem->Get_bodylist()->size();
  if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
    throw std::runtime_error("Size of the external data does not match the "
                             "ChSystem; thrown from Copy_External_To_ChSystem "
                             "!\n");
  }
  //#pragma omp parallel for // Arman: you can bring it back later, when you
  //have a lot of bodies
  for (int i = 0; i < numBodies; i++) {
    auto mBody = mphysicalSystem->Get_bodylist()->at(i);
    mBody->SetPos(
        ChFsiTypeConvert::Real3ToChVector(chronoRigidBackup->pos_ChSystemH[i]));
    mBody->SetPos_dt(
        ChFsiTypeConvert::Real3ToChVector(chronoRigidBackup->vel_ChSystemH[i]));
    mBody->SetPos_dtdt(
        ChFsiTypeConvert::Real3ToChVector(chronoRigidBackup->acc_ChSystemH[i]));

    mBody->SetRot(ChFsiTypeConvert::Real4ToChQuaternion(
        chronoRigidBackup->quat_ChSystemH[i]));
    mBody->SetWvel_par(ChFsiTypeConvert::Real3ToChVector(
        chronoRigidBackup->omegaVelGRF_ChSystemH[i]));
    chrono::ChVector<> acc = ChFsiTypeConvert::Real3ToChVector(
        chronoRigidBackup->omegaAccGRF_ChSystemH[i]);
    mBody->SetWacc_par(acc);
  }
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_ChSystem_to_External() {
  //	// Arman, assume no change in chrono num bodies. the resize is done in
  //initializaiton.
  int numBodies = mphysicalSystem->Get_bodylist()->size();
  if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
    throw std::runtime_error("Size of the external data does not match the "
                             "ChSystem; thrown from Copy_ChSystem_to_External "
                             "!\n");
  }
  //	chronoRigidBackup->resize(numBodies);
  //#pragma omp parallel for // Arman: you can bring it back later, when you
  //have a lot of bodies
  for (int i = 0; i < numBodies; i++) {
    auto mBody = mphysicalSystem->Get_bodylist()->at(i);
    chronoRigidBackup->pos_ChSystemH[i] =
        ChFsiTypeConvert::ChVectorToReal3(mBody->GetPos());
    chronoRigidBackup->vel_ChSystemH[i] =
        ChFsiTypeConvert::ChVectorToReal3(mBody->GetPos_dt());
    chronoRigidBackup->acc_ChSystemH[i] =
        ChFsiTypeConvert::ChVectorToReal3(mBody->GetPos_dtdt());

    chronoRigidBackup->quat_ChSystemH[i] =
        ChFsiTypeConvert::ChQuaternionToReal4(mBody->GetRot());
    chronoRigidBackup->omegaVelGRF_ChSystemH[i] =
        ChFsiTypeConvert::ChVectorToReal3(mBody->GetWvel_par());
    chronoRigidBackup->omegaAccGRF_ChSystemH[i] =
        ChFsiTypeConvert::ChVectorToReal3(mBody->GetWacc_par());
  }
}
//------------------------------------------------------------------------------------
// FSI_Bodies_Index_H[i] is the the index of the i_th sph represented rigid body
// in ChSystem
void ChFsiInterface::Copy_fsiBodies_ChSystem_to_FluidSystem(
    FsiBodiesDataD *fsiBodiesD) {
  //#pragma omp parallel for // Arman: you can bring it back later, when you
  //have a lot of bodies
  int num_fsiBodies_Rigids = fsiBodeisPtr->size();
  for (int i = 0; i < num_fsiBodies_Rigids; i++) {
    auto bodyPtr = (*fsiBodeisPtr)[i];
    fsiBodiesH->posRigid_fsiBodies_H[i] =
        ChFsiTypeConvert::ChVectorToReal3(bodyPtr->GetPos());
    fsiBodiesH->velMassRigid_fsiBodies_H[i] =
        ChFsiTypeConvert::ChVectorRToReal4(bodyPtr->GetPos_dt(),
                                           bodyPtr->GetMass());
    fsiBodiesH->accRigid_fsiBodies_H[i] =
        ChFsiTypeConvert::ChVectorToReal3(bodyPtr->GetPos_dtdt());

    fsiBodiesH->q_fsiBodies_H[i] =
        ChFsiTypeConvert::ChQuaternionToReal4(bodyPtr->GetRot());
    fsiBodiesH->omegaVelLRF_fsiBodies_H[i] =
        ChFsiTypeConvert::ChVectorToReal3(bodyPtr->GetWvel_loc());
    fsiBodiesH->omegaAccLRF_fsiBodies_H[i] =
        ChFsiTypeConvert::ChVectorToReal3(bodyPtr->GetWacc_loc());
  }
  fsiBodiesD->CopyFromH(*fsiBodiesH);
}
//------------------------------------------------------------------------------------
void ChFsiInterface::ResizeChronoBodiesData() {
  int numBodies = mphysicalSystem->Get_bodylist()->size();
  chronoRigidBackup->resize(numBodies);
}

} // end namespace fsi
} // end namespace chrono
