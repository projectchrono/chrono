// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

//------------------------------------------------------------------------------------
ChFsiInterface::ChFsiInterface(
		FsiBodiesDataH * other_fsiBodiesH,
		ChronoBodiesDataH * other_chronoRigidBackup,
		chrono::ChSystemParallelDVI * other_mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> > * other_fsiBodeisPtr,
		thrust::device_vector<Real3> * other_rigid_FSI_ForcesD,
		thrust::device_vector<Real3> * other_rigid_FSI_TorquesD) :
fsiBodiesH(other_fsiBodiesH),
chronoRigidBackup(other_chronoRigidBackup),
mphysicalSystem(other_mphysicalSystem),
fsiBodeisPtr(other_fsiBodeisPtr),
rigid_FSI_ForcesD(other_rigid_FSI_ForcesD),
rigid_FSI_TorquesD(other_rigid_FSI_TorquesD)
{}
//------------------------------------------------------------------------------------
// FSI_Bodies_Index_H[i] is the the index of the i_th sph represented rigid body in ChSystem
void ChFsiInterface::Add_Rigid_ForceTorques_To_ChSystem() {
	int numRigids = fsiBodeisPtr->size();
	//#pragma omp parallel for // Arman: you can bring it back later, when you have a lot of bodies
	for (int i = 0; i < numRigids; i++) {
		chrono::ChSharedPtr<chrono::ChBody> bodyPtr = (*fsiBodeisPtr)[i];

//		// --------------------------------
//		// Add forces to bodies: Version 1
//		// --------------------------------
//
//		bodyPtr->Empty_forces_accumulators();
//		Real3 mforce = (*rigid_FSI_ForcesD)[i];
//
//		printf("\n\n\n\n\n\n\n rigid forces %e %e %e \n", mforce.x, mforce.y,
//				mforce.z);
//		std::cout << "body name: " << bodyPtr->GetName() << "\n\n\n\n\n";
//		bodyPtr->Empty_forces_accumulators();
//
//		bodyPtr->Accumulate_force(ConvertRealToChVector(mforce),
//				bodyPtr->GetPos(), false);
//
//		Real3 mtorque = (*rigid_FSI_TorquesD)[i];
//		bodyPtr->Accumulate_torque(ConvertRealToChVector(mtorque), false);


		// --------------------------------
		// Add forces to bodies: Version 2
		// --------------------------------

		//	string forceTag("hydrodynamics_force");
		char forceTag[] = "fsi_force";
		char torqueTag[] = "fsi_torque";
		chrono::ChSharedPtr<chrono::ChForce> hydroForce = bodyPtr->SearchForce(
				forceTag);
		chrono::ChSharedPtr<chrono::ChForce> hydroTorque = bodyPtr->SearchForce(
				torqueTag);

		if (hydroForce.IsNull()) {
			hydroForce = chrono::ChSharedPtr<chrono::ChForce>(new chrono::ChForce);
			hydroTorque = chrono::ChSharedPtr<chrono::ChForce>(new chrono::ChForce);

			hydroForce->SetMode(FTYPE_FORCE);
			hydroTorque->SetMode(FTYPE_TORQUE);

			hydroForce->SetName(forceTag);
			hydroTorque->SetName(torqueTag);

			bodyPtr->AddForce(hydroForce);
			bodyPtr->AddForce(hydroTorque);
		}

		chrono::ChVector<> mforce = ConvertRealToChVector((*rigid_FSI_ForcesD)[i]);
		chrono::ChVector<> mtorque = ConvertRealToChVector((*rigid_FSI_TorquesD)[i]);

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
// FSI_Bodies_Index_H[i] is the the index of the i_th sph represented rigid body in ChSystem
void ChFsiInterface::Copy_External_To_ChSystem() {
	int numBodies = mphysicalSystem->Get_bodylist()->size();
	if (chronoRigidBackup->pos_ChSystemH.size() != numBodies) {
		throw std::runtime_error ("Size of the external data does not match the ChSystem !\n");
	}
	//#pragma omp parallel for // Arman: you can bring it back later, when you have a lot of bodies
	for (int i = 0; i < numBodies; i++) {
		auto mBody = mphysicalSystem->Get_bodylist()->at(i);
		mBody->SetPos(ConvertRealToChVector(chronoRigidBackup->pos_ChSystemH[i]));
		mBody->SetPos_dt(ConvertRealToChVector(chronoRigidBackup->vel_ChSystemH[i]));
		mBody->SetPos_dtdt(ConvertRealToChVector(chronoRigidBackup->acc_ChSystemH[i]));

		mBody->SetRot(ConvertToChQuaternion(chronoRigidBackup->quat_ChSystemH[i]));
		mBody->SetWvel_par(ConvertRealToChVector(chronoRigidBackup->omegaVelGRF_ChSystemH[i]));
		chrono::ChVector<> acc = ConvertRealToChVector(chronoRigidBackup->omegaAccGRF_ChSystemH[i]);
		mBody->SetWacc_par(acc);
	}
}
//------------------------------------------------------------------------------------
void ChFsiInterface::Copy_ChSystem_to_External() {
	int numBodies = mphysicalSystem->Get_bodylist()->size();
	chronoRigidBackup->pos_ChSystemH.resize(numBodies);
	chronoRigidBackup->vel_ChSystemH.resize(numBodies);
	chronoRigidBackup->acc_ChSystemH.resize(numBodies);
	chronoRigidBackup->quat_ChSystemH.resize(numBodies);
	chronoRigidBackup->omegaVelGRF_ChSystemH.resize(numBodies);
	chronoRigidBackup->omegaAccGRF_ChSystemH.resize(numBodies);
	//#pragma omp parallel for // Arman: you can bring it back later, when you have a lot of bodies
	for (int i = 0; i < numBodies; i++) {
		auto mBody = mphysicalSystem->Get_bodylist()->at(i);
		chronoRigidBackup->pos_ChSystemH[i] = ConvertChVectorToR3(mBody->GetPos());
		chronoRigidBackup->vel_ChSystemH[i] = ConvertChVectorToR3(mBody->GetPos_dt());
		chronoRigidBackup->acc_ChSystemH[i] = ConvertChVectorToR3(mBody->GetPos_dtdt());

		chronoRigidBackup->quat_ChSystemH[i] = ConvertChQuaternionToR4(mBody->GetRot());
		chronoRigidBackup->omegaVelGRF_ChSystemH[i] = ConvertChVectorToR3(mBody->GetWvel_par());
		chronoRigidBackup->omegaAccGRF_ChSystemH[i] = ConvertChVectorToR3(mBody->GetWacc_par());
	}
}
//------------------------------------------------------------------------------------
// FSI_Bodies_Index_H[i] is the the index of the i_th sph represented rigid body in ChSystem
void ChFsiInterface::Copy_fsiBodies_ChSystem_to_FluidSystem(FsiBodiesDataD * fsiBodiesD) {
	int num_fsiBodies_Rigids = fsiBodeisPtr->size();
	if (posRigid_fsiBodies_D.size() != num_fsiBodies_Rigids
			|| posRigid_fsiBodies_H.size() != num_fsiBodies_Rigids) {
		throw std::runtime_error ("number of fsi bodies that are tracked does not match the array size !\n");
	}
	//#pragma omp parallel for // Arman: you can bring it back later, when you have a lot of bodies
	for (int i = 0; i < num_fsiBodies_Rigids; i++) {
		chrono::ChSharedPtr<chrono::ChBody> bodyPtr = (*fsiBodeisPtr)[i];
		fsiBodiesH->posRigid_fsiBodies_H[i] = ConvertChVectorToR3(bodyPtr->GetPos());
		fsiBodiesH->velMassRigid_fsiBodies_H[i] = ConvertChVectorToR4(bodyPtr->GetPos_dt(), bodyPtr->GetMass());
		fsiBodiesH->accRigid_fsiBodies_H[i] = ConvertChVectorToR3(bodyPtr->GetPos_dtdt());

		fsiBodiesH->q_fsiBodies_H[i] = ConvertChQuaternionToR4(bodyPtr->GetRot());
		fsiBodiesH->rigidOmegaLRF_fsiBodies_H[i] = ConvertChVectorToR3(bodyPtr->GetWvel_loc());
		fsiBodiesH->omegaAccLRF_fsiBodies_H[i] = ConvertChVectorToR3(bodyPtr->GetWacc_loc());
	}

	thrust::copy(fsiBodiesH->posRigid_fsiBodies_H.begin(), fsiBodiesH->posRigid_fsiBodies_H.end(),
			fsiBodiesD->posRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH->velMassRigid_fsiBodies_H.begin(),
				fsiBodiesH->velMassRigid_fsiBodies_H.end(), fsiBodiesD->velMassRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH->accRigid_fsiBodies_H.begin(),
				fsiBodiesH->accRigid_fsiBodies_H.end(), fsiBodiesD->accRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH->q_fsiBodies_H.begin(), fsiBodiesH->q_fsiBodies_H.end(),
			fsiBodiesD->q_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH->rigidOmegaLRF_fsiBodies_H.begin(),
			fsiBodiesH->rigidOmegaLRF_fsiBodies_H.end(), fsiBodiesD->rigidOmegaLRF_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH->omegaAccLRF_fsiBodies_H.begin(),
			fsiBodiesH->omegaAccLRF_fsiBodies_H.end(), fsiBodiesD->omegaAccLRF_fsiBodies_D.begin());
}