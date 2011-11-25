#include "common.h"
void System::DoTimeStep() {
	if (mNumCurrentObjects < mNumObjects && mFrameNumber % 5000 == 0) {
		float x = 30, y = 30, z = 300;
		float posX = 0, posY = -14, posZ = 0;
		float radius = .1, mass = 4, mu = .5, rest = 0;
		ShapeType type = SPHERE;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects += x * y * z;
		int mobjNum = 0;

		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					ChVector<> mParticlePos((xx - (x - 1) / 2.0) + posX, (yy) + posY, (zz - (z - 1) / 2.0) + posZ);

					mParticlePos += ChVector<> (rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05);
					ChQuaternion<> quat = ChQuaternion<> (rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000.);
					ChVector<> dim;
					ChVector<> lpos(0, 0, 0);
					quat.Normalize();

					mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
					InitObject(mrigidBody, mass, mParticlePos * .5, quat, mu, mu, rest, true, false, 0, 1);
					mrigidBody->SetPos_dt(ChVector<> (0, -.2, 0));
					switch (type) {
					case SPHERE:
						dim = ChVector<> (radius, 0, 0);
					case ELLIPSOID:
						dim = ChVector<> (radius * 1.3, radius, radius * 1.3);
					case BOX:
						dim = ChVector<> (radius, radius, radius);
					case CYLINDER:
						dim = ChVector<> (radius, radius, radius);
					}
					AddCollisionGeometry(mrigidBody, type, dim, lpos, quat);
					FinalizeObject(mrigidBody);
					mobjNum++;
				}
			}
		}
	}

	//DeactivationPlane(-40, -40, true);
	//BoundingBox(container_R,container_R,container_R);

	//SaveByID(1, "test_ball.txt", true, true, true, false, false);
	//SaveAllData("data/ball_drop",true, false, false, true, false);

	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
}

int main(int argc, char* argv[]) {
	GPUSystem = new System(0);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 10;
	GPUSystem->mNumObjects = 3000;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 1e-5;
	GPUSystem->mOmegaContact = 1.0;
	GPUSystem->mOmegaBilateral = .2;
	float mMu = .5;
	float mWallMu = .5;

	if (argc == 3) {
		GPUSystem->mUseOGL = atoi(argv[1]);
		GPUSystem->mSaveData = atoi(argv[2]);
	}else if(argc == 1) {
		GPUSystem->mUseOGL=0;
		GPUSystem->mSaveData=0;
	}
	float container_R = 10.0, container_T = 1;
	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FREE = ChSharedBodyGPUPtr(new ChBodyGPU);

	GPUSystem->InitObject(L, 100000, ChVector<> (-container_R, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(R, 100000, ChVector<> (container_R, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(F, 100000, ChVector<> (0, 0, -container_R), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(B, 100000, ChVector<> (0, 0, container_R), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM, 100000, ChVector<> (0, -container_R, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

	GPUSystem->AddCollisionGeometry(L, BOX, ChVector<> (container_T, container_R, container_R), lpos, quat);
	GPUSystem->AddCollisionGeometry(R, BOX, ChVector<> (container_T, container_R, container_R), lpos, quat);
	GPUSystem->AddCollisionGeometry(F, BOX, ChVector<> (container_R, container_R, container_T), lpos, quat);
	GPUSystem->AddCollisionGeometry(B, BOX, ChVector<> (container_R, container_R, container_T), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM, BOX, ChVector<> (container_R, container_T, container_R), lpos, quat);

	GPUSystem->FinalizeObject(L);
	GPUSystem->FinalizeObject(R);
	GPUSystem->FinalizeObject(F);
	GPUSystem->FinalizeObject(B);
	GPUSystem->FinalizeObject(BTM);

	/*GPUSystem->InitObject(FREE, 1, ChVector<> (0, 0, 0), quat, mWallMu, mWallMu, 0, true, false, -10, -10);
	GPUSystem->AddCollisionGeometry(FREE, BOX, ChVector<> (1, .1, 1), lpos, quat);
	GPUSystem->FinalizeObject(FREE);

	ChSharedPtr<ChLinkEngine> rotational_motor = ChSharedPtr<ChLinkEngine> (new ChLinkEngine);
	ChSharedBodyPtr ptr1 = ChSharedBodyPtr(BTM);
	ChSharedBodyPtr ptr2 = ChSharedBodyPtr(FREE);

	ChFrame<> f1(ChVector<> (0, 0, 0), Q_from_AngAxis(0, ChVector<> (1, 0, 0)));
	ChFrame<> f2(ChVector<> (0, 0, 0), Q_from_AngAxis(0, ChVector<> (1, 0, 0)));
	rotational_motor->Initialize(ptr1, ptr2, ChCoordsys<> (ChVector<> (1.5, 1, -1), chrono::Q_from_AngAxis(CH_C_PI / 2.0, VECT_Y)));

	rotational_motor->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
	rotational_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);

	GPUSystem->mSystem->AddLink(rotational_motor);
	ChFunction_Const* mfun = (ChFunction_Const*) rotational_motor->Get_tor_funct();
	mfun->Set_yconst(CH_C_PI * 2.0);*/

	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	return 0;
}