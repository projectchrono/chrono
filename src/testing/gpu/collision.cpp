#include "common.h"
uint numY = 1;
void System::DoTimeStep() {
	if (mNumCurrentObjects < mNumObjects && mFrameNumber % 100 == 0) {
		float x = 10, y = numY, z = 10;
		float posX = 0, posY = 0, posZ = 0;
		srand(1);
		float mass = 1, mu = .5, rest = 0;
		ShapeType type = SPHERE;
		ChBODYSHAREDPTR mrigidBody;
		mNumCurrentObjects += x * y * z;
		int mobjNum = 0;
		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					type = rand() % 3;
					float radius = .5;//(rand()%1000)/3000.0+.05;
					ChVector<> mParticlePos((xx - (x - 1) / 2.0) + posX, (yy) + posY, (zz - (z - 1) / 2.0) + posZ);

					mParticlePos += ChVector<> (rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05);
					ChQuaternion<> quat = ChQuaternion<> (rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000.);
					ChVector<> dim;
					ChVector<> lpos(0, 0, 0);
					quat.Normalize();

					mrigidBody = ChBODYSHAREDPTR(new ChBODY);
					InitObject(mrigidBody, mass, mParticlePos * 1.5, quat, mu, mu, rest, true, false, 0, 1);
					mrigidBody->SetPos_dt(ChVector<> (0, -4, 0));
					switch (type) {
						case SPHERE:
							dim = ChVector<> (radius, 0, 0);
							break;
						case ELLIPSOID:
							dim = ChVector<> (radius * 1.3, radius, radius * 1.3);
							break;
						case BOX:
							dim = ChVector<> (radius, radius, radius);
							break;
						case CYLINDER:
							dim = ChVector<> (radius, radius, radius);
							break;
					}
					AddCollisionGeometry(mrigidBody, type, dim, lpos, quat);
					FinalizeObject(mrigidBody);
					mobjNum++;
				}
			}
		}
	}
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
	GPUSystem->PrintStats();
}

int main(int argc, char* argv[]) {
	omp_set_nested(1);
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 10;
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 500;
	GPUSystem->mTolerance = 1e-3;
	GPUSystem->mOmegaContact = .5;
	GPUSystem->mOmegaBilateral = .2;
	float mMu = .5;
	float mWallMu = .5;

	if (argc == 3) {
		numY = atoi(argv[1]);
		GPUSystem->mUseOGL = atoi(argv[2]);
	} else {
		cout << "ARGS: number of particle layers in y direction | display in OPENGL 1= true 0= false" << endl;
		exit(1);
	}
	float container_R = 10.0, container_T = 1;
	ChQuaternion<> quat(1, 0, 0, 0);
	ChQuaternion<> quat2(1, 0, 0, 0);
	quat2.Q_from_AngAxis(PI / 10.0, ChVector<> (1, 0, 0));
	ChVector<> lpos(0, 0, 0);
	ChBODYSHAREDPTR L = ChBODYSHAREDPTR(new ChBODY);
	ChBODYSHAREDPTR R = ChBODYSHAREDPTR(new ChBODY);
	ChBODYSHAREDPTR F = ChBODYSHAREDPTR(new ChBODY);
	ChBODYSHAREDPTR B = ChBODYSHAREDPTR(new ChBODY);
	ChBODYSHAREDPTR BTM = ChBODYSHAREDPTR(new ChBODY);
	ChBODYSHAREDPTR FREE = ChBODYSHAREDPTR(new ChBODY);

	GPUSystem->InitObject(L, 100000, ChVector<> (-container_R, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(R, 100000, ChVector<> (container_R, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(F, 100000, ChVector<> (0, 0, -container_R), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(B, 100000, ChVector<> (0, 0, container_R), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM, 100000, ChVector<> (0, -container_R * .8, 0), quat2, mWallMu, mWallMu, 0, true, true, -20, -20);

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

	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	return 0;
}
