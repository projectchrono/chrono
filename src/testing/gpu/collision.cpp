#include "common.h"
uint numY = 1;
void System::DoTimeStep() {
	if (mNumCurrentObjects < mNumObjects && mFrameNumber % 100 == 0) {
		float x = 10, y = 10, z = 10;
		float posX = 0, posY = 0, posZ = 0;
		srand(1);
		float mass = 1, mu = .5, rest = 0;
		ShapeType type = SPHERE;
		CHBODYSHAREDPTR mrigidBody;
		mNumCurrentObjects += x * y * z;
		int mobjNum = 0;
		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					type = SPHERE;//rand() % 3;
					float radius = .1;//(rand()%1000)/3000.0+.05;
					ChVector<> mParticlePos((xx - (x - 1) / 2.0) + posX, (yy) + posY, (zz - (z - 1) / 2.0) + posZ);

					///mParticlePos += ChVector<> (rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05);
					ChQuaternion<> quat = ChQuaternion<> (rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000.);
					ChVector<> dim;
					ChVector<> lpos(0, 0, 0);
					quat.Normalize();

					mrigidBody = CHBODYSHAREDPTR(new CHBODY);
					InitObject(mrigidBody, mass, mParticlePos * .2, quat, mu, mu, rest, true, false, 0, 1);

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
//cout<<mobjNum<<endl;
					mrigidBody->SetPos_dt(ChVector<> (0, -4, 0));
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
	omp_set_dynamic(true);
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 10;
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 100;
	GPUSystem->mTolerance = 1e-3;
	GPUSystem->mOmegaContact = .5;
	GPUSystem->mOmegaBilateral = .2;
	float mMu = .5;
	float mWallMu = .5;
	numY = 100;
	GPUSystem->mUseOGL =1;
//	if (argc == 3) {
//
//		GPUSystem->mUseOGL = atoi(argv[2]);
//	} else {
//		cout << "ARGS: number of particle layers in y direction | display in OPENGL 1= true 0= false" << endl;
//		exit(1);
//	}
	float container_R = 10.0, container_T = 1;
	ChQuaternion<> quat(1, 0, 0, 0);
	ChQuaternion<> quat2(1, 0, 0, 0);
	quat2.Q_from_AngAxis(PI / 10.0, ChVector<> (1, 0, 0));
	ChVector<> lpos(0, 0, 0);
	CHBODYSHAREDPTR L = CHBODYSHAREDPTR(new CHBODY);
	CHBODYSHAREDPTR R = CHBODYSHAREDPTR(new CHBODY);
	CHBODYSHAREDPTR F = CHBODYSHAREDPTR(new CHBODY);
	CHBODYSHAREDPTR B = CHBODYSHAREDPTR(new CHBODY);
	CHBODYSHAREDPTR BTM = CHBODYSHAREDPTR(new CHBODY);
	CHBODYSHAREDPTR FREE = CHBODYSHAREDPTR(new CHBODY);

	GPUSystem->InitObject(L, 1, ChVector<>(-20, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(R, 1, ChVector<>(20, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(F, 1, ChVector<>(0, 0, -20), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(B, 1, ChVector<>(0, 0, 20), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM, 1, ChVector<>(0, -20, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(FREE, 1, ChVector<>(0, 20, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

	GPUSystem->AddCollisionGeometry(L, BOX, ChVector<>(1, 20, 20), lpos, quat);
	GPUSystem->AddCollisionGeometry(R, BOX, ChVector<>(1, 20, 20), lpos, quat);
	GPUSystem->AddCollisionGeometry(F, BOX, ChVector<>(20, 20, 1), lpos, quat);
	GPUSystem->AddCollisionGeometry(B, BOX, ChVector<>(20, 20, 1), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM, BOX, ChVector<>(20, 1, 20), lpos, quat);
	GPUSystem->AddCollisionGeometry(FREE, BOX, ChVector<>(20, 1, 20), lpos, quat);

	GPUSystem->FinalizeObject(L);
	GPUSystem->FinalizeObject(R);
	GPUSystem->FinalizeObject(F);
	GPUSystem->FinalizeObject(B);
	GPUSystem->FinalizeObject(BTM);
	GPUSystem->FinalizeObject(FREE);
	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	return 0;
}
