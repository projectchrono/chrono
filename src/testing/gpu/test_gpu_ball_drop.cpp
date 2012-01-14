#include "common.h"
//---------------------------Constraints--------------------------------
//--------------------------Global_Objects------------------------------
ChSharedBodyGPUPtr Ball;
//---------------------------SolverParams-------------------------------
//-----------------------------SimParams--------------------------------

//-----------------------------GranularParams---------------------------
float particle_radius = .0035;
float particle_rho = 1560;
float particle_friction_ang = 23;
int particle_max = 1;
float container_width = .250;
float container_thickness = .03;
//-----------------------------CODE--------------------------------
float force = 0;
string ballFile = "ball_data_1.txt";

void System::DoTimeStep() {
	if (mNumCurrentObjects < mNumObjects && mFrameNumber % 500 == 0) {
		float x = 60, y = 20, z = 60;
		float posX = 0, posY = -30, posZ = 0;
		float radius = particle_radius, mass = particle_rho * 4.0 / 3.0 * PI * radius * radius * radius, mu = tan(particle_friction_ang * PI / 180.0), rest = 0;
		ShapeType type = SPHERE;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects += x * y * z;
		int mobjNum = 0;

		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					ChVector<> mParticlePos((xx - (x - 1) / 2.0) + posX, (yy) + posY, (zz - (z - 1) / 2.0) + posZ);

					mParticlePos += ChVector<> (rand() % 1000 / 1000.0 - .05, rand() % 1000 / 1000.0 - .05, rand() % 1000 / 1000.0 - .05);
					ChQuaternion<> quat = ChQuaternion<> (rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000.);
					ChVector<> dim;
					ChVector<> lpos(0, 0, 0);
					quat.Normalize();
					mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
					InitObject(mrigidBody, mass, mParticlePos * .007, quat, mu, mu, rest, true, false, 0, 1);
					mrigidBody->SetPos_dt(ChVector<> (0, -.2, 0));
					ChMatrix33<> mat(quat);
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
	//SaveByObject(Ball.get_ptr(), ballFile);
	DeactivationPlane(-container_width - container_thickness * 2, 0, false);
		if (mFrameNumber % (int(1.0 / mTimeStep / 5.0)) == 0) {
			SaveAllData("data/ball");
		}
	if (mCurrentTime >= 4.0) {

		//Ball->SetBodyFixed(false);
		//Ball->SetPos(ChVector<> (0, 0, 0));
	}
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
}

int main(int argc, char* argv[]) {
	//omp_set_nested(1);
	int gpu= atoi(argv[3]);

	GPUSystem = new System(gpu);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 30;
	GPUSystem->mNumObjects = particle_max;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 1e-10;
	GPUSystem->mOmegaContact = 1.0;
	GPUSystem->mOmegaBilateral = .2;
	GPUSystem->SetTimingFile("ball_1.txt");
	SCALE = .1;
	float mMu = 1;
	float mWallMu = 1;

	GPUSystem->mUseOGL = atoi(argv[1]);
	GPUSystem->mSaveData = atoi(argv[2]);

	GPUSystem->Setup();

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM1 = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM2 = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FREE = ChSharedBodyGPUPtr(new ChBodyGPU);
	float wscale = 1;

	GPUSystem->InitObject(L, 100000, ChVector<> (-container_width * wscale, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(R, 100000, ChVector<> (container_width * wscale, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(F, 100000, ChVector<> (0, 0, -container_width), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(B, 100000, ChVector<> (0, 0, container_width), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM, 100000, ChVector<> (0, -container_width, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM1, 100000, ChVector<> (0, -container_width, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM2, 100000, ChVector<> (0, -container_width, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

	GPUSystem->AddCollisionGeometry(L, BOX, ChVector<> (container_thickness, container_width, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(R, BOX, ChVector<> (container_thickness, container_width, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(F, BOX, ChVector<> (container_width * wscale, container_width, container_thickness), lpos, quat);
	GPUSystem->AddCollisionGeometry(B, BOX, ChVector<> (container_width * wscale, container_width, container_thickness), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM, BOX, ChVector<> (container_width * wscale, container_thickness, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM1, BOX, ChVector<> (container_width * wscale, container_thickness, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM2, BOX, ChVector<> (container_width * wscale, container_thickness, container_width), lpos, quat);

	GPUSystem->FinalizeObject(L);
	GPUSystem->FinalizeObject(R);
	GPUSystem->FinalizeObject(F);
	GPUSystem->FinalizeObject(B);
	GPUSystem->FinalizeObject(BTM);
	GPUSystem->FinalizeObject(BTM1);
	GPUSystem->FinalizeObject(BTM2);
	//GPUSystem->LoadSpheres("ball150.txt", 7, particle_rho * 4.0 / 3.0 * PI * particle_radius * particle_radius * particle_radius, particle_radius, tan(particle_friction_ang * PI / 180.0));
//	Ball = ChSharedBodyGPUPtr(new ChBodyGPU);
//	GPUSystem->InitObject(Ball, .2291, ChVector<> (0, 0, 0), quat, .7, .7, 0, true, false, -10, -10);
//	GPUSystem->AddCollisionGeometry(Ball, SPHERE, ChVector<> (.0191, 0, 0), lpos, quat);
//	GPUSystem->FinalizeObject(Ball);
//	Ball->SetPos_dt(ChVector<> (0, -4.4, 0));
//	Ball->SetBodyFixed(true);

	SimulationLoop(argc, argv);

	return 0;
}

