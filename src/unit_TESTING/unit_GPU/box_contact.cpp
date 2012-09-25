#include "common.h"

void System::DoTimeStep() {

	if (mNumCurrentObjects < mNumObjects && mFrameNumber % 50 == 0) {
		float x = 1, y = 1, z = 1;
		float scalef = .2;
		float posX = 0, posY = 2, posZ = 0;
		float scaler = 1;
		/*if(mFrameNumber==0){
		 x = 5;
		 y = 1;
		 z = 5;
		 scalef=1.6;
		 posY=0.5;
		 scaler=fminf(rand()%400/20.0+5,25);
		 }*/

		float radius = (rand() % 10000) / 100000.0;
		float mass = .01;
		float mu = 1;
		float rest = 0;
		ShapeType type = SPHERE;

		ChBODYSHAREDPTR mrigidBody;
		mNumCurrentObjects += x * y * z;
		int mobjNum = 0;

		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					int t = 0;//(rand() % 4);
					//		if(mFrameNumber==0){t=0;}

					float radius = fmaxf((rand() % 10000) / 130000.0, .05) * scaler;
					ChVector<> mParticlePos((xx - (x - 1) / 2.0), (yy), (zz - (z - 1) / 2.0));

					//mParticlePos += ChVector<>(rand() % 1000 / 1000.0 - .5, rand() % 1000 / 1000.0 - .5, rand() % 1000 / 1000.0 - .5);
					ChQuaternion<> quat = ChQuaternion<>(rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000.);
					ChVector<> dim;
					ChVector<> lpos(0, 0, 0);
					quat.Normalize();
					mrigidBody = ChBODYSHAREDPTR(new ChBODY);
					InitObject(mrigidBody, mass, mParticlePos * scalef + ChVector<>(posX, posY, posZ), quat, mu, mu, rest, true, false, 0, 1);

					ChMatrix33<> mat(quat);
					switch (t) {
					case 0:
						dim = ChVector<>(radius, 0, 0);
						type = SPHERE;
						break;
					case 1:
						dim = ChVector<>(radius * fmaxf((rand() % 100 + 15) / 75.0, .5), radius, radius * fmaxf((rand() % 100 + 15) / 75.0, .5));
						type = ELLIPSOID;
						break;
					case 2:
						dim = ChVector<>(radius * fmaxf((rand() % 100 + 15) / 75.0, .5), radius, radius * fmaxf((rand() % 100 + 15) / 75.0, .5));
						type = BOX;
						break;
					case 3:
						dim = ChVector<>(radius, radius * fmaxf((rand() % 100 + 15) / 75.0, .5), radius);
						type = CYLINDER;
						break;
					}
					AddCollisionGeometry(mrigidBody, type, dim, lpos, ChQuaternion<>(1, 0, 0, 0));
					FinalizeObject(mrigidBody);
					mrigidBody->SetPos_dt(ChVector<>(0, -1, 0));
					mobjNum++;
				}
			}
		}
	}

	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
	GPUSystem->PrintStats();
	//cout << endl;

//	for (int i = 0; i < mSystem->gpu_data_manager->host_norm_data.size(); i++) {
//		float3 N = mSystem->gpu_data_manager->host_norm_data[i];
//		float3 Pa = mSystem->gpu_data_manager->host_cpta_data[i];
//		float3 Pb = mSystem->gpu_data_manager->host_cptb_data[i];
//		float D = mSystem->gpu_data_manager->host_dpth_data[i];
//		int2 ID= mSystem->gpu_data_manager->host_bids_data[i];
//
//		printf("[%f %f %f] [%f %f %f] [%f %f %f] [%f] [%d %d]\n",Pa.x,Pa.y,Pa.z, Pb.x,Pb.y,Pb.z,N.x,N.y,N.z, D, ID.x, ID.y);
//	}

}

int main(int argc, char* argv[]) {
	omp_set_nested(1);
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 10;
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 1e-8;
	GPUSystem->mOmegaContact = .6;
	GPUSystem->mOmegaBilateral = .2;
	stepMode = true;
	float mass = .01, mu = .5, rest = 0;
	GPUSystem->mUseOGL = 1;
	SCALE = .1;

	ChQuaternion<> quat(1, .1, 0, 0);
	quat.Normalize();
	ChVector<> lpos(0, 0, 0);

	ChBODYSHAREDPTR BTM = ChBODYSHAREDPTR(new ChBODY);
	//GPUSystem->InitObject(BTM, .001, ChVector<>(0, 0, 0), quat, mu, mu, rest, true, true, -20, -20);
	////GPUSystem->AddCollisionGeometry(BTM, BOX, ChVector<>(15, .1, 15), lpos, quat);
	//GPUSystem->FinalizeObject(BTM);

	float x = 0.0, y = 0.0, z = 0.0;

	ChBODYSHAREDPTR mrigidBody;

	double L = .9075;
	double R = .39;
	double H1 = .8;
	double H2 = .739;
	double D = 1.692;
	double W = .15;

	mrigidBody = ChBODYSHAREDPTR(new ChBODY);
	GPUSystem->InitObject(mrigidBody, mass, ChVector<>(x, y, z), quat, mu, mu, rest, true, true, 0, 1);
	GPUSystem->AddCollisionGeometry(mrigidBody, CYLINDER, ChVector<>(R, D * .5, R), ChVector<>(L, 0, 0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X));
	//GPUSystem->AddCollisionGeometry(mrigidBody, ELLIPSOID, ChVector<>(R, D * .5, R), ChVector<>(-L, 0, 0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X));
	//GPUSystem->AddCollisionGeometry(mrigidBody, BOX, ChVector<>((L + 2 * R), (H1 + H2) * .5, R * .5), ChVector<>(0, H1 - (H1 + H2) * .5, D - R * .5), quat);
	//GPUSystem->AddCollisionGeometry(mrigidBody, BOX, ChVector<>((L + 2 * R), (H1 + H2) * .5, R * .5), ChVector<>(0, H1 - (H1 + H2) * .5, -D + R * .5), quat);
	GPUSystem->AddCollisionGeometry(mrigidBody, BOX, ChVector<>((L + 2 * R), W, D), ChVector<>(0, -H2 + W, 0), quat);
	GPUSystem->FinalizeObject(mrigidBody);
	((ChLcpSolverGPU*) (GPUSystem->mSystem->GetLcpSolverSpeed()))->SetContactFactor(.5);
	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	return 0;
}

