#include "common.h"

void System::DoTimeStep() {
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
	//GPUSystem->PrintStats();

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
	GPUSystem->mIterations = 2000;
	GPUSystem->mTolerance = 1e-5;
	GPUSystem->mOmegaContact = .1;
	GPUSystem->mOmegaBilateral = .2;
	stepMode = true;
	float mass = .01, mu = .5, rest = 0;
	GPUSystem->mUseOGL = 1;
	SCALE = .1;

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);

	CHBODYSHAREDPTR BTM = CHBODYSHAREDPTR(new CHBODY);
	GPUSystem->InitObject(BTM, .001, ChVector<> (0, 0, 0), quat, mu, mu, rest, true, true, -20, -20);
	GPUSystem->AddCollisionGeometry(BTM, BOX, ChVector<> (15, .1, 15), lpos, quat);
	GPUSystem->FinalizeObject(BTM);

	float x = 0.0, y = 3.0, z = 0.0;

	CHBODYSHAREDPTR mrigidBody;

	mrigidBody = CHBODYSHAREDPTR(new CHBODY);
	GPUSystem->InitObject(mrigidBody, mass, ChVector<> (x, y, z), quat, mu, mu, rest, true, false, 0, 1);
	GPUSystem->AddCollisionGeometry(mrigidBody, BOX, ChVector<> (1, .2, .2), lpos, quat);
	GPUSystem->FinalizeObject(mrigidBody);

	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	return 0;
}

