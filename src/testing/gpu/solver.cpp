#include "common.h"
#define TOLERANCE  1e-4
ChBODYSHAREDPTR FREE,FIXED;

void System::DoTimeStep() {
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;


	GPUSystem->PrintStats();
}

int main(int argc, char* argv[]) {
	omp_set_nested(1);
	stepMode = true;
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .0001;
	GPUSystem->mEndTime = 10;
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 0;
	GPUSystem->mOmegaContact = .5;
	GPUSystem->mOmegaBilateral = .9;
	GPUSystem->mUseOGL = 1;
	GPUSystem->mSaveData = 0;

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	FREE = ChBODYSHAREDPTR(new ChBODY);
	FIXED = ChBODYSHAREDPTR(new ChBODY);

	GPUSystem->InitObject(FREE, 1.0, ChVector<> (0, 0, 0), quat, 1, 1, 0, true, true, -1, -1);
	GPUSystem->InitObject(FIXED, 1.0, ChVector<> (0, 2, 0), quat, 1, 1, 0, true, false, -2, -2);

	GPUSystem->AddCollisionGeometry(FREE, SPHERE, ChVector<> (1, 1, 1), lpos, quat);
	GPUSystem->AddCollisionGeometry(FIXED, SPHERE, ChVector<> (1, 1, 1), lpos, quat);
	GPUSystem->FinalizeObject(FREE);
	GPUSystem->FinalizeObject(FIXED);

	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	cout<<"PASS"<<endl;
	return 0;
}
