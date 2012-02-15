#include "common.h"
void System::DoTimeStep() {
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;

	cout<<2*PI*sqrtf(10.0/9.80665);
}

int main(int argc, char* argv[]) {
	omp_set_nested(1);
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 30;
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 1e-8;
	GPUSystem->mOmegaContact = .5;
	GPUSystem->mOmegaBilateral = .1;
	GPUSystem->mUseOGL = 1;
	GPUSystem->mSaveData = 0;

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	CHBODYSHAREDPTR FXED = CHBODYSHAREDPTR(new CHBODY);
	CHBODYSHAREDPTR FREE = CHBODYSHAREDPTR(new CHBODY);

	GPUSystem->InitObject(FXED, 1.0, ChVector<> (0, 0, 0), quat, 0, 0, 0, false, true, -20, -20);
	GPUSystem->InitObject(FREE, 1.0, ChVector<> (-10, 0, 0), quat, 0, 0, 0, false, false, -20, -20);

	GPUSystem->AddCollisionGeometry(FXED, BOX, ChVector<> (1, 1, 1), lpos, quat);
	GPUSystem->AddCollisionGeometry(FREE, BOX, ChVector<> (1, 1, 1), lpos, quat);

	GPUSystem->FinalizeObject(FXED);
	GPUSystem->FinalizeObject(FREE);

	ChSharedBodyPtr ptr1 = ChSharedBodyPtr(FXED);
	ChSharedBodyPtr ptr2 = ChSharedBodyPtr(FREE);

	ChSharedPtr<ChLinkLockSpherical> my_link_12(new ChLinkLockSpherical);
	my_link_12->Initialize(ptr1, ptr2, ChCoordsys<> (ChVector<> (0, 0, 0)));
	GPUSystem->mSystem->AddLink(my_link_12);

	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	return 0;
}
