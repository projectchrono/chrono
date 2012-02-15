#include "common.h"
#define TOLERANCE .05
CHBODYSHAREDPTR FREE;

double compute_period(double L, double g, double theta) {

	if (theta == PI / 2.0) {
		return 7.416298708 * sqrt(L / 9.80665);
	} else {
		return 2 * PI * sqrt(L / g) + 2 * PI * sqrt(L / g) * 1.0 / 16.0 * pow(theta, 2) + 2 * PI * sqrt(L / g) * 11.0 / 3072.0 * pow(theta, 4);
	}

}

void System::DoTimeStep() {
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
	//GPUSystem->PrintStats();
}

int main(int argc, char* argv[]) {
	omp_set_nested(1);
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .005;
	GPUSystem->mEndTime = compute_period(1, 9.80665, PI / 2.0);
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 100;
	GPUSystem->mTolerance = 1e-8;
	GPUSystem->mOmegaContact = .5;
	GPUSystem->mOmegaBilateral = .1;
	GPUSystem->mUseOGL = 0;
	GPUSystem->mSaveData = 0;

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	CHBODYSHAREDPTR FXED = CHBODYSHAREDPTR(new CHBODY);
	FREE = CHBODYSHAREDPTR(new CHBODY);

	GPUSystem->InitObject(FXED, 1.0, ChVector<> (0, 0, 0), quat, 0, 0, 0, false, true, -20, -20);
	GPUSystem->InitObject(FREE, 1.0, ChVector<> (-1, 0, 0), quat, 0, 0, 0, false, false, -20, -20);

	GPUSystem->AddCollisionGeometry(FXED, SPHERE, ChVector<> (.1, .1, .1), lpos, quat);
	GPUSystem->AddCollisionGeometry(FREE, SPHERE, ChVector<> (.1, .1, .1), lpos, quat);

	GPUSystem->FinalizeObject(FXED);
	GPUSystem->FinalizeObject(FREE);

	ChSharedBodyPtr ptr1 = ChSharedBodyPtr(FXED);
	ChSharedBodyPtr ptr2 = ChSharedBodyPtr(FREE);

	ChSharedPtr<ChLinkLockRevolute> my_link_12(new ChLinkLockRevolute);
	my_link_12->Initialize(ptr1, ptr2, ChCoordsys<> (ChVector<> (0, 0, 0)));
	GPUSystem->mSystem->AddLink(my_link_12);

	GPUSystem->Setup();
	SimulationLoop(argc, argv);

	float correct = 0;
	float current = FREE->GetPos_dt().Length();
	if (fabs(correct - current) > TOLERANCE) {
		cout << "FAIL at T=" << GPUSystem->mSystem->GetChTime() << " correct: " << correct << " current: " << current << " " << fabs(correct - current) << endl;
		exit(1);
	};
	cout << "PASS" << endl;
	return 0;
}
