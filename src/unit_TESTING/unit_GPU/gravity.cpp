#include "common.h"
#define TOLERANCE  1e-4
CHBODYSHAREDPTR FREE;

void System::DoTimeStep() {
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;

	float correct = 9.80665 * mCurrentTime;
	float current = FREE->GetPos_dt().Length();
	//cout<< mSystem->GetChTime()<<endl;
	if (fabs(correct - current) > TOLERANCE) {
		cout << "FAIL at T=" << mSystem->GetChTime() << " correct: " << correct << " current: " << current << " " << fabs(correct - current) << endl;
		exit(1);
	}

}

int main(int argc, char* argv[]) {
	omp_set_num_threads(2);
	GPUSystem = new System(1);
	GPUSystem->mTimeStep = .001;
	GPUSystem->mEndTime = 1;
	GPUSystem->mNumObjects = 1;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 0;
	GPUSystem->mOmegaContact = .5;
	GPUSystem->mOmegaBilateral = .9;
	GPUSystem->mUseOGL = 1;
	GPUSystem->mSaveData = 0;

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	FREE = CHBODYSHAREDPTR(new CHBODY);

	GPUSystem->InitObject(FREE, 1.0, ChVector<>(0, 0, 0), quat, 0, 0, 0, false, false, -20, -20);
	GPUSystem->AddCollisionGeometry(FREE, SPHERE, ChVector<>(1, 1, 1), lpos, quat);
	GPUSystem->FinalizeObject(FREE);

	GPUSystem->Setup();
	SimulationLoop(argc, argv);
	cout << "PASS" << endl;
	return 0;
}
