#include "common.h"
#define TOLERANCE  1e-2
CHBODYSHAREDPTR FREE;

int frame_number=0;
float step_size=.001;
float current_time=0;

void DoTimeStep() {
	frame_number++;
	mSystem->DoStepDynamics(step_size);
	current_time += step_size;

	float correct = 9.80665 * current_time;
	float current = FREE->GetPos_dt().Length();
	//cout<< mSystem->GetChTime()<<endl;
	if (fabs(correct - current) > TOLERANCE) {
		cout << "FAIL at T=" << mSystem->GetChTime() << " correct: " << correct << " current: " << current << " " << fabs(correct - current) << endl;
		exit(1);
	}

}

int main(int argc, char* argv[]) {
	mSystem = new CHSYS();
	mGPUDescriptor = (CHLCPDESC*) mSystem->GetLcpSystemDescriptor();
	mGPUsolverSpeed = (CHSOLVER*) mSystem->GetLcpSolverSpeed();
	mSystem->SetIntegrationType(ChSystem::INT_ANITESCU);
	mSystem->Set_G_acc(ChVector<>(0, -9.80665, 0));
	mSystem->SetStep(step_size);
	mGPUsolverSpeed->SetMaxIterations(1000);
	mGPUsolverSpeed->SetOmega(.4);
	mGPUsolverSpeed->SetTolerance(1e-8);
	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	FREE = CHBODYSHAREDPTR(new CHBODY);

	InitObject(FREE, 1.0, ChVector<>(0, 0, 0), quat, 0, 0, 0, false, false, -20, -20);
	AddCollisionGeometry(FREE, SPHERE, ChVector<>(1, 1, 1), lpos, quat);
	FinalizeObject(FREE);
	while (current_time<5) {
		DoTimeStep();
	}
	cout << "PASS" << endl;
	return 0;
}
