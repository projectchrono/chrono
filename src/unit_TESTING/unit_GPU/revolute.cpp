#include "common.h"
#define TOLERANCE  1e-1
CHBODYSHAREDPTR GROUND, PENDULUM;


int frame_number=0;
float step_size=.001;
float current_time=0;
string temp;
ifstream ifile ("../data/testing/revolute.txt");

double compute_period(double L, double g, double theta) {

	if (theta == PI / 2.0) {
		return 7.416298708 * sqrt(L / 9.80665);
	} else {
		return 2 * PI * sqrt(L / g) + 2 * PI * sqrt(L / g) * 1.0 / 16.0 * pow(theta, 2) + 2 * PI * sqrt(L / g) * 11.0 / 3072.0 * pow(theta, 4);
	}

}

void DoTimeStep() {
if(frame_number%50==0){
if(ifile.fail()==false){
getline(ifile,temp);

stringstream ss;
ss<<temp;
float time, px, py, p, vx, vy, v, ax, ay, a, fx, fy, f;
ss>>time>>px>>py>>p>>vx>>vy>>v>>ax>>ay>>a>>fx>>fy>>f;
float dx=PENDULUM->GetPos().x-px;
float dy=PENDULUM->GetPos().y-py;
float ddx=PENDULUM->GetPos_dt().x-vx;
float ddy=PENDULUM->GetPos_dt().y-vy;

if(!((dx<TOLERANCE)&&(dy<TOLERANCE)&&(ddx<TOLERANCE)&&(ddy<TOLERANCE))){
printf("%f %f [%f,%f] [%f,%f]\n",time,current_time,dx,dy,ddx,ddy);

exit(1);}

}else{
exit(1);
}
}

	frame_number++;
	mSystem->DoStepDynamics(step_size);
	current_time += step_size;
	//cout<< mSystem->GetChTime()<<endl;




	
	/*float correct = 9.80665 * current_time;
	float current = FREE->GetPos_dt().Length();
	
	if (fabs(correct - current) > TOLERANCE) {
		cout << "FAIL at T=" << mSystem->GetChTime() << " correct: " << correct << " current: " << current << " " << fabs(correct - current) << endl;
		exit(1);
	}*/

}

int main(int argc, char* argv[]) {

getline(ifile,temp);
	mSystem = new CHSYS();
	mGPUDescriptor = (CHLCPDESC*) mSystem->GetLcpSystemDescriptor();
	mGPUsolverSpeed = (CHSOLVER*) mSystem->GetLcpSolverSpeed();
	mSystem->SetIntegrationType(ChSystem::INT_ANITESCU);
	mSystem->Set_G_acc(ChVector<>(0, -9.8, 0));

	mGPUDescriptor = new CHLCPDESC();
	mGPUContactContainer = new CHCONTACTCONT();
	mGPUCollisionEngine = new CHCOLLISIONSYS();
/*	mGPUsolverSpeed = new CHSOLVER(mGPUContactContainer);

	mSystem->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mSystem->ChangeLcpSystemDescriptor(mGPUDescriptor);
	mSystem->ChangeContactContainer(mGPUContactContainer);
	mSystem->ChangeLcpSolverSpeed(mGPUsolverSpeed);
	mSystem->ChangeCollisionSystem(mGPUCollisionEngine);
*/

	mSystem->SetStep(step_size);
	mGPUsolverSpeed->SetMaxIterations(1000);
	//mGPUsolverSpeed->SetOmega(.4);
	mGPUsolverSpeed->SetTolerance(1e-8);
	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);

	GROUND = CHBODYSHAREDPTR(new CHBODY);
	PENDULUM = CHBODYSHAREDPTR(new CHBODY);

	ChSharedBodyPtr ptr1 = ChSharedBodyPtr(GROUND);
	ChSharedBodyPtr ptr2 = ChSharedBodyPtr(PENDULUM);

	InitObject(GROUND, 1.0, ChVector<>(0, 0, 0), quat, 0, 0, 0, false, true, -20, -20);
	InitObject(PENDULUM, 1.0, ChVector<>(2, 0, 0), quat, 0, 0, 0, false, false, -20, -20);

	AddCollisionGeometry(GROUND, SPHERE, ChVector<>(1, 1, 1), lpos, quat);
	AddCollisionGeometry(PENDULUM, SPHERE, ChVector<>(1, 1, 1), lpos, quat);

	FinalizeObject(GROUND);
	FinalizeObject(PENDULUM);


	ChSharedPtr<ChLinkLockRevolute>  revoluteJoint(new ChLinkLockRevolute);
	revoluteJoint->Initialize(ptr1, ptr2, ChCoordsys<>(ChVector<>(0,0,0)));
	mSystem->AddLink(revoluteJoint);


	while (current_time<5) {
		DoTimeStep();
	}
	cout << "PASS" << endl;
	return 0;
}
