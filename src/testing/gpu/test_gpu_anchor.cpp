#include "common.h"
//-----------------------------------------------------------------------
// Test 01: Angle: 00 Torque .05 Force 10 Force 30
// Test 02: Angle: 30 Torque .05 Force 10 Force 30
// Test 03: Angle: 45 Torque .05 Force 10 Force 30
// Test 04: Angle: 60 Torque .05 Force 10 Force 30

// Test 05: Angle: 00 Torque .01 Force 10 Force 30
// Test 06: Angle: 00 Torque .10 Force 10 Force 30

// Test 07: Angle: 00 Torque .05 Force 00 Force 30
// Test 08: Angle: 00 Torque .05 Force 20 Force 30

// Test 09: Angle: 00 Torque .05 Force 10 Force 20
// Test 10: Angle: 00 Torque .05 Force 10 Force 40

//---------------------------Constraints--------------------------------
ChSharedPtr<ChLinkEngine> rotational_motor;
ChSharedPtr<ChLinkLockPrismatic> link_align;
//--------------------------Global_Objects------------------------------
ChSharedBodyGPUPtr Anchor;
//---------------------------SolverParams-------------------------------
float mOmega = .5;
float mTimeStep = .0001;
int   mIteations = 800;
//-----------------------------SimParams--------------------------------
float mEndTime = 6;
float mMu = .5;
float mWallMu = .5;
float mEnvelope = 0;
int   mDevice = 0;
bool  OGL = 0;

float anchor_penetration_ang	= -10;	//deg
float anchor_torque 			= -.01; //N-m
float anchor_penetration_force	= -10; 	//N
float anchor_pullout_force		= -10; 	//N
//-----------------------------GranularParams---------------------------
float particle_radius 		= .0035;
float particle_rho 			= 1560;
float particle_friction_ang = 23;
int   particle_max 			= 100;
float anchor_mass 			= 1.0;
float container_width 		= .200;
float container_thickness 	= .007;
//-----------------------------CODE--------------------------------
void LoadObjects(string fname) {
	float radius = particle_radius;
	float mass = particle_rho * 4.0 / 3.0 * PI * radius * radius * radius;
	float mu = tan(particle_friction_ang * PI / 180.0);
	float rest = 0;
	int type = 0;

	ifstream ifile(fname.c_str());
	string data;
	int i=0;
	while(ifile.fail()==false){
		getline(ifile, data);
		if (i > 7) {
			for (int j = 0; j < data.size(); j++) {
				if (data[j] == ',') {
					data[j] = '\t';
				}
			}
			stringstream ss(data);
			float x, y, z, rx, ry, rz;
			ss >> x >> y >> z >> rx >> ry >> rz;
			ChSharedBodyGPUPtr mrigidBody;
			mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
			GPUSystem->MakeSphere(mrigidBody, radius, particle_rho * 4.0 / 3.0 * PI * radius * radius * radius, ChVector<> (x, y, z), mu, mu, rest, true);
		}
	++i;
	}


}
void CreateObjects() {

	if (GPUSystem->mNumCurrentObjects < GPUSystem->mNumObjects && GPUSystem->mFrameNumber % 50 == 0) {
		float x = 20, y = 40, z = 40;
		float posX = 0;
		float posY = -25;
		float posZ = 0;

		float radius = particle_radius;
		float mass = particle_rho * 4.0 / 3.0 * PI * radius * radius * radius;
		float mu = tan(particle_friction_ang * PI / 180.0);
		float rest = 0;
		int type = 0;
		ChSharedBodyGPUPtr mrigidBody;
		GPUSystem->mNumCurrentObjects += x * y * z;
		int mobjNum = 0;
		ChQuaternion<> quat = ChQuaternion<> (1, 0, 0, 0);
		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					ChVector<> mParticlePos((xx - (x - 1) / 2.0) + posX, (yy) + posY, (zz - (z - 1) / 2.0) + posZ);
					//mParticlePos += ChVector<> (rand() % 1000 / 1000.0 - .5, rand() % 1000 / 1000.0 - .5, rand() % 1000 / 1000.0 - .5) * .5;

					mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
					if (type == 0) {
						GPUSystem->MakeSphere(mrigidBody, radius, mass, mParticlePos * .007, mu, mu, rest, true);
					}
					if (type == 1) {
						GPUSystem->MakeBox(mrigidBody, ChVector<> (radius, radius, radius), mass, mParticlePos, quat, mu, mu, rest, mobjNum, mobjNum, true, false);
					}
					if (type == 2) {
						GPUSystem->MakeEllipsoid(mrigidBody, ChVector<> (radius, radius, radius), mass, mParticlePos, quat, mu, mu, rest, true);
					}
					if (type == 3) {
						GPUSystem->MakeCylinder(mrigidBody, ChVector<> (radius, radius, radius), mass, mParticlePos, quat, mu, mu, rest, true);
					}
					mobjNum++;
				}
			}
		}
	}
}
float force = 0;
void System::DoTimeStep() {
	//CreateObjects();
	Anchor->Empty_forces_accumulators();
	if (mCurrentTime <= 3) {
		ChFunction_Const* mfun = (ChFunction_Const*) rotational_motor->Get_tor_funct();
		mfun->Set_yconst(anchor_torque);
		Anchor->Accumulate_force(ChVector<>(0,anchor_penetration_force,0),ChVector<>(0,0,0),1);
	}
	if (mCurrentTime > 3 && mCurrentTime <= 4) {
		ChFunction_Const* mfun = (ChFunction_Const*) rotational_motor->Get_spe_funct();
		mfun->Set_yconst(0);
		Anchor->Accumulate_force(ChVector<>(0,0,0),ChVector<>(0,0,0),1);
	}
	if (mCurrentTime > 4 && mCurrentTime <= 6) {
		Anchor->Accumulate_force(ChVector<>(0,anchor_pullout_force,0),ChVector<>(0,0,0),1);
	}
	if (mCurrentTime > 6) {
		Anchor->SetBodyFixed(true);
	}
	stringstream ss;
	ss << "helical_anchor_data"<<anchor_penetration_ang<<"__"<<anchor_torque<<"__"<<anchor_penetration_force<<"__"<<anchor_pullout_force<<".txt";
	SaveByObject(Anchor.get_ptr(), ss.str(), true, true, true, true, true);

	DeactivationPlane(-container_width - container_thickness * 2, 0, false);
	if (mFrameNumber % (int(1.0 / mTimeStep / 100.0)) == 0) {
		SaveAllData("data/anchor", true, false, false, true, false);
	}
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
}

int main(int argc, char* argv[]) {
	Scale = .01;
	if(argc!=7){cout<<"ERROR ARGS:| Graphics | Store_Data | Angle [Deg] | Torque [N-m] | Penetration_Force [N] | Pullout_Force [N]|"<<endl;		return(0);
	}else{
		OGL 					= atoi(argv[1]);
		saveData 				= atoi(argv[2]);
		anchor_penetration_ang	= atof(argv[3]);	//deg
		anchor_torque 			= atof(argv[4]); 	//N-m
		anchor_penetration_force= atof(argv[5]); 	//N
		anchor_pullout_force	= atof(argv[6]); 	//N
	}
	//cudaSetDevice(mDevice);
	int mDevice = 0;
	cudaGetDevice(&mDevice);
	cout << "Device NUM: " << mDevice << endl;

	ChLcpSystemDescriptorGPU mGPUDescriptor;
	ChContactContainerGPUsimple mGPUContactContainer;
	ChCollisionSystemGPU mGPUCollisionEngine(&mGPUDescriptor, mEnvelope,false);
	ChLcpIterativeSolverGPUsimple mGPUsolverSpeed(&mGPUContactContainer, &mGPUDescriptor, mIteations, mTimeStep, 1e-5, mOmega, false);

	ChSystemGPU SysG(1000, 50);
	SysG.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	SysG.ChangeContactContainer(&mGPUContactContainer);
	SysG.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	SysG.ChangeCollisionSystem(&mGPUCollisionEngine);
	SysG.SetIntegrationType(ChSystem::INT_ANITESCU);
	SysG.Set_G_acc(ChVector<> (0, GRAV, 0));
	GPUSystem = new System(&SysG, particle_max, "anchor.txt");
	GPUSystem->mMu = mMu;
	GPUSystem->mTimeStep = mTimeStep;
	GPUSystem->mEndTime = mEndTime;

	ChQuaternion<> base(1, 0, 0, 0);
	Anchor = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM  = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM1 = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM2 = ChSharedBodyGPUPtr(new ChBodyGPU);
	//ChSharedBodyGPUPtr TOP = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FREE = ChSharedBodyGPUPtr(new ChBodyGPU);
	float wscale=.5;
	GPUSystem->MakeBox(L, ChVector<> (container_thickness, container_width, container_width), 100000, ChVector<> (-container_width*wscale, 0, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	GPUSystem->MakeBox(R, ChVector<> (container_thickness, container_width, container_width), 100000, ChVector<> (container_width*wscale, 0, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	GPUSystem->MakeBox(F, ChVector<> (container_width*wscale, container_width, container_thickness), 100000, ChVector<> (0, 0, -container_width), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	GPUSystem->MakeBox(B, ChVector<> (container_width*wscale, container_width, container_thickness), 100000, ChVector<> (0, 0, container_width), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	GPUSystem->MakeBox(BTM ,ChVector<>(container_width*wscale, container_thickness, container_width),100000, ChVector<> (0, -container_width, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	GPUSystem->MakeBox(BTM1,ChVector<>(container_width*wscale, container_thickness, container_width),100000, ChVector<> (0, -container_width, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	GPUSystem->MakeBox(BTM2,ChVector<>(container_width*wscale, container_thickness, container_width),100000, ChVector<> (0, -container_width, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//GPUSystem->MakeBox(TOP, ChVector<> (container_width, container_thickness, container_width),  100000, ChVector<> (0, container_width, 0), base, mWallMu, mWallMu, 0,  -20, -20, true, true);
	LoadObjects("anchor600.txt");

	ChQuaternion<> anchor_rot;

	//GPUSystem->LoadTriangleMesh(Anchor,"Helical_Anchor2.obj",.5,ChVector<> (0,0,0),anchor_rot,1,.5,.5,0,-21,-21 );
	vector<float3> pos;
	vector<float3> dim;
	vector<float4> quats;
	vector<ShapeType> tpe;
	vector<float> masses;
	for (int i = 0; i < 69; i++) {
		if (i == 0) {
			dim.push_back(F3(.005, 0, 0));
			tpe.push_back(SPHERE);
			quats.push_back(F4(1, 0, 0, 0));
			pos.push_back(F3(0, 0, 0));
			masses.push_back(.001);
		} else if (i == 1) {
			dim.push_back(F3(.005, .05, .005));
			tpe.push_back(CYLINDER);
			quats.push_back(F4(1, 0, 0, 0));
			pos.push_back(F3(0, .05, 0));
			masses.push_back(.1);
		} else {
			dim.push_back(F3(.0025, .001, .025));
			tpe.push_back(BOX);
			ChQuaternion<> quat;
			ChQuaternion<> quat2;
			quat.Q_from_AngAxis(i / 69.0 * 2 * PI, ChVector<> (0, 1, 0));
			quat2.Q_from_AngAxis(6 * 2 * PI / 360.0, ChVector<> (0, 0, 1));
			quat = quat % quat2;
			quats.push_back(F4(quat.e0, quat.e1, quat.e2, quat.e3));
			pos.push_back(F3(sin(i / 69.0 * 2 * PI) * 2.8 * .01, i / 69.0 * 4.0 * .01 + .005, cos(i / 69.0 * 2 * PI) * 2.8 * .01));
			masses.push_back(.00107);
		}
	}
	GPUSystem->MakeCompound(Anchor, ChVector<> (0, 0, -.06), base, .25, pos, dim, quats, tpe, masses, .5, .5, 0, true);
	Anchor->SetMass(.04823);
	Anchor->SetInertiaXX(ChVector<> (1e-5, 1e-5, 1e-5));
	anchor_rot.Q_from_AngAxis(TORAD(anchor_penetration_ang), ChVector<> (1, 0, 0));
	Anchor->SetRot(anchor_rot);

	GPUSystem->MakeBox(FREE, ChVector<> (.001, .001, .001), 1, ChVector<> (0, 0, -.06), base, 0, 0, 0, -20, -20, false, false);
	FREE->SetRot(anchor_rot);
	ChSharedBodyPtr ptr1 = ChSharedBodyPtr(BTM);
	ChSharedBodyPtr ptr2 = ChSharedBodyPtr(FREE);
	ChSharedBodyPtr ptr3 = ChSharedBodyPtr(Anchor);

	ChFrame<> f0(ChVector<> (0, 0, 0), Q_from_AngAxis(0, ChVector<> (0, 1, 0)));
	ChFrame<> f1(ChVector<> (0, 0, -.06), Q_from_AngAxis(PI / 2.0 + TORAD(anchor_penetration_ang), ChVector<> (1, 0, 0)));
	ChFrame<> f2(ChVector<> (0, 0, 0), Q_from_AngAxis(0, ChVector<> (0, 1, 0)));
	ChFrame<> f3(ChVector<> (0, 0, -.06), Q_from_AngAxis(PI / 2.0 + TORAD(anchor_penetration_ang), ChVector<> (1, 0, 0)));

	link_align = ChSharedPtr<ChLinkLockPrismatic> (new ChLinkLockPrismatic);
	link_align->Initialize(ptr1, ptr2, (f0 >> f1).GetCoord());

	rotational_motor = ChSharedPtr<ChLinkEngine> (new ChLinkEngine);
	rotational_motor->Initialize(ptr2, ptr3, (f2 >> f3).GetCoord());
	rotational_motor->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
	rotational_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);

	SysG.AddLink(link_align);
	SysG.AddLink(rotational_motor);


#pragma omp parallel sections
	{
#pragma omp section
		{
			while (GPUSystem->mSystem->GetChTime() <= GPUSystem->mEndTime) {
				GPUSystem->renderScene();
			}
		}
#pragma omp section
		{
			if (OGL) {
				initGLUT(string("anchor"), argc, argv);
			}
		}
	}
	return 0;
}

