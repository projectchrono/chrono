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
int mIteations = 800;
//-----------------------------SimParams--------------------------------
float mEndTime = 6;
float mMu = .5;
float mWallMu = .5;

float anchor_penetration_ang = -10; //deg
float anchor_torque = -.01; //N-m
float anchor_penetration_force = -10; //N
float anchor_pullout_force = -10; //N
//-----------------------------GranularParams---------------------------
float particle_radius = .0035;
float particle_rho = 1560;
float particle_friction_ang = 23;
int particle_max = 10;
float anchor_mass = 1.0;
float container_width = .200;
float container_thickness = .007;
//-----------------------------CODE--------------------------------
float force = 0;

void System::DoTimeStep() {
	if (mNumCurrentObjects < mNumObjects && mFrameNumber % 5000 == 0) {
		float x = 0, y = 0, z = 0;
		float posX = 0, posY = -25, posZ = 0;
		float radius = particle_radius, mass = particle_rho * 4.0 / 3.0 * PI * radius * radius * radius, mu = tan(particle_friction_ang * PI / 180.0), rest = 0;
		ShapeType type = SPHERE;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects += x * y * z;
		int mobjNum = 0;

		for (int xx = 0; xx < x; xx++) {
			for (int yy = 0; yy < y; yy++) {
				for (int zz = 0; zz < z; zz++) {
					ChVector<> mParticlePos((xx - (x - 1) / 2.0) + posX, (yy) + posY, (zz - (z - 1) / 2.0) + posZ);

					mParticlePos += ChVector<> (rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05, rand() % 1000 / 10000.0 - .05);
					ChQuaternion<> quat = ChQuaternion<> (rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000., rand() % 1000 / 1000.);
					ChVector<> dim;
					ChVector<> lpos(0, 0, 0);
					quat.Normalize();
					mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
					InitObject(mrigidBody, mass, mParticlePos * .007, quat, mu, mu, rest, true, false, 0, 1);
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

	Anchor->Empty_forces_accumulators();
	if (mCurrentTime <= 3) {
		ChFunction_Const* mfun = (ChFunction_Const*) rotational_motor->Get_tor_funct();
		mfun->Set_yconst(anchor_torque);
		Anchor->Accumulate_force(ChVector<> (0, anchor_penetration_force, 0), ChVector<> (0, 0, 0), 1);
	}
	if (mCurrentTime > 3 && mCurrentTime <= 4) {
		ChFunction_Const* mfun = (ChFunction_Const*) rotational_motor->Get_spe_funct();
		mfun->Set_yconst(0);
		Anchor->Accumulate_force(ChVector<> (0, 0, 0), ChVector<> (0, 0, 0), 1);
	}
	if (mCurrentTime > 4 && mCurrentTime <= 6) {
		Anchor->Accumulate_force(ChVector<> (0, anchor_pullout_force, 0), ChVector<> (0, 0, 0), 1);
	}
	if (mCurrentTime > 6) {
		Anchor->SetBodyFixed(true);
	}
	stringstream ss;
	ss << "helical_anchor_data" << anchor_penetration_ang << "__" << anchor_torque << "__" << anchor_penetration_force << "__" << anchor_pullout_force << ".txt";
	SaveByObject(Anchor.get_ptr(), ss.str());

	DeactivationPlane(-container_width - container_thickness * 2, 0, false);
	if (mFrameNumber % (int(1.0 / mTimeStep / 100.0)) == 0) {
		SaveAllData("data/anchor");
	}
	mFrameNumber++;
	mSystem->DoStepDynamics(mTimeStep);
	mCurrentTime += mTimeStep;
}

int main(int argc, char* argv[]) {
	GPUSystem = new System(0);
	GPUSystem->mTimeStep = .0001;
	GPUSystem->mEndTime = 8;
	GPUSystem->mNumObjects = particle_max;
	GPUSystem->mIterations = 1000;
	GPUSystem->mTolerance = 1e-8;
	GPUSystem->mOmegaContact = 1.0;
	GPUSystem->mOmegaBilateral = .2;
	GPUSystem->SetTimingFile("anchor.txt");
	SCALE = .1;
	float mMu = .5;
	float mWallMu = .5;

	if (argc != 7) {
		cout << "ERROR ARGS:| Graphics | Store_Data | Angle [Deg] | Torque [N-m] | Penetration_Force [N] | Pullout_Force [N]|" << endl;
		return (0);
	} else {
		GPUSystem->mUseOGL = atoi(argv[1]);
		GPUSystem->mSaveData = atoi(argv[2]);
		anchor_penetration_ang = atof(argv[3]); //deg
		anchor_torque = atof(argv[4]); //N-m
		anchor_penetration_force = atof(argv[5]); //N
		anchor_pullout_force = atof(argv[6]); //N
	}
	GPUSystem->Setup();

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	Anchor = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM1 = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM2 = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FREE = ChSharedBodyGPUPtr(new ChBodyGPU);
	float wscale = .5;

	GPUSystem->InitObject(L, 100000, ChVector<> (-container_width * wscale, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(R, 100000, ChVector<> (container_width * wscale, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(F, 100000, ChVector<> (0, 0, -container_width), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(B, 100000, ChVector<> (0, 0, container_width), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM, 100000, ChVector<> (0, -container_width, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM1, 100000, ChVector<> (0, -container_width, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(BTM2, 100000, ChVector<> (0, -container_width, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	GPUSystem->InitObject(FREE, .01, ChVector<> (0, 0, -.06), quat, mWallMu, mWallMu, 0, false, false, -1, -1);

	GPUSystem->AddCollisionGeometry(L, BOX, ChVector<> (container_thickness, container_width, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(R, BOX, ChVector<> (container_thickness, container_width, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(F, BOX, ChVector<> (container_width * wscale, container_width, container_thickness), lpos, quat);
	GPUSystem->AddCollisionGeometry(B, BOX, ChVector<> (container_width * wscale, container_width, container_thickness), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM, BOX, ChVector<> (container_width * wscale, container_thickness, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM1, BOX, ChVector<> (container_width * wscale, container_thickness, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(BTM2, BOX, ChVector<> (container_width * wscale, container_thickness, container_width), lpos, quat);
	GPUSystem->AddCollisionGeometry(FREE, BOX, ChVector<> (.001, .001, .001), lpos, quat);

	GPUSystem->FinalizeObject(L);
	GPUSystem->FinalizeObject(R);
	GPUSystem->FinalizeObject(F);
	GPUSystem->FinalizeObject(B);
	GPUSystem->FinalizeObject(BTM);
	GPUSystem->FinalizeObject(BTM1);
	GPUSystem->FinalizeObject(BTM2);
	GPUSystem->FinalizeObject(FREE);

	//	GPUSystem->MakeBox(L, ChVector<> (container_thickness, container_width, container_width), 100000, ChVector<> (-container_width*wscale, 0, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(R, ChVector<> (container_thickness, container_width, container_width), 100000, ChVector<> (container_width*wscale, 0, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(F, ChVector<> (container_width*wscale, container_width, container_thickness), 100000, ChVector<> (0, 0, -container_width), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(B, ChVector<> (container_width*wscale, container_width, container_thickness), 100000, ChVector<> (0, 0, container_width), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(BTM ,ChVector<>(container_width*wscale, container_thickness, container_width),100000, ChVector<> (0, -container_width, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(BTM1,ChVector<>(container_width*wscale, container_thickness, container_width),100000, ChVector<> (0, -container_width, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(BTM2,ChVector<>(container_width*wscale, container_thickness, container_width),100000, ChVector<> (0, -container_width, 0), base, mWallMu, mWallMu, 0, -20, -20, true, true);
	//	GPUSystem->MakeBox(FREE, ChVector<> (.001, .001, .001), 1, ChVector<> (0, 0, -.06), base, 0, 0, 0, -20, -20, false, false);

	//LoadObjects("anchor600.txt");
	ChQuaternion<> anchor_rot;
	anchor_rot.Q_from_AngAxis(TORAD(anchor_penetration_ang), ChVector<> (1, 0, 0));
	FREE->SetRot(anchor_rot);
	ChVector<> p1(0, 0, 0);
	ChVector<> p2(0,.05, 0);
	GPUSystem->InitObject(Anchor, .04823, ChVector<> (0, 0, -.06), anchor_rot, .5, .5, 0, true, false, -20, -20);
	GPUSystem->AddCollisionGeometry(Anchor, SPHERE, ChVector<> (.005, 0, 0), p1, quat);
	GPUSystem->AddCollisionGeometry(Anchor, CYLINDER, ChVector<> (.005, .05, .005), p2, quat);
	for (int i = 0; i < 69; i++) {
		ChQuaternion<> quat, quat2;
		quat.Q_from_AngAxis(i / 69.0 * 2 * PI, ChVector<> (0, 1, 0));
		quat2.Q_from_AngAxis(6 * 2 * PI / 360.0, ChVector<> (0, 0, 1));
		quat = quat % quat2;
		ChVector<> pos(sin(i / 69.0 * 2 * PI) * 2.8 * .01, i / 69.0 * 4.0 * .01 + .005, cos(i / 69.0 * 2 * PI) * 2.8 * .01);
		ChMatrix33<> mat(quat);
		GPUSystem->AddCollisionGeometry(Anchor, BOX, ChVector<> (.0025, .001, .025), pos, quat);
	}
	GPUSystem->FinalizeObject(Anchor);
	Anchor->SetInertiaXX(ChVector<> (1e-5, 1e-5, 1e-5));

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

	GPUSystem->mSystem->AddLink(link_align);
	GPUSystem->mSystem->AddLink(rotational_motor);

	SimulationLoop(argc, argv);
	return 0;
}

