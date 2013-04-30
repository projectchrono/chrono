///////////////////////////////////////////////////
//
//   Demo code about
//
//     - using GPU solver - Tasora implementation.
//
//	 NOTE! this program should be copied
//   on multiple hosts of a cluster and executed
//   using the launcher utility of the MPICH2
//   toolchain (ex. mpiexec or wmpiexec.exe).
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "core/ChLinearAlgebra.h"
#include "physics/ChSystemOpenMP.h"
#include "assets/ChSphereShape.h"
#include "physics/ChApidll.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "physics/ChContactContainer.h"
#include "unit_OPENGL/ChOpenGL.h"
#include "unit_POSTPROCESS/ChMitsubaRender.h"

#include "unit_GPU/ChSystemGPU.h"
#include "unit_GPU/ChLcpSolverGPU.h"

//Defines for selecting CPU or GPU solver

//#define CHBODYSHAREDPTR ChSharedBodyPtr
//#define CHBODY ChBody
//#define CHCOLLISIONSYS ChCollisionSystemBullet
//#define CHLCPDESC ChLcpSystemDescriptor
//#define CHCONTACTCONT ChContactContainer
//#define CHSOLVER ChLcpIterativeJacobi
//#define CHMODEL ChModelBullet
//#define CHSYS ChSystem
#define CHBODYSHAREDPTR ChSharedBodyGPUPtr
#define CHBODY ChBodyGPU
#define CHCOLLISIONSYS ChCollisionSystemGPU
#define CHLCPDESC ChLcpSystemDescriptorGPU
#define CHCONTACTCONT ChContactContainerGPU
#define CHSOLVER ChLcpIterativeJacobi
#define CHMODEL ChModelGPU
#define CHSYS ChSystemGPU

// Use the namespace of Chrono
using namespace chrono;

template<class T>
void AddBoundary(T* mSystem, double L, double W, double H, double th, float mu1) {
	ChVector<> pos_p0(0, 0, 0);

	CHBODYSHAREDPTR BTM = CHBODYSHAREDPTR(new CHBODY);
	BTM->SetIdentifier(1);
	InitObject(BTM, 100, Vector(0, 0, 0), QUNIT, mu1, mu1, 0, true, true, -20, -20);
	AddCollisionGeometry(BTM, BOX, Vector(L / 2.0, th / 2.0, W / 2.0), VNULL, QUNIT);
	FinalizeObject(BTM, (CHSYS*) mSystem);

	CHBODYSHAREDPTR RIGHT = CHBODYSHAREDPTR(new CHBODY);
	RIGHT->SetIdentifier(2);
	InitObject(RIGHT, 100, Vector(L / 2.0, H / 2.0, 0), QUNIT, mu1, mu1, 0, true, true, -20, -20);
	AddCollisionGeometry(RIGHT, BOX, Vector(th / 2.0, H / 2.0, W / 2.0), VNULL, QUNIT);
	FinalizeObject(RIGHT, (CHSYS*) mSystem);

	CHBODYSHAREDPTR LEFT = CHBODYSHAREDPTR(new CHBODY);
	LEFT->SetIdentifier(3);
	InitObject(LEFT, 100, Vector(-L / 2.0, H / 2.0, 0), QUNIT, mu1, mu1, 0, true, true, -20, -20);
	AddCollisionGeometry(LEFT, BOX, Vector(th / 2.0, H / 2.0, W / 2.0), VNULL, QUNIT);
	FinalizeObject(LEFT, (CHSYS*) mSystem);

	CHBODYSHAREDPTR FRONT = CHBODYSHAREDPTR(new CHBODY);
	FRONT->SetIdentifier(4);
	InitObject(FRONT, 100, Vector(0.0, H / 2.0, W / 2.0), QUNIT, mu1, mu1, 0, true, true, -20, -20);
	AddCollisionGeometry(FRONT, BOX, Vector(L / 2.0, H / 2.0, th / 2.0), VNULL, QUNIT);
	FinalizeObject(FRONT, (CHSYS*) mSystem);

	CHBODYSHAREDPTR BACK = CHBODYSHAREDPTR(new CHBODY);
	BACK->SetIdentifier(5);
	InitObject(BACK, 100, Vector(0.0, H / 2.0, -W / 2.0), QUNIT, mu1, mu1, 0, true, true, -20, -20);
	AddCollisionGeometry(BACK, BOX, Vector(L / 2.0, H / 2.0, th / 2.0), VNULL, QUNIT);
	FinalizeObject(BACK, (CHSYS*) mSystem);
}

template<class T>
void AddSomeParticles(T* mSystem, double rad, double mass, double L, double W, double h, int n_x, int n_z, int n_curr_spheres, float mu2) {
	int id_offset = 300;

	double box_x = L - 5 * rad;
	double box_z = W - 5 * rad;
	double spacing_x = box_x / (n_x - 1);
	double spacing_z = box_z / (n_z - 1);
	double first_x = -box_x / 2;
	double first_z = -box_z / 2;
	int cid = 0;

	for (int ii = 0; ii < n_x; ii++) {
		for (int jj = 0; jj < n_z; jj++) {
			double perturb_x = ChRandom() * rad - rad / 2;
			double perturb_z = ChRandom() * rad - rad / 2;
			Vector particle_pos(first_x + ii * spacing_x + perturb_x, h, first_z + jj * spacing_z + perturb_z);

			CHBODYSHAREDPTR mybody = CHBODYSHAREDPTR(new CHBODY);
			mybody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1 * rad);
			InitObject(mybody, mass, particle_pos, QUNIT, mu2, mu2, 0, true, false, 32, 64);
			AddCollisionGeometry(mybody, SPHERE, Vector(rad, rad, rad), VNULL, QUNIT);
			mybody->SetInertiaXX((2.0 / 5.0) * mass * pow(rad, 2) * ChVector<>(1, 1, 1));
			FinalizeObject(mybody, (CHSYS*) mSystem);

			cid++;
		}
	}
}

template<class T>
int add_particles_fromFile(T* mSystem, int n_load, double body_radius, double particle_mass, float fric, std::string tfilename) {
	CHBODYSHAREDPTR mrigidBody;

	cout << tfilename;
	ChStreamInAsciiFile terrain_file(tfilename.c_str());
	int num_p = 0;
	printf("Loading particles...");
	int pid = 0;
	double xx = 0.0;
	double yy = 0.0;
	double zz = 0.0;
	double r1 = 0.0;
	double r2 = 0.0;
	double r3 = 0.0;
	while (num_p < n_load) {
		terrain_file >> pid;
		terrain_file >> xx;
		terrain_file >> yy;
		terrain_file >> zz;
		terrain_file >> r1;
		terrain_file >> r2;
		terrain_file >> r3;
		if (pid > 36257) //36257 for RE, 40824 for orig
				{
			mrigidBody = CHBODYSHAREDPTR(new CHBODY);
			mrigidBody->SetIdentifier(num_p + 10);
			mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1 * body_radius);
			InitObject(mrigidBody, particle_mass, Vector(xx, yy, zz), QUNIT, fric, fric, 0, true, false, 32, 64);
			AddCollisionGeometry(mrigidBody, SPHERE, Vector(body_radius, body_radius, body_radius), VNULL, QUNIT);
			mrigidBody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(body_radius, 2) * ChVector<>(1, 1, 1));
			FinalizeObject(mrigidBody, (CHSYS*) mSystem);
			num_p++;
		}

		if (num_p % 4000 == 0)
			printf("loaded %i\n", num_p);
	}
	printf("Done loading %i particles\n", num_p);
	return num_p;
}

template<class T>
int add_particles_fromFile_V2(T* mSystem, int n_load, double body_radius, double particle_mass, float fric, std::string tfilename) {
	CHBODYSHAREDPTR mrigidBody;

	cout << tfilename;

	ifstream terrain_file(tfilename.c_str());
	int num_p = 0;
	printf("Loading particles...");
	int pid = 0;
	double xx = 0.0;
	double yy = 0.0;
	double zz = 0.0;
	double r1 = 0.0;
	double r2 = 0.0;
	double r3 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	double v3 = 0.0;
	while (num_p < n_load) {
		terrain_file >> pid;
		terrain_file >> xx;
		terrain_file >> yy;
		terrain_file >> zz;
		terrain_file >> r1;
		terrain_file >> r2;
		terrain_file >> r3;
		terrain_file >> v1;
		terrain_file >> v2;
		terrain_file >> v3;
		if (pid > 5) //
				{
			mrigidBody = CHBODYSHAREDPTR(new CHBODY);
			mrigidBody->SetIdentifier(num_p + 10);
			mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1 * body_radius);
			InitObject(mrigidBody, particle_mass, Vector(xx, yy, zz), QUNIT, fric, fric, 0, true, false, 32, 64);
			AddCollisionGeometry(mrigidBody, SPHERE, Vector(body_radius, body_radius, body_radius), VNULL, QUNIT);
			mrigidBody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(body_radius, 2) * ChVector<>(1, 1, 1));
			mrigidBody->SetPos_dt(Vector(v1, v2, v3));
			FinalizeObject(mrigidBody, (CHSYS*) mSystem);
			num_p++;
		}

		if (num_p % 4000 == 0)
			printf("loaded %i\n", num_p);
	}
	printf("Done loading %i particles\n", num_p);
	return num_p;
}

template<class T>
void AddBall(T* mSystem, double Px, double Py, double Pz, double Vy, double bRad, double bMass, float mu1) {
	CHBODYSHAREDPTR theBall = CHBODYSHAREDPTR(new CHBODY);
	theBall->SetIdentifier(-1);
	theBall->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1 * bRad);
	InitObject(theBall, bMass, Vector(Px, Py, Pz), QUNIT, mu1, mu1, 0, true, false, 8, 16);
	ChVector<> inertias((2.0 / 5.0) * bMass * bRad * bRad, (2.0 / 5.0) * bMass * bRad * bRad, (2.0 / 5.0) * bMass * bRad * bRad);
	theBall->SetInertiaXX(inertias);
	AddCollisionGeometry(theBall, SPHERE, Vector(bRad, bRad, bRad), VNULL, QUNIT);
	theBall->SetPos_dt(Vector(0.0, Vy, 0.0));
	FinalizeObject(theBall, (CHSYS*) mSystem);
}

int main(int argc, char* argv[]) {
	omp_set_num_threads(8);
	//for the particles
	double prad = 0.0005; //r = 500 um = 0.5 mm => d=1mm
	double pmass = 2500.0 * (4.0 / 3.0) * CH_C_PI * pow(prad, 3); //density=2500 kg/m^3
	int curr_particles = 0;
	int num_batch_x = 60; //120
	int num_batch_z = 60; //120
	int batchID = 0;
	int num_particles = 500000; //2000000, 10000000
	float mu = 0.3;

	double ballR = 0.0127; //d=2.54 cm = 1 in
	double ballM = 0.00600617265; //6.006 g
	double Py = 0.0592 + prad + ballR;
	double drop_h = 0.1; // 10 cm
	double rho_b = 700;

	//for the boundary
	double LL = 0.1; //0.2
	double WW = 0.1; //0.2
	double HH = 0.15; //0.19
	double th = 0.01;

	//for the time-stepping
	double current_time = 0.0;
	double time_step = 0.00002; //0.001
	double end_time = time_step; //0.05
	int outMult = 5; //for step of 1e-3, use 30
	int addMult = 1; //for step of 1e-5, use 1000
	bool output = true;
	int frame_number = 0;
	int fOuts = outMult;
	int fAdds = addMult;

	int num_to_load = 0;
	char* ofileBase = "testData";

	if (argc == 5) {
		num_to_load = atoi(argv[1]);
		drop_h = atof(argv[2]);
		rho_b = atof(argv[3]);
		ofileBase = argv[4];
	} else {
		std::cout << "Invalid arguments, please try again.\n";
		return 0;
	}
	double Vy = -sqrt(2 * 9.80665 * drop_h);
	ballM = rho_b * (4.0 / 3.0) * CH_C_PI * pow(ballR, 3);

	CHSYS* mSys = new CHSYS();
	CHLCPDESC *mdescriptor = new CHLCPDESC();
	CHCONTACTCONT *mcontactcontainer = new CHCONTACTCONT();
	CHCOLLISIONSYS *mcollisionengine = new CHCOLLISIONSYS();
	//ChLcpIterativeJacobi *msolver = new ChLcpIterativeJacobi();
	//ChLcpIterativeSOR *msolver = new ChLcpIterativeSOR();
	//ChLcpIterativeMINRES *msolver = new ChLcpIterativeMINRES();
	//ChLcpIterativePMINRES *msolver = new ChLcpIterativePMINRES();
	//ChLcpIterativeBB *msolver = new ChLcpIterativeBB();

	mcollisionengine->SetCollisionEnvelope(0.1 * prad);

	mSys->ChangeLcpSystemDescriptor(mdescriptor);
	mSys->ChangeContactContainer(mcontactcontainer);
	//mSys->ChangeLcpSolverSpeed(msolver);
	mSys->ChangeCollisionSystem(mcollisionengine);

	mSys->SetIntegrationType(ChSystem::INT_ANITESCU);
	mSys->Set_G_acc(Vector(0, -9.80665, 0));

	mSys->SetTolSpeeds(0.001);
	mSys->SetIterLCPmaxItersSpeed(100);
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
	//((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetSolverType(CONJUGATE_GRADIENT);
	//((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetSolverType(BLOCK_JACOBI);
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetCompliance(0.0, 0.0, 0.0);
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetContactRecoverySpeed(0.6); //0.6
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetTolerance(0.001); //1e-3
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetMaxIteration(100); //1000

	mcollisionengine->broadphase.setBinsPerAxis(R3(200, 200, 200));
	mcollisionengine->broadphase.setBodyPerBin(100, 50);

	//Create the system:
	AddBoundary(mSys, LL, WW, HH, th, mu);
	//curr_particles+=add_particles_fromFile(mSys, num_to_load, prad, pmass, mu, "input_data/SpherePos_500K_RE.dat");
	curr_particles += add_particles_fromFile_V2(mSys, num_to_load, prad, pmass, mu, "Sphere_Ball_Drop.dat");
	AddBall(mSys, 0.0, Py, 0.0, Vy, ballR, ballM, mu); //NOTE: this is commented out for a re-settling sim, also I scaled up y-positions, and changed contact recovery speed to 0.2, and max its to 100 from 1000

	//Do the simulation
	chrono::Quaternion bodyRot;
	chrono::Vector bodyAngs;
	while (current_time < end_time) {
		if (curr_particles < num_particles && fAdds == addMult) {
			AddSomeParticles(mSys, prad, pmass, LL - th, WW - th, th / 2 + 2 * prad + (2 * batchID) * prad, num_batch_x, num_batch_z, curr_particles, mu);
			curr_particles += num_batch_x * num_batch_z;
			batchID++;
			fAdds = 0;
		}
		if (output && fOuts == outMult) {

			char padnumber[256];
			sprintf(padnumber, "%d", (frame_number + 10000));
			char filename[256];
			sprintf(filename, "%s/pos%s.txt", ofileBase, padnumber + 1);

			ChStreamOutAsciiFile data_spheres_positions(filename);
			std::vector<ChBody*>::iterator abody = mSys->Get_bodylist()->begin();
			while (abody != mSys->Get_bodylist()->end()) {
				if (ChBody* bbb = dynamic_cast<ChBody*>(*abody)) {
					ChVector<> bodypos = bbb->GetPos();
					ChVector<> bodyvel = bbb->GetPos_dt();
					bodyRot = bbb->GetRot();
					bodyAngs = bodyRot.Q_to_NasaAngles();
					data_spheres_positions << bbb->GetIdentifier() << ", ";
					data_spheres_positions << float(bodypos.x) << ", ";
					data_spheres_positions << float(bodypos.y) << ", ";
					data_spheres_positions << float(bodypos.z) << ", ";
					data_spheres_positions << float(bodyAngs.x) << ", ";
					data_spheres_positions << float(bodyAngs.y) << ", ";
					data_spheres_positions << float(bodyAngs.z) << ", ";
					data_spheres_positions << float(bodyvel.x) << ", ";
					data_spheres_positions << float(bodyvel.y) << ", ";
					data_spheres_positions << float(bodyvel.z) << ",\n";
				}
				abody++;
			}

			frame_number++;
			fOuts = 0;
		}

		mSys->DoStepDynamics(time_step);
		GetLog() << "Residual: " << ((ChLcpSolverGPU *) (mSys->GetLcpSolverSpeed()))->GetResidual() << "\n";
		GetLog() << "ITER: " << ((ChLcpSolverGPU *) (mSys->GetLcpSolverSpeed()))->GetTotalIterations() << "\n";
		GetLog() << "OUTPUT STEP: Time= " << current_time << " bodies= " << mSys->GetNbodies() << " contacts= " << mSys->GetNcontacts() << " step time=" << mSys->GetTimerStep() << " lcp time="
				<< mSys->GetTimerLcp() << " CDbroad time=" << mSys->GetTimerCollisionBroad() << " CDnarrow time=" << mSys->GetTimerCollisionNarrow() << " Iterations="
				<< ((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->GetTotalIterations() << "\n";
		fOuts++;
		fAdds++;
		current_time += time_step;
	}

	return 0;
}
