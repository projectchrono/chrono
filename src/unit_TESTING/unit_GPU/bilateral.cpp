#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativePCG.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpIterativeAPGD.h"
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
#include "unit_GPU/ChSystemGPU.h"
#include "physics/ChContactContainer.h"
#include "unit_OPENGL/ChOpenGL.h"
#include <iostream> // For writing to output file
//#include "omp.h"
#include "unit_POSTPROCESS/ChMitsubaRender.h"

#include "unit_GPU/ChSystemGPU.h"
#include "unit_GPU/ChLcpSolverGPU.h"
#include "unit_GPU/solver/ChSolverGPU.cuh"
// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

//float SF = 1;	// Scaling factor
#define SF 1000	// Scaling factor - scales to mm
float GAP_OPEN = 0;
float FRIC = 0; // Friction coef
float wall_dist = .009525 * SF * .5;
float wall_th = .001 * SF; // Wall thickness
float wall_len = .03175 * SF * .55; // Wall length
float mTimeStep = .001; //0.001 has some penetration???
bool open_gap = false;
bool saveData = true;
int mFrameNumber = 0;
int mNumCurrentObjects = 0;
int mFileNumber = 0;

ofstream mfr_file;
using namespace chrono;
using namespace postprocess;
#define SIM_USE_GPU_MODE
#ifdef SIM_USE_GPU_MODE
#define CHBODYSHAREDPTR ChSharedBodyGPUPtr
#define CHBODY ChBodyGPU
#define CHCOLLISIONSYS ChCollisionSystemGPU
#define CHLCPDESC ChLcpSystemDescriptorGPU
#define CHCONTACTCONT ChContactContainerGPU
#define CHSOLVER ChSolverGPU
#define CHMODEL ChModelGPU
#define CHSYS ChSystemGPU
#else
#define CHBODYSHAREDPTR ChSharedBodyPtr
#define CHBODY ChBody
#define CHCOLLISIONSYS ChCollisionSystemBullet
#define CHLCPDESC ChLcpSystemDescriptor
#define CHCONTACTCONT ChContactContainer
#define CHSOLVER ChLcpIterativeAPGD
#define CHMODEL ChModelBullet
#define CHSYS ChSystem
#endif
//*/
ChBODYSHAREDPTR S, sphere2, sphere3;
int counter = 0;
char* ofileBase = "testData";
char* ifileBase = "testData";
void AddEllipsoids(ChSystem * mSys);

void dump_matrices(ChLcpSystemDescriptor &mdescriptor) {
	chrono::ChSparseMatrix mdM;
	chrono::ChSparseMatrix mdCq;
	chrono::ChSparseMatrix mdE;
	chrono::ChMatrixDynamic<double> mdf;
	chrono::ChMatrixDynamic<double> mdb;
	chrono::ChMatrixDynamic<double> mdfric;
	mdescriptor.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
	chrono::ChStreamOutAsciiFile file_M("output_MFR_SCALED_J/dump_M.dat");
	mdM.StreamOUTsparseMatlabFormat(file_M);
	chrono::ChStreamOutAsciiFile file_Cq("output_MFR_SCALED_J/dump_Cq.dat");
	mdCq.StreamOUTsparseMatlabFormat(file_Cq);
	chrono::ChStreamOutAsciiFile file_E("output_MFR_SCALED_J/dump_E.dat");
	mdE.StreamOUTsparseMatlabFormat(file_E);
	chrono::ChStreamOutAsciiFile file_f("output_MFR_SCALED_J/dump_f.dat");
	mdf.StreamOUTdenseMatlabFormat(file_f);
	chrono::ChStreamOutAsciiFile file_b("output_MFR_SCALED_J/dump_b.dat");
	mdb.StreamOUTdenseMatlabFormat(file_b);
	chrono::ChStreamOutAsciiFile file_fric("output_MFR_SCALED_J/dump_fric.dat");
	mdfric.StreamOUTdenseMatlabFormat(file_fric);
}

float GetMFR(CHSYS * mSys, double height) {
	float mass = 0;
	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBODY *abody = (ChBODY*) (mSys->Get_bodylist()->at(i));
		if (abody->IsActive() == true) {
			if (abody->GetPos().y < height) {
				mass += abody->GetMass();
				//abody->SetCollide(false);
				//abody->SetBodyFixed(true);
				//abody->SetPos_dt(ChVector<>(0, 0, 0));
			}
		}
	}
	return mass;
}

void SaveAllData(CHSYS * mSys, string prefix) {
	ofstream ofile;
	stringstream ss;

	char padnumber[100];
	sprintf(padnumber, "%d", (mFileNumber + 10000));
	char filename[100];

	//ss << prefix << mFileNumber << ".txt";
	ss << prefix << padnumber + 1 << ".txt";
	ofile.open(ss.str().c_str());
	//string output_text = "";
	stringstream line;
	ChVector<> spos = S->GetPos();
	ChQuaternion<> squat = S->GetRot();
	ChVector<> srot = squat.Q_to_NasaAngles();
	ChVector<> svel = S->GetPos_dt();
	line << -1 << "," << spos.x << "," << spos.y << "," << spos.z << ",";
	line << svel.x << "," << svel.y << "," << svel.z << ",";
	line << srot.x << "," << srot.y << "," << srot.z << "," << 0 << ",";
	line << endl;
	int counter = 0;
	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBODY * abody = (ChBODY *) mSys->Get_bodylist()->at(i);
		if (abody->IsActive() == true) {
			ChVector<> pos = abody->GetPos();
			//ChVector<> rot = abody->GetRot().Q_to_NasaAngles();
			ChQuaternion<> quat = abody->GetRot();
			ChVector<> vel = abody->GetPos_dt();
			ChVector<> acc = abody->GetPos_dtdt();
			//ChVector<> fap = abody->GetAppliedForce();
			line << counter << "," << pos.x << "," << pos.y << "," << pos.z << ",";
			line << vel.x << "," << vel.y << "," << vel.z << ",";
			line << quat.e0 << "," << quat.e1 << "," << quat.e2 << "," << quat.e3 << ",";
			line << endl;
			counter++;
		}
	}
	ofile << line.str();
	ofile.close();
	mFileNumber++;
}
template<class T>
void RunTimeStep(T * mSys, const int frame) {
	if (frame % 40 == 0)
		AddEllipsoids(mSys);


	float final = ((sqrt(2.0) / 2.0) * wall_th) + wall_th + GAP_OPEN;
	ChVector<> pos1 = S->GetPos();
	if (pos1.z < final) {
		//(GAP_OPEN / 1000.0 + wall_th) * .001*GAP_OPEN
		S->SetPos(pos1 + ChVector<>(0.0, 0.0, mTimeStep * 1.0));
		//S->SetPos_dt(ChVector<>(0, 0, mTimeStep / 1000.0) / mTimeStep);
		S->SetPos_dt(ChVector<>(0.0, 0.0, 1.0));
		open_gap = true;
	} else {
		S->SetPos_dt(ChVector<>(0.0, 0.0, 0.0));
	}

	int fps = float(1.0 / mTimeStep); // Frames per second
	int save_every = 1.0 / (60. / fps);
//	if (saveData && frame % 25 == 0) {
//		stringstream ss;
//		ss << ofileBase << "/G_" << GAP_OPEN << "_data";
//		SaveAllData(mSys, ss.str().c_str());
//		//saveData = false;
//	}

	//float mfr_output = GetMFR(mSys, -wall_len * 1.1);
//	stringstream ss;
//	ss << ofileBase << "/MFR_" << GAP_OPEN << "_" << FRIC << ".txt";
//	mfr_file.open(ss.str().c_str(), ios_base::app);
//	mfr_file << mSys->GetChTime() << " " << mfr_output << endl;
//	mfr_file.close();
	mFrameNumber++;
}

void LoadData(string fname, ChSystem * mSys) {
	ifstream ifile(fname.c_str());
	//ShapeType type = SPHERE;
	ShapeType type = ELLIPSOID;
	//ShapeType type = CYLINDER;
	ChBODYSHAREDPTR mrigidBody;
	float mu = FRIC, rest = 0;
	float rad_a = (3.0e-4);
	float rad_b = (2.0e-4);
	float rad_c = (2.0e-4);
	float mass = 2500.0 * (4.0 / 3.0) * CH_C_PI * rad_a * rad_b * rad_c;
	//float mass = 1.63153846e-7;
	rad_a = SF * rad_a;
	rad_b = SF * rad_b;
	rad_c = SF * rad_c;
	ChQuaternion<> quat = ChQuaternion<>(0, 0, 0, 0);
	ChVector<> dim = ChVector<>(rad_a, rad_b, rad_c);
	ChVector<> lpos(0, 0, 0);
	ChVector<> mParticlePos;
	ChVector<> mParticleVel;
	int iii;
	while (ifile.fail() == false && mNumCurrentObjects < 39000) {
		string temp;
		getline(ifile, temp);
		for (int i = 0; i < temp.size(); i++) {
			if (temp[i] == ',') {
				temp[i] = ' ';
			}
		}
		stringstream ss(temp);
		mrigidBody = ChBODYSHAREDPTR(new ChBODY);
		mrigidBody->SetIdentifier(mNumCurrentObjects + 1);
		ss >> iii;
#ifdef SIM_USE_GPU_MODE
		//mrigidBody->SetCollisionModelBullet();
#endif
		ss >> mParticlePos.x >> mParticlePos.y >> mParticlePos.z >> mParticleVel.x >> mParticleVel.y >> mParticleVel.z >> quat.e0 >> quat.e1 >> quat.e2 >> quat.e3;
		mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1 * rad_b);
		//mParticlePos=SF*mParticlePos; //this is already done in the file MFR_DATA_RE_SCALED.txt
		mParticlePos.y = (mParticlePos.y + wall_len) * 1.2 - wall_len;
		//mParticlePos.y=mParticlePos.y-SF*(5e-4);
		InitObject(mrigidBody, mass, mParticlePos, quat, mu, mu, rest, true, false, 0, 1);
		AddCollisionGeometry(mrigidBody, type, dim, lpos, QUNIT);
		//AddCollisionGeometry(mrigidBody, type, dim, Vector(-2*rad_b,0,0), QUNIT);
		//AddCollisionGeometry(mrigidBody, type, dim, Vector(0,0,0), QUNIT);
		//AddCollisionGeometry(mrigidBody, type, dim, Vector(2*rad_b,0,0), QUNIT);
		mrigidBody->SetInertiaXX((1.0 / 5.0) * mass * ChVector<>(rad_b * rad_b + rad_c * rad_c, rad_c * rad_c + rad_a * rad_a, rad_a * rad_a + rad_b * rad_b));
		FinalizeObject(mrigidBody, (CHSYS*) mSys);
		//mrigidBody->SetPos_dt(SF*mParticleVel);
		mNumCurrentObjects++;
		cout << mNumCurrentObjects << ", "; //Loaded 16318 on CPU before Segmentation Fault
	}
	cout << "READ IN " << mNumCurrentObjects << " OBJECTS" << endl;
}

void AddBodies(ChSystem * mSys) {
	//ShapeType type = SPHERE;
	//ShapeType type = ELLIPSOID;
	ShapeType type = BOX;
	//ShapeType type = CYLINDER;
	ChBODYSHAREDPTR mrigidBody;
	float mu = FRIC, rest = 0;
	float rad_a = (10.0e-4);
	float rad_b = (2.0e-4);
	float rad_c = (2.0e-4);
	float mass = 2500.0 * (4.0 / 3.0) * CH_C_PI * rad_a * rad_b * rad_c;
	//float mass = 1.63153846e-7;
	rad_a = SF * rad_a;
	rad_b = SF * rad_b;
	rad_c = SF * rad_c;
	ChQuaternion<> quat = ChQuaternion<>(1, 0, 0, 0);
	ChVector<> dim = ChVector<>(1, .2, .2);
	ChVector<> lpos(0, 0, 0);
	ChVector<> mParticlePos;
	int iii;

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 2; j++) {
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_len,2.0*ChRandom()*(wall_len-2*wall_th)-(wall_len-2*wall_th));
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_th,-ChRandom()*(wall_len-2*wall_th));
			mParticlePos.Set(j * (wall_dist) - wall_dist / 2.0, wall_th, -(wall_len - 2 * wall_th) + i * (wall_len / 6.0));
			quat.Set(ChRandom(), ChRandom(), ChRandom(), ChRandom());
			quat.Normalize();

			mrigidBody = ChBODYSHAREDPTR(new ChBODY);
			mrigidBody->SetIdentifier(mNumCurrentObjects + 1);
#ifdef SIM_USE_GPU_MODE
			//mrigidBody->SetCollisionModelBullet();
#endif
			mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1 * rad_b);
			InitObject(mrigidBody, mass, mParticlePos, quat, mu, mu, rest, true, false, 0, 1);

			//AddCollisionGeometry(mrigidBody, type, dim, Vector(-2*rad_a,0,0), QUNIT);
			//AddCollisionGeometry(mrigidBody, type, dim, Vector(0,0,0), QUNIT);
			//AddCollisionGeometry(mrigidBody, type, dim, Vector(2*rad_a,0,0), QUNIT);
			AddCollisionGeometry(mrigidBody, type, dim, lpos, QUNIT);
			mrigidBody->SetInertiaXX((1.0 / 5.0) * mass * ChVector<>(rad_b * rad_b + rad_c * rad_c, rad_c * rad_c + rad_a * rad_a, rad_a * rad_a + rad_b * rad_b));
			FinalizeObject(mrigidBody, (CHSYS*) mSys);
			mNumCurrentObjects++;
		}
	}
}

void AddBoxes(ChSystem * mSys) {
	ShapeType type = BOX;
	ChBODYSHAREDPTR mrigidBody;
	float mu = FRIC, rest = 0;
	float rad_a = (10.0e-4);
	float rad_b = (2.5e-4);
	float rad_c = (2.5e-4);
	float mass = 2500.0 * rad_a * rad_b * rad_c;
	//float mass = 1.63153846e-7;
	rad_a = SF * rad_a;
	rad_b = SF * rad_b;
	rad_c = SF * rad_c;
	ChQuaternion<> quat = ChQuaternion<>(1, 0, 0, 0);
	ChVector<> dim = ChVector<>(rad_a, rad_b, rad_c);
	ChVector<> lpos(0, 0, 0);
	ChVector<> mParticlePos;
	int iii;

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 2; j++) {
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_len,2.0*ChRandom()*(wall_len-2*wall_th)-(wall_len-2*wall_th));
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_th,-ChRandom()*(wall_len-2*wall_th));
			mParticlePos.Set(j * (wall_dist) - wall_dist / 2.0, wall_th, -(wall_len - 2 * wall_th) + i * (wall_len / 6.0));
			quat.Set(ChRandom(), ChRandom(), ChRandom(), ChRandom());
			quat.Normalize();

			mrigidBody = ChBODYSHAREDPTR(new ChBODY);
			mrigidBody->SetIdentifier(mNumCurrentObjects + 1);
#ifdef SIM_USE_GPU_MODE
			//mrigidBody->SetCollisionModelBullet();
#endif
			mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.5 * rad_b);
			InitObject(mrigidBody, mass, mParticlePos, quat, mu, mu, rest, true, false, 0, 1);
			AddCollisionGeometry(mrigidBody, type, dim, lpos, QUNIT);
			mrigidBody->SetInertiaXX((1.0 / 12.0) * mass * ChVector<>(rad_b * rad_b + rad_c * rad_c, rad_c * rad_c + rad_a * rad_a, rad_a * rad_a + rad_b * rad_b));
			FinalizeObject(mrigidBody, (CHSYS*) mSys);
			mNumCurrentObjects++;
		}
	}
}

void AddCylinders(ChSystem * mSys) {
	ShapeType type = CYLINDER;
	ChBODYSHAREDPTR mrigidBody;
	float mu = FRIC, rest = 0;
	float rad_a = (2.5e-4);
	float rad_b = (2.5e-4);
	float rad_c = (10.0e-4);
	float mass = 2500.0 * CH_C_PI * rad_a * rad_b * rad_c;
	//float mass = 1.63153846e-7;
	rad_a = SF * rad_a;
	rad_b = SF * rad_b;
	rad_c = SF * rad_c;
	ChQuaternion<> quat = ChQuaternion<>(1, 0, 0, 0);
	ChVector<> dim = ChVector<>(rad_a, rad_b, rad_c);
	ChVector<> lpos(0, 0, 0);
	ChVector<> mParticlePos;
	int iii;

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 2; j++) {
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_len,2.0*ChRandom()*(wall_len-2*wall_th)-(wall_len-2*wall_th));
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_th,-ChRandom()*(wall_len-2*wall_th));
			mParticlePos.Set(j * (wall_dist) - wall_dist / 2.0, wall_th, -(wall_len - 2 * wall_th) + i * (wall_len / 6.0));
			quat.Set(ChRandom(), ChRandom(), ChRandom(), ChRandom());
			quat.Normalize();

			mrigidBody = ChBODYSHAREDPTR(new ChBODY);
			mrigidBody->SetIdentifier(mNumCurrentObjects + 1);
#ifdef SIM_USE_GPU_MODE
			//mrigidBody->SetCollisionModelBullet();
#endif
			mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.5 * rad_b);
			InitObject(mrigidBody, mass, mParticlePos, quat, mu, mu, rest, true, false, 0, 1);
			AddCollisionGeometry(mrigidBody, type, dim, lpos, QUNIT);
			mrigidBody->SetInertiaXX(
					ChVector<>((1.0 / 12.0) * mass * (3 * rad_b * rad_b + rad_c * rad_c), (1.0 / 2.0) * mass * rad_a * rad_b, (1.0 / 12.0) * mass * (3 * rad_b * rad_b + rad_c * rad_c)));
			FinalizeObject(mrigidBody, (CHSYS*) mSys);
			mNumCurrentObjects++;
		}
	}
}

void AddEllipsoids(ChSystem * mSys) {
	ShapeType type = ELLIPSOID;
	ChBODYSHAREDPTR mrigidBody;
	float mu = FRIC, rest = 0;
	float rad_a = (10.0e-4);
	float rad_b = (2.5e-4);
	float rad_c = (2.5e-4);
	float mass = 2500.0 * (4.0 / 3.0) * CH_C_PI * rad_a * rad_b * rad_c;
	//float mass = 1.63153846e-7;
	rad_a = SF * rad_a;
	rad_b = SF * rad_b;
	rad_c = SF * rad_c;
	ChQuaternion<> quat = ChQuaternion<>(1, 0, 0, 0);
	ChVector<> dim = ChVector<>(1, .25, .25);
	ChVector<> lpos(0, 0, 0);
	ChVector<> mParticlePos;
	int iii;

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 2; j++) {
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_len,2.0*ChRandom()*(wall_len-2*wall_th)-(wall_len-2*wall_th));
			//mParticlePos.Set(-wall_dist+2*ChRandom()*wall_dist,wall_th,-ChRandom()*(wall_len-2*wall_th));
			mParticlePos.Set(j * (wall_dist) - wall_dist / 2.0, wall_th, -(wall_len - 2 * wall_th) + i * (wall_len / 6.0));
			quat.Set(ChRandom()/1000.0, ChRandom()/1000.0, ChRandom()/1000.0, ChRandom()/1000.0);
			quat.Normalize();

			mrigidBody = ChBODYSHAREDPTR(new ChBODY);
			mrigidBody->SetIdentifier(mNumCurrentObjects + 1);
#ifdef SIM_USE_GPU_MODE
			//mrigidBody->SetCollisionModelBullet();
#endif
			mrigidBody->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.5 * rad_b);
			InitObject(mrigidBody, mass, mParticlePos, quat, mu, mu, rest, true, false, 0, 1);
			AddCollisionGeometry(mrigidBody, type, dim, lpos, QUNIT);
			mrigidBody->SetInertiaXX((1.0 / 5.0) * mass * ChVector<>(rad_b * rad_b + rad_c * rad_c, rad_c * rad_c + rad_a * rad_a, rad_a * rad_a + rad_b * rad_b));
			FinalizeObject(mrigidBody, (CHSYS*) mSys);
			mNumCurrentObjects++;
		}
	}
}

int main(int argc, char *argv[]) {
	//feenableexcept(FE_ALL_EXCEPT);

	FRIC = 0.3; //atof(argv[2]);

	float mMu = FRIC;
	float mWallMu = 0.9; //1.05



	// Parse the command line arguments
	if (argc == 4) {
		GAP_OPEN = atof(argv[1]);
		ifileBase = argv[2];
		ofileBase = argv[3];
	} else {
		std::cout << "Invalid arguments, please try again.\n";
		return 0;
	}

	//	return 0;
#ifdef SIM_USE_GPU_MODE
	omp_set_num_threads(4);
#endif
	CHSYS *mSys;
#ifdef SIM_USE_GPU_MODE
	mSys = new CHSYS(50000);
#else
	mSys = new CHSYS(50000,SF*50.0,true);
#endif
	ChLCPDESC *mdescriptor = new ChLCPDESC();
	ChCONTACTCONT *mcontactcontainer = new ChCONTACTCONT();
	ChCOLLISIONSYS *mcollisionengine;
#ifdef SIM_USE_GPU_MODE
	mcollisionengine = new ChCOLLISIONSYS();
#else
	mcollisionengine = new ChCOLLISIONSYS(50000, SF*50.0);
#endif
	ChSOLVER *msolver;
#ifndef SIM_USE_GPU_MODE
	msolver = new ChSOLVER();
	mSys->ChangeLcpSolverSpeed(msolver);
#endif

	mSys->ChangeLcpSystemDescriptor(mdescriptor);
	mSys->ChangeContactContainer(mcontactcontainer);
	mSys->ChangeCollisionSystem(mcollisionengine);
	mSys->SetIntegrationType(ChSystem::INT_ANITESCU);
	mSys->Set_G_acc(Vector(0, -9.80665 * SF, 0));

#ifdef SIM_USE_GPU_MODE
	mSys->SetTolSpeeds( 1e-3);
	mSys->SetIterLCPmaxItersSpeed(500);
	//BLOCK_JACOBI
	//ACCELERATED_PROJECTED_GRADIENT_DESCENT
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetCompliance(0,0,0);
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetContactRecoverySpeed(0.01 ); //0.6
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetTolerance( 1e-3); //1e-3
	((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->SetMaxIteration(500); //1000
#else
			mSys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_APGD);
			//mSys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
			mSys->SetIterLCPmaxItersSpeed(500);
			mSys->SetTolSpeeds(SF*1e-3);
			mSys->SetMaxPenetrationRecoverySpeed(0.01*SF);// used by Anitescu stepper only 0.6
			mSys->SetIterLCPomega(1.0);
			mSys->SetIterLCPwarmStarting(false);
#endif

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);

	stringstream ss;
	ss << ofileBase << "/MFR_" << GAP_OPEN << "_" << FRIC << ".txt";
	mfr_file.open(ss.str().c_str());
	mfr_file.close();
	ChBODYSHAREDPTR L = ChBODYSHAREDPTR(new ChBODY);
	L->SetIdentifier(-1);
	ChBODYSHAREDPTR R = ChBODYSHAREDPTR(new ChBODY);
	R->SetIdentifier(-2);
	ChBODYSHAREDPTR F = ChBODYSHAREDPTR(new ChBODY);
	F->SetIdentifier(-3);
	ChBODYSHAREDPTR BASE = ChBODYSHAREDPTR(new ChBODY);
	BASE->SetIdentifier(-4);

	ChBODYSHAREDPTR BR = ChBODYSHAREDPTR(new ChBODY);
	BR->SetIdentifier(-5);
	ChBODYSHAREDPTR BL = ChBODYSHAREDPTR(new ChBODY);
	BL->SetIdentifier(-6);
	ChBODYSHAREDPTR BF = ChBODYSHAREDPTR(new ChBODY);
	BF->SetIdentifier(-7);
	ChBODYSHAREDPTR BB = ChBODYSHAREDPTR(new ChBODY);
	BB->SetIdentifier(-8);

	S = ChBODYSHAREDPTR(new ChBODY);
	S->SetIdentifier(-9);

#ifdef SIM_USE_GPU_MODE
//	L->SetCollisionModelBullet();
//	R->SetCollisionModelBullet();
//	F->SetCollisionModelBullet();
//	BASE->SetCollisionModelBullet();
//	BR->SetCollisionModelBullet();
//	BL->SetCollisionModelBullet();
//	BF->SetCollisionModelBullet();
//	BB->SetCollisionModelBullet();
//	S->SetCollisionModelBullet();
#endif

	L->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	R->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	F->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	BASE->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	BR->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	BL->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	BF->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	BB->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);
	S->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.1);

	ChQuaternion<> slope(1, 0, 0, 0);
	slope.Q_from_AngAxis(-CH_C_PI / 4.0, ChVector<>(1, 0, 0));

	InitObject(L, 1, ChVector<>(-wall_dist - wall_th, 0, 0), quat, mWallMu, mWallMu, 0, true, true, 32, 32);
	InitObject(R, 1, ChVector<>(wall_dist + wall_th, 0, 0), quat, mWallMu, mWallMu, 0, true, true, 20, 20);
	InitObject(F, 1, ChVector<>(0, 0, -wall_len), quat, mWallMu, mWallMu, 0, true, true, 20, 20);
	InitObject(BASE, 1, ChVector<>(0, -wall_len * 2, -wall_len), quat, 0, 0, 0, true, true, 20, 20);
	InitObject(BR, 1, ChVector<>(0, -wall_len, wall_len), quat, mWallMu, mWallMu, 0, true, true, 20, 20);
	InitObject(BL, 1, ChVector<>(0, -wall_len, -wall_len * 3), quat, mWallMu, mWallMu, 0, true, true, 20, 20);
	InitObject(BF, 1, ChVector<>(wall_len * 2, -wall_len, -wall_len), quat, mWallMu, mWallMu, 0, true, true, 20, 20);
	InitObject(BB, 1, ChVector<>(-wall_len * 2, -wall_len, -wall_len), quat, mWallMu, mWallMu, 0, true, true, 20, 20);
	//InitObject(S, 1, ChVector<>(0, 0, ((sqrt(2.0)/2.0)*wall_th) + wall_th + SF*(5e-4) ), slope, mWallMu, mWallMu, 0, true, true, 20, 20);
	InitObject(S, 1, ChVector<>(0, 0, ((sqrt(2.0) / 2.0) * wall_th) - wall_th), slope, mWallMu, mWallMu, 0, true, true, 20, 20);

	AddCollisionGeometry(L, BOX, ChVector<>(wall_th, wall_len, wall_len), lpos, quat);
	AddCollisionGeometry(R, BOX, ChVector<>(wall_th, wall_len, wall_len), lpos, quat);
	AddCollisionGeometry(F, BOX, ChVector<>(wall_dist + wall_th * 2, wall_len, wall_th), lpos, quat);
	AddCollisionGeometry(BASE, BOX, ChVector<>(wall_len * 2, wall_th, wall_len * 2), lpos, quat);
	AddCollisionGeometry(BR, BOX, ChVector<>(wall_len * 2, wall_len, wall_th), lpos, quat);
	AddCollisionGeometry(BL, BOX, ChVector<>(wall_len * 2, wall_len, wall_th), lpos, quat);
	AddCollisionGeometry(BF, BOX, ChVector<>(wall_th, wall_len, wall_len * 2), lpos, quat);
	AddCollisionGeometry(BB, BOX, ChVector<>(wall_th, wall_len, wall_len * 2), lpos, quat);
	AddCollisionGeometry(S, BOX, ChVector<>(wall_dist + wall_th * 2, wall_th, sqrt(wall_len * wall_len + wall_len * wall_len)), lpos, quat);

	FinalizeObject(L, (CHSYS*) mSys);
	FinalizeObject(R, (CHSYS*) mSys);
	FinalizeObject(F, (CHSYS*) mSys);
	FinalizeObject(S, (CHSYS*) mSys);
	FinalizeObject(BASE, (CHSYS*) mSys);
	FinalizeObject(BR, (CHSYS*) mSys);
	FinalizeObject(BL, (CHSYS*) mSys);
	FinalizeObject(BF, (CHSYS*) mSys);
	FinalizeObject(BB, (CHSYS*) mSys);

	cout << "LOAD DATA" << endl;
	//LoadData(ifileBase,mSys);
	//AddBodies(mSys);
	//mSys->Set_G_acc(ChVector<>(0, -9.80665 * SF, 0));
	mSys->SetStep(mTimeStep);
	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, mSys, 800, 600, 0, 0, "Test_Solvers");
	//openGLView.AddSystem(system_cpu);
	openGLView.SetCustomCallback(RunTimeStep);
	openGLView.StartSpinning(window_manager);
	window_manager->CallGlutMainLoop();




	//
	while (counter < 7.0 / mTimeStep) {
		//if (counter % 15 == 0)
			//AddEllipsoids(mSys);
		RunTimeStep(mSys, counter);
		mSys->DoStepDynamics(mTimeStep);

		double TIME = mSys->GetChTime();
		double STEP = mSys->GetTimerStep();
		double BROD = mSys->GetTimerCollisionBroad();
		double NARR = mSys->GetTimerCollisionNarrow();
		double LCP = mSys->GetTimerLcp();
		double UPDT = mSys->GetTimerUpdate();
		int BODS = mSys->GetNbodies();
		int CNTC = mSys->GetNcontacts();
		//int REQ_ITS=((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->GetCurrentIteration();
		int REQ_ITS = ((ChLcpSolverGPU*) (mSys->GetLcpSolverSpeed()))->GetTotalIterations();

		printf("%7.4f|%7.4f|%7.4f|%7.4f|%7.4f|%7.4f|%7d|%7d|%7d\n", TIME, STEP, BROD, NARR, LCP, UPDT, BODS, CNTC, REQ_ITS);

		cout << counter << endl;
		counter++;
		//
	}
	return 0;
}

