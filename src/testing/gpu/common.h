#include "render.h"

class System {
	public:
		System(int cudaDevice);
		~System() {
		}
		void Setup() {
			mGPUCollisionEngine->mGPU->collision_envelope = mEnvelope;
			mGPUsolverSpeed->mDt = mTimeStep;
			mGPUsolverSpeed->mMaxIterations = mIterations;
			mGPUsolverSpeed->mOmegaContact = mOmegaContact;
			mGPUsolverSpeed->mOmegaBilateral = mOmegaBilateral;
			mGPUsolverSpeed->mTolerance = mTolerance;
		}
		void SetTimingFile(string fname) {
			mTimingFile.open(fname.c_str());
		}

		double GetKE();
		double GetMFR(double height);
		void DoTimeStep();
		void PrintStats();
		void InitObject(ChSharedPtr<ChBodyGPU> &body, double mass, ChVector<> pos, ChQuaternion<> rot, double sfric, double kfric, double restitution, bool collide, bool fixed, int family,
				int nocolwith);
		void AddCollisionGeometry(ChSharedPtr<ChBodyGPU> &body, ShapeType type, ChVector<> dim, ChVector<> lPos, ChQuaternion<> lRot);
		void FinalizeObject(ChSharedPtr<ChBodyGPU> body);
		void DeactivationPlane(float y, float h, bool disable);
		void BoundingPlane(float y);
		void BoundingBox(float x, float y, float z, float offset);
		void LoadSpheres(string fname, int skip, float mass, float rad, float mu);
		void SaveByID(int id, string fname);
		void SaveByObject(ChBodyGPU *abody, string fname);
		void SaveAllData(string prefix);
		void drawAll();
		ChSystemGPU *mSystem;

		ChLcpSystemDescriptorGPU *mGPUDescriptor;
		ChContactContainerGPUsimple *mGPUContactContainer;
		ChCollisionSystemGPU *mGPUCollisionEngine;
		ChLcpIterativeSolverGPUsimple *mGPUsolverSpeed;

		int mNumCurrentSpheres, mNumCurrentObjects;

		double mTimeStep, mTotalTime, mCurrentTime, mEndTime;

		double mOffsetX, mOffsetY, mOffsetZ;
		int mNumObjects;
		int mFrameNumber;
		int mFileNumber;
		int mCudaDevice;
		int mIterations;

		float mOmegaContact, mOmegaBilateral, mEnvelope, mTolerance;

		bool mUseOGL;
		bool mSaveData;
		ChTimer<double> mTimer;

		ofstream mTimingFile;
};

System::System(int cudaDevice) {
	mGPUDescriptor = new ChLcpSystemDescriptorGPU();
	mGPUContactContainer = new ChContactContainerGPUsimple();
	mGPUCollisionEngine = new ChCollisionSystemGPU(mGPUDescriptor);
	mGPUsolverSpeed = new ChLcpIterativeSolverGPUsimple(mGPUContactContainer, mGPUDescriptor);

	mSystem = new ChSystemGPU();
	mSystem->ChangeLcpSystemDescriptor(mGPUDescriptor);
	mSystem->ChangeContactContainer(mGPUContactContainer);
	mSystem->ChangeLcpSolverSpeed(mGPUsolverSpeed);
	mSystem->ChangeCollisionSystem(mGPUCollisionEngine);
	mSystem->SetIntegrationType(ChSystem::INT_ANITESCU);
	mSystem->Set_G_acc(ChVector<> (0, -9.80665, 0));

	mCurrentTime = 0;
	//mCameraX = 0, mCameraY = 0, mCameraZ = 0;

	mNumCurrentObjects = 0;
	mFrameNumber = 0;
	mFileNumber = 0;
	mTotalTime = 0;
	mOmegaBilateral = .2;
	mOmegaContact = .2;
	mEnvelope = 0;
	mTolerance = 1e-3;
	mIterations = 1000;
	mCudaDevice = 0;
	mUseOGL = 0;
	mCudaDevice = cudaDevice;
	cudaSetDevice(mCudaDevice);
}

void System::PrintStats() {
	double A = mSystem->GetChTime();
	double B = mSystem->GetTimerStep();
	double C = mSystem->GetTimerCollisionBroad();
	double D = mSystem->GetTimerLcp();
	int E = mSystem->GetNbodies();
	int F = mSystem->GetNcontacts();
	int I = ((ChLcpIterativeSolverGPUsimple*) (mSystem->GetLcpSolverSpeed()))->gpu_solver->iteration_number;
	mTotalTime += mTimer();
	double KE = GetKE();
	char numstr[512];
	printf("%7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7.7f | %d\n", A, B, C, D, mTotalTime, E, F, KE, I);
	sprintf(numstr, "%7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %f | %d\n", A, B, C, D, mTotalTime, E, F, KE, I);

	if (mTimingFile.fail() == false) {
		mTimingFile << numstr;
	}
}

void System::LoadSpheres(string fname, int skip, float mass, float rad, float mu) {
	ifstream ifile(fname.c_str());
	if(ifile.fail()==true){cout<<"FAIL"<<endl;}
	string temp;
	int counter = 0;
	ChSharedBodyGPUPtr mrigidBody;

	ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0,0,0);
	ChVector<> dim(rad,0,0);
	while (ifile.fail() == false) {
		getline(ifile, temp);
		if (counter >= skip) {
			for (int i = 0; i < temp.size(); i++) {
				if (temp[i] == ',') {
					temp[i] = ' ';
				}
			}
			stringstream ss(temp);

			ChVector<> pos, vel, acc;
			ss >> pos.x >> pos.y >> pos.z;
			ss >> vel.x >> vel.y >> vel.z;
			ss >> acc.x >> acc.y >> acc.z;
			mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
			InitObject(mrigidBody, mass, pos, quat, mu, mu, 0, true, false, 0, 1);
			AddCollisionGeometry(mrigidBody, SPHERE, dim, lpos, quat);
			FinalizeObject(mrigidBody);
			mNumCurrentObjects++;

		}
counter++;
	}

}

void System::SaveByID(int id, string fname) {
	ofstream ofile;
	ofile.open(fname.c_str(), ios_base::app);

	ChBodyGPU * abody = (ChBodyGPU *) mSystem->Get_bodylist()->at(id);
	ChVector<> pos = abody->GetPos();
	ChVector<> rot = abody->GetRot().Q_to_NasaAngles();
	ChVector<> vel = abody->GetPos_dt();
	ChVector<> acc = abody->GetPos_dtdt();
	ChVector<> fap = abody->GetAppliedForce();
	if (isnan(rot.x)) {
		rot.x = 0;
	}
	if (isnan(rot.y)) {
		rot.y = 0;
	}
	if (isnan(rot.z)) {
		rot.z = 0;
	}
	ofile << pos.x << "," << pos.y << "," << pos.z << ",";
	ofile << vel.x << "," << vel.y << "," << vel.z << ",";
	ofile << acc.x << "," << acc.y << "," << acc.z << ",";
	ofile << rot.x << "," << rot.y << "," << rot.z << ",";
	ofile << fap.x << "," << fap.y << "," << fap.z << ",";
	ofile << endl;

	ofile.close();
}

void System::SaveAllData(string prefix) {
	ofstream ofile;
	stringstream ss;
	ss << prefix << mFileNumber << ".txt";
	ofile.open(ss.str().c_str());
	ofile.close();
	for (int i = 0; i < mSystem->Get_bodylist()->size(); i++) {
		SaveByID(i, ss.str().c_str());
	}
	mFileNumber++;
}

void System::SaveByObject(ChBodyGPU *abody, string fname) {
	ofstream ofile;
	ofile.open(fname.c_str(), ios_base::app);
	ChVector<> pos = abody->GetPos();
	ChVector<> rot = abody->GetRot().Q_to_NasaAngles();
	ChVector<> vel = abody->GetPos_dt();
	ChVector<> acc = abody->GetPos_dtdt();
	ChVector<> fap = abody->GetAppliedForce();

	if (isnan(rot.x)) {
		rot.x = 0;
	}
	if (isnan(rot.y)) {
		rot.y = 0;
	}
	if (isnan(rot.z)) {
		rot.z = 0;
	}

	ofile << pos.x << "," << pos.y << "," << pos.z << ",";
	ofile << vel.x << "," << vel.y << "," << vel.z << ",";
	ofile << acc.x << "," << acc.y << "," << acc.z << ",";
	ofile << rot.x << "," << rot.y << "," << rot.z << ",";
	ofile << fap.x << "," << fap.y << "," << fap.z << ",";
	ofile << endl;

	ofile.close();
}
double System::GetKE() {
	return mSystem->GetKineticEnergy() / SCALE;
}

double System::GetMFR(double height) {
	return 0;
}

void System::InitObject(ChSharedPtr<ChBodyGPU> &body, double mass, ChVector<> pos, ChQuaternion<> rot, double sfric, double kfric, double restitution, bool collide, bool fixed, int family,
		int nocolwith) {
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetCollide(collide);
	body->SetBodyFixed(fixed);
	body->SetImpactC(restitution);
	body->SetSfriction(sfric);
	body->SetKfriction(kfric);
	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->SetFamily(family);
	body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(nocolwith);
	body->SetLimitSpeed(true);

}
void System::AddCollisionGeometry(ChSharedPtr<ChBodyGPU> &body, ShapeType type, ChVector<> dim, ChVector<> lPos, ChQuaternion<> lRot) {
	ChMatrix33<> *rotation = new ChMatrix33<> (lRot);
	ChCollisionModelGPU * model = (ChCollisionModelGPU*) body->GetCollisionModel();
	if (type == SPHERE) {
		model->AddSphere(dim.x, &lPos);
	}
	if (type == ELLIPSOID) {
		model->AddEllipsoid(dim.x, dim.y, dim.z, &lPos, rotation);
	}
	if (type == BOX) {
		model->AddBox(dim.x, dim.y, dim.z, &lPos, rotation);
	}
	if (type == CYLINDER) {
		model->AddCylinder(dim.x, dim.y, dim.z, &lPos, rotation);
	}

}
void System::FinalizeObject(ChSharedPtr<ChBodyGPU> newbody) {
	newbody->GetCollisionModel()->BuildModel();
	mSystem->AddBody(newbody);

}
void System::DeactivationPlane(float y, float h, bool disable) {
	for (int i = 0; i < mSystem->Get_bodylist()->size(); i++) {
		ChBodyGPU *abody = (ChBodyGPU*) (mSystem->Get_bodylist()->at(i));
		if (abody->GetPos().y < y) {
			abody->SetCollide(!disable);
			abody->SetBodyFixed(disable);
			abody->SetPos(ChVector<> (0, h, 0));
		}
	}
}
void System::BoundingPlane(float y) {
	for (int i = 0; i < mSystem->Get_bodylist()->size(); i++) {
		ChBodyGPU *abody = (ChBodyGPU*) (mSystem->Get_bodylist()->at(i));
		if (abody->GetPos().y < y && abody->GetBodyFixed() == false) {
			abody->SetPos(ChVector<> (abody->GetPos().x, y, abody->GetPos().z));
			abody->SetPos_dt(ChVector<> (0, 0, 0));
		}
	}
}
void System::BoundingBox(float x, float y, float z, float offset) {
	for (int i = 0; i < mSystem->Get_bodylist()->size(); i++) {
		ChBodyGPU *abody = (ChBodyGPU*) (mSystem->Get_bodylist()->at(i));

		if (abody->GetBodyFixed() == true) {
			continue;
		}
		ChVector<> pos = abody->GetPos();
		ChVector<> vel = abody->GetPos_dt();
		ChVector<> force(0, 0, 0);
		if (pos.x + offset > x || pos.x - offset < -x) {
			force.x = (x - pos.x);
		}
		if (pos.y + offset > y || pos.y - offset < -y) {
			force.y = (y - pos.y);
		}
		if (pos.z + offset > z || pos.z - offset < -z) {
			force.z = (z - pos.z);
		}

		abody->Set_Scr_force(force * .1);
	}
}

void System::drawAll() {

	if (updateDraw && (mSystem->GetChTime() > this->mTimeStep * 2)) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();

		float4 pitch_quat = CreateFromAxisAngle(cross(dir, camera_up), camera_pitch);
		float4 heading_quat = CreateFromAxisAngle(camera_up, camera_heading);

		dir = quatRotate(dir, normalize(mult(pitch_quat, heading_quat)));
		camera_pos += camera_pos_delta;
		look_at = camera_pos + dir * 1;

		camera_heading *= .5;
		camera_pitch *= .5;
		camera_pos_delta *= .5;
		newaverage = 0;
		gluLookAt(camera_pos.x, camera_pos.y, camera_pos.z, look_at.x, look_at.y, look_at.z, camera_up.x, camera_up.y, camera_up.z);
		//points.resize(mSystem->Get_bodylist()->size());
		points.clear();
		for (int i = 0; i < mSystem->Get_bodylist()->size(); i++) {
			ChBodyGPU* abody = (ChBodyGPU*) mSystem->Get_bodylist()->at(i);
			drawObject(abody);
		}
		averageVal = newaverage / mSystem->Get_bodylist()->size();

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(3, GL_FLOAT, sizeof(Point), &points[0].x);
		glColorPointer(3, GL_FLOAT, sizeof(Point), &points[0].r);
		glPointSize(10.0);
		glDrawArrays(GL_POINTS, 0, points.size());
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);

		if (showContacts) {
			ChLcpSystemDescriptorGPU* mGPUDescriptor = (ChLcpSystemDescriptorGPU *) mSystem->GetLcpSystemDescriptor();
			for (int i = 0; i < mGPUDescriptor->gpu_collision->data_container->device_norm_data.size(); i++) {
				float3 N = mGPUDescriptor->gpu_collision->data_container->device_norm_data[i];
				float3 Pa = mGPUDescriptor->gpu_collision->data_container->device_cpta_data[i];
				float3 Pb = mGPUDescriptor->gpu_collision->data_container->device_cptb_data[i];
				float D = mGPUDescriptor->gpu_collision->data_container->device_dpth_data[i];
				float3 color = GetColour(D, 0, .0001);
				glColor3f(color.x, color.y, color.z);
				glBegin(GL_LINES);
				glVertex3f(Pa.x, Pa.y, Pa.z);
				glVertex3f(Pb.x, Pb.y, Pb.z);
				glEnd();
				glBegin(GL_POINTS);
				glVertex3f(Pa.x, Pa.y, Pa.z);
				glVertex3f(Pb.x, Pb.y, Pb.z);
				glEnd();

			}
		}
#if defined( _WINDOWS )
		Sleep( 40 );
#else
		usleep(40 * 1000);
#endif
		glutSwapBuffers();
	}

}
System *GPUSystem;

void renderSceneAll() {
	GPUSystem->drawAll();
}
void initGLUT(string name, int argc, char* argv[]) {

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(1024, 512);
	glutCreateWindow(name.c_str());
	glutDisplayFunc(renderSceneAll);
	glutIdleFunc(renderSceneAll);
	glutReshapeFunc(changeSize);
	glutIgnoreKeyRepeat(0);
	glutKeyboardFunc(processNormalKeys);
	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMove);
	initScene();
	glutMainLoop();

}

void SimulationLoop(int argc, char* argv[]) {

#pragma omp parallel sections
	{
#pragma omp section
		{

			while (GPUSystem->mSystem->GetChTime() <= GPUSystem->mEndTime) {
				GPUSystem->mTimer.start();
				GPUSystem->DoTimeStep();
				GPUSystem->mTimer.stop();
				GPUSystem->PrintStats();

			}
			cout << "Simulation Complete" << endl;
		}
#pragma omp section
		{
			if (GPUSystem->mUseOGL) {
				initGLUT(string("test"), argc, argv);
			}
		}
	}
}

