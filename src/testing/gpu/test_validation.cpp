#include "test_validation.h"
System::System(ChSystem * Sys, bool GPU){
	mSystem=Sys;
	mGPUSys=GPU;
	mTimeStep=.00005;
	mCurrentTime=0;
	mEndTime=1;
	mFrameNumber=0;
	mTriangleScale=1;
}

double System::GetKE(){
	mKE=0;
	unsigned int counter=0;
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBody *abody=mSystem->Get_bodylist()->at(i);

		double mass=abody->GetMass();
		ChVector<> inertia=abody->GetInertiaXX();
		ChVector<> omega=abody->GetWvel_par();
		double vel2=abody->GetPos_dt().Length2();
		mKE+=.5*mass*vel2+0.5 * ((omega*omega).Dot(inertia));
		counter++;
	}
	return mKE/SCALE;
}

void System::MakeSphere(SHAREDPTR &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide){
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetInertiaXX(ChVector<>(2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius));
	body->GetCollisionModel()->ClearModel();
	(BODYPTR)(body->GetCollisionModel())->AddSphere(radius);
	body->GetCollisionModel()->BuildModel();
	body->SetCollide(collide);
	body->SetImpactC(0);
	body->SetSfriction(sfric);
	body->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::MakeBox(SHAREDPTR &body, ChVector<> dim, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,int family,int nocolwith,bool collide, bool fixed){
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetInertiaXX(ChVector<>(1/12.0*mass*(dim.y*dim.y+dim.z*dim.z),1/12.0*mass*(dim.x*dim.x+dim.z*dim.z),1/12.0*mass*(dim.x*dim.x+dim.y*dim.y)));
	body->GetCollisionModel()->ClearModel();
	(BODYPTR)(body->GetCollisionModel())->AddBox(dim.x,dim.y,dim.z);
	body->GetCollisionModel()->BuildModel();
	body->SetCollide(collide);
	body->SetBodyFixed(fixed);
	body->SetImpactC(0.);
	body->SetSfriction(sfric);
	body->SetKfriction(kfric);
	mSystem->AddBody(body);
	//body->GetCollisionModel()->SetFamily(family);
	//body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(nocolwith);
}
void System::MakeEllipsoid(SHAREDPTR &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->GetCollisionModel()->ClearModel();
	(BODYPTR)(body->GetCollisionModel())->AddEllipsoid(radius.x,radius.y,radius.z);
	body->GetCollisionModel()->BuildModel();
	body->SetCollide(collide);
	body->SetImpactC(0);
	body->SetSfriction(sfric);
	body->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::drawAll(){
	if(updateDraw){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glFrontFace(GL_CCW);
		glCullFace(GL_BACK);
		glEnable(GL_CULL_FACE);
		glDepthFunc(GL_LEQUAL);
		glClearDepth(1.0);

		glPointSize(2);
		glLoadIdentity();


		float4 pitch_quat=CreateFromAxisAngle(cross(dir,camera_up), camera_pitch);
		float4 heading_quat=CreateFromAxisAngle(camera_up, camera_heading);

		dir=quatRotate(dir,normalize(mult(pitch_quat,heading_quat)));
		camera_pos+=camera_pos_delta;
		look_at=camera_pos+dir*1;

		camera_heading*=.5;
		camera_pitch*=.5;
		camera_pos_delta*=.5;

		gluLookAt(	
			camera_pos.x, camera_pos.y, camera_pos.z,
			look_at.x, look_at.y,  look_at.z,
			camera_up.x, camera_up.y,  camera_up.z);

		std::vector<ChBody*>::iterator abody = mSystem->Get_bodylist()->begin();
		while (abody != mSystem->Get_bodylist()->end()){
			BODYPTR cModel = (BODYPTR)((*abody)->GetCollisionModel());
			if(cModel->GetShapeType()==SPHERE){
				drawSphere(*abody);
			}
			if(cModel->GetShapeType()==BOX){
				drawBox(*abody);
			}
			if(cModel->GetShapeType()==ELLIPSOID){
				drawEllipsoid(*abody);
			}
			if(cModel->GetShapeType()==TRIANGLEMESH){
				glColor3f (0,0,0);
				drawTriMesh(TriMesh,(*abody));
			}
			abody++;
		}
		Sleep(30);
		glutSwapBuffers();
	}
}

void System::LoadTriangleMesh(string name, ChVector<> Pos, ChQuaternion<> Rot, float mass){
	ChSharedBodyPtr mrigidBody;
	ifstream ifile(name.c_str());
	string temp,j;
	vector<float3> pos, tri;
	float3 tempF;
	while(ifile.fail()==false){
		getline(ifile,temp);
		if(temp.size()>2&&temp[0]!='#'&&temp[0]!='g'){
			if(temp[0]=='v'){
				stringstream ss(temp);
				ss>>j>>tempF.x>>tempF.y>>tempF.z;
				pos.push_back(tempF);
			}
			if(temp[0]=='f'){
				stringstream ss(temp);
				ss>>j>>tempF.x>>tempF.y>>tempF.z;
				tri.push_back(tempF);
			}
		}
	}

	mrigidBody = ChSharedBodyPtr(new BODYTYPE);
	for(int i=0; i<tri.size(); i++){
		ChVector<> A(pos[tri[i].x-1].x,pos[tri[i].x-1].y,pos[tri[i].x-1].z);
		ChVector<> B(pos[tri[i].y-1].x,pos[tri[i].y-1].y,pos[tri[i].y-1].z);
		ChVector<> C(pos[tri[i].z-1].x,pos[tri[i].z-1].y,pos[tri[i].z-1].z);
		A*=mTriangleScale;
		B*=mTriangleScale;
		C*=mTriangleScale;
		TriMesh.addTriangle(A,B,C);
	}
	mrigidBody->SetMass(100000);
	ChVector<> particle_pos2(-30,-70,0);
	mrigidBody->SetPos(Pos);

	mrigidBody->GetCollisionModel()->ClearModel();
	mrigidBody->GetCollisionModel()->AddTriangleMesh(TriMesh,true,false);
	mrigidBody->GetCollisionModel()->BuildModel();
	mrigidBody->SetBodyFixed(true);
	mrigidBody->SetCollide(true);
	mrigidBody->SetImpactC(0.0);
	mrigidBody->SetSfriction(.9);
	mrigidBody->SetKfriction(.9);
	mrigidBody->SetRot(Rot);
	mSystem->AddBody(mrigidBody);
	//mrigidBody->GetCollisionModel()->SetFamily(1);
	//mrigidBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);



}

void System::DoTimeStep(){
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	if(mSystem->GetChTime()>=mEndTime){exit(0);}
}
void System::PrintStats(){
	double A=mSystem->GetChTime();
	double B=mSystem->GetTimerStep();
	double C=mSystem->GetTimerCollisionBroad();
	double D=mSystem->GetTimerLcp();
	int E=mSystem->GetNbodies();
	int F=mSystem->GetNcontacts();
	double G=GetKE();
	char numstr[512];
	//printf("%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %14.8f\n",A,B,C,D,E,F,G);
	sprintf(numstr,"%7.4f\t%7.4f\t%7.4f\t%7.4f\t%7d\t%7d\t%14.8f",A,B,C,D,E,F,G);
	mTimingFile<<numstr<<endl;
}
void initScene(){
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };			
	glClearColor (1.0, 1.0, 1.0, 0.0);							
	glShadeModel (GL_SMOOTH);									
	glEnable(GL_COLOR_MATERIAL);								
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);			
	glEnable(GL_LIGHTING);										
	glEnable(GL_LIGHT0);
	glEnable (GL_POINT_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_POINT_SMOOTH_HINT, GL_DONT_CARE);

}


System *GPUSystem;
void renderSceneAll(){
	if(OGL){GPUSystem->drawAll();}
}
int main(int argc, char* argv[]){
	float mOmega=.3;
	int mIteations=50;
	float mTimeStep=.001;
	float mEnvelope=0;
	float mSphereMu=.15;
	float mWallMu=.05;
	int mDevice=0;
	
	//mIteations=atoi(argv[1]);
	//mTimeStep=atof(argv[2]);
	//mOmega=atof(argv[3]);
	//mEnvelope=atof(argv[4]);
	//mSphereMu=atof(argv[5]);
	//mWallMu=atof(argv[6]);
	//mDevice=atoi(argv[7]);

	ChSystem Sys(1000, 50); 
#ifdef USEBULLET

	Sys.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	Sys.SetIterLCPmaxItersSpeed(200);
	Sys.SetIterLCPmaxItersStab(50);
	//Sys.SetMaxPenetrationRecoverySpeed(1.6); // used by Anitescu stepper only
	Sys.SetIterLCPwarmStarting(false);

#else
	cudaSetDevice(mDevice);

	ChLcpSystemDescriptorGPU		mGPUDescriptor;
	ChContactContainerGPUsimple		mGPUContactContainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(&mGPUDescriptor, mEnvelope);
	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUContactContainer,&mGPUDescriptor,  mIteations,mTimeStep, 1e-5, mOmega, false);
	Sys.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	Sys.ChangeContactContainer(&mGPUContactContainer);
	Sys.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	Sys.ChangeCollisionSystem(&mGPUCollisionEngine);
	Sys.SetIntegrationType(ChSystem::INT_ANITESCU);

#endif
	Sys.Set_G_acc(ChVector<>(0,GRAV,0));
	Sys.SetStep(mTimeStep);

	GPUSystem=new System(&Sys,1);
	GPUSystem->mMu=mSphereMu;
	GPUSystem->mTimeStep=mTimeStep;

	//GPUSystem->LoadTriangleMesh("capsule2.obj");
	char numstr[256]; // enough to hold all numbers up to 64-bits
	sprintf(numstr, "TEST_%d_%f_%f_%f_%f_%f_%f.txt", mIteations,mTimeStep,mOmega);
	GPUSystem->mTimingFile.open(numstr);
	GPUSystem->mTimingFile<<"Time:\tTotal:\tCD:\tSolver:\tBodies:\tContacts:\tKE:"<<endl;

	ChQuaternion<> base(1,0,0,0);

	SHAREDPTR floor	=	SHAREDPTR(new BODYTYPE);
	GPUSystem->MakeBox(floor,	ChVector<>(10,.01,1),1,ChVector<>(0.0,-.01,0),base,.2,.2,0,0,0,true,true);

	SHAREDPTR ball1	=	SHAREDPTR(new BODYTYPE);
	GPUSystem->MakeSphere(ball1,1,1,ChVector<>(-5,1,0),.2,.2,0,1);
	ball1->SetPos_dt(ChVector<>(2,0,0));
	
#pragma omp parallel sections
	{

#pragma omp section
		{
			ofstream ball_file("ball_data.txt");

			while(true){
				GPUSystem->renderScene();
				ChVector<> pos, vel, acc, rot;
				pos=ball1->GetPos();
				vel=ball1->GetPos_dt();
				acc=ball1->GetPos_dtdt();
				rot=ball1->GetWvel_par();



				double mass=ball1->GetMass();
				ChVector<> inertia=ball1->GetInertiaXX();
				double vel2=vel.Length2();
				double ke=.5*mass*vel2+0.5 * ((rot*rot).Dot(inertia));
				char numstr[512];
				sprintf(numstr,"%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f\t%14.8f",pos.x,pos.y,pos.z,vel.x,vel.y,vel.z,acc.x,acc.y,acc.z,rot.x,rot.y,rot.z,ke);
				ball_file<<numstr<<endl;
			}
			ball_file.close();
		}
#pragma omp section
		{
			if(OGL){
				glutInit(&argc, argv);									
				glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);	
				glutInitWindowPosition(0,0);								
				glutInitWindowSize(1024	,512);
				glutCreateWindow("MAIN");
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
		}
	}
	return 0;
}