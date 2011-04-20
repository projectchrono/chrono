#include "demo_gpuCD.h"
ChSharedBodyGPUPtr rightWall;
ChSharedBodyGPUPtr leftWall;	
ChSharedBodyGPUPtr flatWall;	
ChSharedBodyGPUPtr movingWall;
ChSharedBodyGPUPtr ground;

System::System(ChSystem * Sys, bool GPU){
	mSystem=Sys;
	mGPUSys=GPU;
	mNumCurrentSpheres=0;
	saveTimingData=0;
	saveSimData=false;
	mSphereRadius=2.5e-4*SCALE;
	mSphereEnvelope=0;
	mSphereMass = 1.632e-7*SCALE;
	mSphereRestitution=0;
	mTimeStep=.00005;
	mCurrentTime=0;
	mMu=.15;
	mTriangleScale=20;
	mEndTime=5;
	mCameraX=0, mCameraY=0, mCameraZ=0;
	mBoundingBoxSize=40;
	mNumSpheres=39000;
	mOffsetX=0;
	mOffsetY=6;
	mOffsetZ=0;
	mNumCurrentSpheres=0;
	mFrameNumber=0;
	mFileNumber=0;
	mSaveEveryX=6;
	mCudaDevice=0;
}

double System::GetKE(){
	mKE=0;
	unsigned int counter=0;
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBody *abody=mSystem->Get_bodylist()->at(i);
		double mass=abody->GetMass();
		double vel2=abody->GetPos_dt().Length2();
		//cout<<vel2<<endl;
		mKE+=.5*mass*vel2;
		counter++;
	}
	return mKE/SCALE;
}

double System::GetMFR(double height){
	mMassFlowRate=0;
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBody *abody=mSystem->Get_bodylist()->at(i);
		if(abody->GetPos().y<height){
			double mass=abody->GetMass();
			mMassFlowRate+=mass;
		}
	}
	return (mMassFlowRate)/SCALE;

}


void System::MakeSphere(ChSharedBodyPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetInertiaXX(ChVector<>(2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius));
	body.get_ptr()->GetCollisionModel()->ClearModel();
	if(mGPUSys){(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddSphere(mSphereRadius);}
	else{body.get_ptr()->GetCollisionModel()->AddSphere(radius);}
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::MakeBox(ChSharedBodyGPUPtr &body, ChVector<> dim, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,int family,int nocolwith,bool collide, bool fixed){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetRot(rot);
	body.get_ptr()->SetInertiaXX(ChVector<>(1/12.0*mass*(dim.y*dim.y+dim.z*dim.z),1/12.0*mass*(dim.x*dim.x+dim.z*dim.z),1/12.0*mass*(dim.x*dim.x+dim.y*dim.y)));
	body.get_ptr()->GetCollisionModel()->ClearModel();
	(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddBox(dim.x,dim.y,dim.z);
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetBodyFixed(fixed);
	body.get_ptr()->SetImpactC(0.);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
	(body.get_ptr()->GetCollisionModel())->SetFamily(family);
	(body.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(nocolwith);
}
void System::MakeEllipsoid(ChSystem* mSystem,ChSharedBodyPtr &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetRot(rot);
	body.get_ptr()->GetCollisionModel()->ClearModel();
	if(mGPUSys){(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddEllipsoid(radius.x,radius.y,radius.z);}
	else{body.get_ptr()->GetCollisionModel()->AddEllipsoid(radius.x,radius.y,radius.z);}
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::CreateSpheres(int x, int y, int z, double posX, double posY, double posZ, bool rnd){
	if(load_file){
		ifstream ifile ("OUTDATA.txt");
		ChSharedBodyPtr mrigidBodya;
		for (int zz=0; zz<39480; zz++){
			//ChVector<> mParticlePos((xx),(yy)+4,(zz));
			ChVector<> mParticlePos(0,0,0);
			ifile>>mParticlePos.x>>mParticlePos.y>>mParticlePos.z;
			mrigidBodya = ChSharedBodyPtr(new ChBodyGPU);
			MakeSphere(mrigidBodya, mSphereRadius, mSphereMass, mParticlePos, mMu, mMu, mSphereRestitution, true);
		}
		ifile.close();
	}
	
	mNumCurrentSpheres+=x*y*z;
	ChSharedBodyPtr mrigidBody;
	for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
			for (int zz=0; zz<z; zz++){
				ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
				if(rnd){
					mParticlePos.x+=(rand()%1000)/4000.f;
					mParticlePos.y+=(rand()%1000)/4000.f;
					mParticlePos.z+=(rand()%1000)/4000.f;
				}
				mrigidBody = ChSharedBodyPtr(new ChBodyGPU);
				MakeSphere(mrigidBody, mSphereRadius, mSphereMass, mParticlePos*.0006*SCALE, mMu, mMu, mSphereRestitution, true);
			}
		}
	}
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
			ChCollisionModelGPU* cModel = (ChCollisionModelGPU*)((*abody)->GetCollisionModel());
			if(cModel->GetShapeType()==SPHERE){
				drawSphere((*abody)->GetPos(),(*abody)->GetPos_dt().Length(), mSphereRadius);
			}
			if(cModel->GetShapeType()==BOX){
				drawBox(*abody,mSphereRadius,mGPUSys);
			}
			if(cModel->GetShapeType()==ELLIPSOID){
				drawSphere(*abody,mGPUSys);
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

	mrigidBody = ChSharedBodyPtr(new ChBodyGPU);
	for(int i=0; i<tri.size(); i++){
		ChVector<> A(pos[tri[i].x-1].x,pos[tri[i].x-1].y,pos[tri[i].x-1].z);
		ChVector<> B(pos[tri[i].y-1].x,pos[tri[i].y-1].y,pos[tri[i].y-1].z);
		ChVector<> C(pos[tri[i].z-1].x,pos[tri[i].z-1].y,pos[tri[i].z-1].z);
		A*=mTriangleScale;
		B*=mTriangleScale;
		C*=mTriangleScale;
		TriMesh.addTriangle(A,B,C);
	}
	mrigidBody.get_ptr()->SetMass(100000);
	ChVector<> particle_pos2(-30,-70,0);
	mrigidBody.get_ptr()->SetPos(Pos);

	mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
	mrigidBody.get_ptr()->GetCollisionModel()->AddTriangleMesh(TriMesh,true,false);
	mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
	mrigidBody.get_ptr()->SetBodyFixed(true);
	mrigidBody.get_ptr()->SetCollide(true);
	mrigidBody.get_ptr()->SetImpactC(0.0);
	mrigidBody.get_ptr()->SetSfriction(.9);
	mrigidBody.get_ptr()->SetKfriction(.9);
	mrigidBody.get_ptr()->SetRot(Rot);
	mSystem->AddBody(mrigidBody);
	mrigidBody.get_ptr()->GetCollisionModel()->SetFamily(1);
	mrigidBody.get_ptr()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);



}

void System::DoTimeStep(){
	if(mNumCurrentSpheres<mNumSpheres&&mFrameNumber%1==0&&!load_file){CreateSpheres(1+mFrameNumber, 1, 14, -48+mFrameNumber/2.0, -55+mFrameNumber, 0, true);}
	ChVector<> pos=movingWall->GetPos();

	if(pos.x<(.0025)*SCALE&&movewall){
		movingWall->SetPos(movingWall->GetPos()+ChVector<>(1/1000.0*mTimeStep,0,0)*SCALE);
	}
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
		if(abody->GetPos().y<-60){
			abody->SetCollide(false);
			abody->SetBodyFixed(true);
			abody->SetPos(ChVector<>(0,-60,0));
		}
	}
	if(savenow){
		ofstream ofile("OUTDATA.txt");
		for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
			ChVector<> pos=mSystem->Get_bodylist()->at(i)->GetPos();
			ofile<<pos.x<<"\t"<<pos.y<<"\t"<<pos.z<<endl;
		}
		ofile.close();
		savenow=false;
	}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	if(mSystem->GetChTime()>=mEndTime){exit(0);}
	if(mSystem->GetChTime()>=1.0){movewall=true;}
	if(mSystem->GetNbodies()==0){exit(0);}
}
void System::PrintStats(){
	double A=mSystem->GetChTime();
	double B=mSystem->GetTimerStep();
	double C=mSystem->GetTimerCollisionBroad();
	double D=mSystem->GetTimerLcp();
	int E=mSystem->GetNbodies();
	int F=mSystem->GetNcontacts();
	double G=GetKE();
	double H=GetMFR(-50);
	char numstr[512];
	printf("%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %14.8f | %14.8f\n",A,B,C,D,E,F,G,H);
	sprintf(numstr,"%7.4f\t%7.4f\t%7.4f\t%7.4f\t%7d\t%7d\t%14.8f\t%14.8f",A,B,C,D,E,F,G,H);

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


	float mOmega=.5;
	int mIteations=500;
	float mTimeStep=.00005;
	float mEnvelope=0.25e-4;
	float mSphereMu=.15;
	float mWallMu=.05;

	mIteations=atoi(argv[1]);
	mTimeStep=atof(argv[2]);
	mOmega=atof(argv[3]);
	mEnvelope=atof(argv[4]);
	mSphereMu=atof(argv[5]);
	mWallMu=atof(argv[6]);
	int mDevice=atoi(argv[7]);

	cudaSetDevice(mDevice);

	ChLcpSystemDescriptorGPU		mGPUDescriptor;
	ChContactContainerGPUsimple		mGPUContactContainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(&mGPUDescriptor, mEnvelope*SCALE);
	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUContactContainer,&mGPUDescriptor,  mIteations,mTimeStep, 1e-5, mOmega, false);

	ChSystem SysG(1000, 50); 
	SysG.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	SysG.ChangeContactContainer(&mGPUContactContainer);
	SysG.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	SysG.ChangeCollisionSystem(&mGPUCollisionEngine);
	SysG.SetIntegrationType(ChSystem::INT_ANITESCU);
	SysG.Set_G_acc(ChVector<>(0,GRAV*SCALE,0));
	GPUSystem=new System(&SysG,1);
	GPUSystem->mMu=mSphereMu;
	GPUSystem->mTimeStep=mTimeStep;

	float width=0.009525;
	rightWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	leftWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	flatWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	movingWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	//ground		=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChQuaternion<> base(1,0,0,0);
	ChQuaternion<> quat;
	double angle=PI/4.0;
	quat.Q_from_NasaAngles(ChVector<>(0,0,angle));
	GPUSystem->MakeBox(rightWall,	ChVector<>(.04,.035,.00005)*SCALE,100000,ChVector<>(0.0,-.01,-width/2.0)*SCALE,base,mWallMu,mWallMu,0,1,1,true,true);
	GPUSystem->MakeBox(leftWall,	ChVector<>(.04,.035,.00005)*SCALE,100000,ChVector<>(0.0,-.01,width/2.0)*SCALE,base,mWallMu,mWallMu,0,1,1,true,true);
	GPUSystem->MakeBox(flatWall,	ChVector<>(.00005,.03,width)*SCALE,100000,ChVector<>(-.03,-.01,0)*SCALE,base,mWallMu,mWallMu,0,1,1,true,true);
	GPUSystem->MakeBox(movingWall,	ChVector<>(.0425,.00005,width)*SCALE,100000,ChVector<>(0,-.01,0)*SCALE,quat,mWallMu,mWallMu,0,1,1,true,true);
	rightWall->SetPos_dt(ChVector<>(0,0,0));
	leftWall->SetPos_dt(ChVector<>(0,0,0));
	flatWall->SetPos_dt(ChVector<>(0,0,0));
	movingWall->SetPos_dt(ChVector<>(0,0,0));

	
	if(load_file){GPUSystem->CreateSpheres(0, 0, 0, 0, 0, 0, true);}
	
	//GPUSystem->LoadTriangleMesh("capsule2.obj");
		char numstr[256]; // enough to hold all numbers up to 64-bits
	sprintf(numstr, "DEMO_%d_%f_%f_%f_%f_%f_%f.txt", mIteations,mTimeStep,mOmega,mEnvelope,mSphereMu,mWallMu);
	GPUSystem->mTimingFile.open(numstr);
	GPUSystem->mTimingFile<<"Time:\tTotal:\tCD:\tSolver:\tBodies:\tContacts:\tKE:"<<endl;

#pragma omp parallel sections
	{

#pragma omp section
		{
			while(true){
				GPUSystem->renderScene();
			}
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