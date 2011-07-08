#include "demo_gpuCD.h"

System::System(ChSystem * Sys, bool GPU){
	mSystem=Sys;
	mGPUSys=GPU;
	mNumCurrentSpheres=0;
	saveTimingData=0;
	saveSimData=false;
	mSphereRadius=.2;
	mSphereEnvelope=0;
	mSphereMass = 1;
	mSphereRestitution=0;
	mTimeStep=.001;
	mCurrentTime=0;
	mMu=.15;
	mTriangleScale=20;
	mEndTime=4;
	mCameraX=0, mCameraY=0, mCameraZ=0;
	mBoundingBoxSize=40;
	mNumSpheres=50000;
	mOffsetX=0;
	mOffsetY=6;
	mOffsetZ=0;
	mNumCurrentObjects=0;
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


void System::MakeSphere(ChSharedBodyGPUPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide){
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
void System::MakeEllipsoid(ChSharedBodyGPUPtr &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
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
void System::CreateObjects(int x, int y, int z, double posX, double posY, double posZ, bool rnd,int type){

	mNumCurrentSpheres+=x*y*z;
	ChSharedBodyGPUPtr mrigidBody;
	for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
			for (int zz=0; zz<z; zz++){
				ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
				ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);;
				if(rnd){
					mParticlePos.x+=(rand()%1000)/4000.f;
					mParticlePos.y+=(rand()%1000)/4000.f;
					mParticlePos.z+=(rand()%1000)/4000.f;
					quat.Q_from_NasaAngles(ChVector<>((rand()%4000)/1000.0f,(rand()%4000)/1000.f,(rand()%4000)/1000.f));
				}
				mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
				if(type==0){MakeSphere(mrigidBody, mSphereRadius, mSphereMass, mParticlePos, mMu, mMu, mSphereRestitution, true);}
				if(type==1){MakeBox(mrigidBody, ChVector<>(mSphereRadius,mSphereRadius,mSphereRadius), mSphereMass, mParticlePos,quat, mMu, mMu, mSphereRestitution,mNumCurrentObjects,mNumCurrentObjects,true, false);}
				if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(mSphereRadius*2,mSphereRadius,mSphereRadius*2), mSphereMass, mParticlePos,quat, mMu, mMu, mSphereRestitution, true);}
				mNumCurrentObjects++;
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

		for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
			ChBody* abody=mSystem->Get_bodylist()->at(i);
			if(abody->GetCollisionModel()->GetShapeType()==SPHERE){
				drawSphere((abody)->GetPos(),(abody)->GetPos_dt().Length(), mSphereRadius);
			}
			if(abody->GetCollisionModel()->GetShapeType()==BOX){
				drawBox(abody,mSphereRadius,mGPUSys);
			}
			if(abody->GetCollisionModel()->GetShapeType()==ELLIPSOID){
				drawSphere(abody,mGPUSys);
			}
			if(abody->GetCollisionModel()->GetShapeType()==TRIANGLEMESH){
				glColor3f (0,0,0);
				drawTriMesh(TriMesh,(abody));
			}
		}

#if defined( _WINDOWS )
		Sleep( 30 );
#else
		usleep( 30 * 1000 );
#endif
		glutSwapBuffers();
	}
}

void System::DoTimeStep(){
	if(mNumCurrentSpheres<mNumSpheres&&mFrameNumber%600==0){
		//CreateObjects(10, 1, 10, 1, 1+mFrameNumber*3, 0, true, 0);
		//CreateObjects(10, 1, 10, 1, 2+mFrameNumber*3, 0, true, 1);
		CreateObjects(10, 10, 10, 1, 15, 0, false, 2);

	}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	if(mSystem->GetChTime()>=mEndTime){exit(0);}
	if(mSystem->GetNbodies()==0){exit(0);}
}
void System::PrintStats(){
	double A=mSystem->GetChTime();
	double B=mSystem->GetTimerStep();
	double C=mSystem->GetTimerCollisionBroad();
	double D=mSystem->GetTimerLcp();
	int E=mSystem->GetNbodies();
	int F=mSystem->GetNcontacts();
	//double G=0;//GetKE();
	//double H=GetMFR(-50);
	char numstr[512];
	printf("%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d\n",A,B,C,D,E,F);
	//sprintf(numstr,"%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %14.8f",A,B,C,D,E,F,G);
	//mTimingFile<<numstr<<endl;
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
	float mOmega=.1;
	int mIteations=200;
	float mTimeStep=.001;
	float mEnvelope=0;
	float mSphereMu=.15;
	float mWallMu=.05;
	int mDevice=0;
	cudaSetDevice(mDevice);

	ChLcpSystemDescriptorGPU		mGPUDescriptor;
	ChContactContainerGPUsimple		mGPUContactContainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(&mGPUDescriptor, mEnvelope);
	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUContactContainer,&mGPUDescriptor,  mIteations,mTimeStep, 1e-5, mOmega, false);

	ChSystem SysG(1000, 50); 
	SysG.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	SysG.ChangeContactContainer(&mGPUContactContainer);
	SysG.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	SysG.ChangeCollisionSystem(&mGPUCollisionEngine);
	SysG.SetIntegrationType(ChSystem::INT_ANITESCU);
	SysG.Set_G_acc(ChVector<>(0,GRAV,0));
	GPUSystem=new System(&SysG,1);
	GPUSystem->mMu=mSphereMu;
	GPUSystem->mTimeStep=mTimeStep;

	ChQuaternion<> base(1,0,0,0);
	ChQuaternion<> quat1,quat2,quat3,quat4;
	double angle=0;//PI/4.0;

	ChSharedBodyGPUPtr L	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr W	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B	=	ChSharedBodyGPUPtr(new ChBodyGPU);

	GPUSystem->MakeBox(L,	ChVector<>(.1,10,15), 100000,ChVector<>(-10,-10,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(.1,10,15), 100000,ChVector<>(10,-10,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(15,10,.1), 100000,ChVector<>(0,-10,-10),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(W,	ChVector<>(15,10,.1), 100000,ChVector<>(0,-10,10), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(15,.1,15), 100000,ChVector<>(0,-15,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	
	//GPUSystem->mTimingFile.open("timing.txt");


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
