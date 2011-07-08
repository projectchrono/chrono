#include "common.h"

ChSharedPtr<ChLinkLockAlign> my_motor;
ChSharedBodyGPUPtr BTM;
void System::DoTimeStep(){
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%50==0){
		float x=35;	float posX=0;
		float y=60;	float posY=20;
		float z=35;	float posZ=0;
		
		float radius	=.5;
		float mass	=.0003456;
		float mu	=1;
		float rest	=0;
		int type 	=0;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects+=x*y*z;
		int mobjNum=0;

		for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
		for (int zz=0; zz<z; zz++){
			ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
			mParticlePos+ChVector<>(rand()%1000/1000.0-.5,rand()%1000/1000.0-.5,rand()%1000/1000.0-.5);
			ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);;
			mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
			if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*1.2, mu, mu, rest, true);}
			if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
			if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			mobjNum++;
			}
		}
	}

	}
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
		if(abody->GetPos().y<-40){
			abody->SetCollide(false);
			abody->SetBodyFixed(true);
			abody->SetPos(ChVector<>(15,-40,15));
		}
	}
	if(mFrameNumber%60==0){
		ofstream ofile;
		stringstream ss;
		ss<<"data/anchor"<<mFileNumber<<".txt";
		ofile.open(ss.str().c_str());
		for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
			ChBody* abody = mSystem->Get_bodylist()->at(i);
			ChVector<> pos=abody->GetPos();
			ChVector<> rot=abody->GetRot().Q_to_NasaAngles();
			ChVector<> vel=abody->GetPos_dt();
			ChVector<> acc=abody->GetPos_dtdt();
			ChVector<> trq=abody->Get_gyro();
			if(isnan(rot.x)){rot.x=0;}
			if(isnan(rot.y)){rot.y=0;}
			if(isnan(rot.z)){rot.z=0;}
			ofile<<pos.x<<","<<pos.y<<","<<pos.z<<","<<rot.x<<","<<rot.y<<","<<rot.z<<","<<vel.x<<","<<vel.y<<","<<vel.z<<","<<acc.x<<","<<acc.y<<","<<acc.z<<","<<trq.x<<","<<trq.y<<","<<trq.z<<","<<endl;
		}
		ofile.close();
		mFileNumber++;
	}
	
	
	if(mSystem->GetChTime()>5){
		BTM->SetBodyFixed(false);
		BTM->SetCollide(true);
	}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	if(mSystem->GetChTime()>=mEndTime){exit(0);}
	if(mSystem->GetNbodies()==0){exit(0);}
}

System *GPUSystem;
void renderSceneAll(){
	GPUSystem->drawAll();
}
int main(int argc, char* argv[]){
	float mOmega=.1;
	int mIteations=200;
	float mTimeStep=.005;
	float mEnvelope=0;
	float mMu=1;
	float mWallMu=1;
	int mDevice=0;
	float mEndTime=20;
	/*if(argc>1){
	mIteations=atoi(argv[1]);
	mTimeStep=atof(argv[2]);
	mOmega=atof(argv[3]);
	mEnvelope=atof(argv[4]);
	mMu=atof(argv[5]);
	mWallMu=atof(argv[6]);
	mDevice=atoi(argv[7]);
	}*/
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
	GPUSystem=new System(&SysG,400,"anchor.txt");
	GPUSystem->mMu=mMu;
	GPUSystem->mTimeStep=mTimeStep;
	GPUSystem->mEndTime=mEndTime;
	ChQuaternion<> base(1,0,0,0);

	ChSharedBodyGPUPtr L	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	BTM	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FXED	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr I	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	
	GPUSystem->MakeBox(L,	ChVector<>(7,30,30), 100000,ChVector<>(-30,0,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(7,30,30), 100000,ChVector<>(30,0,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(30,30,7), 100000,ChVector<>(0,0,-30),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(30,30,7), 100000,ChVector<>(0,0,30), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(FXED,	ChVector<>(50,4,50), 100000,ChVector<>(0,-30,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	
	GPUSystem->MakeBox(BTM,ChVector<>(2,20,2), .1,ChVector<>(0,60,0 ), base,mWallMu,mWallMu,0,100000,100000,true,false);
	//GPUSystem->MakeSphere(I, 5, .03, ChVector<>(0,0,0), mMu, mMu, 0, true);
	
	my_motor= ChSharedPtr<ChLinkLockAlign> (new ChLinkLockAlign);
	ChSharedBodyPtr ptr1=ChSharedBodyPtr(FXED);
	ChSharedBodyPtr ptr2=ChSharedBodyPtr(BTM);
	
	my_motor->Initialize(ptr1,ptr2,ChCoordsys<>(ChVector<>(0,1,0)));
	//ChFunction_Sine *vibrationFunc=new ChFunction_Sine(0,50,1);
	//my_motor->SetMotion_Y(vibrationFunc);
	SysG.AddLink(my_motor);
	//my_motor->SetDisabled(true);
	BTM->SetBodyFixed(true);
	BTM->SetCollide(false);
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
				glutCreateWindow("Anchor ");
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
