#include "common.h"

void System::DoTimeStep(){
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%50==0){
		float x=1;	float posX=0;
		float y=1;	float posY=0;
		float z=1;	float posZ=0;
		
		float radius	=.5;
		float mass	=10;
		float mu	=.5;
		float rest	=0;
		int type 	=0;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects+=x*y*z;
		int mobjNum=0;

		for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
		for (int zz=0; zz<z; zz++){
			ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
			//mParticlePos+ChVector<>(rand()%1000/1000.0-.5,rand()%1000/1000.0-.5,rand()%1000/1000.0-.5);
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
	if(mFrameNumber%3==0&&saveData){
		ofstream ofile;
		stringstream ss;
		ss<<"data/test"<<mFileNumber<<".txt";
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
			ofile<<pos.x<<","<<pos.y<<","<<pos.z<<","<<rot.x<<","<<rot.y<<","<<rot.z<<","<<endl;
		}
		ofile.close();
		mFileNumber++;
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
	int   mIteations=200;
	float mTimeStep=.005;
	float mEnvelope=0;
	float mMu=.5;
	float mWallMu=.5;
	int   mDevice=0;
	float mEndTime=20;
	bool OGL=0;
	if(argc==3){OGL=atoi(argv[1]);	saveData=atoi(argv[2]);}

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
	GPUSystem=new System(&SysG,1,"test.txt");
	GPUSystem->mMu=mMu;
	GPUSystem->mTimeStep=mTimeStep;
	GPUSystem->mEndTime=mEndTime;
	ChQuaternion<> base(1,0,0,0);

	ChSharedBodyGPUPtr P1	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	GPUSystem->MakeBox(P1,ChVector<>(10,5,20), 100000,ChVector<>(-20,-30,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	ChSharedBodyGPUPtr P2	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	GPUSystem->MakeBox(P2,ChVector<>(15,5,20), 100000,ChVector<>(0,-30,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	ChSharedBodyGPUPtr P3	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	GPUSystem->MakeBox(P3,ChVector<>(10,5,20), 100000,ChVector<>(20,-30,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	
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
				glutCreateWindow("test");
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
