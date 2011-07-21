#include "common.h"

ChSharedPtr<ChLinkLockAlign> my_motor;
ChSharedBodyGPUPtr BTM;
void System::DoTimeStep(){
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%50==0){
		float x=35;	float posX=0;
		float y=60;	float posY=20;
		float z=35;	float posZ=0;
		
		float radius	=.5;
		float mass	=793.3;
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
	DeactivationPlane(-40);
	
	if(mFrameNumber%3==0&&saveData){
		SaveAllData("data/anchor",true, false, false, true, false);
	}
	
	
	if(mSystem->GetChTime()>5){
		BTM->SetBodyFixed(false);
		BTM->SetCollide(true);
	}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
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
	
	ChSystemGPU SysG(1000, 50);
	SysG.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	SysG.ChangeContactContainer(&mGPUContactContainer);
	SysG.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	SysG.ChangeCollisionSystem(&mGPUCollisionEngine);
	SysG.SetIntegrationType(ChSystem::INT_ANITESCU);
	SysG.Set_G_acc(ChVector<>(0,GRAV,0));
	GPUSystem=new System(&SysG,73500,"anchor.txt");
	GPUSystem->mMu=mMu;
	GPUSystem->mTimeStep=mTimeStep;
	GPUSystem->mEndTime=mEndTime;
	ChQuaternion<> base(1,0,0,0);

	ChSharedBodyGPUPtr L	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	BTM			=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FXED	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr I	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	
	GPUSystem->MakeBox(L,	ChVector<>(7,30,30), 100000,ChVector<>(-30,0,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(7,30,30), 100000,ChVector<>(30,0,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(30,30,7), 100000,ChVector<>(0,0,-30),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(30,30,7), 100000,ChVector<>(0,0,30), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(FXED,ChVector<>(50,4,50), 100000,ChVector<>(0,-30,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	
	GPUSystem->MakeBox(BTM,ChVector<>(2,20,2), 644282,ChVector<>(0,60,0 ), base,mWallMu,mWallMu,0,100000,100000,true,false);
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
			while(GPUSystem->mSystem->GetChTime()<=GPUSystem->mEndTime){GPUSystem->renderScene();}
		}
#pragma omp section
		{
			if(OGL){initGLUT(string("anchor"),argc, argv);}
		}
	}
	return 0;
}
