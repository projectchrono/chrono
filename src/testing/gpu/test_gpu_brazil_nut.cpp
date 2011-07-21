#include "common.h"

ChSharedPtr<ChLinkLockLock> my_motor;
ChSharedBodyGPUPtr BTM;
void System::DoTimeStep(){
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%50==0){
		float x=60;	float posX=0;
		float y=60;	float posY=-10;
		float z=60;	float posZ=0;
		
		float radius	=.0015;
		float mass	=.00001696;
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
			if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*.005, mu, mu, rest, true);}
			if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
			if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			mobjNum++;
			}
		}
	}

	}
	DeactivationPlane(-1);
	
	if(mFrameNumber%30==0&&saveData){
		SaveAllData("data/brazil_nut",true, false, false, true, false);
	}
	
	
	if(mSystem->GetChTime()>.5){
		BTM->SetBodyFixed(false);
		//BTM->SetCollide(true);
		my_motor->SetDisabled(false);
	}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
}

int main(int argc, char* argv[]){
Scale=.01;
	float mOmega=.1;
	int   mIteations=200;
	float mTimeStep=.0005;
	float mEnvelope=0;
	float mMu=.5;
	float mWallMu=.5;
	int   mDevice=0;
	float mEndTime=8;
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
	GPUSystem=new System(&SysG,79000,"brazil.txt");
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
	
	GPUSystem->MakeBox(L,	ChVector<>(.05,.25,.25), 100000,ChVector<>(-.25,0,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(.05,.25,.25), 100000,ChVector<>(.25,0,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(.25,.25,.05), 100000,ChVector<>(0,0,-.25),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(.25,.25,.05), 100000,ChVector<>(0,0,.25), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(FXED,ChVector<>(.05,.04,.05), 100000,ChVector<>(0,-.40,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	
	GPUSystem->MakeBox(BTM,ChVector<>(.25,.05,.25), 100000,ChVector<>(0,-.25,0 ), base,mWallMu,mWallMu,0,-20,-20,true,false);
	GPUSystem->MakeSphere(I, .04,.4061, ChVector<>(0,-.15,0), mMu, mMu, 0, true);
	
	my_motor= ChSharedPtr<ChLinkLockLock> (new ChLinkLockLock);
	ChSharedBodyPtr ptr1=ChSharedBodyPtr(FXED);
	ChSharedBodyPtr ptr2=ChSharedBodyPtr(BTM);
	
	my_motor->Initialize(ptr1,ptr2,ChCoordsys<>(ChVector<>(0,1,0)));
	ChFunction_Sine *vibrationFunc=new ChFunction_Sine(0,30,.005);
	my_motor->SetMotion_Y(vibrationFunc);
	SysG.AddLink(my_motor);
	my_motor->SetDisabled(true);
	BTM->SetBodyFixed(true);
#pragma omp parallel sections
	{
#pragma omp section
		{
			while(GPUSystem->mSystem->GetChTime()<=GPUSystem->mEndTime){GPUSystem->renderScene();}
		}
#pragma omp section
		{
			if(OGL){initGLUT(string("test"),argc, argv);}
		}
	}
	return 0;
}
