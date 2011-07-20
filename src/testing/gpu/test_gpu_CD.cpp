#include "common.h"

void System::DoTimeStep(){
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%100==0){
		float x=1;	float posX=1;
		float y=1;	float posY=-10;
		float z=1;	float posZ=1;
		
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
			if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*1.5, mu, mu, rest, true);}
			if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
			if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			if(type==3){MakeCylinder(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			mobjNum++;
			}
		}
	}

	}

	DeactivationPlane(-40);

	SaveByID(1,"test_ball.txt",true,true,true,false,false);
	//SaveAllData("data/ball_drop",true, false, false, true, false);
	
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
	showContacts=1;
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
	GPUSystem=new System(&SysG,4,"test.txt");
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
	//ChSharedBodyGPUPtr P1	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	//GPUSystem->MakeCylinder(P1, ChVector<>(10,10,10), 10000, ChVector<>(0,-30,0),base, mWallMu, mWallMu, 0, true);
	//P1.get_ptr()->SetBodyFixed(true);
	
#pragma omp parallel sections
	{
#pragma omp section
		{
			while(GPUSystem->mSystem->GetChTime()<=GPUSystem->mEndTime){
				GPUSystem->renderScene();
			}
		}
#pragma omp section
		{
			if(OGL){initGLUT(string("test"),argc, argv);}
		}
	}
	return 0;
}
