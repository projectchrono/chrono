#include "common.h"
//specify parameters:
float particle_R []={.0035, .012,.0005, .0017};
float particle_Rho []={1560, 720, 5490, 5490};
float particle_Theta []={23,  28,   24,   24};
float impactor_R []={.095, .013, .015, .020, .025, .035, .040, .045, .050, .013, .019, .026, .0125, .019};
float impactor_M []={.034, .083, .130, .287, .531,1.437,2.099,3.055,4.079, .064, .201, .518,  .009, .018};
float container_R=   .3;
float container_T=  .03;

void System::DoTimeStep(){
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%500==0){
		float x=50;	float posX=0;
		float y=50;	float posY=0;
		float z=50;	float posZ=0;
		
		float radius	=particle_R[0];
		float mass	=particle_Rho[0]*4.0/3.0*PI*radius*radius*radius;
		float mu	=tan(particle_Theta[0]*PI/180.0);
		float rest	=0;
		int type 	=0;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects+=x*y*z;
		int mobjNum=0;

		for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
		for (int zz=0; zz<z; zz++){
			ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
			mParticlePos+=ChVector<>(rand()%1000/1000.0-.5,rand()%1000/1000.0-.5,rand()%1000/1000.0-.5)*.5;
			ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);;
			mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
			if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*.01, mu, mu, rest, true);}
			if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
			if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			if(type==3){MakeCylinder(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			mobjNum++;
			}
		}
	}

	}
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
		if(abody->GetPos().y<-1){
			abody->SetCollide(false);
			abody->SetBodyFixed(true);
			abody->SetPos(ChVector<>(1,-1,1));
		}
	}
	if(mFrameNumber%3==0&&saveData){
		ofstream ofile;
		stringstream ss;
		ss<<"data/validation"<<mFileNumber<<".txt";
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
	float mTimeStep=.0001;
	float mEnvelope=0;
	float mMu=.5;
	float mWallMu=.5;
	int   mDevice=0;
	float mEndTime=20;
	bool OGL=0;
	Scale=.05;
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
	GPUSystem=new System(&SysG,100000,"validation.txt");
	GPUSystem->mMu=mMu;
	GPUSystem->mTimeStep=mTimeStep;
	GPUSystem->mEndTime=mEndTime;
	ChQuaternion<> base(1,0,0,0);

	ChSharedBodyGPUPtr L	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	
	GPUSystem->MakeBox(L,	ChVector<>(container_T,container_R,container_R), 100000,ChVector<>(-container_R,0,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(container_T,container_R,container_R), 100000,ChVector<>(container_R,0,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(container_R,container_R,container_T), 100000,ChVector<>(0,0,-container_R),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(container_R,container_R,container_T), 100000,ChVector<>(0,0,container_R), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(BTM, ChVector<>(container_R,container_T,container_R), 100000,ChVector<>(0,-container_R,0),base,mWallMu,mWallMu,0,-20,-20,true,true);

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
				glutCreateWindow("Validation");
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
