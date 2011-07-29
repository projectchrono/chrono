#include "common.h"
//specify parameters:
float particle_R []={.0035, .012,.0005, .0017};
float particle_Rho []={1560, 720, 5490, 5490};
float particle_Theta []={23,  28,   24,   24};
float impactor_R []={.095, .013, .015, .020, .025, .035, .040, .045, .050, .013, .019, .026, .0125, .019};
float impactor_M []={.034, .083, .130, .287, .531,1.437,2.099,3.055,4.079, .064, .201, .518,  .009, .018};
float container_R=   .3;
float container_T=  .03;
float mOmega=.1;
int   mIteations=200;
float mTimeStep=.0005;
float mEnvelope=0;
float mMu=1;
float mWallMu=1;
int   mDevice=0;
float mEndTime=2.0;
bool  OGL=0;
int   var=0;
ChSharedBodyGPUPtr Ball;
void System::DoTimeStep(){
	//if(mNumCurrentObjects<mNumObjects&&mFrameNumber%50==0){
	//	float x=50;	float posX=0;
	//	float y=100;	float posY=-25;
	//	float z=50;	float posZ=0;
	//	
	//	float radius	=particle_R[0];
	//	float mass	=particle_Rho[0]*4.0/3.0*PI*radius*radius*radius;
	//	float mu	=tan(particle_Theta[0]*PI/180.0);
	//	float rest	=0;
	//	int   type 	=0;
	//	ChSharedBodyGPUPtr mrigidBody;
	//	mNumCurrentObjects+=x*y*z;
	//	int mobjNum=0;
	//	ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);
	//	for (int xx=0; xx<x; xx++){
	//	for (int yy=0; yy<y; yy++){
	//	for (int zz=0; zz<z; zz++){
	//		ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
	//		//mParticlePos+=ChVector<>(rand()%1000/1000.0-.5,rand()%1000/1000.0-.5,rand()%1000/1000.0-.5)*.5;

	//		mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
	//		if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*.01, mu, mu, rest, true);}
	//		if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
	//		if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
	//		if(type==3){MakeCylinder(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
	//		mobjNum++;
	//		}
	//	}
	//}
	//}
	DeactivationPlane(-.4);
	stringstream ss;
	ss<<"dropped_ball"<<var<<".txt";
	SaveByID(5,ss.str(),true,true,true,false,false);

	//	ChLcpSystemDescriptorGPU* mGPUDescriptor=(ChLcpSystemDescriptorGPU *)mSystem->GetLcpSystemDescriptor();
	//	if(mGPUDescriptor->gpu_collision->contact_data_host.size()>0)
	//		{
	//			ofstream ofile ("contacts.txt");
	//			for(int i=0; i<mGPUDescriptor->gpu_collision->contact_data_host.size(); i++){
	//											float3 N=mGPUDescriptor->gpu_collision->contact_data_host[i].N;
	//											float3 Pa=mGPUDescriptor->gpu_collision->contact_data_host[i].Pa;
	//											float3 Pb=mGPUDescriptor->gpu_collision->contact_data_host[i].Pb;
	//											float D=mGPUDescriptor->gpu_collision->contact_data_host[i].I.x;
	//											ofile<<N.x<<" "<<N.y<<" "<<N.z<<" "<<Pa.x<<" "<<Pa.y<<" "<<Pa.z<<" "<<Pb.x<<" "<<Pb.y<<" "<<Pb.z<<" "<<D<<endl;
	//
	//
	//											//glBegin(GL_LINES);
	//											//glVertex3f(Pa.x, Pa.y, Pa.z);
	//											//float3 Pb=Pa+N*-D*10;
	//											//glVertex3f(Pb.x, Pb.y, Pb.z);
	//											//glEnd();
	//
	//										}
	//			ofile.close();
	//			exit(0);
	//	}
	if(saveData||mFrameNumber%50==0){SaveAllData("data/ball_drop",true, false, false, true, false);}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	//if(mCurrentTime>=mEndTime){SaveAllData("data/ball_drop",true, false, false, true, false);}
}

int main(int argc, char* argv[]){
	Scale=.01;

	OGL=atoi(argv[1]);
	saveData=atoi(argv[2]);
	mDevice=atoi(argv[3]);
	var=atoi(argv[4]);
	if(var>=14){exit(0);}
	cudaSetDevice(mDevice);
	bool copyContacts=OGL;
	ChLcpSystemDescriptorGPU		mGPUDescriptor;
	ChContactContainerGPUsimple		mGPUContactContainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(&mGPUDescriptor, mEnvelope, copyContacts);
	ChLcpIterativeSolverGPUsimple		mGPUsolverSpeed(&mGPUContactContainer,&mGPUDescriptor,  mIteations,mTimeStep, 1e-5, mOmega, false);

	ChSystemGPU SysG(1000, 50);
	SysG.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	SysG.ChangeContactContainer(&mGPUContactContainer);
	SysG.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	SysG.ChangeCollisionSystem(&mGPUCollisionEngine);
	SysG.SetIntegrationType(ChSystem::INT_ANITESCU);
	SysG.Set_G_acc(ChVector<>(0,GRAV,0));

	stringstream ss;
	ss<<"ball_drop_timer_"<<var<<".txt";

	GPUSystem=new System(&SysG,10,ss.str());
	GPUSystem->mMu=mMu;
	GPUSystem->mTimeStep=mTimeStep;
	GPUSystem->mEndTime=mEndTime;

	ChQuaternion<> base(1,0,0,0);
	ChSharedBodyGPUPtr L	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr Ball	=	ChSharedBodyGPUPtr(new ChBodyGPU);

	GPUSystem->MakeBox(L,	ChVector<>(container_T,container_R,container_R), 100000,ChVector<>(-container_R,0,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(container_T,container_R,container_R), 100000,ChVector<>(container_R,0,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(container_R,container_R,container_T), 100000,ChVector<>(0,0,-container_R),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(container_R,container_R,container_T), 100000,ChVector<>(0,0,container_R), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(BTM, ChVector<>(container_R,container_T,container_R), 100000,ChVector<>(0,-container_R,0),base,mWallMu,mWallMu,0,-20,-20,true,true);

	float radius	=particle_R[0];
	float mass	=particle_Rho[0]*4.0/3.0*PI*radius*radius*radius;
	float mu	=.4244748;//tan(particle_Theta[0]*PI/180.0);
	float rest	=0;
	int   type 	=0;

	//GPUSystem->MakeSphere(Ball, impactor_R [var], impactor_M[var], ChVector<>(0,1,0), .3, .3, rest, true);

	ifstream ifile("ball_drop_start.txt");
	string data;
	for(int i=0; i<250005; i++){
		getline(ifile,data);
		if(i>=5){
			for(int j=0; j<data.size(); j++){
				if(data[j]==','){data[j]='\t';}
			}
			stringstream ss(data);
			float x,y,z,rx,ry,rz;
			ss>>x>>y>>z>>rx>>ry>>rz;
			ChSharedBodyGPUPtr mrigidBody;
			mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
			GPUSystem->MakeSphere(mrigidBody, radius, mass, ChVector<>(x,y,z), mu, mu, rest, true);
		}
	}


#pragma omp parallel sections
	{
#pragma omp section
		{
			simulationLoop();
		}
#pragma omp section
		{
			if(OGL){initGLUT(string("validation"),argc, argv);}
		}
	}
	return 0;
}
