#include "common.h"

float particle_R []={.0035, .012,.0005, .0017};
float particle_Rho []={1560, 720, 5490, 5490};
float particle_Theta []={23,  28,   24,   24};
float impactor_R []={.095, .013, .015, .020, .025, .035, .040, .045, .050, .013, .019, .026, .0125, .019};
float impactor_M []={.034, .083, .130, .287, .531,1.437,2.099,3.055,4.079, .064, .201, .518,  .009, .018};

float container_R=   .3;
float container_T=  .03;

float mOmega=.1;
int   mIteations=200;
float mTimeStep=.0001;
float mEnvelope=0;
float mMu=.5;
float mWallMu=.5;
int   mDevice=0;
float mEndTime=12;
bool OGL=0;

ChSharedPtr<ChLinkLockPointLine> my_motor;
ChSharedBodyGPUPtr Anchor;
void System::DoTimeStep(){

/*
	if(mNumCurrentObjects<mNumObjects&&mFrameNumber%50==0){
		float x=4;	float posX=0;
		float y=4;	float posY=0;
		float z=4;	float posZ=0;

		float radius	=.005;
		float mass	=1;
		float mu	=.5;
		float rest	=0;
		int type 	=3;
		ChSharedBodyGPUPtr mrigidBody;
		mNumCurrentObjects+=x*y*z;
		int mobjNum=0;

		for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
		for (int zz=0; zz<z; zz++){
			ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
			//mParticlePos+=ChVector<>(rand()%1000/1000.0-.5,rand()%1000/1000.0-.5,rand()%1000/1000.0-.5);
			ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);;
			mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
			if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*.01, mu, mu, rest, true);}
			if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius*2,radius), mass, mParticlePos*.03,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
			if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
			if(type==3){MakeCylinder(mrigidBody, ChVector<>(radius,radius*2,radius), mass, mParticlePos*.02,quat, mu, mu, rest, true);}
			
			mobjNum++;
			}
		}
	}

	}
*/
if(mCurrentTime<4){
	Anchor->Set_Scr_torque(ChVector<>(0,-10,0));
	Anchor->Set_Scr_force(ChVector<>(0,-10,0));
}
if(mCurrentTime>6&&mCurrentTime<8){
	Anchor->Set_Scr_torque(ChVector<>(0,0,0));
	Anchor->Set_Scr_force(ChVector<>(0,20,0));
}

	DeactivationPlane(-.4);
	
	if(mFrameNumber%150==0&&saveData){
		SaveAllData("data/anchor",true, false, false, true, false);
	}
	
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
}

int main(int argc, char* argv[]){
	Scale=.01;
	
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
	GPUSystem=new System(&SysG,10,"anchor.txt");
	GPUSystem->mMu=mMu;
	GPUSystem->mTimeStep=mTimeStep;
	GPUSystem->mEndTime=mEndTime;
	ChQuaternion<> base(1,0,0,0);
	Anchor			=	ChSharedBodyGPUPtr(new ChBodyGPU);
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
	
	ChQuaternion<> anchor_rot;
	float angle =-90*PI/180.0;
	ChVector<double> axis(1,0.,0);
	anchor_rot.Q_from_AngAxis(angle,axis);

	//GPUSystem->LoadTriangleMesh(Anchor,"Helical_Anchor2.obj",.5,ChVector<> (0,0,0),anchor_rot,1,.5,.5,0,-21,-21 );
	//GPUSystem->MakeCylinder(Anchor, ChVector<>(.005,.31/2.0,.005), .05, ChVector<> (0,.5,0),base, .5, .5, 0, true);
	vector<float3> pos;
	vector<float3> dim;
	vector<float4> quats;
	vector<ShapeType> tpe;
	vector<float> masses;

	ifstream anchorFile("Helical_Anchor.txt");
	string dat;
	float3 p;
	float4 r;
	for(int i=0; i<69; i++){
		//getline(anchorFile,dat);
		//anchorFile>>p.x>>p.y>>p.z>>r.x>>r.y>>r.z>>r.w;
		anchorFile>>p.x>>p.y>>p.z>>r.w>>r.x>>r.y>>r.z;
		//p.x*=-1;
		//p.z*=-1;
		//r=normalize(r);
		//ChQuaternion<> qq;
		//qq.Q_from_AngAxis(r.x*PI/180.0,ChVector<> (r.y,r.z,r.w));
		if      (i==0){
			dim.push_back(F3(.005,0,0));
			tpe.push_back(SPHERE);
			quats.push_back(r);
			pos.push_back(F3(0,0,0));
			masses.push_back(.1);
		}
		else if (i==1){
			dim.push_back(F3(.005,.3*.5,.005));
			tpe.push_back(CYLINDER);
			quats.push_back(F4(1,0,0,0));
			pos.push_back(F3(0,.3*.5,0));
			masses.push_back(.1);
		}
		else{
			dim.push_back(F3(.004,.0005,.04));
			tpe.push_back(BOX);
			ChQuaternion<> quat;
			quat.Q_from_AngAxis(i/69.0*2*PI,ChVector<>(0,1,0));
			quats.push_back(F4(quat.e0,quat.e1,quat.e2,quat.e3));
			pos.push_back(F3(sin(i/69.0*2*PI)*4.0*.01,i/69.0*4.0*.01+.005,cos(i/69.0*2*PI)*4.0*.01));
			masses.push_back(1);
		}
	}
	GPUSystem->MakeCompound(Anchor, ChVector<> (0,0,0),base,1,pos,dim,quats,tpe,masses, .5, .5, 0, true);
	
	float radius	=particle_R[0];
	float mass	=particle_Rho[0]*4.0/3.0*PI*radius*radius*radius;
	float mu	=tan(particle_Theta[0]*PI/180.0);
	float rest	=0;
	int   type 	=0;


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
	GPUSystem->MakeSphere(mrigidBody, radius, 1560*4.0/3.0*PI*radius*radius*radius, ChVector<>(x,y,z), mu, mu, rest, true);
	}
	}

	//GPUSystem->MakeSphere(I, 5, .03, ChVector<>(0,0,0), mMu, mMu, 0, true);
	
	//my_motor= ChSharedPtr<ChLinkLockPointLine> (new ChLinkLockPointLine);
	//ChSharedBodyPtr ptr1=ChSharedBodyPtr(Anchor);
	//ChSharedBodyPtr ptr2=ChSharedBodyPtr(BTM);
	
	//my_motor->Initialize(ptr1,ptr2,ChCoordsys<>(ChVector<>(0,1,0)));
	//ChFunction_Sine *vibrationFunc=new ChFunction_Sine(0,50,1);
	//my_motor->SetMotion_Y(vibrationFunc);
	//SysG.AddLink(my_motor);
	//my_motor->SetDisabled(true);
	//BTM->SetBodyFixed(true);
	//BTM->SetCollide(false);
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
