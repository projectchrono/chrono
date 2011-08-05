#include "common.h"

float particle_R []={.0035, .012,.0005, .0017};
float particle_Rho []={1560, 720, 5490, 5490};
float particle_Theta []={23,  28,   24,   24};
float impactor_R []={.095, .013, .015, .020, .025, .035, .040, .045, .050, .013, .019, .026, .0125, .019};
float impactor_M []={.034, .083, .130, .287, .531,1.437,2.099,3.055,4.079, .064, .201, .518,  .009, .018};

float container_R=   .105;
float container_T=   .007;

float mOmega=.1;
int   mIteations=500;
float mTimeStep=.0010;
float mEnvelope=0;
float mMu=.5;
float mWallMu=.5;
int   mDevice=0;
float mEndTime=12;
bool OGL=0;

ChSharedPtr<ChLinkEngine> my_motor;
ChSharedPtr<ChLinkLinActuator> link_align;
ChSharedBodyGPUPtr Anchor;
ChSharedBodyGPUPtr FREE;
void System::DoTimeStep(){
//code to create objects if needed
/*
		if(mNumCurrentObjects<mNumObjects&&mFrameNumber%200==0){
			float x=25;	float posX=0;
			float y=1;	float posY=-40+mFrameNumber/200;
			float z=25;	float posZ=0;

			float radius	=particle_R[0];
			float mass	=particle_Rho[0]*4.0/3.0*PI*radius*radius*radius;
			float mu	=tan(particle_Theta[0]*PI/180.0);
			float rest	=0;
			int   type 	=0;
			ChSharedBodyGPUPtr mrigidBody;
			mNumCurrentObjects+=x*y*z;
			int mobjNum=0;
			ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);
			for (int xx=0; xx<x; xx++){
			for (int yy=0; yy<y; yy++){
			for (int zz=0; zz<z; zz++){
				ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
				mParticlePos+=ChVector<>(rand()%1000/1000.0-.5,rand()%1000/1000.0-.5,rand()%1000/1000.0-.5)*.5;

				mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
				if(type==0){MakeSphere(mrigidBody, radius, mass, mParticlePos*.007, mu, mu, rest, true);}
				if(type==1){MakeBox(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest,mobjNum,mobjNum,true, false);}
				if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
				if(type==3){MakeCylinder(mrigidBody, ChVector<>(radius,radius,radius), mass, mParticlePos,quat, mu, mu, rest, true);}
				mobjNum++;
				}
			}
		}
		}*/

	if(mCurrentTime<1){

		//Anchor->Set_Scr_torque(ChVector<>(0,-10,0));
		//FREE->Set_Scr_force(ChVector<>(0,0,0));

		ChFunction_Const* mfun=(ChFunction_Const* )my_motor->Get_spe_funct();
		mfun->Set_yconst(PI);

	}
	if(mCurrentTime>1&&mCurrentTime<2){
		ChFunction_Const* mfun=(ChFunction_Const* )my_motor->Get_spe_funct();
		mfun->Set_yconst(0);
		//Anchor->Set_Scr_torque(ChVector<>(0,0,0));
		//Anchor->Set_Scr_force(ChVector<>(0,20,0));
		//link_align->Set_lin_offset(.5);
		ChFunction_Const* dist_fun= (ChFunction_Const* )link_align->Get_dist_funct();
		dist_fun->Set_yconst(mFrameNumber/100.);
	}

	DeactivationPlane(-.35, -.15, false);
	//BoundingPlane(-container_R*3-container_T);
	//BoundingBox(.1,.3,.1,.0035);
	if(mFrameNumber%166==0){
		SaveAllData("data/anchor",true, false, false, true, false);
	}

	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
}

int main(int argc, char* argv[]){
	Scale=.01;

	if(argc==3){OGL=atoi(argv[1]);	saveData=atoi(argv[2]);}
	//cudaSetDevice(mDevice);

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
	GPUSystem=new System(&SysG,20000,"anchor.txt");
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
	FREE	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	//GPUSystem->MakeBox(FREE, ChVector<>(.001,.001,.001), 100000,ChVector<>(0,0,0),base,mWallMu,mWallMu,0,-20,-20,false,false);

	GPUSystem->MakeBox(L,	ChVector<>(container_T,container_R,container_R), 100000,ChVector<>(-container_R,-.2005,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(container_T,container_R,container_R), 100000,ChVector<>(container_R,-.2005,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(container_R,container_R,container_T), 100000,ChVector<>(0,-.2005,-container_R),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(container_R,container_R,container_T), 100000,ChVector<>(0,-.2005,container_R), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(BTM, ChVector<>(container_R,container_T,container_R), 100000,ChVector<>(0,-container_R-.2005,0),base,mWallMu,mWallMu,0,-20,-20,true,true);

	ChQuaternion<> anchor_rot;
	float angle =-90*PI/180.0;
	ChVector<double> axis(1,0.,0);
	anchor_rot.Q_from_AngAxis(angle,axis);

	//GPUSystem->LoadTriangleMesh(Anchor,"Helical_Anchor2.obj",.5,ChVector<> (0,0,0),anchor_rot,1,.5,.5,0,-21,-21 );
	vector<float3> pos;
	vector<float3> dim;
	vector<float4> quats;
	vector<ShapeType> tpe;
	vector<float> masses;


	for(int i=0; i<69; i++){
		if (i==0){
			dim.push_back(F3(.005,0,0));
			tpe.push_back(SPHERE);
			quats.push_back(F4(1,0,0,0));
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
			dim.push_back(F3(.0025,.0005,.025));
			tpe.push_back(BOX);
			ChQuaternion<> quat;
			ChQuaternion<> quat2;
			quat.Q_from_AngAxis(i/69.0*2*PI,ChVector<>(0,1,0));
			quat2.Q_from_AngAxis(6*2*PI/360.0,ChVector<>(0,0,1));
			quat=quat%quat2;
			quats.push_back(F4(quat.e0,quat.e1,quat.e2,quat.e3));
			pos.push_back(F3(sin(i/69.0*2*PI)*2.8*.01,i/69.0*4.0*.01+.005,cos(i/69.0*2*PI)*2.8*.01));
			masses.push_back(1);
		}
	}
	GPUSystem->MakeCompound(Anchor, ChVector<> (0,-.1,0),base,.25,pos,dim,quats,tpe,masses, .5, .5, 0, true);

	float radius	=particle_R[0];
	float mass	=particle_Rho[0]*4.0/3.0*PI*radius*radius*radius;
	float mu	=tan(particle_Theta[0]*PI/180.0);
	float rest	=0;
	int   type 	=0;


	ifstream ifile("anchor6.txt");
	string data;
	for(int i=0; i<0; i++){
		getline(ifile,data);
		if(i>=4){
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

	my_motor= ChSharedPtr<ChLinkEngine> (new ChLinkEngine);
	link_align= ChSharedPtr<ChLinkLinActuator> (new ChLinkLinActuator);

	//ChSharedBodyPtr ptr1=ChSharedBodyPtr(FREE);
	ChSharedBodyPtr ptr2=ChSharedBodyPtr(BTM);
	ChSharedBodyPtr ptr3=ChSharedBodyPtr(Anchor);

	ChFrame<> f0( ChVector<>(0, 0, 0) , Q_from_AngAxis(CH_C_PI/2.0, ChVector<>(1,0,0) )  );
	ChFrame<> f1( VNULL ,  Q_from_AngAxis(0*(CH_C_2PI/3.0) , ChVector<>(0,1,0) )  );

	my_motor->Initialize(ptr3,ptr2,(f0>>f1).GetCoord());
	link_align->Initialize(ptr3,ptr2,(f0>>f1).GetCoord());
	ChFunction_Const* dist_fun= (ChFunction_Const* )link_align->Get_dist_funct();
	dist_fun->Set_yconst(0);

	//my_motor->SetMotion_axis(ChVector<>(1,0,0));

	my_motor->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_PRISM);
	my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	ChFunction_Const* mfun= (ChFunction_Const* )my_motor->Get_spe_funct();
	mfun->Set_yconst(PI);


	SysG.AddLink(my_motor);
	//SysG.AddLink(link_align);

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
