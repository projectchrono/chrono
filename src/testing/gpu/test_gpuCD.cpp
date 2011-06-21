#include "test_gpuCD.h"
ChSharedBodyGPUPtr rightWall;
ChSharedBodyGPUPtr leftWall;	
ChSharedBodyGPUPtr flatWall;	
ChSharedBodyGPUPtr movingWall;
ChSharedBodyGPUPtr ground;

#define SPHERES 50*52*15
float mSettleTime=0;
System::System(ChSystem * Sys, bool GPU){
	mSystem=Sys;
	mGPUSys=GPU;
	mNumCurrentSpheres=0;
	saveTimingData=0;
	saveSimData=false;
	mSphereRadius=2.5e-4*SCALE;
	mSphereEnvelope=0;
	mSphereMass = 1.631e-7*SCALE;
	mSphereRestitution=0;
	mTimeStep=.00005;
	mCurrentTime=0;
	mMu=.15;
	mTriangleScale=20;
	mEndTime=7;
	mCameraX=0, mCameraY=0, mCameraZ=0;
	mBoundingBoxSize=40;

	mOffsetX=0;
	mOffsetY=6;
	mOffsetZ=0;
	mNumSpheres=SPHERES;
	mNumCurrentSpheres=0;
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
		mKE+=.5*mass*vel2;
		counter++;
	}
	return mKE;
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
	return (mMassFlowRate)*1000;

}


void System::MakeSphere(ChSharedBodyPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->GetCollisionModel()->ClearModel();
	if(mGPUSys){(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddSphere(mSphereRadius);}
	else{body.get_ptr()->GetCollisionModel()->AddSphere(mSphereRadius);}
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
}
void System::MakeBox(ChSharedBodyPtr &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetRot(rot);
	body.get_ptr()->GetCollisionModel()->ClearModel();
	if(mGPUSys){(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddBox(radius.x,radius.y,radius.z);}
	else{body.get_ptr()->GetCollisionModel()->AddBox(radius.x,radius.y,radius.z);}
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
}
void System::MakeEllipsoid(ChSharedBodyPtr &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
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
}
void System::CreateSpheres(int x, int y, int z, double posX, double posY, double posZ, bool rnd){

	if(load_file){
		ifstream ifile ("OUTDATA.txt");

		mNumCurrentSpheres+=x*y*z;
		ChSharedBodyPtr mrigidBodya;
		for (int zz=0; zz<39480; zz++){
			//ChVector<> mParticlePos((xx),(yy)+4,(zz));
			ChVector<> mParticlePos(0,0,0);
			ifile>>mParticlePos.x>>mParticlePos.y>>mParticlePos.z;
			mrigidBodya = ChSharedBodyPtr(new ChBodyGPU);
			MakeSphere(mrigidBodya, mSphereRadius, mSphereMass, mParticlePos, mMu, mMu, mSphereRestitution, true);
			mSystem->AddBody(mrigidBodya);
		}
	}else{



		mNumCurrentSpheres+=x*y*z;
		ChSharedBodyPtr mrigidBodya;
		for (int xx=0; xx<x; xx++){
			for (int yy=0; yy<y; yy++){
				for (int zz=0; zz<z; zz++){
					//ChVector<> mParticlePos((xx),(yy)+4,(zz));
					ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
					if(rnd){
						mParticlePos.x+=(rand()%1000)/4000.f;
						mParticlePos.y+=(rand()%1000)/4000.f;
						mParticlePos.z+=(rand()%1000)/4000.f;
					}
					//ChVector<> mParticlePos((xx-x/2.0)+rand()%100/100.0,(yy-y/2.0)+rand()%100/100.0,(zz-z/2.0)+rand()%100/100.0);
					if(mGPUSys){mrigidBodya = ChSharedBodyPtr(new ChBodyGPU);}
					else{mrigidBodya = ChSharedBodyPtr(new ChBody);}
					MakeSphere(mrigidBodya, mSphereRadius, mSphereMass, mParticlePos*.0006*SCALE, mMu, mMu, mSphereRestitution, true);
					mSystem->AddBody(mrigidBodya);
				}
			}
		}
	}
}


float3 camera_pos=make_float3(-10,0,-10);
float3 look_at=make_float3(.1,0,.1);
float3 camera_up=make_float3(0,1,0);
float camera_heading=0, camera_pitch=0;
float3 dir=make_float3(0,0,1);
float2 mouse_pos=make_float2(0,0);
float3 camera_pos_delta=make_float3(0,0,0);
float4 CreateFromAxisAngle(float3 axis, float degrees){
	float angle = float((degrees / 180.0f) * PI);
	float result = (float)sinf( angle / 2.0f );
	float4 camera_quat;
	camera_quat.w = (float)cosf( angle / 2.0f );
	camera_quat.x = float(axis.x * result);
	camera_quat.y = float(axis.y * result);
	camera_quat.z = float(axis.z * result);
	return camera_quat;
}

inline float4 operator ~(const float4& a){
	return (1.0/(dot(a,a)))*(F4(-1*F3(a),a.w));
}
inline float4 mult(const float4 &a, const float4 &b){
	return F4(a.w*F3(b)+b.w*F3(a)+cross(F3(a),F3(b)),a.w*b.w-dot(F3(a),F3(b)));
}
inline float3 quatRotate(const float3 &v, const float4 &q){
	return make_float3(mult(mult(q,make_float4(v,0)),~(q)));
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

		std::vector<ChBody*>::iterator abody = mSystem->Get_bodylist()->begin();
		while (abody != mSystem->Get_bodylist()->end()){
			ChCollisionModelGPU* cModel = (ChCollisionModelGPU*)((*abody)->GetCollisionModel());
			if(cModel->GetShapeType()==SPHERE){
				drawSphere((*abody)->GetPos(),(*abody)->GetPos_dt().Length(), mSphereRadius);
			}
			if(cModel->GetShapeType()==BOX){
				drawBox(*abody,mSphereRadius,mGPUSys);
			}
			if(cModel->GetShapeType()==ELLIPSOID){
				drawSphere(*abody,mGPUSys);
			}
			if(cModel->GetShapeType()==TRIANGLEMESH){
				glColor3f (0,0,0);
				drawTriMesh(TriMesh,(*abody));
			}
			abody++;
		}
		//Sleep(30);
		glutSwapBuffers();
	}

}

void System::LoadTriangleMesh(string name){
	ChSharedBodyPtr mrigidBody;
	ifstream ifile(name.c_str());
	string temp,j;
	vector<float3> pos, tri;
	float3 tempF;
	while(ifile.fail()==false){
		getline(ifile,temp);
		if(temp.size()>2&&temp[0]!='#'&&temp[0]!='g'){
			if(temp[0]=='v'){
				stringstream ss(temp);
				ss>>j>>tempF.x>>tempF.y>>tempF.z;
				pos.push_back(tempF);
			}
			if(temp[0]=='f'){
				stringstream ss(temp);
				ss>>j>>tempF.x>>tempF.y>>tempF.z;
				tri.push_back(tempF);
			}
		}
	}

	if(mGPUSys){mrigidBody = ChSharedBodyPtr(new ChBodyGPU);}
	else{mrigidBody = ChSharedBodyPtr(new ChBody);}
	for(int i=0; i<tri.size(); i++){
		ChVector<> A(pos[tri[i].x-1].x,pos[tri[i].x-1].y,pos[tri[i].x-1].z);
		ChVector<> B(pos[tri[i].y-1].x,pos[tri[i].y-1].y,pos[tri[i].y-1].z);
		ChVector<> C(pos[tri[i].z-1].x,pos[tri[i].z-1].y,pos[tri[i].z-1].z);
		A*=mTriangleScale;
		B*=mTriangleScale;
		C*=mTriangleScale;
		TriMesh.addTriangle(A,B,C);
	}

	mrigidBody.get_ptr()->SetMass(100000);
	ChVector<> particle_pos2(-30,-70,0);
	mrigidBody.get_ptr()->SetPos(particle_pos2);

	mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
	mrigidBody.get_ptr()->GetCollisionModel()->AddTriangleMesh(TriMesh,true,false);
	mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
	mrigidBody.get_ptr()->SetBodyFixed(true);
	mrigidBody.get_ptr()->SetCollide(true);
	mrigidBody.get_ptr()->SetImpactC(0.0);
	mrigidBody.get_ptr()->SetSfriction(.9);
	mrigidBody.get_ptr()->SetKfriction(.9);
	//ChMatrix33<> rotation;
	//ChQuaternion<> quat(-3.1415,ChVector<>(0,0,1));
	//quat.Normalize();
	//rotation.Set_A_quaternion(quat);
	//mrigidBody.get_ptr()->SetRot(rotation);
	mSystem->AddBody(mrigidBody);
	mrigidBody.get_ptr()->GetCollisionModel()->SetFamily(1);
	mrigidBody.get_ptr()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);



}

void System::DoTimeStep(){
	if(mNumCurrentSpheres<mNumSpheres&&mFrameNumber%1==0&&!load_file){CreateSpheres(1+mFrameNumber, 1, 14, -48+mFrameNumber/2.0, -55+mFrameNumber, 0, true);}
	if(mSystem->GetChTime()>mSettleTime&&movewall){
		movingWall->SetPos(ChVector<>(.0025+.00015,-.01,0)*SCALE);
		movewall=false;
	}
	if(savenow){
		ofstream ofile("OUTDATA.txt");
		for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
			ChBody *abody=mSystem->Get_bodylist()->at(i);
			ChVector<> pos=abody->GetPos();
			ofile<<pos.x<<"\t"<<pos.y<<"\t"<<pos.z<<endl;
		}
		ofile.close();
		savenow=false;
	}

	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
		if(abody->GetPos().y<-60){
			abody->SetCollide(false);
			abody->SetBodyFixed(true);
			//abody->GetCollisionModel()->SetFamily(1);
			//abody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);

			abody->SetPos(ChVector<>(0,-60,0));
		}
	}

	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;

	if(mSystem->GetChTime()>=mEndTime){exit(0);}
}
void System::PrintStats(){
	cout<<"T:   "<<mSystem->GetChTime()
		<<"\tC: "<<mSystem->GetTimerStep()
		<<"\tG: "<<mSystem->GetTimerCollisionBroad()
		<<"\tG: "<<mSystem->GetTimerLcp()
		<<"\tB: "<<mSystem->GetNbodies()
		<<"\tC: "<<mSystem->GetNcontacts()
		<<"\tKE: "<<GetKE()<<" MFR: "<<GetMFR(-50)<<endl;
	mTimingFile
		<<mSystem->GetChTime()<<"\t"
		<<mSystem->GetTimerStep()<<"\t"
		<<mSystem->GetTimerCollisionBroad()<<"\t"
		<<mSystem->GetTimerLcp()<<"\t"
		<<mSystem->GetNbodies()<<"\t"
		<<mSystem->GetNcontacts()<<"\t"
		<<GetKE()<<"\t"
		<<GetMFR(-50)<<endl;

}
GLuint g_textureID = -1;
void initScene(){
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };			
	glClearColor (1.0, 1.0, 1.0, 0.0);							
	glShadeModel (GL_SMOOTH);									
	glEnable(GL_COLOR_MATERIAL);								
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);			
	glEnable(GL_LIGHTING);										
	glEnable(GL_LIGHT0);
	//glAlphaFunc(GL_GREATER, 0.1);
	//glEnable(GL_ALPHA_TEST);
	glEnable (GL_POINT_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_POINT_SMOOTH_HINT, GL_DONT_CARE);

}


System *GPUSystem;

void renderSceneAll(){
	if(OGL){GPUSystem->drawAll();}

}
float m_MaxPitchRate=5;
float m_MaxHeadingRate=5;
void ChangePitch(GLfloat degrees)
{
	if(fabs(degrees) < fabs(m_MaxPitchRate)){
		camera_pitch += degrees;
	}else{
		if(degrees < 0){
			camera_pitch -= m_MaxPitchRate;
		}else{
			camera_pitch += m_MaxPitchRate;
		}
	}
	if(camera_pitch > 360.0f){
		camera_pitch -= 360.0f;
	}else if(camera_pitch < -360.0f){
		camera_pitch += 360.0f;
	}
}

void ChangeHeading(GLfloat degrees)
{
	if(fabs(degrees) < fabs(m_MaxHeadingRate)){
		if(camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)){
			camera_heading -= degrees;
		}
		else{
			camera_heading += degrees;
		}
	}
	else{
		if(degrees < 0){
			if((camera_pitch > 90 && camera_pitch < 270) || (camera_pitch < -90 && camera_pitch > -270)){
				camera_heading += m_MaxHeadingRate;
			}else{
				camera_heading -= m_MaxHeadingRate;
			}
		}else{
			if(camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)){
				camera_heading -= m_MaxHeadingRate;
			}else{
				camera_heading += m_MaxHeadingRate;
			}
		}
	}
	if(camera_heading > 360.0f){
		camera_heading -= 360.0f;
	}else if(camera_heading < -360.0f){
		camera_heading += 360.0f;
	}
}


void processNormalKeys(unsigned char key, int x, int y) { 	
	if (key=='w'){camera_pos_delta+=dir*.0005*SCALE;}
	if (key=='s'){camera_pos_delta-=dir*.0005*SCALE;}
	if (key=='d'){camera_pos_delta+=cross(dir,camera_up)*.0005*SCALE;}
	if (key=='a'){camera_pos_delta-=cross(dir,camera_up)*.0005*SCALE;}
	if (key=='q'){camera_pos_delta+=camera_up*.0005*SCALE;}
	if (key=='e'){camera_pos_delta-=camera_up*.0005*SCALE;}
	if (key=='u'){updateDraw=(updateDraw)? 0:1;}
	if (key=='i'){showSphere=(showSphere)? 0:1;}
	if (key=='z'){savenow=1;}
	if (key=='x'){movewall=1;}


}
void pressKey(int key, int x, int y) {} 
void releaseKey(int key, int x, int y) {} 
void mouseMove(int x, int y) { 
	float2 mouse_delta=mouse_pos-make_float2(x,y);
	ChangeHeading(.2 * mouse_delta.x);
	ChangePitch(.2 * mouse_delta.y);
	mouse_pos=make_float2(x,y);
}

void mouseButton(int button, int state, int x, int y) {
	mouse_pos=make_float2(x,y);
}

void changeSize(int w, int h) {
	if(h == 0) {h = 1;}
	float ratio = 1.0* w / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45,ratio,.00001*SCALE,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0,0.0,0.0,		0.0,0.0,-7,		0.0f,1.0f,0.0f);
}

int main(int argc, char* argv[]){
	
	
	float mOmega=.5;
	int mIteations=500;
	float mTimeStep=.00005;
	float mEnvelope=0.25e-4;

	float mSphereMu=.15;
	float mWallMu=.05;

	mIteations=atoi(argv[1]);
	mTimeStep=atof(argv[2]);
	mOmega=atof(argv[3]);
	mEnvelope=atof(argv[4]);
	mSphereMu=atof(argv[5]);
	mWallMu=atof(argv[6]);
	mSettleTime=atof(argv[7]);
	int mDevice=atoi(argv[8]);

	cudaSetDevice(mDevice);
	cout<<mIteations<<" "<<mTimeStep<<" "<<mOmega<<" "<<mEnvelope<<" "<<mSphereMu<<" "<<mWallMu<<" "<<mSettleTime<<" "<<mDevice<<endl;

	ChLcpSystemDescriptorGPU		mGPUDescriptor;
	ChContactContainerGPUsimple		mGPUContactContainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(&mGPUDescriptor, mEnvelope*SCALE);
	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUContactContainer,&mGPUDescriptor,  mIteations,mTimeStep, 1e-5, mOmega, false);

	ChSystem SysG(1000, 50); 



	SysG.ChangeLcpSystemDescriptor(&mGPUDescriptor);
	SysG.ChangeContactContainer(&mGPUContactContainer);
	SysG.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	SysG.ChangeCollisionSystem(&mGPUCollisionEngine);

	SysG.SetIntegrationType(ChSystem::INT_ANITESCU);
	SysG.SetIterLCPwarmStarting(false);
	SysG.Set_G_acc(ChVector<>(0,-9.80665*SCALE,0));
	GPUSystem=new System(&SysG,1);
	GPUSystem->mMu=mSphereMu;
	GPUSystem->mTimeStep=mTimeStep;


	float width=0.009525;	//[m]

	rightWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	leftWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	flatWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	movingWall	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ground		=	ChSharedBodyGPUPtr(new ChBodyGPU);
	rightWall->SetMass(100000);
	leftWall->SetMass(100000);
	flatWall->SetMass(100000);
	movingWall->SetMass(100000);
	ground->SetMass(100000);

	rightWall->SetPos(ChVector<>(0.0,-.01,-width/2.0-.00005)*SCALE);
	leftWall->SetPos(ChVector<>(0.0,-.01,width/2.0+.00005)*SCALE);

	flatWall->SetPos(ChVector<>(-.03,-.01,0)*SCALE);
	movingWall->SetPos(ChVector<>(0,-.01,0)*SCALE);
	ground->SetPos(ChVector<>(0,-70,0));


	(rightWall.get_ptr()->GetCollisionModel())->ClearModel();
	(rightWall.get_ptr()->GetCollisionModel())->AddBox(.03*SCALE,.035*SCALE,.0003*SCALE); 
	(rightWall.get_ptr()->GetCollisionModel())->BuildModel();

	(leftWall.get_ptr()->GetCollisionModel())->ClearModel();
	(leftWall.get_ptr()->GetCollisionModel())->AddBox(.03*SCALE,.035*SCALE,.0003*SCALE); 
	(leftWall.get_ptr()->GetCollisionModel())->BuildModel();

	(flatWall.get_ptr()->GetCollisionModel())->ClearModel();
	(flatWall.get_ptr()->GetCollisionModel())->AddBox(.0003*SCALE,.03*SCALE,width/1.5*SCALE); 
	(flatWall.get_ptr()->GetCollisionModel())->BuildModel();


	(movingWall.get_ptr()->GetCollisionModel())->ClearModel();
	(movingWall.get_ptr()->GetCollisionModel())->AddBox(.0425*SCALE,.0003*SCALE,width/2.0*SCALE); 
	(movingWall.get_ptr()->GetCollisionModel())->BuildModel();

	(ground.get_ptr()->GetCollisionModel())->ClearModel();
	(ground.get_ptr()->GetCollisionModel())->AddBox(100,1,100); 
	(ground.get_ptr()->GetCollisionModel())->BuildModel();


	ChQuaternion<> quat;
	double angle=PI/4.0;
	quat.Q_from_NasaAngles(ChVector<>(0,0,angle));
	movingWall->SetRot(quat);


	rightWall->SetBodyFixed(true);
	leftWall->SetBodyFixed(true);
	flatWall->SetBodyFixed(true);
	movingWall->SetBodyFixed(true);
	ground->SetBodyFixed(true);



	rightWall->SetCollide(true);
	leftWall->SetCollide(true);
	flatWall->SetCollide(true);
	movingWall->SetCollide(true);
	ground->SetCollide(true);

	rightWall->SetSfriction(mWallMu);
	leftWall->SetSfriction(mWallMu);
	flatWall->SetKfriction(mWallMu);
	movingWall->SetKfriction(mWallMu);
	ground->SetKfriction(mWallMu);

	rightWall->SetImpactC(0.0);
	leftWall->SetImpactC(0.0);
	flatWall->SetImpactC(0.0);
	movingWall->SetImpactC(0.0);
	ground->SetImpactC(0.0);

	SysG.AddBody(rightWall);
	SysG.AddBody(leftWall);
	SysG.AddBody(flatWall);
	SysG.AddBody(movingWall);

	(rightWall.get_ptr()->GetCollisionModel())->SetFamily(1);
	(rightWall.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(1);

	(leftWall.get_ptr()->GetCollisionModel())->SetFamily(1);
	(leftWall.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(1);

	(movingWall.get_ptr()->GetCollisionModel())->SetFamily(1);
	(movingWall.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(1);

	(flatWall.get_ptr()->GetCollisionModel())->SetFamily(1);
	(flatWall.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(1);

	(ground.get_ptr()->GetCollisionModel())->SetFamily(1);
	(ground.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(1);


	//SysG.AddBody(ground);

	//GPUSystem->CreateSpheres(15, 15, 15, 0, 0, 0, true);

	//GPUSystem->LoadTriangleMesh("capsule2.obj");

	char numstr[256]; // enough to hold all numbers up to 64-bits
sprintf(numstr, "R_%d_%f_%f_%f_%f_%f_%f.txt", mIteations,mTimeStep,mOmega,mEnvelope,mSphereMu,mWallMu,mSettleTime);

	GPUSystem->mTimingFile.open(numstr);
	GPUSystem->mTimingFile<<"Time:\tTotal:\tCD:\tSolver:\tBodies:\tContacts:\tKE:"<<endl;

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
				//glutSpecialFunc(pressKey);
				//glutSpecialUpFunc(releaseKey);
				glutMouseFunc(mouseButton);
				glutMotionFunc(mouseMove);
				//glutPassiveMotionFunc(mouseMove);
				initScene();
				glutMainLoop();	
			}
		}
	}
	return 0;
}