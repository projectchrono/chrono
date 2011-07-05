#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <cutil_inline.h>
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChContactContainer.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "unit_GPU/ChLcpIterativeSolverGPUsimple.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
#include "unit_GPU/ChCCollisionModelGPU.h"
#include "unit_GPU/ChBodyGPU.h"
#include <GL/freeglut.h>
#include "omp.h"


#if defined( _WINDOWS )
#include <windows.h>
#else
#include <unistd.h>
#endif
using namespace chrono;
using namespace std;
#define SCALE 1
#define PI			3.14159265358979323846
#define OGL 0
#define GRAV -9.80665

bool load_file=false;
bool updateDraw=true;
bool showSphere=true;
bool movewall=true;
bool savenow=false;
bool moveGround=false;
class System{
public:
	System(ChSystem * Sys, bool GPU);
	~System(){};

	void RenderFrame();
	void RunGLUTLoop();
	double GetKE();
	double GetMFR(double height);
	void CreateObjects(int x, int y, int z, double posX, double posY, double posZ, bool rand, int type);
	void DoTimeStep();
	void PrintStats();
	void MakeSphere(ChSharedBodyGPUPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide);
	void MakeBox(ChSharedBodyGPUPtr &body, ChVector<> radius, double mass,ChVector<> pos,ChQuaternion<> rot,double sfric,double kfric,double restitution,int family,int nocolwith,bool collide, bool fixed);
	void MakeEllipsoid(ChSharedBodyGPUPtr &body, ChVector<> radius, double mass,ChVector<> pos,ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide);
	void LoadTriangleMesh(string name, ChVector<> pos, ChQuaternion<> rot, float mass);
	void drawAll();
	void renderScene(){	PrintStats();	DoTimeStep();}
	ChSystem *mSystem;

	double mKE,mMassFlowRate;
	bool openGL, mGPUSys;
	int mNumCurrentSpheres,mNumCurrentObjects;
	bool saveTimingData;
	bool saveSimData;

	double mSphereRadius,mSphereEnvelope,mSphereMass,mSphereRestitution;
	double mTimeStep;
	double mCurrentTime;
	double mMu;
	double mTriangleScale;
	double mEndTime;
	double mCameraX, mCameraY, mCameraZ;
	double mBoundingBoxSize;

	double mOffsetX,mOffsetY,mOffsetZ;
	int mNumSpheres;
	int mNumBoxes;
	int mNumTriangles;
	int mFrameNumber;
	int mFileNumber;
	int mSaveEveryX;
	int mCudaDevice;
	geometry::ChTriangleMesh TriMesh;

	ofstream mTimingFile;
	FILE *mDataFile;
};


float3 GetColour(double v,double vmin,double vmax){
	float3 c = {1.0,1.0,1.0}; // white
	double dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		c.x = 0;
		c.y = 4 * (v - vmin) / dv;
	} else if (v < (vmin + 0.5 * dv)) {
		c.x = 0;
		c.z = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
	} else if (v < (vmin + 0.75 * dv)) {
		c.x = 4 * (v - vmin - 0.5 * dv) / dv;
		c.z = 0;
	} else {
		c.y = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
		c.z = 0;
	}
	return(c);
}

void drawSphere(ChBody *abody, bool gpu, int a){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,500);
	glColor3f (color.x, color.y,color.z);
	glPushMatrix();
	glTranslatef(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);
	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].A;
	if(showSphere){glutSolidSphere(h.w,10,10);}
	else{
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex3f(0, 0, 0);	
		glEnd();
	}

	glPopMatrix();
}

void drawSphere(ChBody *abody, bool gpu){

	float3 color=GetColour(abody->GetPos_dt().Length(),0,500);
	glColor3f (color.x, color.y,color.z);
	glPushMatrix();
	double angle;
	ChVector<> axis;

	glTranslatef(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);

	abody->GetRot().Q_to_AngAxis(angle,axis);
	glRotatef(angle*180.0/3.1415, axis.x, axis.y, axis.z);
	if(gpu){
		float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
		glScalef(h.x,h.y,h.z);
	}
	glutSolidSphere(1,10,10);
	if(showSphere){glutSolidSphere(1,10,10);}
	else{
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex3f(0, 0, 0);	
		glEnd();
	}
	glPopMatrix();
}
void drawBox(ChBody *abody, float x, bool gpu){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,10);
	glColor4f (color.x, color.y,color.z, 1);
	glPushMatrix();
	double angle;
	ChVector<> axis;


	glTranslatef(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);

	abody->GetRot().Q_to_AngAxis(angle,axis);
	glRotatef(angle*180.0/PI, axis.x, axis.y, axis.z);

	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
	glScalef(h.x*2,h.y*2,h.z*2);

	glutSolidCube(1);
	glPopMatrix();
}

void drawTriMesh(ChTriangleMesh &TriMesh,ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,50);
	glColor3f (color.x, color.y,color.z);
	for(int i=0; i<TriMesh.getNumTriangles(); i++){
		geometry::ChTriangle tri=TriMesh.getTriangle(i);
		ChVector<> gA = (abody)->GetCoord().TrasformLocalToParent(tri.p1);
		ChVector<> gB = (abody)->GetCoord().TrasformLocalToParent(tri.p2);
		ChVector<> gC = (abody)->GetCoord().TrasformLocalToParent(tri.p3);
		glColor4f (0, 0,0,.3);
		glBegin(GL_LINE_LOOP);
		glVertex3f(gA.x,gA.y,gA.z);	
		glVertex3f(gB.x,gB.y,gB.z);	
		glVertex3f(gC.x,gC.y,gC.z);	
		glEnd();
	}
}

float m_MaxPitchRate=5;
float m_MaxHeadingRate=5;
float3 camera_pos=make_float3(-.1,0,-.1);
float3 look_at=make_float3(0,0,0);
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

void ChangeHeading(GLfloat degrees){
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
	if (key=='w'){camera_pos_delta+=dir*1*SCALE;}
	if (key=='s'){camera_pos_delta-=dir*1*SCALE;}
	if (key=='d'){camera_pos_delta+=cross(dir,camera_up)*1*SCALE;}
	if (key=='a'){camera_pos_delta-=cross(dir,camera_up)*1*SCALE;}
	if (key=='q'){camera_pos_delta+=camera_up*1*SCALE;}
	if (key=='e'){camera_pos_delta-=camera_up*1*SCALE;}
	if (key=='u'){updateDraw=(updateDraw)? 0:1;}
	if (key=='i'){showSphere=(showSphere)? 0:1;}
	if (key=='z'){savenow=1;}
	if (key=='x'){moveGround=1;}
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
	gluPerspective(45,ratio,.1,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0,0.0,0.0,		0.0,0.0,-7,		0.0f,1.0f,0.0f);
}





ChSharedPtr<ChLinkLockLock> my_motor;
ChSharedBodyGPUPtr BTM;

System::System(ChSystem * Sys, bool GPU){
	mSystem=Sys;
	mGPUSys=GPU;
	mNumCurrentSpheres=0;
	saveTimingData=0;
	saveSimData=false;
	mSphereRadius=.5;
	mSphereEnvelope=0;
	mSphereMass = .000136;
	mSphereRestitution=0;
	mTimeStep=.001;
	mCurrentTime=0;
	mMu=.15;
	mTriangleScale=20;
	mEndTime=10;
	mCameraX=0, mCameraY=0, mCameraZ=0;
	mBoundingBoxSize=40;
	mNumSpheres=78400;
	mOffsetX=0;
	mOffsetY=6;
	mOffsetZ=0;
	mNumCurrentObjects=0;
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
	return mKE/SCALE;
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
	return (mMassFlowRate)/SCALE;

}


void System::MakeSphere(ChSharedBodyGPUPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetInertiaXX(ChVector<>(2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius));
	body.get_ptr()->GetCollisionModel()->ClearModel();
	if(mGPUSys){(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddSphere(radius);}
	else{body.get_ptr()->GetCollisionModel()->AddSphere(radius);}
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::MakeBox(ChSharedBodyGPUPtr &body, ChVector<> dim, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,int family,int nocolwith,bool collide, bool fixed){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetRot(rot);
	body.get_ptr()->SetInertiaXX(ChVector<>(1/12.0*mass*(dim.y*dim.y+dim.z*dim.z),1/12.0*mass*(dim.x*dim.x+dim.z*dim.z),1/12.0*mass*(dim.x*dim.x+dim.y*dim.y)));
	body.get_ptr()->GetCollisionModel()->ClearModel();
	(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddBox(dim.x,dim.y,dim.z);
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetBodyFixed(fixed);
	body.get_ptr()->SetImpactC(0.);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
	(body.get_ptr()->GetCollisionModel())->SetFamily(family);
	(body.get_ptr()->GetCollisionModel())->SetFamilyMaskNoCollisionWithFamily(nocolwith);
}
void System::MakeEllipsoid(ChSharedBodyGPUPtr &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
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
	mSystem->AddBody(body);
}
void System::CreateObjects(int x, int y, int z, double posX, double posY, double posZ, bool rnd,int type){

	mNumCurrentSpheres+=x*y*z;
	ChSharedBodyGPUPtr mrigidBody;
	for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
			for (int zz=0; zz<z; zz++){
				ChVector<> mParticlePos((xx-(x-1)/2.0)+posX,(yy)+posY,(zz-(z-1)/2.0)+posZ);
				ChQuaternion<> quat=ChQuaternion<>(1,0,0,0);;
				if(rnd){
					mParticlePos.x+=(rand()%1000)/4000.f;
					mParticlePos.y+=(rand()%1000)/4000.f;
					mParticlePos.z+=(rand()%1000)/4000.f;
					quat.Q_from_NasaAngles(ChVector<>((rand()%4000)/1000.0f,(rand()%4000)/1000.f,(rand()%4000)/1000.f));
				}
				mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
				if(type==0){MakeSphere(mrigidBody, mSphereRadius, mSphereMass, mParticlePos*1.2, mMu, mMu, mSphereRestitution, true);}
				if(type==1){MakeBox(mrigidBody, ChVector<>(mSphereRadius,mSphereRadius,mSphereRadius), mSphereMass, mParticlePos,quat, mMu, mMu, mSphereRestitution,mNumCurrentObjects,mNumCurrentObjects,true, false);}
				if(type==2){MakeEllipsoid(mrigidBody, ChVector<>(mSphereRadius,mSphereRadius,mSphereRadius), mSphereMass, mParticlePos,quat, mMu, mMu, mSphereRestitution, true);}
				mNumCurrentObjects++;
			}
		}
	}
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

		for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
			ChBody* abody=mSystem->Get_bodylist()->at(i);
			if(abody->GetCollisionModel()->GetShapeType()==SPHERE){
				drawSphere(abody,mGPUSys,0);
			}
			if(abody->GetCollisionModel()->GetShapeType()==BOX){
				drawBox(abody,mSphereRadius,mGPUSys);
			}
			if(abody->GetCollisionModel()->GetShapeType()==ELLIPSOID){
				drawSphere(abody,mGPUSys);
			}
			if(abody->GetCollisionModel()->GetShapeType()==TRIANGLEMESH){
				glColor3f (0,0,0);
				drawTriMesh(TriMesh,(abody));
			}
		}

#if defined( _WINDOWS )
		Sleep( 30 );
#else
		usleep( 30 * 1000 );
#endif
		glutSwapBuffers();
	}
}

void System::DoTimeStep(){
	if(mNumCurrentSpheres<mNumSpheres&&mFrameNumber%50==0){
		//CreateObjects(10, 1, 10, 1, 1+mFrameNumber*3, 0, true, 0);
		//CreateObjects(10, 1, 10, 1, 2+mFrameNumber*3, 0, true, 1);
		CreateObjects(70, 16, 70, 0, -10, 0, false, 0);
	}
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
	ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
	if(abody->GetPos().y<-40){
	abody->SetCollide(false);
	abody->SetBodyFixed(true);
	abody->SetPos(ChVector<>(15,-40,15));
	}
	}
	 if(mFrameNumber%120==0){
		 ofstream ofile;
		 stringstream ss;
		 ss<<"data/data"<<mFileNumber<<".txt";
		ofile.open(ss.str().c_str());
		 for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
			 ChBody* abody = mSystem->Get_bodylist()->at(i);
			 ChVector<> pos=abody->GetPos();
			 ChVector<> rot=abody->GetRot().Q_to_NasaAngles();
			 if(isnan(rot.x)){rot.x=0;}
			 if(isnan(rot.y)){rot.y=0;}
			 if(isnan(rot.z)){rot.z=0;}

			 ofile<<pos.x<<","<<pos.y<<","<<pos.z<<","<<rot.x<<","<<rot.y<<","<<rot.z<<","<<endl;
		 }
		 ofile.close();
		 mFileNumber++;
	 }
	
	
	if(moveGround||mNumCurrentSpheres>mNumSpheres){
	BTM->SetBodyFixed(false);
	my_motor->SetDisabled(false);
	}
	mFrameNumber++;
	mSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	if(mSystem->GetChTime()>=mEndTime){exit(0);}
	if(mSystem->GetChTime()>=1.0){movewall=true;}
	if(mSystem->GetNbodies()==0){exit(0);}
}
void System::PrintStats(){
	double A=mSystem->GetChTime();
	double B=mSystem->GetTimerStep();
	double C=mSystem->GetTimerCollisionBroad();
	double D=mSystem->GetTimerLcp();
	int E=mSystem->GetNbodies();
	int F=mSystem->GetNcontacts();
	char numstr[512];
	printf("%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d\n",A,B,C,D,E,F);
	sprintf(numstr,"%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d",A,B,C,D,E,F);
	mTimingFile<<numstr<<endl;
}
void initScene(){
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };			
	glClearColor (1.0, 1.0, 1.0, 0.0);							
	glShadeModel (GL_SMOOTH);									
	glEnable(GL_COLOR_MATERIAL);								
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);			
	glEnable(GL_LIGHTING);										
	glEnable(GL_LIGHT0);
	glEnable (GL_POINT_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_POINT_SMOOTH_HINT, GL_DONT_CARE);
}


System *GPUSystem;
void renderSceneAll(){
	if(OGL){GPUSystem->drawAll();}
}
int main(int argc, char* argv[]){
	float mOmega=.1;
	int mIteations=200;
	float mTimeStep=.00025;
	float mEnvelope=0;
	float mSphereMu=.5;
	float mWallMu=.5;
	int mDevice=1;
	/*if(argc>1){
	mIteations=atoi(argv[1]);
	mTimeStep=atof(argv[2]);
	mOmega=atof(argv[3]);
	mEnvelope=atof(argv[4]);
	mSphereMu=atof(argv[5]);
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
	SysG.Set_G_acc(ChVector<>(0,GRAV,0)*1000);
	GPUSystem=new System(&SysG,1);
	GPUSystem->mMu=mSphereMu;
	GPUSystem->mTimeStep=mTimeStep;

	ChQuaternion<> base(1,0,0,0);

	ChSharedBodyGPUPtr L	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	BTM	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr FXED	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr I	=	ChSharedBodyGPUPtr(new ChBodyGPU);
	
	GPUSystem->MakeBox(L,	ChVector<>(3,30,60), 100000,ChVector<>(-60,0,0),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(R,	ChVector<>(3,30,60), 100000,ChVector<>(60,0,0), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(F,	ChVector<>(60,30,3), 100000,ChVector<>(0,0,-60),base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(B,	ChVector<>(60,30,3), 100000,ChVector<>(0,0,60), base,mWallMu,mWallMu,0,-20,-20,true,true);
	GPUSystem->MakeBox(BTM,	ChVector<>(80,4,80), 100000,ChVector<>(0,-30,0),base,mWallMu,mWallMu,0,-20,-20,true,false);
	
	GPUSystem->MakeBox(FXED,ChVector<>(5,5,5), 100000,ChVector<>(0,-70,0 ), base,mWallMu,mWallMu,0,-20,-20,true,true);
	
	my_motor= ChSharedPtr<ChLinkLockLock> (new ChLinkLockLock);
	ChSharedBodyPtr ptr1=ChSharedBodyPtr(BTM);
	ChSharedBodyPtr ptr2=ChSharedBodyPtr(FXED);
	
	my_motor->Initialize(ptr1,ptr2,ChCoordsys<>(ChVector<>(0,1,0)));
	ChFunction_Sine *vibrationFunc=new ChFunction_Sine(0,50,1);
	my_motor->SetMotion_Y(vibrationFunc);
	SysG.AddLink(my_motor);
	my_motor->SetDisabled(true);
	BTM->SetBodyFixed(true);
	GPUSystem->mTimingFile.open("vibration.txt");

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
				glutMouseFunc(mouseButton);
				glutMotionFunc(mouseMove);
				initScene();
				glutMainLoop();	
			}
		}
	}
	return 0;
}
