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
#define PI	3.14159265358979323846
#define GRAV 	-9.80665
#define OGL 1
bool showSphere = true;
bool updateDraw=true;
bool savenow =false;
bool moveGround=false;
bool movewall=false;
class System{
public:
	System(ChSystem * Sys, int nobjects, string timingfile);
	~System(){};

	void RenderFrame();
	void RunGLUTLoop();
	double GetKE();
	double GetMFR(double height);
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
	int mNumCurrentSpheres,mNumCurrentObjects;
	bool saveTimingData;
	bool saveSimData;

	double mTimeStep;
	double mCurrentTime;
	double mMu;
	double mEndTime;
	double mCameraX, mCameraY, mCameraZ;

	double mOffsetX,mOffsetY,mOffsetZ;
	int mNumObjects;
	int mFrameNumber;
	int mFileNumber;
	int mCudaDevice;

	ofstream mTimingFile;
};

System::System(ChSystem * Sys, int nobjects, string timingfile){
	mSystem=Sys;
	mCurrentTime=0;
	mEndTime=5;
	mCameraX=0, mCameraY=0, mCameraZ=0;
	mNumObjects=nobjects;
	mNumCurrentObjects=0;
	mFrameNumber=0;
	mFileNumber=0;
	mTimingFile.open(timingfile.c_str());
}
void System::drawAll(){
	if(updateDraw){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
				drawSphere(abody,0);
			}
			if(abody->GetCollisionModel()->GetShapeType()==BOX){
				drawBox(abody);
			}
			if(abody->GetCollisionModel()->GetShapeType()==ELLIPSOID){
				drawSphere(abody);
			}
			//if(abody->GetCollisionModel()->GetShapeType()==TRIANGLEMESH){
			//	glColor3f (0,0,0);
			//	drawTriMesh(TriMesh,(abody));
			//}
}
#if defined( _WINDOWS )
		Sleep( 30 );
#else
		usleep( 30 * 1000 );
#endif
		glutSwapBuffers();
	}
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

void System::MakeSphere(ChSharedBodyGPUPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetInertiaXX(ChVector<>(2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius,2.0/5.0*mass*radius*radius));
	body.get_ptr()->GetCollisionModel()->ClearModel();
	(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddSphere(radius);
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
	(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddEllipsoid(radius.x,radius.y,radius.z);
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
}


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

void drawSphere(ChBody *abody, int a){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,500);
	glColor3f (color.x, color.y,color.z);
	glPushMatrix();
	glTranslatef(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);
	double angle;
	ChVector<> axis;
	abody->GetRot().Q_to_AngAxis(angle,axis);
	glRotatef(angle*180.0/3.1415, axis.x, axis.y, axis.z);
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

void drawSphere(ChBody *abody){

	float3 color=GetColour(abody->GetPos_dt().Length(),0,500);
	glColor3f (color.x, color.y,color.z);
	glPushMatrix();
	double angle;
	ChVector<> axis;

	glTranslatef(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z);

	abody->GetRot().Q_to_AngAxis(angle,axis);
	glRotatef(angle*180.0/3.1415, axis.x, axis.y, axis.z);
	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
	glScalef(h.x,h.y,h.z);
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

void drawBox(ChBody *abody){
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

	glutWireCube(1);
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
	glEnable(GL_DEPTH_TEST);
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0);
	glPointSize(2);
}

