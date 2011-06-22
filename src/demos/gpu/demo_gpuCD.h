#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>

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
#define OGL 1
#define GRAV -9.80665

bool load_file=false;
bool updateDraw=true;
bool showSphere=true;
bool movewall=true;
bool savenow=false;

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

void drawSphere(ChVector<> gPos, double velocity, double & mSphereRadius){
	float3 color=GetColour(velocity,0,10);
	glColor4f (color.x, color.y,color.z,1.0f);
	glPushMatrix();
	glTranslatef (gPos.x, gPos.y, gPos.z);
	if(showSphere){glutSolidSphere(mSphereRadius,10,10);}
	else{
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex3f(0, 0, 0);	
		glEnd();
	}

	glPopMatrix();
}

void drawSphere(ChBody *abody, bool gpu){

	float3 color=GetColour(abody->GetPos_dt().Length(),0,10);
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
	if (key=='w'){camera_pos_delta+=dir*.5*SCALE;}
	if (key=='s'){camera_pos_delta-=dir*.5*SCALE;}
	if (key=='d'){camera_pos_delta+=cross(dir,camera_up)*.5*SCALE;}
	if (key=='a'){camera_pos_delta-=cross(dir,camera_up)*.5*SCALE;}
	if (key=='q'){camera_pos_delta+=camera_up*.5*SCALE;}
	if (key=='e'){camera_pos_delta-=camera_up*.5*SCALE;}
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
	gluPerspective(45,ratio,.1,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0,0.0,0.0,		0.0,0.0,-7,		0.0f,1.0f,0.0f);
}


