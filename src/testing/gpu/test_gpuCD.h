#include <string>

#include <fstream>
#include <sstream>
#include <GL/glut.h>
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
#include "omp.h"
#include <windows.h>
using namespace chrono;
using namespace std;
#define SCALE 1000
#define PI			3.14159265358979323846
#define OGL 1

bool load_file=false;
bool updateDraw=true;
bool showSphere=false;
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
	void CreateSpheres(int x, int y, int z, double posX, double posY, double posZ, bool rand);
	void DoTimeStep();
	void PrintStats();
	void MakeSphere(ChSharedBodyPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,bool collide);
	void MakeBox(ChSharedBodyPtr &body, ChVector<> radius, double mass,ChVector<> pos,ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide);
	void MakeEllipsoid(ChSharedBodyPtr &body, ChVector<> radius, double mass,ChVector<> pos,ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide);
	void LoadTriangleMesh(string name);
	void drawAll();
	void renderScene(){	PrintStats();	DoTimeStep();}
	ChSystem *mSystem;



	double mKE,mMassFlowRate;
	bool openGL, mGPUSys;
	int mNumCurrentSpheres;
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

	ChSharedBodyPtr mgroundBody;
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

void drawSphere(ChVector<> &gPos, double velocity, double mSphereRadius){
	float3 color=GetColour(velocity,0,.1*SCALE);
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

	float3 color=GetColour(abody->GetPos_dt().Length(),0,.001*SCALE);
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
	glPopMatrix();
}
void drawBox(ChBody *abody, float x, bool gpu){
	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	float3 color=GetColour(abody->GetPos_dt().Length(),0,.001*SCALE);
	glColor4f (color.x, color.y,color.z, .3);
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
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
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

