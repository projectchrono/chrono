#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "unit_GPU/ChCuda.h"
#include "collision\bullet\btBulletCollisionCommon.h"
#include "collision\ChCModelBullet.h"
#include "collision\ChCModelBulletBody.h"
#include "cutil_math.h"
#include "unit_GPU/ChLcpIterativeSolverGPUsimple.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
#include <windows.h>
#include <tchar.h>
#pragma comment(lib,"glut64.lib")
#include <GL/glut.h>
using namespace std;
using namespace chrono::collision;
using namespace chrono;
float mult=20.0;
struct Contact{
	int a,b;
	float x1,y1,z1,x2,y2,z2,nx,ny,nz;
};

vector<float4> mData;

void bullet_sphere_sphere(int num_Bodies,vector<Contact> &CPU_contact_storage, vector<float4> &bData, int &totalC);
void gpu_sphere_sphere(int num_Bodies,vector<Contact> &CPU_contact_storage, vector<float4> &bData, int &totalC);




void changeSize(int w, int h) {
	if(h == 0) {h = 1;}
	float ratio = 1.0* w / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45,ratio,.1,10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(
		0.0,0.0,0.0, 
		0.0,0.0,-1,
		0.0f,1.0f,0.0f);
}

bool sphere_sphere(int);
//void sphere_triangle();
//void sphere_box();
bool verify(vector<Contact> A, vector<Contact> B);
bool test_func(Contact one, Contact two);

int testNumber=0;

int main(int argc, char** argv){
	cudaSetDevice(0);
	for(int i=0; i<1; i++){
		sphere_sphere(1000000);	//tests n random spheres against each other
		//sphere_triangle();
		//sphere_box();
		testNumber++;
	}
	//glutInit(&argc, argv);

	return 0;
}
float x=0,y=0,z=0;

void renderScene(){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK); 
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0);

	glPointSize(2);
	glLoadIdentity();
	gluLookAt(cos(x)*40*mult, 0, sin(z)*40*mult, 
		0,0,0,
		0.0f,-20.0f*mult,0.0f);
	glColor3f (1,1,1);
	glutWireCube(20*mult);
	for(int i=0; i<mData.size(); i++){
		glPushMatrix();
		glTranslatef (mData[i].x, mData[i].y, mData[i].z);
		glutSolidSphere(mData[i].w,10,10);
		glPopMatrix();
	}
	glutSwapBuffers();
	x+=.01;
	y+=.01;
	z+=.01;

}


void bullet_sphere_sphere(int num_Bodies,vector<Contact> &CPU_contact_storage, vector<float4> &bData, int &totalC){
	__int64 ctr1 = 0, ctr2 = 0, freq = 0;
	QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
	QueryPerformanceCounter((LARGE_INTEGER *)&ctr1);

	btCollisionWorld					*collisionWorld			= 0;
	btCollisionDispatcher				*dispatcher				= 0;
	btDefaultCollisionConfiguration		*collisionConfiguration	= 0;
	btCollisionObject					*object					= 0;
	btSphereShape						*sphereTemp				= 0;
	btDbvtBroadphase					*broadphase				= 0;

	collisionConfiguration	= new btDefaultCollisionConfiguration();
	dispatcher				= new	btCollisionDispatcher(collisionConfiguration);
	broadphase				= new btDbvtBroadphase;
	collisionWorld			= new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
	object					= new btCollisionObject[num_Bodies];

	btMatrix3x3 basisTemp;
	basisTemp.setIdentity();

	for(int i=0; i<num_Bodies; ++i){
		sphereTemp=new btSphereShape(bData[i].w);
		object[i].getWorldTransform().setBasis(basisTemp);
		object[i].setCollisionShape(sphereTemp);
		object[i].getWorldTransform().setOrigin(btVector3(bData[i].x,bData[i].y,bData[i].z));
		object[i].setCompanionId(i);
		collisionWorld->addCollisionObject(&object[i]);
	}
	if (collisionWorld){
		//collision start time
		//float c_start=clock()/1000.0;
		collisionWorld->performDiscreteCollisionDetection();
		//cout<<"Collision Detection Completed"<<endl;
		//cout<<endl<<"Collision Detection took: "<<(clock()/1000.0-c_start)<<" seconds"<<endl<<endl;
	}
	QueryPerformanceCounter((LARGE_INTEGER *)&ctr2);
	cout<<"CPU: "<<((ctr2 - ctr1) * 1.0 / freq)<<"\t";

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
	for (int k=0;k<numManifolds;k++)
	{
		btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(k);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		int numContacts = contactManifold->getNumContacts();

		totalC+= numContacts;
		//for (int j=0;j<numContacts;j++)
		//{
		//	btManifoldPoint& pt = contactManifold->getContactPoint(j);
		//	Contact temp;
		//	temp.a=obA->getCompanionId();
		//	temp.b=obB->getCompanionId();
		//	temp.x1=pt.m_positionWorldOnA.x();
		//	temp.y1=pt.m_positionWorldOnA.y();
		//	temp.z1=pt.m_positionWorldOnA.z();
		//	temp.x2=pt.m_positionWorldOnB.x();
		//	temp.y2=pt.m_positionWorldOnB.y();
		//	temp.z2=pt.m_positionWorldOnB.z();
		//	temp.nx=pt.m_normalWorldOnB.x();
		//	temp.ny=pt.m_normalWorldOnB.y();
		//	temp.nz=pt.m_normalWorldOnB.z();

		//	CPU_contact_storage.push_back(temp);
		//}
		contactManifold->clearManifold();
	}
	//////cout<<totalC<<" Contacts found"<<endl;
	//sort(CPU_contact_storage.begin(),CPU_contact_storage.end(),test_func);
}
void gpu_sphere_sphere(int num_Bodies,vector<Contact> &contact_storage, vector<float4> &bData, int &totalC){
	__int64 ctr1 = 0, ctr2 = 0, freq = 0;
	QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
	QueryPerformanceCounter((LARGE_INTEGER *)&ctr1);
	float envelope =0;
	float bin_size= 1;
	int mMaxContact=4000000;

	ChLcpSystemDescriptorGPU		newdescriptor(num_Bodies+100,mMaxContact, 1 );
	ChCollisionSystemGPU			mGPUCollisionEngine(envelope);
	mGPUCollisionEngine.SetSystemDescriptor(&newdescriptor);
	//totalC+= numContacts;
	mGPUCollisionEngine.mGPU.cMax=make_float3(-FLT_MAX  ,-FLT_MAX  ,-FLT_MAX  );
	mGPUCollisionEngine.mGPU.cMin=make_float3(FLT_MAX  ,FLT_MAX  ,FLT_MAX  );
	mGPUCollisionEngine.mGPU.mNBodies=num_Bodies;
	mGPUCollisionEngine.mGPU.mNSpheres=num_Bodies;

	float maxrad=0;

	mGPUCollisionEngine.mGPU.mAuxData.clear();
	for(int i=0; i<num_Bodies; ++i){
		mGPUCollisionEngine.mGPU.mDataSpheres.push_back(bData[i]);

		mGPUCollisionEngine.mGPU.cMax.x=max(mGPUCollisionEngine.mGPU.cMax.x,(float)bData[i].x+bData[i].w);
		mGPUCollisionEngine.mGPU.cMax.y=max(mGPUCollisionEngine.mGPU.cMax.y,(float)bData[i].y+bData[i].w);
		mGPUCollisionEngine.mGPU.cMax.z=max(mGPUCollisionEngine.mGPU.cMax.z,(float)bData[i].z+bData[i].w);
		mGPUCollisionEngine.mGPU.cMin.x=min(mGPUCollisionEngine.mGPU.cMin.x,(float)bData[i].x-bData[i].w);
		mGPUCollisionEngine.mGPU.cMin.y=min(mGPUCollisionEngine.mGPU.cMin.y,(float)bData[i].y-bData[i].w);
		mGPUCollisionEngine.mGPU.cMin.z=min(mGPUCollisionEngine.mGPU.cMin.z,(float)bData[i].z-bData[i].w);

		maxrad=max(maxrad,bData[i].w);

		float4 temp=F4(i,1,0,1);
		mGPUCollisionEngine.mGPU.mAuxData.push_back(temp);
	}
	mGPUCollisionEngine.mGPU.mMaxRad=maxrad;

	mGPUCollisionEngine.mGPU.InitCudaCollision();
	mGPUCollisionEngine.mGPU.CudaCollision();
	totalC=mGPUCollisionEngine.mGPU.mNumContacts;

	QueryPerformanceCounter((LARGE_INTEGER *)&ctr2);
	cout<<"GPU: "<<((ctr2 - ctr1) * 1.0 / freq)<<"\t";

	//cout<<"COPY"<<endl;
	//vector<float4> GPU_contacts=mGPUCollisionEngine.mGPU.CopyContactstoHost();
	//Contact temp;



	//CData[offset+mMaxContactD*0]=make_float4(n,0);
	//	CData[offset+mMaxContactD*3]=make_float4(onA,A);
	//	CData[offset+mMaxContactD*6]=make_float4(onB,B);
	//	CData[offset+mMaxContactD*9]=make_float4(0,0,0,mKF1*mKF2);



	//for(int i=0; i<totalC; i++){
	//	temp.nx=GPU_contacts[i*3].x;
	//	temp.ny=GPU_contacts[i*3].y;
	//	temp.nz=GPU_contacts[i*3].z;

	//	temp.x1=GPU_contacts[i*3+1].x;
	//	temp.y1=GPU_contacts[i*3+1].y;
	//	temp.z1=GPU_contacts[i*3+1].z;
	//	temp.a =GPU_contacts[i*3+1].w;

	//	temp.x2=GPU_contacts[i*3+2].x;
	//	temp.y2=GPU_contacts[i*3+2].y;
	//	temp.z2=GPU_contacts[i*3+2].z;
	//	temp.b =GPU_contacts[i*3+2].w;
	//	contact_storage.push_back(temp);
	//}
	//////cout<<"SORT"<<endl;
	//sort(contact_storage.begin(),contact_storage.end(),test_func);
	mGPUCollisionEngine.mGPU.mNBodies=0;
}

bool sphere_sphere(int num_Bodies){
	//generate data on host
	mData.clear();
	for(int a=0; a<num_Bodies; a++){
		float w=rand()%100/10.0+.1;
		float randx=rand()%1000-(1000)/2.0;
		float randy=rand()%1000-(1000)/2.0;
		float randz=rand()%1000-(1000)/2.0;
		float4 temp=make_float4(randx,randy,randz,w);
		mData.push_back(temp);


	}
	vector<Contact> A;
	vector<Contact> B;
	int totalCPU=0;
	int totalGPU=0;

	bullet_sphere_sphere(num_Bodies,A, mData, totalCPU);
	cout<<totalCPU<<"\t";
	gpu_sphere_sphere(num_Bodies,B, mData, totalGPU);
	cout<<"\t"<<totalGPU;

	//cout<<bData[32].x<<" "<<bData[32].y<<" "<<bData[32].z<<" "<<bData[32].w<<endl;
	//cout<<bData[45].x<<" "<<bData[45].y<<" "<<bData[45].z<<" "<<bData[45].w<<endl;

	if(totalCPU!=totalGPU){cout<<"\t FAIL"<<endl;}
	//if(!verify(A,B)){cout<<"\t FAIL"<<endl;}
	else {cout<<"\t PASS"<<endl;}

	//A.clear();
	//B.clear();
	/*glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(600	,600);
	glutCreateWindow("Wave Tank");

	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glutDisplayFunc(renderScene);
	glutIdleFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutMainLoop();*/













	return true;
}

bool test_func(Contact one, Contact two){
	if (one.a == two.a)
	{return one.b < two.b;}
	else
	{return one.a < two.a;}
}

bool verify(vector<Contact> A, vector<Contact> B){
	float angle_tolerance=cos(double((10.0*3.14)/180.0));
	for(int i=0; i<A.size(); ++i){
		//cout<<A[i].a<<" "<<B[i].a<<" "<<A[i].b<<" "<<B[i].b<<endl;
		if(A[i].a!=B[i].a||A[i].b!=B[i].b){return false;}
		if(B[i].x1>A[i].x1+.1||B[i].x1<A[i].x1-.1){return false;}
		if(B[i].y1>A[i].y1+.1||B[i].y1<A[i].y1-.1){return false;}
		if(B[i].z1>A[i].z1+.1||B[i].z1<A[i].z1-.1){return false;}

		if(B[i].x2>A[i].x2+.1||B[i].x2<A[i].x2-.1){return false;}
		if(B[i].y2>A[i].y2+.1||B[i].y2<A[i].y2-.1){return false;}
		if(B[i].z2>A[i].z2+.1||B[i].z2<A[i].z2-.1){return false;}
		float dotp=A[i].nx*B[i].nx+A[i].ny*B[i].ny+A[i].nz*B[i].nz;
		if (dotp<angle_tolerance){return false;}
	}
	return true;
}


