#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "collision\bullet\btBulletCollisionCommon.h"
#include "collision\ChCModelBullet.h"
#include "collision\ChCModelBulletBody.h"
#include "cutil_math.h"
#include "unit_GPU/ChLcpIterativeSolverGPUsimple.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
using namespace std;
using namespace chrono::collision;
using namespace chrono;

struct Contact{
	int a,b;
	float x1,y1,z1,x2,y2,z2,nx,ny,nz;
};
void bullet_sphere_sphere(int num_Bodies,vector<Contact> &CPU_contact_storage, vector<float4> &bData, int &totalC);
void gpu_sphere_sphere(int num_Bodies,vector<Contact> &CPU_contact_storage, vector<float4> &bData, int &totalC);

bool sphere_sphere(int);
//void sphere_triangle();
//void sphere_box();
bool verify(vector<Contact> A, vector<Contact> B);
bool test_func(Contact one, Contact two);


int main(){
	cudaSetDevice(0);
	cout<<"Bullet Collision Detection "<<endl;
	sphere_sphere(10000);	//tests n random spheres against each other
	//sphere_triangle();
	//sphere_box();

	return 0;
}
void bullet_sphere_sphere(int num_Bodies,vector<Contact> &CPU_contact_storage, vector<float4> &bData, int &totalC){
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
		float c_start=clock()/1000.0;
		collisionWorld->performDiscreteCollisionDetection();
		cout<<"Collision Detection Completed"<<endl;
		cout<<endl<<"Collision Detection took: "<<(clock()/1000.0-c_start)<<" seconds"<<endl<<endl;
	}

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
	for (int k=0;k<numManifolds;k++)
	{
		btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(k);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		int numContacts = contactManifold->getNumContacts();

		totalC+= numContacts;
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			Contact temp;
			temp.a=obA->getCompanionId();
			temp.b=obB->getCompanionId();
			temp.x1=pt.m_positionWorldOnA.x();
			temp.y1=pt.m_positionWorldOnA.y();
			temp.z1=pt.m_positionWorldOnA.z();
			temp.x2=pt.m_positionWorldOnB.x();
			temp.y2=pt.m_positionWorldOnB.y();
			temp.z2=pt.m_positionWorldOnB.z();
			temp.nx=pt.m_normalWorldOnB.x();
			temp.ny=pt.m_normalWorldOnB.y();
			temp.nz=pt.m_normalWorldOnB.z();

			CPU_contact_storage.push_back(temp);
		}
		contactManifold->clearManifold();
	}
	//cout<<totalC<<" Contacts found"<<endl;
	sort(CPU_contact_storage.begin(),CPU_contact_storage.end(),test_func);
}
void gpu_sphere_sphere(int num_Bodies,vector<Contact> &contact_storage, vector<float4> &bData, int &totalC){
	float envelope =0;
	float bin_size= .5;
	int mMaxContact=num_Bodies*5;


	ChLcpSystemDescriptorGPU		newdescriptor(num_Bodies+100,mMaxContact, 1 );
	ChCollisionSystemGPU			mGPUCollisionEngine(envelope,bin_size, mMaxContact);
	mGPUCollisionEngine.SetSystemDescriptor(&newdescriptor);
	//totalC+= numContacts;
	mGPUCollisionEngine.mGPU.cMax=make_float3(-FLT_MAX  ,-FLT_MAX  ,-FLT_MAX  );
	mGPUCollisionEngine.mGPU.cMin=make_float3(FLT_MAX  ,FLT_MAX  ,FLT_MAX  );
	mGPUCollisionEngine.mGPU.mNBodies=num_Bodies;
	mGPUCollisionEngine.mGPU.mNSpheres=num_Bodies;
	mGPUCollisionEngine.mGPU.mMaxRad=100;
	for(int i=0; i<num_Bodies; ++i){
		mGPUCollisionEngine.mGPU.mDataSpheres.push_back(bData[i]);

		mGPUCollisionEngine.mGPU.cMax.x=max(mGPUCollisionEngine.mGPU.cMax.x,(float)bData[i].x);
		mGPUCollisionEngine.mGPU.cMax.y=max(mGPUCollisionEngine.mGPU.cMax.y,(float)bData[i].y);
		mGPUCollisionEngine.mGPU.cMax.z=max(mGPUCollisionEngine.mGPU.cMax.z,(float)bData[i].z);
		mGPUCollisionEngine.mGPU.cMin.x=min(mGPUCollisionEngine.mGPU.cMin.x,(float)bData[i].x);
		mGPUCollisionEngine.mGPU.cMin.y=min(mGPUCollisionEngine.mGPU.cMin.y,(float)bData[i].y);
		mGPUCollisionEngine.mGPU.cMin.z=min(mGPUCollisionEngine.mGPU.cMin.z,(float)bData[i].z);

		int3f temp=I3F(i,1,0,1);
		mGPUCollisionEngine.mGPU.mAuxData.push_back(temp);
	}

	cout<<"RUN"<<endl;
	mGPUCollisionEngine.mGPU.InitCudaCollision();
	mGPUCollisionEngine.mGPU.CudaCollision();
	totalC=mGPUCollisionEngine.mGPU.mNumContacts;

	vector<float4> GPU_contacts=mGPUCollisionEngine.mGPU.CopyContactstoHost();
	Contact temp;

	//CData[offset+mMaxContactD*0]=make_float4(n,0);
	//	CData[offset+mMaxContactD*3]=make_float4(onA,A);
	//	CData[offset+mMaxContactD*6]=make_float4(onB,B);
	//	CData[offset+mMaxContactD*9]=make_float4(0,0,0,mKF1*mKF2);



	for(int i=0; i<totalC; i++){
		temp.nx=GPU_contacts[i*3].x;
		temp.ny=GPU_contacts[i*3].y;
		temp.nz=GPU_contacts[i*3].z;

		temp.x1=GPU_contacts[i*3+1].x;
		temp.y1=GPU_contacts[i*3+1].y;
		temp.z1=GPU_contacts[i*3+1].z;
		temp.a =GPU_contacts[i*3+1].w;

		temp.x2=GPU_contacts[i*3+2].x;
		temp.y2=GPU_contacts[i*3+2].y;
		temp.z2=GPU_contacts[i*3+2].z;
		temp.b =GPU_contacts[i*3+2].w;
		contact_storage.push_back(temp);
	}
	sort(contact_storage.begin(),contact_storage.end(),test_func);
	mGPUCollisionEngine.mGPU.mNBodies=0;
}
bool sphere_sphere(int num_Bodies){
	//generate data on host
	vector<float4> bData;
	for(int i=0; i<num_Bodies; i++){
		float4 temp=make_float4(rand()%10000/100.0,rand()%10000/100.0,rand()%10000/100.0,rand()%1000/1000.0);
		bData.push_back(temp);
	}

	vector<Contact> CPU_contact_storage;
	vector<Contact> GPU_contact_storage;
	int totalCPU=0;
	int totalGPU=0;

	bullet_sphere_sphere(num_Bodies,CPU_contact_storage, bData, totalCPU);
	gpu_sphere_sphere(num_Bodies,GPU_contact_storage, bData, totalGPU);
	cout<<totalCPU<<endl;
	cout<<totalGPU<<endl;
	if(totalCPU!=totalGPU){cout<<"FAIL"<<endl;}
	else if(!verify(CPU_contact_storage,GPU_contact_storage)){
	cout<<"FAIL VERIFY"<<endl;
	}
	else {cout<<"PASS"<<endl;}


	CPU_contact_storage.clear();
	GPU_contact_storage.clear();
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
		if(A[i].a!=B[i].a){cout<<B[i].a<<" "<<A[i].a<<endl;return false;}
		if(A[i].b!=B[i].b){cout<<B[i].b<<" "<<A[i].b<<endl;return false;}

		if(B[i].x1>A[i].x1+.1||B[i].x1<A[i].x1-.1){cout<<B[i].x1<<" "<<A[i].x1<<endl; return false;}
		if(B[i].y1>A[i].y1+.1||B[i].y1<A[i].y1-.1){cout<<B[i].y1<<" "<<A[i].y1<<endl; return false;}
		if(B[i].z1>A[i].z1+.1||B[i].z1<A[i].z1-.1){cout<<B[i].z1<<" "<<A[i].z1<<endl; return false;}

		if(B[i].x2>A[i].x2+.1||B[i].x2<A[i].x2-.1){cout<<B[i].x2<<" "<<A[i].x2<<endl; return false;}
		if(B[i].y2>A[i].y2+.1||B[i].y2<A[i].y2-.1){cout<<B[i].y2<<" "<<A[i].y2<<endl; return false;}
		if(B[i].z2>A[i].z2+.1||B[i].z2<A[i].z2-.1){cout<<B[i].z2<<" "<<A[i].z2<<endl; return false;}

		float dotp=A[i].nx*B[i].nx+A[i].ny*B[i].ny+A[i].nz*B[i].nz;

		if (dotp<angle_tolerance){cout<<A[i].nx<<" "<<B[i].nx<<" "<<A[i].ny<<" "<<B[i].ny<<" "<<A[i].nz<<" "<<B[i].nz<<endl; return false;}
	}
return true;
}

//bool verify(void) {
//	cout<<"Verifying Results"<<endl;
//
//	ifstream f1("bullet_output.txt");
//	ifstream f2("cuda_output.txt");
//
//	int indexb0=0,indexb1=0, indexc0=0, indexc1=0; 
//	float ab0x=0,ab0y=0,ab0z=0,ab1x=0,ab1y=0,ab1z=0,ac0x=0,ac0y=0,ac0z=0,ac1x=0,ac1y=0,ac1z=0;
//	float nbx=0,nby=0,nbz=0,ncx=0,ncy=0,ncz=0;
//	float dotp=0;
//	float angle_tolerance=cos(double((10.0*3.14)/180.0));
//	for(int i=0; i<totalC; ++i){
//
//		f1>>indexb0>>indexb1>>ab0x>>ab0y>>ab0z>>ab1x>>ab1y>>ab1z>>nbx>>nby>>nbz;
//		f2>>indexc0>>indexc1>>ac0x>>ac0y>>ac0z>>ac1x>>ac1y>>ac1z>>ncx>>ncy>>ncz;
//		if(indexb0!=indexc0){return false;}
//		else if(indexb1!=indexc1){return false;}
//
//		if(ac0x>ab0x+.1||ac0x<ab0x-.1){return false;}
//		else if(ac0y>ab0y+.1||ac0y<ab0y-.1){return false;}
//		else if(ac0z>ab0z+.1||ac0z<ab0z-.1){return false;}
//
//		else if(ac1x>ab1x+.1||ac1x<ab1x-.1){return false;}
//		else if(ac1y>ab1y+.1||ac1y<ab1y-.1){return false;}
//		else if(ac1z>ab1z+.1||ac1z<ab1z-.1){return false;}
//
//		dotp=nbx*ncx+nby*ncy+nbz*ncz;
//
//		if (dotp<angle_tolerance){return false;}
//	}
//	f1.close();
//	f2.close();
//	cout<<"Verification Complete"<<endl;
//	return true;
//}
