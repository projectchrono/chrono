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
#include "unit_GPU/ChSystemGPU.h"
#include <GL/freeglut.h>
#include "omp.h"
#include "unit_GPU/ChCuda.h"
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
float Scale=1;
bool showSphere = true;
bool showSolid = false;
bool updateDraw=true;
bool saveData =false;
bool showContacts=false;
int detail=1;
class System{
public:
	System(ChSystemGPU * Sys, int nobjects, string timingfile);
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
	void MakeCylinder(ChSharedBodyGPUPtr &body, ChVector<> radius, double mass,ChVector<> pos,ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide);
	void MakeCompound(ChSharedBodyGPUPtr &body,ChVector<> p, ChQuaternion<> r,float mass, vector<float3> pos, vector<float3> dim, vector<float4> quats, vector<ShapeType> type, vector<float> masses,double sfric,double kfric,double restitution,bool collide);
	void LoadTriangleMesh(ChSharedBodyGPUPtr &mrigidBody,string name, float scale, ChVector<> pos, ChQuaternion<> rot, float mass,double sfric,double kfric,double restitution, int family,int nocolwith);
	void DeactivationPlane(float y, float h, bool disable);
	void BoundingPlane(float y);
	void BoundingBox(float x,float y, float z,float offset);

	void SaveByID(int id, string fname, bool pos, bool vel, bool acc, bool rot, bool omega);
	void SaveAllData(string prefix, bool p, bool v, bool a, bool r, bool o);
	void drawAll();
	void renderScene(){	PrintStats();	DoTimeStep();}
	ChSystemGPU *mSystem;

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

System::System(ChSystemGPU * Sys, int nobjects, string timingfile){
	mSystem=Sys;
	mCurrentTime=0;
	mCameraX=0, mCameraY=0, mCameraZ=0;
	mNumObjects=nobjects;
	mNumCurrentObjects=0;
	mFrameNumber=0;
	mFileNumber=0;
	mTimingFile.open(timingfile.c_str());
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
	double KE=GetKE();
	char numstr[512];
	        printf("%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %f\n",A,B,C,D,E,F,KE);
	sprintf(numstr,"%7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %f",A,B,C,D,E,F,KE);
	mTimingFile<<numstr<<endl;
}
void System::LoadTriangleMesh(ChSharedBodyGPUPtr &mrigidBody,string name, float scale, ChVector<> position, ChQuaternion<> rot, float mass,double sfric,double kfric,double restitution, int family,int nocolwith){
	geometry::ChTriangleMesh TriMesh;
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

	for(int i=0; i<tri.size(); i++){
		ChVector<> A(pos[tri[i].x-1].x,pos[tri[i].x-1].y,pos[tri[i].x-1].z);
		ChVector<> B(pos[tri[i].y-1].x,pos[tri[i].y-1].y,pos[tri[i].y-1].z);
		ChVector<> C(pos[tri[i].z-1].x,pos[tri[i].z-1].y,pos[tri[i].z-1].z);
		A*=scale;
		B*=scale;
		C*=scale;
		TriMesh.addTriangle(A,B,C);
	}

	mrigidBody.get_ptr()->SetMass(mass);
	mrigidBody.get_ptr()->SetPos(position);

	mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
	mrigidBody.get_ptr()->GetCollisionModel()->AddTriangleMesh(TriMesh,false,false);
	mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
	mrigidBody.get_ptr()->SetBodyFixed(false);
	mrigidBody.get_ptr()->SetCollide(true);
	mrigidBody.get_ptr()->SetImpactC(0.0);
	mrigidBody.get_ptr()->SetSfriction(sfric);
	mrigidBody.get_ptr()->SetKfriction(kfric);
	mrigidBody.get_ptr()->SetRot(rot);
	mSystem->AddBody(mrigidBody);
	mrigidBody.get_ptr()->GetCollisionModel()->SetFamily(family);
	mrigidBody.get_ptr()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(nocolwith);
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
void System::MakeCylinder(ChSharedBodyGPUPtr &body, ChVector<> radius, double mass,ChVector<> pos, ChQuaternion<> rot,double sfric,double kfric,double restitution,bool collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->SetRot(rot);
	body.get_ptr()->GetCollisionModel()->ClearModel();
	(ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel())->AddCylinder(radius.x,radius.y,radius.z);
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::MakeCompound(ChSharedBodyGPUPtr &body,ChVector<> p, ChQuaternion<> r,float mass, vector<float3> pos, vector<float3> dim, vector<float4> quats, vector<ShapeType> type, vector<float> masses,double sfric,double kfric,double restitution,bool collide){

	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(p);
	body.get_ptr()->SetRot(r);


	body.get_ptr()->GetCollisionModel()->ClearModel();
	((ChCollisionModelGPU *)(body.get_ptr()->GetCollisionModel()))->AddCompoundBody(pos,dim,quats,type,masses);
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(0);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
	mSystem->AddBody(body);
}
void System::DeactivationPlane(float y, float h, bool disable){
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
		if(abody->GetPos().y<y){
			abody->SetCollide(!disable);
			abody->SetBodyFixed(disable);
			abody->SetPos(ChVector<>(0,h,0));
		}
	}
}
void System::BoundingPlane(float y){
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));
		if(abody->GetPos().y<y&&abody->GetBodyFixed()==false){
			abody->SetPos(ChVector<>(abody->GetPos().x,y,abody->GetPos().z));
			abody->SetPos_dt(ChVector<>(0,0,0));
		}
	}
}
void System::BoundingBox(float x,float y, float z, float offset){
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBodyGPU *abody=(ChBodyGPU*)(mSystem->Get_bodylist()->at(i));

		if(abody->GetBodyFixed()==true){continue;}
		ChVector<> pos=abody->GetPos();
		ChVector<> vel=abody->GetPos_dt();
		ChVector<> force(0,0,0);
		if(pos.x+offset> x||pos.x-offset<-x){force.x=(x-pos.x);}
		if(pos.y+offset> y||pos.y-offset<-y){force.y=(y-pos.y);}
		if(pos.z+offset> z||pos.z-offset<-z){force.z=(z-pos.z);}

		abody->Set_Scr_force(force*.1);
	}
}
void System::SaveByID(int id, string fname, bool p, bool v, bool a, bool r, bool o){
	ofstream ofile;
	ofile.open(fname.c_str(),ios_base::app);

	ChBody* abody = mSystem->Get_bodylist()->at(id);
	ChVector<> pos=abody->GetPos();
	ChVector<> rot=abody->GetRot().Q_to_NasaAngles();
	ChVector<> vel=abody->GetPos_dt();
	ChVector<> acc=abody->GetPos_dtdt();
	ChVector<> trq=abody->Get_gyro();

	if(isnan(rot.x)){rot.x=0;}
	if(isnan(rot.y)){rot.y=0;}
	if(isnan(rot.z)){rot.z=0;}

	if(p){ofile<<pos.x<<","<<pos.y<<","<<pos.z<<",";}
	if(v){ofile<<vel.x<<","<<vel.y<<","<<vel.z<<",";}
	if(a){ofile<<acc.x<<","<<acc.y<<","<<acc.z<<",";}
	if(r){ofile<<rot.x<<","<<rot.y<<","<<rot.z<<",";}
	if(o){ofile<<trq.x<<","<<trq.y<<","<<trq.z<<",";}
	ofile<<endl;

	ofile.close();
}


void System::SaveAllData(string prefix, bool p, bool v, bool a, bool r, bool o){
	ofstream ofile;
	stringstream ss;
	ss<<prefix<<mFileNumber<<".txt";
	ofile.open(ss.str().c_str());
	for(int i=0; i<mSystem->Get_bodylist()->size(); i++){
		ChBody* abody = mSystem->Get_bodylist()->at(i);
		ChVector<> pos=abody->GetPos();
		ChVector<> rot=abody->GetRot().Q_to_NasaAngles();
		ChVector<> vel=abody->GetPos_dt();
		ChVector<> acc=abody->GetPos_dtdt();
		ChVector<> trq=abody->Get_gyro();
		if(isnan(rot.x)){rot.x=0;}
		if(isnan(rot.y)){rot.y=0;}
		if(isnan(rot.z)){rot.z=0;}
		if(p){ofile<<pos.x<<","<<pos.y<<","<<pos.z<<",";}
		if(v){ofile<<vel.x<<","<<vel.y<<","<<vel.z<<",";}
		if(a){ofile<<acc.x<<","<<acc.y<<","<<acc.z<<",";}
		if(r){ofile<<rot.x<<","<<rot.y<<","<<rot.z<<",";}
		if(o){ofile<<trq.x<<","<<trq.y<<","<<trq.z<<",";}
		ofile<<endl;
	}
	ofile.close();
	mFileNumber++;
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
void makeSphere(float3 pos, float rad, float angle, float3 axis, float3 scale){
	glPushMatrix();
	glTranslatef(pos.x,pos.y,pos.z);
	glRotatef(angle*180.0/PI, axis.x, axis.y, axis.z);
	glScalef(scale.x,scale.y,scale.z);
	if(showSphere){
		if(!showSolid){glutWireSphere(rad,10,10);}else{glutSolidSphere(rad,10,10);}
	}
	else{
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex3f(0, 0, 0);
		glEnd();
	}
	glPopMatrix();
}
void makeBox(float3 pos, float rad, float angle, float3 axis, float3 scale){
	glPushMatrix();
	glTranslatef(pos.x,pos.y,pos.z);
	//glRotatef(90, 1, 0, 0);
	glRotatef(angle*180.0/PI, axis.x, axis.y, axis.z);
	glScalef(scale.x*2,scale.y*2,scale.z*2);
	if(!showSolid){glutWireCube(1);}else{glutSolidCube(1);}
	glPopMatrix();
}
void makeCyl(float3 pos, float rad, float angle, float3 axis, float3 scale){

	GLUquadric *quad=gluNewQuadric();

	if(!showSolid){gluQuadricDrawStyle(quad,GLU_LINE);}else{gluQuadricDrawStyle(quad,GLU_FILL);}
	glPushMatrix();
	glTranslatef(pos.x,pos.y,pos.z);
	glRotatef(90, 1, 0, 0);
	glRotatef(angle*180.0/PI, axis.x, axis.y, axis.z);
	//glScalef(scale.x*2,scale.y*2,scale.z*2);
	gluCylinder(quad,scale.x,scale.z,scale.y*2,10,10);

	glPopMatrix();
}

void drawSphere(ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,.1);
	glColor3f (color.x, color.y,color.z);
	double angle;
	ChVector<> axis;
	abody->GetRot().Q_to_AngAxis(angle,axis);
	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
	makeSphere(F3(abody->GetPos().x,abody->GetPos().y,abody->GetPos().z),h.x,angle, F3(axis.x,axis.y,axis.z) , F3(1,1,1));
}

void drawEllipsoid(ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,1);
	glColor3f (color.x, color.y,color.z);
	double angle;
	ChVector<> axis;
	abody->GetRot().Q_to_AngAxis(angle,axis);
	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
	makeSphere(F3(abody->GetPos().x,abody->GetPos().y,abody->GetPos().z),1,angle, F3(axis.x,axis.y,axis.z) , F3(h.x,h.y,h.z));
}

void drawBox(ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,1);
	glColor4f (color.x, color.y,color.z, 1);
	double angle;
	ChVector<> axis;
	abody->GetRot().Q_to_AngAxis(angle,axis);
	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
	makeBox(F3(abody->GetPos().x,abody->GetPos().y,abody->GetPos().z),1,angle, F3(axis.x,axis.y,axis.z) , F3(h.x,h.y,h.z));
}
void drawCyl(ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,1);
	glColor4f (color.x, color.y,color.z, 1);
	double angle;
	ChVector<> axis;
	float4 h=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[0].B;
	abody->GetRot().Q_to_AngAxis(angle,axis);
	makeBox(F3(abody->GetPos().x,abody->GetPos().y,abody->GetPos().z),1,angle, F3(axis.x,axis.y,axis.z) , F3(h.x,h.y,h.z));
}
void drawTriMesh(ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,1);
	glColor3f (color.x, color.y,color.z);
	int numtriangles=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData.size();
	for(int i=0; i<numtriangles; i++){
		float4 p1=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[i].A;
		float4 p2=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[i].B;
		float4 p3=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[i].C;

		ChVector<> gA = (abody)->GetCoord().TrasformLocalToParent(ChVector<>(p1.x,p1.y,p1.z));
		ChVector<> gB = (abody)->GetCoord().TrasformLocalToParent(ChVector<>(p2.x,p2.y,p2.z));
		ChVector<> gC = (abody)->GetCoord().TrasformLocalToParent(ChVector<>(p3.x,p3.y,p3.z));
		glColor4f (0, 0,0,.3);
		glBegin(GL_LINE_LOOP);
		glVertex3f(gA.x,gA.y,gA.z);
		glVertex3f(gB.x,gB.y,gB.z);
		glVertex3f(gC.x,gC.y,gC.z);
		glEnd();
	}
}
void drawCompound(ChBody *abody){
	float3 color=GetColour(abody->GetPos_dt().Length(),0,1);
	glColor3f (color.x, color.y,color.z);
	int numobjects=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData.size();
	for(int i=0; i<numobjects; i++){
		float4 A=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[i].A;
		float4 B=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[i].B;
		float4 C=((ChCollisionModelGPU *)(abody->GetCollisionModel()))->mData[i].C;
		ChVector<> pos = (abody)->GetCoord().TrasformLocalToParent(ChVector<>(A.x,A.y,A.z));
		int type=B.w;
		double angle;
		ChVector<> axis;

		ChQuaternion<> quat=abody->GetRot();
		quat=quat%ChQuaternion<>(C.x,C.y,C.z,C.w);
		quat.Normalize();
		quat.Q_to_AngAxis(angle,axis);
		if(type==0){makeSphere(F3(pos.x,pos.y,pos.z),B.x,angle, F3(axis.x,axis.y,axis.z) , 	F3(1,1,1));}
		if(type==2){makeBox(F3(pos.x,pos.y,pos.z),1,angle, F3(axis.x,axis.y,axis.z) , 		F3(B.x,B.y,B.z));}
		if(type==3){makeSphere(F3(pos.x,pos.y,pos.z),1,angle, F3(axis.x,axis.y,axis.z) , 	F3(B.x,B.y,B.z));}
		if(type==4){makeBox(F3(pos.x,pos.y,pos.z),1,angle, F3(axis.x,axis.y,axis.z) , 		F3(B.x,B.y,B.z));}
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
	if (key=='w'){camera_pos_delta+=dir*Scale;}
	if (key=='s'){camera_pos_delta-=dir*Scale;}
	if (key=='d'){camera_pos_delta+=cross(dir,camera_up)*Scale;}
	if (key=='a'){camera_pos_delta-=cross(dir,camera_up)*Scale;}
	if (key=='q'){camera_pos_delta+=camera_up*Scale;}
	if (key=='e'){camera_pos_delta-=camera_up*Scale;}
	if (key=='u'){updateDraw=(updateDraw)? 0:1;}
	if (key=='i'){showSphere=(showSphere)? 0:1;}
	if (key=='o'){showSolid=(showSolid)? 0:1;}
	if (key=='z'){saveData=1;}
	if (key=='['){detail=max(1,detail-1);}
	if (key==']'){detail++;}
	if (key=='c'){showContacts=(showContacts)? 0:1;}
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
	gluPerspective(45,ratio,.01,1000);
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
	//glFrontFace(GL_CCW);
	//glCullFace(GL_BACK);
	//glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0);
	glPointSize(2);
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
			if(abody->GetCollisionModel()->GetShapeType()==SPHERE&&i%detail==0){
				drawSphere(abody);
			}
			if(abody->GetCollisionModel()->GetShapeType()==BOX){
				drawBox(abody);
			}
			if(abody->GetCollisionModel()->GetShapeType()==ELLIPSOID){
				drawEllipsoid(abody);
			}
			if(abody->GetCollisionModel()->GetShapeType()==CYLINDER){
				drawCyl(abody);
			}

			if(abody->GetCollisionModel()->GetShapeType()==TRIANGLEMESH){
				glColor3f (0,0,0);
				drawTriMesh(abody);
			}
			if(abody->GetCollisionModel()->GetShapeType()==COMPOUND){
				glColor3f (0,0,0);
				drawCompound(abody);
			}
		}
		if(showContacts){
			ChLcpSystemDescriptorGPU* mGPUDescriptor=(ChLcpSystemDescriptorGPU *)mSystem->GetLcpSystemDescriptor();
			for(int i=0; i<mGPUDescriptor->gpu_collision->contact_data_host.size(); i++){
				float3 N=mGPUDescriptor->gpu_collision->contact_data_host[i].N;
				float3 Pa=mGPUDescriptor->gpu_collision->contact_data_host[i].Pa;
				float3 Pb=mGPUDescriptor->gpu_collision->contact_data_host[i].Pb;
				float D=mGPUDescriptor->gpu_collision->contact_data_host[i].I.x;
				float3 color=GetColour(D,0,.0001);
				glColor3f (color.x, color.y,color.z);
				glBegin(GL_LINES);
				glVertex3f(Pa.x, Pa.y, Pa.z);
				glVertex3f(Pb.x, Pb.y, Pb.z);
				glEnd();
				glBegin(GL_POINTS);
				glVertex3f(Pa.x, Pa.y, Pa.z);
				glVertex3f(Pb.x, Pb.y, Pb.z);
				glEnd();

			}
		}
#if defined( _WINDOWS )
			Sleep( 40 );
#else
	usleep( 40 * 1000 );
#endif
	glutSwapBuffers();
	}

}
System *GPUSystem;
void renderSceneAll(){
	GPUSystem->drawAll();
}
void initGLUT(string name, int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(1024	,512);
	glutCreateWindow(name.c_str());
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

void simulationLoop(){
	while(GPUSystem->mSystem->GetChTime()<=GPUSystem->mEndTime){
		GPUSystem->renderScene();
	}
	cout<< "Simulation Complete"<<endl;
#if defined( _WINDOWS )
	Sleep( 2000 );
#else
	usleep( 2000 * 1000 );
#endif
	exit(0);
}
