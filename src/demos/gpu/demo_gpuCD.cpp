#include <string>
#include <fstream>
#include <sstream>
#include <GL/glut.h>
#include <cutil_inline.h>
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChContactContainerBase.h"
#include "unit_GPU/ChLcpIterativeSolverGPUsimple.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
#include "unit_GPU/ChCCollisionModelGPU.h"
#include "unit_GPU/ChBodyGPU.h"
using namespace chrono;
using namespace std;

bool useOpenGL=1;
bool saveTimingData=0;
bool saveSimData;

double mSphereRadius=.4;
double mSphereEnvelope=.01;
double mSphereMass = .001;
double mSphereRestitution=0.0;
double mTimeStep=.01;
double mCurrentTime=0;
double mMu=.1;
double mTriangleScale=1/10.0;
double mEndTime=10.0;
double mCameraX=0, mCameraY=0, mCameraZ=0;
double mBoundingBoxSize=10;

double mOffsetX=0;
double mOffsetY=0;
double mOffsetZ=0;
int mNumSpheres=10000;
int mNumCurrentSpheres=0;
int mNumBoxes=1;
int mNumTriangles=0;
int mFrameNumber=0;
int mSaveEveryX=(1/mTimeStep)/100.0;
int mCudaDevice=0;

ChSystem* mPhysicalSystem;

ofstream mTimingFile;
ofstream mDataFile;

ChSharedBodyPtr mgroundBody;

void makeSphere(ChSharedBodyPtr &body, double radius, double mass,ChVector<> pos,double sfric,double kfric,double restitution,double collide){
	body.get_ptr()->SetMass(mass);
	body.get_ptr()->SetPos(pos);
	body.get_ptr()->GetCollisionModel()->ClearModel();
	body.get_ptr()->GetCollisionModel()->AddSphere(mSphereRadius);
	body.get_ptr()->GetCollisionModel()->BuildModel();
	body.get_ptr()->SetCollide(collide);
	body.get_ptr()->SetImpactC(restitution);
	body.get_ptr()->SetSfriction(sfric);
	body.get_ptr()->SetKfriction(kfric);
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


//creates a grid of spheres at a certain location
void createSpheres(int x, int y, int z, double posX, double posY, double posZ){
	ChSharedBodyPtr mrigidBody;
	for (int xx=0; xx<x; xx++){
		for (int yy=0; yy<y; yy++){
			for (int zz=0; zz<z; zz++){
				ChVector<> mParticlePos((xx-x/2.0)-mOffsetX+rand()%1000/10000.0,yy-mOffsetY+rand()%1000/10000.0,(zz-z/2.0)-mOffsetZ+rand()%1000/10000.0);
				mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
				makeSphere(mrigidBody, mSphereRadius, mSphereMass, mParticlePos, mMu, mMu, mSphereRestitution, true);
				mPhysicalSystem->AddBody(mrigidBody);
			}
		}
	}
}


void loadTriangleMesh(string name){
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
	for(int i=0; i<tri.size(); i++){
		ChVector<> A(pos[tri[i].x-1].x,pos[tri[i].x-1].y,pos[tri[i].x-1].z);
		ChVector<> B(pos[tri[i].y-1].x,pos[tri[i].y-1].y,pos[tri[i].y-1].z);
		ChVector<> C(pos[tri[i].z-1].x,pos[tri[i].z-1].y,pos[tri[i].z-1].z);
		A*=mTriangleScale;
		B*=mTriangleScale;
		C*=mTriangleScale;

		mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
		mrigidBody.get_ptr()->SetMass(100000);
		ChVector<> particle_pos2(0,-mOffsetY,0);
		mrigidBody.get_ptr()->SetPos(particle_pos2);
		mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
		((ChCollisionModelGPU*)mrigidBody.get_ptr()->GetCollisionModel())->AddTriangle(A,B,C);
		mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
		mrigidBody.get_ptr()->SetBodyFixed(true);
		mrigidBody.get_ptr()->SetCollide(true);
		mrigidBody.get_ptr()->SetImpactC(0.0);
		mrigidBody.get_ptr()->SetSfriction(0.05);
		mrigidBody.get_ptr()->SetKfriction(0.05);
		mPhysicalSystem->AddBody(mrigidBody);

	}
}
void doTimeStep(){
	cout<<"T:   "<<mPhysicalSystem->GetChTime()
		<<"\tC: "<<mPhysicalSystem->GetTimerStep()
		<<"\tG: "<<mPhysicalSystem->GetTimerCollisionBroad()
		<<"\tG: "<<mPhysicalSystem->GetTimerLcp()
		<<"\tB: "<<mPhysicalSystem->GetNbodies()
		<<"\tC: "<<((ChLcpSystemDescriptorGPU*)(mPhysicalSystem->GetLcpSystemDescriptor()))->nContactsGPU<<endl;

	if(mNumCurrentSpheres<mNumSpheres&&mFrameNumber%1000==0){
		createSpheres(10, 10, 10, 0, 0, 0);
		mNumCurrentSpheres+=10*10*10;
	}
	mPhysicalSystem->DoStepDynamics( mTimeStep );
	mCurrentTime+=mTimeStep;
	mFrameNumber++;
	if(mPhysicalSystem->GetChTime()>=mEndTime){exit(0);}
}

void renderScene(){
	doTimeStep();

	if(useOpenGL){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glFrontFace(GL_CCW);
		glCullFace(GL_BACK); 
		glEnable(GL_CULL_FACE);
		glDepthFunc(GL_LEQUAL);
		glClearDepth(1.0);

		glPointSize(2);
		glLoadIdentity();
		gluLookAt(cos(mCameraX)*mBoundingBoxSize*5, mCameraY, sin(mCameraZ)*mBoundingBoxSize*5, 
			0,0,0,
			0.0f,1.0f,0.0f);
		glColor3f (1,1,1);
		glutWireCube(mBoundingBoxSize*2);
	}
	std::vector<ChBody*>::iterator abody = mPhysicalSystem->Get_bodylist()->begin();
	while (abody != mPhysicalSystem->Get_bodylist()->end()){
		//ChCollisionModelGPU* bpointer = (chrono::collision::ChCollisionModelGPU*)(*abody);
		ChCollisionModelGPU* cModel = (ChCollisionModelGPU*)((*abody)->GetCollisionModel());
		int type=cModel->GetType();
		if(useOpenGL){
			if(type==0){

				ChVector<> gPos = (*abody)->GetCoord().TrasformLocalToParent(cModel->GetSpherePos(0));

				float3 color=GetColour((*abody)->GetPos_dt().Length(),0,10);
				glColor3f (color.x, color.y,color.z);
				glPushMatrix();
				glTranslatef (gPos.x, gPos.y, gPos.z);
				glutSolidSphere(cModel->GetSphereR(0),10,10);
				glPopMatrix();
			}
			if(type==1){

				for(int i=0; i<cModel->GetNObjects(); i++){
					ChVector<> gPos = (*abody)->GetCoord().TrasformLocalToParent(cModel->GetSpherePos(i));
					float3 color=GetColour((*abody)->GetPos_dt().Length(),0,10);
					glColor3f (color.x, color.y,color.z);
					glPushMatrix();
					glTranslatef (gPos.x, gPos.y, gPos.z);
					glutSolidSphere(cModel->GetSphereR(i),10,10);
					glPopMatrix();
				}

			}
			if(type==3){

				for(int i=0; i<cModel->GetNObjects(); i++){
					ChVector<> AA(cModel->mData[0].A.x,cModel->mData[0].A.y,cModel->mData[0].A.z);
					ChVector<> BB(cModel->mData[0].B.x,cModel->mData[0].B.y,cModel->mData[0].B.z);
					ChVector<> CC(cModel->mData[0].C.x,cModel->mData[0].C.y,cModel->mData[0].C.z);

					ChVector<> gA = (*abody)->GetCoord().TrasformLocalToParent(AA);
					ChVector<> gB = (*abody)->GetCoord().TrasformLocalToParent(BB);
					ChVector<> gC = (*abody)->GetCoord().TrasformLocalToParent(CC);
					float3 color=GetColour(10,0,10);
					glColor3f (color.x, color.y,color.z);
					glBegin(GL_LINE_LOOP);//start drawing a line loop
					glVertex3f(gA.x,gA.y,gA.z);//left of window
					glVertex3f(gB.x,gB.y,gB.z);//bottom of window
					glVertex3f(gC.x,gC.y,gC.z);//right of window
					glEnd();//end drawing of line loop
				}

			}
		}
		abody++;
	}
	if(useOpenGL){
		glutSwapBuffers();
		mCameraX+=.01;
		mCameraZ+=.01;
	}


}
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
		0.0,0.0,-mBoundingBoxSize,
		0.0f,1.0f,0.0f);
}
int main(int argc, char* argv[]){ 
	cudaSetDevice(mCudaDevice);
	DLL_CreateGlobals();
	ChLcpSystemDescriptorGPU		mGPUDescriptor(mNumSpheres+100,mNumSpheres*5, 1 );
	ChContactContainerGPUsimple		mGPUContactContainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(mSphereEnvelope,0, mNumSpheres*5);

	ChSystem mphysicalSystem(mNumSpheres+100, 50); 

	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUContactContainer, 150, 1e-3, 0.2);
	mGPUsolverSpeed.SetDt(mTimeStep);
	mphysicalSystem.ChangeLcpSystemDescriptor(&mGPUDescriptor);

	mphysicalSystem.ChangeContactContainer(&mGPUContactContainer);
	mphysicalSystem.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	mphysicalSystem.ChangeCollisionSystem(&mGPUCollisionEngine);

	mGPUCollisionEngine.SetSystemDescriptor(&mGPUDescriptor);
	mGPUsolverSpeed.SetSystemDescriptor(&mGPUDescriptor);

	mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU);
	mphysicalSystem.SetIterLCPmaxItersSpeed(10);
	mphysicalSystem.SetMaxPenetrationRecoverySpeed(.1);
	mphysicalSystem.SetIterLCPwarmStarting(false);
	mphysicalSystem.Set_G_acc(ChVector<>(0,-9.834,0));
	mPhysicalSystem=&mphysicalSystem;

	ChVector<> cgpos(0,0,0);
	mgroundBody= ChSharedBodyGPUPtr(new ChBodyGPU);
	mgroundBody->SetMass(100000);
	mgroundBody->SetPos(ChVector<>(0.0,0.0,0.0));
	((ChCollisionModelGPU*)mgroundBody.get_ptr()->GetCollisionModel())->ClearModel();
	((ChCollisionModelGPU*)mgroundBody.get_ptr()->GetCollisionModel())->AddBox(mBoundingBoxSize,mBoundingBoxSize,mBoundingBoxSize,&cgpos); 
	((ChCollisionModelGPU*)mgroundBody.get_ptr()->GetCollisionModel())->BuildModel();
	mgroundBody->SetBodyFixed(true);
	mgroundBody->SetCollide(true);
	mgroundBody->SetSfriction(0.05);
	mgroundBody->SetKfriction(0.05);
	mgroundBody->SetImpactC(0.0);
	mPhysicalSystem->AddBody(mgroundBody);

	if(!useOpenGL){
		while(true){
			renderScene();
		}
	}
	if(useOpenGL){
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
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
		glutMainLoop();
	}

	return 0;
}