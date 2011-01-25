#define OGL 1
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "unit_GPU/ChCCollisionModelGPU.h"
#include "unit_GPU/ChBodyGPU.h"
#include "physics/ChContactContainerBase.h"
#include <string>
#include <fstream>
#include <sstream>
#include <cutil_inline.h>
#include "unit_GPU/ChLcpIterativeSolverGPUsimple.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
#include <GL/glut.h>


using namespace chrono;
using namespace std;



float3 GetColour(double v,double vmin,double vmax)
{
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

double 
particle_radius = .004,
tank_radius=1,
envelope = 0.01,//0.0025
time_step= 0.005,
current_time = 0.0,
particle_friction=.03,
bin_size=.2;

unsigned int NSPSide=2;
bool ogl=0;
int num_particles = 5000;
int pieces=1,frame_number = 0,numP=0;
bool save=false;
int every_x_file=(1/.001)/100.0; 
ChSystem* mphysicalSystemG;
ofstream timer_file;
ofstream out_file;
ChSharedBodyPtr mgroundBody;




void create_falling_items(ChSystem* mphysicalSystem, double body_radius, float fric, int n_bodies,double tank_radius){
	double particle_mass = .0001;
	ChSharedBodyPtr mrigidBody;
	//ifstream domfile("domino.txt");
	vector<float4> spheres;
	float scale =10.0;
	float rad=particle_radius*scale*20;
	//for(int i=0; i<4; i++){
	//	float4 temp;
	//	domfile>>temp.x>>temp.z>>temp.y;
	//	temp.w=rad;
	//	cout<<temp.x<<"\t"<<temp.y<<"\t"<<temp.z<<endl;;
	//	spheres.push_back(temp/scale);
	//}
	//spheres.push_back(make_float4(0,	0,	0	,rad*10)/scale);
	/*spheres.push_back(make_float4(-.3,	0,	-.3	,rad)/scale);
	spheres.push_back(make_float4(.3,	0,	-.3	,rad)/scale);
	spheres.push_back(make_float4(-.3,	0,	.3	,rad)/scale);
	spheres.push_back(make_float4(.3,	0,	.3	,rad)/scale);
	spheres.push_back(make_float4(-.3,	.3,	-.3	,rad)/scale);
	spheres.push_back(make_float4(.3,	.3,	-.3	,rad)/scale);
	spheres.push_back(make_float4(-.3,	.3,	.3	,rad)/scale);
	spheres.push_back(make_float4(.3,	.3,	.3	,rad)/scale);*/
	//ifstream ifile("dom.txt");
	float x,y,z;
	float a=6;
	for (int yy=0; yy<10; yy++){
		for (int zz=0; zz<10; zz++){
			for (int xx=0; xx<10; xx++){
				//ChVector<> particle_pos((xx-5)/a+4,yy/a+23,(zz-5)/a-4.2);
				ChVector<> particle_pos((xx-5)/a+rand()%1000/10000.0,yy/a-tank_radius+3+rand()%1000/10000.0,(zz-5)/a+rand()%1000/10000.0);					//flow
				//ChVector<> particle_pos((xx-10)/a+rand()%1000/10000.0,yy/a+3,(zz-10)/a+rand()%1000/10000.0); //teapot
				//ChVector<> particle_pos(rand()%100/100.0-.5,-tank_radius+.2+rand()%100/10000.0,rand()%100/100.0-.5);
				mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
				mrigidBody.get_ptr()->SetMass(particle_mass);
				mrigidBody.get_ptr()->SetPos(particle_pos);
				mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
				//cModel->AddCompoundBody(8, spheres);
				mrigidBody.get_ptr()->GetCollisionModel()->AddSphere(rad/scale);
				mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
				mrigidBody.get_ptr()->SetCollide(true);
				mrigidBody.get_ptr()->SetImpactC(0.0);
				//mrigidBody->SetInertiaXX(chrono::Vector(.000005333,.000005333,.000005333));
				///mrigidBody->SetInertiaXY(chrono::Vector(0,0,0));
				mrigidBody.get_ptr()->SetSfriction(0.05);
				mrigidBody.get_ptr()->SetKfriction(0.05);
				mphysicalSystem->AddBody(mrigidBody);
			}
		}
	}
	//ChVector<> particle_pos(0,0,0);
	//mrigidBody = ChSharedBodyPtr(new ChBody);
	//mrigidBody.get_ptr()->SetMass(particle_mass);
	//mrigidBody.get_ptr()->SetPos(particle_pos);
	//mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
	//mrigidBody.get_ptr()->GetCollisionModel()->AddSphere(rad/scale);
	//mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
	//mrigidBody->SetBodyFixed(false);
	//mrigidBody.get_ptr()->SetCollide(true);
	//mrigidBody.get_ptr()->SetImpactC(0.0);
	//mrigidBody.get_ptr()->SetSfriction(0.05);
	//mrigidBody.get_ptr()->SetKfriction(0.05);
	//mphysicalSystem->AddBody(mrigidBody);

	//ChVector<> particle_pos2(.1,0,0);
	//mrigidBody = ChSharedBodyPtr(new ChBody);
	//mrigidBody.get_ptr()->SetMass(particle_mass);
	//mrigidBody.get_ptr()->SetPos(particle_pos2);
	//mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
	//mrigidBody.get_ptr()->GetCollisionModel()->AddSphere(rad/scale);
	//mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
	//mrigidBody->SetBodyFixed(false);
	//mrigidBody.get_ptr()->SetCollide(true);
	//mrigidBody.get_ptr()->SetImpactC(0.0);
	//mrigidBody.get_ptr()->SetSfriction(0.05);
	//mrigidBody.get_ptr()->SetKfriction(0.05);
	//mphysicalSystem->AddBody(mrigidBody);

	//ChVector<> particle_pos2(0,-tank_radius+.05,0);
	//mrigidBody = ChSharedBodyPtr(new ChBody);
	//mrigidBody.get_ptr()->SetMass(1);
	//mrigidBody.get_ptr()->SetPos(particle_pos2);
	//mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
	//mrigidBody.get_ptr()->GetCollisionModel()->AddSphere(.05);
	//mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
	//mrigidBody.get_ptr()->SetCollide(true);
	//mrigidBody.get_ptr()->SetImpactC(0.0);
	//mrigidBody.get_ptr()->SetSfriction(0.05);
	//mrigidBody.get_ptr()->SetKfriction(0.05);
	//mphysicalSystem->AddBody(mrigidBody);




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
				//cout<<temp<<endl;
				ss>>j>>tempF.x>>tempF.y>>tempF.z;
				pos.push_back(tempF);
			}
			if(temp[0]=='f'){
				stringstream ss(temp);
				//cout<<temp<<endl;
				ss>>j>>tempF.x>>tempF.y>>tempF.z;
				tri.push_back(tempF);
			}
		}
	}
	for(int i=0; i<tri.size(); i++){
		ChVector<> A(pos[tri[i].x-1].x,pos[tri[i].x-1].y,pos[tri[i].x-1].z);
		ChVector<> B(pos[tri[i].y-1].x,pos[tri[i].y-1].y,pos[tri[i].y-1].z);
		ChVector<> C(pos[tri[i].z-1].x,pos[tri[i].z-1].y,pos[tri[i].z-1].z);
		A/=10.0;
		B/=10.0;
		C/=10.0;

		//cout<<tri[i].x<<" "<<tri[i].y<<" "<<tri[i].z<<endl;//<<" "<<B.x<<" "<<B.y<<" "<<B.z<<" "<<C.x<<" "<<C.y<<" "<<C.z<<endl;
		mrigidBody = ChSharedBodyGPUPtr(new ChBodyGPU);
		mrigidBody.get_ptr()->SetMass(100000);
		ChVector<> particle_pos2(0,-tank_radius,0);
		mrigidBody.get_ptr()->SetPos(particle_pos2);
		mrigidBody.get_ptr()->GetCollisionModel()->ClearModel();
		((ChCollisionModelGPU*)mrigidBody.get_ptr()->GetCollisionModel())->AddTriangle(A,B,C);
		mrigidBody.get_ptr()->GetCollisionModel()->BuildModel();
		mrigidBody.get_ptr()->SetBodyFixed(true);
		mrigidBody.get_ptr()->SetCollide(true);
		mrigidBody.get_ptr()->SetImpactC(0.0);
		mrigidBody.get_ptr()->SetSfriction(0.05);
		mrigidBody.get_ptr()->SetKfriction(0.05);
		mphysicalSystemG->AddBody(mrigidBody);

	}
	cout<<"DONE Loading Triangles"<<endl;
}

float x=.5, y=.5, z=.5;
int filecount=0;
void renderScene(){
	if(numP<num_particles&&frame_number%100==0){
		create_falling_items(mphysicalSystemG, particle_radius, particle_friction, NSPSide*NSPSide*NSPSide,tank_radius);
		numP+=10*10*10;
	}
	mphysicalSystemG->DoStepDynamics( time_step );

	if(ogl){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		glFrontFace(GL_CCW);
		glCullFace(GL_BACK); 
		glEnable(GL_CULL_FACE);
		glDepthFunc(GL_LEQUAL);
		glClearDepth(1.0);

		glPointSize(2);
		glLoadIdentity();
		gluLookAt(cos(x)*tank_radius*5, 0, sin(z)*tank_radius*5, 
			0,0,0,
			0.0f,1.0f,0.0f);
		glColor3f (1,1,1);
		glutWireCube(tank_radius*2);
	}
	std::vector<ChBody*>::iterator abody = mphysicalSystemG->Get_bodylist()->begin();
	//int a=0;

	ofstream ofile;
	stringstream ss; ss<<"dataflow\\frame"<<filecount<<".txt";
	if(frame_number%every_x_file==0&&save){

		ofile.open(ss.str().c_str());
		filecount++;
	}
	if(ofile.fail()){cout<<ss.str()<<endl;exit(1);}


	while (abody != mphysicalSystemG->Get_bodylist()->end()){
		//ChCollisionModelGPU* bpointer = (chrono::collision::ChCollisionModelGPU*)(*abody);
		ChCollisionModelGPU* cModel = (ChCollisionModelGPU*)((*abody)->GetCollisionModel());
		int type=cModel->GetType();
		if(frame_number%every_x_file==0&&save){
			ChVector<> temp=	(*abody)->GetPos();
			ChQuaternion<> tempR=	(*abody)->GetRot();
			tempR.Normalize();
			ofile<<temp.x<<","<<temp.y<<","<<temp.z<<","<<endl;
		}
		if(ogl){
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


					/*glPushMatrix();
					glTranslatef (gPos.x, gPos.y, gPos.z);
					glutSolidSphere(.01,10,10);
					glPopMatrix();
					gPos = (*abody)->GetCoord().TrasformLocalToParent(BB);
					glPushMatrix();
					glTranslatef (gPos.x, gPos.y, gPos.z);
					glutSolidSphere(.01,10,10);
					glPopMatrix();
					gPos = (*abody)->GetCoord().TrasformLocalToParent(CC);
					glPushMatrix();
					glTranslatef (gPos.x, gPos.y, gPos.z);
					glutSolidSphere(.01,10,10);
					glPopMatrix();*/
				}

			}
		}
		abody++;
	}
	if(ogl){
		glutSwapBuffers();
		x+=.01;
		y+=.01;
		z+=.01;
	}
	cout<<"T:   "<<mphysicalSystemG->GetChTime()
		<<"\tC: "<<mphysicalSystemG->GetTimerStep()
		<<"\tG: "<<mphysicalSystemG->GetTimerCollisionBroad()
		<<"\tG: "<<mphysicalSystemG->GetTimerLcp()
		<<"\tB: "<<mphysicalSystemG->GetNbodies()
		<<"\tC: "<<((ChLcpSystemDescriptorGPU*)(mphysicalSystemG->GetLcpSystemDescriptor()))->nContactsGPU<<"\t";
	/*timer_file<<""<<mphysicalSystem->GetChTime()
	<<"\t"<<mphysicalSystem->GetTimerStep()
	<<"\t"<<mphysicalSystem->GetTimerCollisionBroad()
	<<"\t"<<mphysicalSystem->GetTimerLcp()
	<<"\t"<<mphysicalSystem->GetNbodies()
	<<"\t"<<((ChLcpSystemDescriptorGPU*)(mphysicalSystem->GetLcpSystemDescriptor()))->n_Contacts_GPU<<endl;*/
	current_time+=time_step;
	frame_number++;
	if(mphysicalSystemG->GetChTime()>=.005){exit(0);}
	//mgroundBody->SetPos(ChVector<>(sin(frame_number/500.0),0.0,0.0));
	//if(frame_number%every_x_file==0&&save){
	//	ofile.close();
	//}
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
		0.0,0.0,-tank_radius,
		0.0f,1.0f,0.0f);
}
int main(int argc, char* argv[]){ 
	NSPSide=40;
	unsigned int devNum=0;
	//stringstream sA;
	//sA<<argv[1]<<" "<<argv[2]<<" "<<argv[3];
	//sA>>NSPSide>>devNum>>ogl;
	//num_particles=NSPSide*NSPSide*NSPSide;
	//devNum=0;
	ogl=0;
	tank_radius=(NSPSide/10.0);
	if(NSPSide==0){tank_radius=(10*particle_radius)*2;}
	//stringstream ss;
	//ss<<"dataW//"<<num_particles<<".txt";
	//timer_file.open(ss.str().c_str());
	//timer_file<<"WTime: "<<"\tCPU : "<<"\tGPU Coll : "<<"\tGPU LCP : "<<"\tBodies: "<<"\tContacts: "<<endl;
	cudaSetDevice(devNum);
	DLL_CreateGlobals();
	ChLcpSystemDescriptorGPU		newdescriptor(num_particles+100,num_particles*5, 1 );
	ChContactContainerGPUsimple		mGPUcontactcontainer;
	ChCollisionSystemGPU			mGPUCollisionEngine(envelope,bin_size, num_particles*5);

	ChSystem mphysicalSystem(num_particles+100, 50); 

	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUcontactcontainer, 500, false, 0, 0.2, num_particles*5, num_particles+100, 1);
	mGPUsolverSpeed.SetDt(time_step);
	mphysicalSystem.ChangeLcpSystemDescriptor(&newdescriptor);

	mphysicalSystem.ChangeContactContainer(&mGPUcontactcontainer);
	mphysicalSystem.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	mphysicalSystem.ChangeCollisionSystem(&mGPUCollisionEngine);

	mGPUCollisionEngine.SetSystemDescriptor(&newdescriptor);
	mGPUsolverSpeed.SetSystemDescriptor(&newdescriptor);

	mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU);
	//mphysicalSystem.SetIterLCPmaxItersSpeed(10);
	//mphysicalSystem.SetMaxPenetrationRecoverySpeed(.1);
	mphysicalSystem.SetIterLCPwarmStarting(false);
	mphysicalSystem.Set_G_acc(ChVector<>(0,-9.834,0));
	mphysicalSystemG=&mphysicalSystem;

	ChVector<> cgpos(0,0,0);
	mgroundBody= ChSharedBodyGPUPtr(new ChBodyGPU);
	mgroundBody->SetMass(100000);
	mgroundBody->SetPos(ChVector<>(0.0,0.0,0.0));
	((ChCollisionModelGPU*)mgroundBody.get_ptr()->GetCollisionModel())->ClearModel();
	((ChCollisionModelGPU*)mgroundBody.get_ptr()->GetCollisionModel())->AddBox(tank_radius,tank_radius,tank_radius,&cgpos); 
	((ChCollisionModelGPU*)mgroundBody.get_ptr()->GetCollisionModel())->BuildModel();
	mgroundBody->SetBodyFixed(true);
	mgroundBody->SetCollide(true);
	mgroundBody->SetSfriction(0.05);
	mgroundBody->SetKfriction(0.05);
	mgroundBody->SetImpactC(0.0);
	mphysicalSystemG->AddBody(mgroundBody);

	//loadTriangleMesh("plane.txt");
	//vector<float4> spheres;
	//create_falling_items(mphysicalSystemG, particle_radius, particle_friction, 1,tank_radius);

	if(!ogl){
		while(true){
			renderScene();
		}
	}
	if(ogl){
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