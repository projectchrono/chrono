#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <time.h>
#include <cutil_inline.h>
#include <GL/freeglut.h>
#include "omp.h"
#include "unit_GPU/ChSystemGPU.h"
#include "unit_GPU/ChSystemMultiGPU.h"
#include "unit_GPU/ChLcpSolverGPU.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
#include "unit_GPU/ChCCollisionModelGPU.h"
#include "unit_GPU/ChBodyGPU.h"
#include "unit_GPU/ChCuda.h"

#include "physics/ChApidll.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"
#define _CHRONOGPU

#if defined (_CHRONOGPU)
#define CHBODYSHAREDPTR ChSharedBodyGPUPtr
#define CHBODY ChBodyGPU
#define CHCOLLISIONSYS ChCollisionSystemGPU
#define CHLCPDESC ChLcpSystemDescriptorGPU
#define CHCONTACTCONT ChContactContainerGPUsimple
#define CHSOLVER ChLcpSolverGPU
#define CHMODEL ChCollisionModelGPU
#define CHSYS ChSystemGPU
#endif

#ifndef _CHRONOGPU
#define CHBODYSHAREDPTR ChSharedBodyPtr
#define CHBODY ChBody
#define CHCOLLISIONSYS ChCollisionSystemBullet
#define CHLCPDESC ChLcpSystemDescriptor
#define CHCONTACTCONT ChContactContainer
#define CHSOLVER ChLcpIterativeJacobi
#define CHMODEL ChModelBullet
#define CHSYS ChSystem
#endif

using namespace chrono;
#if defined( _WINDOWS )
#include <windows.h>
#define ISNAN(x) _isnan(x)
#else
#include <unistd.h>
#define ISNAN(x) isnan(x)
#endif

using namespace std;
#define TORAD(x) x*PI/180.0

struct Point {
	float x, y, z;
	float r, g, b;
};
std::vector<Point> points;

float m_MaxPitchRate = 5;
float m_MaxHeadingRate = 5;
float3 camera_pos = make_float3(0, 0, -20);
float3 look_at = make_float3(0, 0, 0);
float3 camera_up = make_float3(0, 1, 0);
float camera_heading = 0, camera_pitch = 0;
float3 dir = make_float3(0, 0, 1);
float2 mouse_pos = make_float2(0, 0);
float3 camera_pos_delta = make_float3(0, 0, 0);
float SCALE = 1;

bool DataCopied = false;
int OutputFPS = 60;
bool showSphere = false;
bool showSolid = false;
bool updateDraw = true;
bool saveData = false;
bool showContacts = false;
int detail_level = 1;
int drawType = 1;
bool stepMode = false;
bool stepNext = false;
float averageVal = 1, newaverage = 0;

float4 CreateFromAxisAngle(float3 axis, float degrees) {
	float angle = float((degrees / 180.0f) * PI);
	float result = (float) sinf(angle / 2.0f);
	float4 camera_quat;
	camera_quat.x = (float) cosf(angle / 2.0f);
	camera_quat.y = float(axis.x * result);
	camera_quat.z = float(axis.y * result);
	camera_quat.w = float(axis.z * result);
	return camera_quat;
}

void ChangePitch(GLfloat degrees) {
	if (fabs(degrees) < fabs(m_MaxPitchRate)) {
		camera_pitch += degrees;
	} else {
		if (degrees < 0) {
			camera_pitch -= m_MaxPitchRate;
		} else {
			camera_pitch += m_MaxPitchRate;
		}
	}
	if (camera_pitch > 360.0f) {
		camera_pitch -= 360.0f;
	} else if (camera_pitch < -360.0f) {
		camera_pitch += 360.0f;
	}
}

void ChangeHeading(GLfloat degrees) {
	if (fabs(degrees) < fabs(m_MaxHeadingRate)) {
		if (camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)) {
			camera_heading -= degrees;
		} else {
			camera_heading += degrees;
		}
	} else {
		if (degrees < 0) {
			if ((camera_pitch > 90 && camera_pitch < 270) || (camera_pitch < -90 && camera_pitch > -270)) {
				camera_heading += m_MaxHeadingRate;
			} else {
				camera_heading -= m_MaxHeadingRate;
			}
		} else {
			if (camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)) {
				camera_heading -= m_MaxHeadingRate;
			} else {
				camera_heading += m_MaxHeadingRate;
			}
		}
	}
	if (camera_heading > 360.0f) {
		camera_heading -= 360.0f;
	} else if (camera_heading < -360.0f) {
		camera_heading += 360.0f;
	}
}

void processNormalKeys(unsigned char key, int x, int y) {
	switch (key) {
	case 'w':
		camera_pos_delta += dir * SCALE;
		break;
	case 's':
		camera_pos_delta -= dir * SCALE;
		break;
	case 'd':
		camera_pos_delta += cross(dir, camera_up) * SCALE;
		break;
	case 'a':
		camera_pos_delta -= cross(dir, camera_up) * SCALE;
		break;
	case 'q':
		camera_pos_delta += camera_up * SCALE;
		break;
	case 'e':
		camera_pos_delta -= camera_up * SCALE;
		break;
	case 'u':
		updateDraw = (updateDraw) ? 0 : 1;
		break;
	case 'i':
		showSphere = (showSphere) ? 0 : 1;
		break;
	case 'o':
		showSolid = (showSolid) ? 0 : 1;
		break;
	case '[':
		detail_level = max(1, detail_level - 1);
		break;
	case ']':
		detail_level++;
		break;
	case 'c':
		showContacts = (showContacts) ? 0 : 1;
	case '1':
		drawType = 1;
		break;
	case '2':
		drawType = 2;
		break;
	case '3':
		drawType = 3;
		break;
	case 'x':
		stepMode = (stepMode) ? 0 : 1;
		break;
	case 'z':
		stepNext = 1;
		break;
	case 'b':
		saveData = 1;
		break;
	}
}

void pressKey(int key, int x, int y) {
}
void releaseKey(int key, int x, int y) {
}

void mouseMove(int x, int y) {
	float2 mouse_delta = mouse_pos - make_float2(x, y);
	ChangeHeading(.2 * mouse_delta.x);
	ChangePitch(.2 * mouse_delta.y);
	mouse_pos = make_float2(x, y);
}

void mouseButton(int button, int state, int x, int y) {
	mouse_pos = make_float2(x, y);
}

void changeSize(int w, int h) {
	if (h == 0) {
		h = 1;
	}
	float ratio = 1.0 * w / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45, ratio, .001, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -7, 0.0f, 1.0f, 0.0f);
}
void initScene() {
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_POINT_SMOOTH);
	//glEnable( GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_POINT_SMOOTH_HINT, GL_DONT_CARE);
	//   glEnable(GL_DEPTH_TEST);
	//   glDepthFunc(GL_LESS);
	//glFrontFace(GL_CCW);
	//glCullFace(GL_BACK);
	//glEnable(GL_CULL_FACE);
	//glDepthFunc( GL_LEQUAL);
	//glClearDepth(1.0);
	glPointSize(2);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}

float3 GetColour(double v, double vmin, double vmax) {
	float3 c = { 1.0, 1.0, 1.0 }; // white
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
	return (c);
}
void makeSphere(float3 pos, float rad, float angle, float3 axis, float3 scale) {

	glRotatef(angle * 180.0 / PI, axis.x, axis.y, axis.z);
	glScalef(scale.x, scale.y, scale.z);
	if (!showSolid) {
		glutWireSphere(rad, 10, 10);
	} else {
		glutSolidSphere(rad, 10, 10);
	}
}
void makeBox(float3 pos, float rad, float angle, float3 axis, float3 scale) {
	glRotatef(angle * 180.0 / PI, axis.x, axis.y, axis.z);
	glScalef(scale.x * 2, scale.y * 2, scale.z * 2);
	if (!showSolid) {
		glutWireCube(1);
	} else {
		glutSolidCube(1);
	}
}
void makeCyl(float3 pos, float rad, float angle, float3 axis, float3 scale) {
	GLUquadric *quad = gluNewQuadric();

	if (!showSolid) {
		gluQuadricDrawStyle(quad, GLU_LINE);
	} else {
		gluQuadricDrawStyle(quad, GLU_FILL);
	}

	glRotatef(angle * 180.0 / PI, axis.x, axis.y, axis.z);
	glRotatef(-90, 1, 0, 0);
	//glScalef(scale.x*2,scale.y*2,scale.z*2);
	gluCylinder(quad, scale.x, scale.z, scale.y * 2, 10, 10);
}

void drawObject(ChBODY *abody) {
	float3 color;
	float value;
	if (drawType == 1) {
		value = abody->GetPos_dt().Length();
	}
	if (drawType == 2) {
		value = abody->GetPos_dtdt().Length();
	}
	if (drawType == 3) {
		value = 0; //abody->GetAppliedForce().Length() * 1e5;
	}
	color = GetColour(value, 0, averageVal);
	newaverage += value;
	glColor4f(color.x, color.y, color.z, 1);
	double angle;
	ChVector<> axis;

	ChVector<> pos = (abody)->GetPos();
	ChQuaternion<> quat = abody->GetRot();
	quat.Normalize();
	quat.Q_to_AngAxis(angle, axis);

	ShapeType type = ((ChMODEL *) (abody->GetCollisionModel()))->GetShapeType();

#ifndef _CHRONOGPU
	btVector3 half;
	ChMODEL* model = ((ChMODEL *) (abody->GetCollisionModel()));
	glTranslatef(pos.x, pos.y, pos.z);
	switch (type) {
		case SPHERE:

		if (showSphere) {
			float Rad = ((btSphereShape*) model->GetBulletModel()->getCollisionShape())->getRadius();
			makeSphere(F3(pos.x, pos.y, pos.z), Rad, angle, F3(axis.x, axis.y, axis.z), F3(1, 1, 1));
		} else {
			Point pt;
			pt.x = pos.x;
			pt.y = pos.y;
			pt.z = pos.z;
			pt.r = color.x;
			pt.g = color.y;
			pt.b = color.z;
			points.push_back(pt);
		}

		break;
		//case ELLIPSOID:

		//	break;
		case BOX:

		half = ((btBoxShape*) model->GetBulletModel()->getCollisionShape())->getHalfExtentsWithMargin();

		makeBox(F3(pos.x, pos.y, pos.z), 1, angle, F3(axis.x, axis.y, axis.z), F3(float(half.x()), float(half.y()), float(half.z())));
		break;
		case CYLINDER:
		half = ((btCylinderShape*) model->GetBulletModel()->getCollisionShape())->getHalfExtentsWithMargin();
		makeCyl(F3(pos.x, pos.y, pos.z), 1, angle, F3(axis.x, axis.y, axis.z), F3(float(half.x()), float(half.y()), float(half.z())));
		break;

		default:

		Point pt;
		pt.x = pos.x;
		pt.y = pos.y;
		pt.z = pos.z;
		pt.r = color.x;
		pt.g = color.y;
		pt.b = color.z;
		points.push_back(pt);
		break;
	}
#else

	for (int i = 0; i < ((ChMODEL *) (abody->GetCollisionModel()))->mData.size(); i++) {
		glPushMatrix();
		ChVector<> pos = (abody)->GetPos();
		ChQuaternion<> quat = abody->GetRot();
		glTranslatef(pos.x, pos.y, pos.z);
		quat.Normalize();
		quat.Q_to_AngAxis(angle, axis);
		glRotatef(angle * 180.0 / PI, axis.x, axis.y, axis.z);

		ShapeType type = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].type;
		float3 A = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].A;
		float3 B = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].B;
		float3 C = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].C;
		float4 R = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].R;
		ChCoordsys<> csys((abody)->GetPos(), (abody)->GetRot());

		//if (type == CYLINDER) {
			//	pos = csys.TrasformLocalToParent(ChVector<>(A.x, A.y - B.y, A.z));
			//glTranslatef(A.x, A.y - B.y, A.z);
		//} else {
			//	pos = csys.TrasformLocalToParent(ChVector<>(A.x, A.y, A.z));
			glTranslatef(A.x, A.y, A.z);
		//}
		quat = ChQuaternion<>(R.x, R.y, R.z, R.w);
		quat.Normalize();
		quat.Q_to_AngAxis(angle, axis);

		switch (type) {
		case SPHERE:

			if (showSphere) {
				makeSphere(F3(pos.x, pos.y, pos.z), B.x, angle, F3(axis.x, axis.y, axis.z), F3(1, 1, 1));
			} else {
				Point pt;
				pt.x = pos.x;
				pt.y = pos.y;
				pt.z = pos.z;
				pt.r = color.x;
				pt.g = color.y;
				pt.b = color.z;
				points.push_back(pt);
			}

			break;
		case ELLIPSOID:
			makeSphere(F3(pos.x, pos.y, pos.z), 1, angle, F3(axis.x, axis.y, axis.z), F3(B.x, B.y, B.z));
			break;
		case BOX:
			makeBox(F3(pos.x, pos.y, pos.z), 1, angle, F3(axis.x, axis.y, axis.z), F3(B.x, B.y, B.z));
			break;
		case CYLINDER:
			//makeBox(F3(pos.x, pos.y, pos.z), 1, angle, F3(axis.x, axis.y, axis.z), F3(B.x, B.y, B.z));
			makeSphere(F3(pos.x, pos.y, pos.z), 1, angle, F3(axis.x, axis.y, axis.z), F3(B.x, B.y, B.z));
			break;
		}
		glPopMatrix();
	}

#endif

	//int numobjects = ((ChMODEL *) (abody->GetCollisionModel()))->mData.size();

	//for (int i = 0; i < numobjects; i++) {

	//)->mData[i].type;
	//ChCoordsys<> csys((abody)->GetPos(), (abody)->GetRot());
	//csys.TrasformLocalToParent(ChVector<> (A.x, A.y, A.z));

	//quat = quat % ChQuaternion<> (R.x, R.y, R.z, R.w);

	glPopMatrix();
	//}
}

void drawTriMesh(ChBODY *abody) {
	//	float3 color = GetColour(abody->GetPos_dt().Length(), 0, 1);
	//	glColor3f(color.x, color.y, color.z);
	//	int numtriangles = ((ChMODEL *) (abody->GetCollisionModel()))->mData.size();
	//	for (int i = 0; i < numtriangles; i++) {
	//		float3 p1 = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].A;
	//		float3 p2 = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].B;
	//		float3 p3 = ((ChMODEL *) (abody->GetCollisionModel()))->mData[i].C;
	//
	//		ChVector<> gA = (abody)->GetCoord().TrasformLocalToParent(ChVector<> (p1.x, p1.y, p1.z));
	//		ChVector<> gB = (abody)->GetCoord().TrasformLocalToParent(ChVector<> (p2.x, p2.y, p2.z));
	//		ChVector<> gC = (abody)->GetCoord().TrasformLocalToParent(ChVector<> (p3.x, p3.y, p3.z));
	//		glColor4f(0, 0, 0, .3);
	//		glBegin( GL_LINE_LOOP);
	//		glVertex3f(gA.x, gA.y, gA.z);
	//		glVertex3f(gB.x, gB.y, gB.z);
	//		glVertex3f(gC.x, gC.y, gC.z);
	//		glEnd();
	//	}
}
