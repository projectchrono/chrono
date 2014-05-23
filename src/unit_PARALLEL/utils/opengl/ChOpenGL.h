#ifndef CHOPENGL_H
#define CHOPENGL_H
#include <string>
////#include <unistd.h>
#include "utils/opengl/ChApiOpenGL.h"
#include "utils/opengl/ChOpenGLManager.h"
#include "utils/opengl/ChOpenGLCamera.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeSolver.h"
#include "assets/ChBoxShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChConeShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "ChSystemParallel.h"
#include "collision/ChCCollisionSystemParallel.h"
#include "collision/ChCCollisionSystemBulletParallel.h"
#include "collision/ChCCollisionSystemBullet.h"
using namespace std;

namespace chrono {
	namespace utils{
		class CH_UTILS_OPENGL_API ChOpenGL: public ChOpenGLWindow {
		public:
			enum displayMode {
				POINT_CLOUD, BOUNDING_BOX, WIREFRAME, SOLID
			};
			enum textMode {
				NONE, POS_T, VEL_T, ACC_T
			};
			enum colorMode {
				POS_X, POS_Y, POS_Z, VEL, VEL_X, VEL_Y, VEL_Z, ACC, ACC_X, ACC_Y, ACC_Z, RANDOM
			};
			enum simulationMode {
				RUNNING, STOPPED, PAUSED_RENDER, PAUSED_SIM
			};
			enum contactMode {
				NOCONTACT, POINTS, NORMAL, LAGRANGE
			};
			struct textInfo3D {
				string text;
				Vector pos;
			};

			int height, width;
			int initPositionX, initPositionY;

			ChOpenGL(ChOpenGLManager * window_manager, ChSystem * system, int setWidth, int setHeight, int setInitPositionX, int setInitPositionY, char * title);
			~ChOpenGL();

			void CallBackDisplayFunc(void);
			void CallBackReshapeFunc(int w, int h);
			void CallBackIdleFunc(void);
			void CallBackKeyboardFunc(unsigned char key, int x, int y);
			void CallBackMouseFunc(int button, int state, int x, int y);
			void CallBackMotionFunc(int x, int y);
			void StartSpinning(ChOpenGLManager * window_manager);
			void SetDisplayMode(displayMode mode);
			void DrawObject(ChBody * abody);
			void DrawContactsGPU(ChSystemParallel* system_gpu);
			void DrawContactsBulletGPU(ChCollisionSystemBulletParallel* coll_sys);
			void DrawContactsBullet(ChCollisionSystemBullet* coll_sys);
			void DrawContacts();
			void DrawGrids();
			void DrawText();
			void DrawHUD();
			void RenderBitmapString2D(float x, float y, void *font, string str);
			void RenderBitmapString3D(Vector p, void *font, string str);
			void MakeSphere(Vector pos, float rad, float angle, Vector axis, Vector scale);
			void MakeBox(Vector pos, Vector radius, float angle, Vector axis, Vector scale);
			void MakeCyl(Vector pos, double rad, float angle, Vector axis, Vector scale);
			void MakeCone(Vector pos, Vector radius, float angle, Vector axis);
			void SetOrthographicProjection();
			void RestorePerspectiveProjection();
			void RunStep();
			void SetCustomCallback(void (*ptr)(ChSystem* mSys, const int frame)) {
				set_custom_callback = true;
				custom_callback = ptr;
			}
			void AddSystem(ChSystem * system) {
				physics_system.push_back(system);
			}

			string VectorToString(Vector v) {
				stringstream ss;
				ss << v.x << "," << v.y << "," << v.z;
				return ss.str();
			}
		template<class T, class U>
			string ToString(T label, U value) {
				stringstream ss;
				ss << label << value;
				return ss.str();
			}
			ChOpenGLCamera * render_camera;
	//private:
			displayMode display_mode;
			colorMode color_mode;
			simulationMode sim_mode, render_mode;
			vector<ChSystem *> physics_system;

			int sim_frame;
			float sim_dt, sim_time;
			vector<textInfo3D> text_3d;
			textMode text_mode;
			contactMode contact_mode;

			bool set_custom_callback;
			void (*custom_callback)(ChSystem* mSys, const int frame);

			vector<real3> points;

			GLuint sphere_handle;
			GLuint cylinder_handle;
			GLuint cube_handle;
			GLuint cone_handle;

		};
	}
}

real3 GetColour(real v, real vmin, real vmax) {
	real3 c = R3( 1.0, 1.0, 1.0 );     // white
	real dv;

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

#endif  // END of ChOpenGL.h
