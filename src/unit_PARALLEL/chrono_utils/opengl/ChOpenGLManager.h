// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Implementation of GL manager class based on GlutMaster code by George Stetten 
// and Korin Crawford.
// Authors: Hammad Mazhar
// =============================================================================

#ifndef  CHOPENGLMANAGER_H
#define  CHOPENGLMANAGER_H

#ifdef __APPLE__

#else
#include <GL/freeglut.h>
#endif

#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <time.h>
#include <math.h>
#include "chrono_utils/opengl/ChApiOpenGL.h"
#include "core/ChClassRegister.h"
#include "physics/ChSystem.h"
#include "chrono_utils/opengl/ChOpenGLCamera.h"
#include "chrono_utils/opengl/ChOpenGLWindow.h"


using namespace std;
namespace chrono{
	namespace utils{
#define MAX_NUMBER_OF_WINDOWS 256
		class CH_UTILS_OPENGL_API ChOpenGLManager {
		private:

			static void CallBackDisplayFunc(void);
			static void CallBackIdleFunc(void);
			static void CallBackKeyboardFunc(unsigned char key, int x, int y);
			static void CallBackMotionFunc(int x, int y);
			static void CallBackMouseFunc(int button, int state, int x, int y);
			static void CallBackPassiveMotionFunc(int x, int y);
			static void CallBackReshapeFunc(int w, int h);
			static void CallBackSpecialFunc(int key, int x, int y);
			static void CallBackVisibilityFunc(int visible);

			static int currentIdleWindow;
			static int idleFunctionEnabled;

		public:
			ChOpenGLManager();
			~ ChOpenGLManager();

			void CallGlutCreateWindow(char * setTitle, ChOpenGLWindow * glutWindow);
			void CallGlutMainLoop(void);
			void DisableIdleFunction(void);
			void EnableIdleFunction(void);
			int IdleFunctionEnabled(void);
			int IdleSetToCurrentWindow(void);
			void SetIdleToCurrentWindow(void);
		};
	}
}
#endif  // END of  CHOPENGLMANAGER_H
