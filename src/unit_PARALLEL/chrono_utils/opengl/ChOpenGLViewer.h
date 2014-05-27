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
// Implementation of OpenGL class 
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLVIEWER_H
#define CHOPENGLVIEWER_H

#include "utils/opengl/core/ChApiOpenGL.h"
#include "utils/opengl/ChOpenGLManager.h"
#include "utils/opengl/ChOpenGLWorld.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeSolver.h"
#include "assets/ChBoxShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChConeShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "ChSystemParallel.h"
using namespace std;

namespace chrono {
	namespace utils{
		class CH_UTILS_OPENGL_API ChOpenGLViewer: public ChOpenGLWindow {
		public:

			ChOpenGLViewer(ChOpenGLManager * window_manager, ChSystem * system, glm::ivec2 size, glm::ivec2 position, char * title);
			~ChOpenGLViewer();
			void CallBackDisplayFunc(void);
    		void CallBackReshapeFunc(int w, int h);
    		void CallBackIdleFunc(void);
    		void CallBackKeyboardFunc(unsigned char key, int x, int y);
    		void CallBackMouseFunc(int button, int state, int x, int y);
    		void CallBackMotionFunc(int x, int y);
    		void StartSpinning(ChOpenGLManager * window_manager);
    		
			ChOpenGLWorld main_world;

			glm::ivec2 window_size;
			glm::ivec2 window_position;
			float window_aspect;
			int interval;
		};
	}
}

#endif  // END of CHOPENGLVIEWER_H
