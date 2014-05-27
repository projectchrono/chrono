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
// Implementation of GL window class based on GlutMaster code by George Stetten 
// and Korin Crawford.
// Authors: Hammad Mazhar
// =============================================================================
#ifndef CHOPENGLWINDOW_H
#define CHOPENGLWINDOW_H

#include "utils/opengl/core/ChOpenGLBase.h"

namespace chrono{
	namespace utils{

		class CH_UTILS_OPENGL_API ChOpenGLWindow:ChOpenGLBase {
		protected:
			int windowID;
		public:

			ChOpenGLWindow(void);
			~ChOpenGLWindow();

			virtual void CallBackDisplayFunc();
			virtual void CallBackIdleFunc(void);
			virtual void CallBackKeyboardFunc(unsigned char key, int x, int y);
			virtual void CallBackMotionFunc(int x, int y);
			virtual void CallBackMouseFunc(int button, int state, int x, int y);
			virtual void CallBackPassiveMotionFunc(int x, int y);
			virtual void CallBackReshapeFunc(int w, int h);
			virtual void CallBackSpecialFunc(int key, int x, int y);
			virtual void CallBackVisibilityFunc(int visible);
			void TakeDown(){}
			void SetWindowID(int newWindowID);
			int GetWindowID(void);
		};
	}
}
#endif   // END of CHOPENGLWINDOW_H
