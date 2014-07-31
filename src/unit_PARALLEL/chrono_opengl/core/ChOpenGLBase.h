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
// Base Class for all opengl related classes
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLBASE_H
#define CHOPENGLBASE_H

#include <GL/glew.h>

#ifdef __APPLE__
#define GLFW_INCLUDE_GLCOREARB
#define GL_DO_NOT_WARN_IF_MULTI_GL_VERSION_HEADERS_INCLUDED //fixes warnings
#endif
#include <glfw3.h>

#define GLM_FORCE_RADIANS
#define _CRT_SECURE_NO_WARNINGS

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/constants.hpp>
#include <glm/gtx/spline.hpp>

#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <time.h>
#include <math.h>
#include <vector>
#include <assert.h>

#include "chrono_opengl/core/ChApiOpenGL.h"



namespace chrono {
	namespace opengl{

#ifndef	BAD_GL_VALUE
#define	BAD_GL_VALUE	GLuint(-1)
#endif

		class CH_OPENGL_API ChOpenGLBase {
		public:
			ChOpenGLBase(){}
			~ChOpenGLBase(){}

		    // Children must implement this function
			virtual void TakeDown() = 0;

        //Check for opengl Errors and output if error along with input char strings
			bool GLReturnedError(const char * s) {
				bool return_error = false;
				GLenum glerror;
            //Go through list of errors until no errors remain
				while ((glerror = glGetError()) != GL_NO_ERROR) {
					return_error = true;
					std::cerr << s << ": " << gluErrorString(glerror) << std::endl;
				}
				return return_error;
			}
		};
	}
}


#endif  // END of CHOPENGLBASE_H
