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
// Authors: Hammad Mazhar
// =============================================================================
// Base Class for all opengl related classes
// =============================================================================

#ifndef CHOPENGLBASE_H
#define CHOPENGLBASE_H

#include <GL/glew.h>

#ifdef __APPLE__
#define GLFW_INCLUDE_GLCOREARB
#define GL_DO_NOT_WARN_IF_MULTI_GL_VERSION_HEADERS_INCLUDED  // fixes warnings
#endif

#define GLM_FORCE_RADIANS
#define _CRT_SECURE_NO_WARNINGS

#include <cassert>
#include <iostream>
#include <string>
//#include <string>
//#include <iomanip>
//#include <fstream>
//#include <sstream>
//#include <limits>
//#include <time.h>
//#include <math.h>
//#include <vector>

#include "chrono_opengl/core/ChApiOpenGL.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

#ifndef BAD_GL_VALUE
#define BAD_GL_VALUE GLuint(-1)
#endif

/// Convert error enum to error string.
static std::string GetErrorString(GLenum error) {
    std::string ret_val;
    switch (error) {
        case GL_NO_ERROR:
            break;
        case GL_INVALID_ENUM:
            ret_val = "GL_INVALID_ENUM";
            break;
        case GL_INVALID_VALUE:
            ret_val = "GL_INVALID_VALUE";
            break;
        case GL_INVALID_OPERATION:
            ret_val = "GL_INVALID_OPERATION";
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            ret_val = "GL_INVALID_FRAMEBUFFER_OPERATION";
            break;
        case GL_OUT_OF_MEMORY:
            ret_val = "GL_OUT_OF_MEMORY";
            break;
        case GL_STACK_UNDERFLOW:
            ret_val = "GL_STACK_UNDERFLOW";
            break;
        case GL_STACK_OVERFLOW:
            ret_val = "GL_STACK_OVERFLOW";
            break;
    }
    return ret_val;
}

/// Checks if there are any errors in the opengl context.
static bool GLReturnedError(std::string err) {
    bool return_error = false;
    GLenum glerror;
    while ((glerror = glGetError()) != GL_NO_ERROR) {
        return_error = true;
        std::cerr << err << ": " << GetErrorString(glerror) << std::endl;
    }
    return return_error;
}

/// Base class for all OpenGL related classes.
class CH_OPENGL_API ChOpenGLBase {
  public:
    ChOpenGLBase() {}
    virtual ~ChOpenGLBase() {}

    // Children must implement this function
    virtual void TakeDown() = 0;

    // Check for opengl Errors and output if error along with input char strings
    bool GLReturnedError(const char* s) {
        bool return_error = false;
        GLenum glerror;
        while ((glerror = glGetError()) != GL_NO_ERROR) {
            return_error = true;
            std::cerr << s << ": " << GetErrorString(glerror) << std::endl;
        }
        return return_error;
    }
};

/// @} opengl_module

}
}

#endif
