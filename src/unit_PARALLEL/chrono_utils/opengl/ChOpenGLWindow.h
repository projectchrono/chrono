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

#include "chrono_utils/opengl/core/ChApiOpenGL.h"
#include "chrono_utils/opengl/ChOpenGLViewer.h"
using namespace std;

namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLWindow {
 public:
   static ChOpenGLWindow& getInstance() {
      static ChOpenGLWindow instance;
      return instance;
   }

   void Initialize(
         int size_x,
         int size_y,
         char * title,
         ChSystem * msystem);
   void StartDrawLoop();
   void DoStepDynamics(
         double time_step);
   void Render();
   bool Active() {
      return !glfwWindowShouldClose(window);
   }
   void SetCamera(
         ChVector<> position,
         ChVector<> look_at,
         ChVector<> up) {
      viewer->render_camera.camera_position = glm::vec3(position.x, position.y, position.z);
      viewer->render_camera.camera_look_at = glm::vec3(look_at.x, look_at.y, look_at.z);
      viewer->render_camera.camera_up = glm::vec3(up.x, up.y, up.z);
   }

   static bool GLUGetError(
         string err = "") {
      bool return_error = false;
      GLenum glerror;
      //Go through list of errors until no errors remain
      while ((glerror = glGetError()) != GL_NO_ERROR) {
         return_error = true;
         std::cerr << err << " - " << gluErrorString(glerror) << std::endl;
      }
      return return_error;
   }

   static void GLFWGetVersion(
         GLFWwindow* main_window) {
      int major, minor, rev;
      major = glfwGetWindowAttrib(main_window, GLFW_CONTEXT_VERSION_MAJOR);
      minor = glfwGetWindowAttrib(main_window, GLFW_CONTEXT_VERSION_MINOR);
      rev = glfwGetWindowAttrib(main_window, GLFW_CONTEXT_REVISION);
      fprintf(stdout, "Version: %d.%d.%d\n", major, minor, rev);

      const GLubyte* vendor = glGetString(GL_VENDOR);
      const GLubyte* renderer = glGetString(GL_RENDERER);
      const GLubyte* version = glGetString(GL_VERSION);
      const GLubyte* glsl_ver = glGetString(GL_SHADING_LANGUAGE_VERSION);

      printf("%s : %s (%s)\n >> GLSL: %s\n", vendor, renderer, version, glsl_ver);

   }

 private:
   ChOpenGLWindow() {
   }
   ~ChOpenGLWindow() {
   }
   ChOpenGLWindow(
         ChOpenGLWindow const&);   // Don't Implement.
   void operator=(
         ChOpenGLWindow const&);   // Don't implement
   static void CallbackError(
         int error,
         const char* description);
   static void CallbackReshape(
         GLFWwindow* window,
         int w,
         int h);
   static void CallbackKeyboard(
         GLFWwindow* window,
         int key,
         int scancode,
         int action,
         int mode);
   static void CallbackMouseButton(
         GLFWwindow* window,
         int button,
         int action,
         int mods);
   static void CallbackMousePos(
         GLFWwindow* window,
         double x,
         double y);

   ChOpenGLViewer *viewer;
   GLFWwindow* window;
};
}
}
#endif   // END of CHOPENGLWINDOW_H
