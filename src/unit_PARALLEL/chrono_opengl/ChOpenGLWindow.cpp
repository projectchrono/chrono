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
// OpenGL window singleton, this class manages the opengl context and window
// Authors: Hammad Mazhar
// =============================================================================

#include "chrono_opengl/ChOpenGLWindow.h"

//using namespace glm;
using namespace chrono;
using namespace chrono::opengl;

void ChOpenGLWindow::Initialize(
      int size_x,
      int size_y,
      const char * title,
      ChSystem * msystem) {

   if (!glfwInit()) {
      std::cout << "could not initialize glfw- exiting" << std::endl;
      exit(EXIT_FAILURE);
   }
   //glfwWindowHint(GLFW_SAMPLES, 4);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
   glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

   window = glfwCreateWindow(size_x, size_y, title, NULL, NULL);
   if (!window) {
      std::cout << "could not create window - exiting" << std::endl;
      glfwTerminate();
      exit(EXIT_FAILURE);
   }
   glfwMakeContextCurrent(window);

   //Disable vsync!!
   glfwSwapInterval(0);

   GLUGetError("Initialize GLFW");

   glewExperimental = GL_TRUE;
   GLenum err = glewInit();
   if (err != GLEW_OK) {
      fprintf(stdout, "Failed to initialize GLEW\n");
      fprintf(stdout, "%s\n", glewGetErrorString(err));
      glfwTerminate();
      return;
   }

   glfwSetErrorCallback(CallbackError);
   glfwSetWindowCloseCallback(window, CallbackClose);
   glfwSetFramebufferSizeCallback(window, CallbackReshape);
   glfwSetKeyCallback(window, CallbackKeyboard);
   glfwSetMouseButtonCallback(window, CallbackMouseButton);
   glfwSetCursorPosCallback(window, CallbackMousePos);

   GLUGetError("Initialize GLEW");
   GLFWGetVersion(window);

   viewer = new ChOpenGLViewer(msystem);

   glfwGetFramebufferSize(window, &size_x, &size_y);
   if (size_y > 0) {
      viewer->window_size = glm::ivec2(size_x, size_y);
      viewer->window_aspect = float(size_x) / float(size_y);
   }
   viewer->Initialize();

   glfwSetWindowUserPointer(window, viewer);
   GLUGetError("Initialize Viewer ");
   poll_frame = 0;
}

void ChOpenGLWindow::StartDrawLoop(double time_step) {
   GLUGetError("Start Draw Loop");
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   while (Active()) {
      pointer->Update(time_step);
      Render();
   }
}
bool ChOpenGLWindow::DoStepDynamics(
      double time_step) {
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   return pointer->Update(time_step);
}
void ChOpenGLWindow::Render() {

   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   if (pointer->pause_vis == false) {
      glEnable(GL_BLEND);
      glEnable(GL_DEPTH_TEST);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_CULL_FACE);
      glClearColor(84.0f / 255.0f, 36.0f / 255.0f, 55.0f / 255.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      GLUGetError("Before Render");
      pointer->Render();
      GLUGetError("After Render");
      glfwSwapBuffers(window);
   }

   glfwPollEvents();

}

bool ChOpenGLWindow::Active() {
   return !glfwWindowShouldClose(window);
}

bool ChOpenGLWindow::Running() {
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   if (pointer->pause_sim == true) {
      return false;
   }
   return true;
}

//Pause simulation
void ChOpenGLWindow::Pause() {
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   pointer->pause_sim = true;
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
bool ChOpenGLWindow::GLUGetError(
      std::string err) {
   bool return_error = false;
   GLenum glerror;
//Go through list of errors until no errors remain
   while ((glerror = glGetError()) != GL_NO_ERROR) {
      return_error = true;
      std::cerr << err << " - " << gluErrorString(glerror) << std::endl;
   }
   return return_error;
}
#pragma GCC diagnostic pop

void ChOpenGLWindow::GLFWGetVersion(
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

void ChOpenGLWindow::CallbackError(
      int error,
      const char* description) {
   fputs(description, stderr);
}

void ChOpenGLWindow::CallbackClose(
      GLFWwindow* window) {

   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   pointer->TakeDown();
   glfwSetWindowShouldClose(window, GL_TRUE);

}

void ChOpenGLWindow::CallbackReshape(
      GLFWwindow* window,
      int w,
      int h) {

   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   if (h > 0) {
      pointer->window_size = glm::ivec2(w, h);
      pointer->window_aspect = float(w) / float(h);
   }

}

void ChOpenGLWindow::CallbackKeyboard(
      GLFWwindow* window,
      int key,
      int scancode,
      int action,
      int mode) {

   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));

   if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
      glfwSetWindowShouldClose(window, GL_TRUE);
   }

   if (action == GLFW_PRESS || action == GLFW_REPEAT) {
      pointer->HandleInput(key, 0, 0);
   }
}

void ChOpenGLWindow::CallbackMouseButton(
      GLFWwindow* window,
      int button,
      int state,
      int mods) {
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   double x, y;
   glfwGetCursorPos(window, &x, &y);

   pointer->render_camera.SetPos(button, state, x, y);

}
void ChOpenGLWindow::CallbackMousePos(
      GLFWwindow* window,
      double x,
      double y) {
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));

   pointer->render_camera.Move2D(x, y);

}

