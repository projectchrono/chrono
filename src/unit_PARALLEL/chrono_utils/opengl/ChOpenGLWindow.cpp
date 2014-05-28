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

#include "ChOpenGLWindow.h"

using namespace glm;
using namespace chrono;
using namespace chrono::utils;

GLFWwindow* ChOpenGLWindow::Initialize(
      ivec2 size,
      char * title,
      ChOpenGLViewer* viewer) {

   GLFWwindow* window;

   if (!glfwInit()) {
      cout << "could not initialize glfw- exiting" << endl;
      exit(EXIT_FAILURE);
   }
   glfwWindowHint(GLFW_SAMPLES, 4);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
   glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

   window = glfwCreateWindow(size.x, size.y, "window", NULL, NULL);
   if (!window) {
      cout << "could not create window - exiting" << endl;
      glfwTerminate();
      exit(EXIT_FAILURE);
   }
   glfwMakeContextCurrent(window);

   GLUGetError("Initialize GLFW");

   glewExperimental = GL_TRUE;
   GLenum err = glewInit();
   if (err != GLEW_OK) {
      fprintf(stdout, "Failed to initialize GLEW\n");
      fprintf(stdout, "%s\n", glewGetErrorString(err));
      glfwTerminate();
      return 0;
   }

   glfwSetErrorCallback (CallbackError);
   glfwSetFramebufferSizeCallback(window, CallbackReshape);
   glfwSetKeyCallback(window, CallbackKeyboard);
   glfwSetMouseButtonCallback(window, CallbackMouseButton);
   glfwSetCursorPosCallback(window,CallbackMousePos );

   GLUGetError("Initialize GLEW");
   GLFWGetVersion(window);
   viewer->Initialize();
   if (size.y > 0) {
      viewer->window_size = size;
      viewer->window_aspect = float(size.x) / float(size.y);
   }

   glfwSetWindowUserPointer(window, viewer);
   GLUGetError("Initialize Viewer ");
   return window;
}

void ChOpenGLWindow::SetPointer(
      GLFWwindow* window,
      void* pointer) {

   glfwSetWindowUserPointer(window, pointer);
}

void ChOpenGLWindow::StartDrawLoop(
      GLFWwindow* window) {
   GLUGetError("STARTLOOP");
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));

   while (!glfwWindowShouldClose(window)) {
      //glEnable(GL_POINT_SMOOTH);
      //glEnable(GL_BLEND);
      //glEnable(GL_ALPHA_TEST);
      //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      //glEnable(GL_CULL_FACE);
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      //glViewport(0, 0, pointer->window_size.x, pointer->window_size.y);

      GLUGetError("SingletonRender");

      pointer->Update();
      GLUGetError("AfterUpdate");
      pointer->Render();
      glfwSwapBuffers(window);
      glfwPollEvents();
   }
}

void ChOpenGLWindow::CallbackError(
      int error,
      const char* description) {
   fputs(description, stderr);
}

void ChOpenGLWindow::CallbackReshape(
      GLFWwindow* window,
      int w,
      int h) {

   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));

   if (h > 0) {
      pointer->window_size = ivec2(w, h);
      pointer->window_aspect = float(w) / float(h);
   }

   pointer->SetWindowSize(pointer->window_size);
}

void ChOpenGLWindow::CallbackKeyboard(
      GLFWwindow* window,
      int key,
      int scancode,
      int action,
      int mode) {

   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));

   if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS){
      glfwSetWindowShouldClose(window, GL_TRUE);
   }
   if(action==GLFW_PRESS||action==GLFW_REPEAT){
      pointer->HandleInput(key, 0, 0);
   }
}


void ChOpenGLWindow::CallbackMouseButton(GLFWwindow* window, int button, int state, int mods){
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));
   double x, y;
   glfwGetCursorPos (window,&x,&y );

   pointer->render_camera.SetPos(button, state, x, y);


}
void ChOpenGLWindow::CallbackMousePos(GLFWwindow* window, double x, double y){
   ChOpenGLViewer* pointer = ((ChOpenGLViewer *) (glfwGetWindowUserPointer(window)));

   pointer->render_camera.Move2D(x, y);

}

void ChOpenGLWindow::StartSpinning() {

   //window_manager->SetIdleToCurrentWindow();
   //window_manager->EnableIdleFunction();
}


