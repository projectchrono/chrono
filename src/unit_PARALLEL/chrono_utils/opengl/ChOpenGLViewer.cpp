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

#include "ChOpenGLViewer.h"

using namespace glm;
using namespace chrono;
using namespace chrono::utils;

ChOpenGLViewer::ChOpenGLViewer(
      ChSystem * system) {
   physics_system = system;

   render_camera.SetMode(FREE);
   render_camera.SetPosition(glm::vec3(0, 0, -10));
   render_camera.SetLookAt(glm::vec3(0, 0, 0));
   render_camera.SetClipping(.1, 1000);
   render_camera.SetFOV(45);

   simulation_frame = 0;
   simulation_time = 0;

}

ChOpenGLViewer::~ChOpenGLViewer() {

}

bool ChOpenGLViewer::Initialize() {
   fontfilename = "FreeSans.ttf";
   if (FT_Init_FreeType(&ft)) {
      fprintf(stderr, "Could not init freetype library\n");
      return 0;
   }

   if (FT_New_Face(ft, fontfilename.c_str(), 0, &face)) {
      fprintf(stderr, "Could not open font %s\n", fontfilename.c_str());
      return 0;
   }
//   if (!font_shader.Initialize("text.vert", "text.frag")) {
//      return 0;
//   }

   ChOpenGLMaterial white(glm::vec3(1), glm::vec3(1), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
   ChOpenGLMaterial red(glm::vec3(1, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
   if (!main_shader.Initialize("phong.vert", "phong.frag")) {
      return 0;
   }
   sphere.Initialize(5, 5, white);
   sphere.AttachShader(&main_shader);

   frustum.Initialize(vec2(1, 1), vec2(1, 1), 1, red);
   frustum.AttachShader(&main_shader);

}
void ChOpenGLViewer::Update() {
   render_camera.aspect = window_aspect;
   render_camera.window_width = window_size.x;
   render_camera.window_height = window_size.y;
   render_camera.Update();
   model, view, projection, modelview;
   render_camera.GetMatricies(projection, view, model);

}
void ChOpenGLViewer::Render() {
   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

   frustum.Draw(projection, view * model);
   // DisplayHUD()

}

//void ChOpenGLViewer::DrawString(
//      string line,
//      glm::vec3 pos) {
//
//   glPointSize(GLfloat(1));
//   glLineWidth(GLfloat(1));
//   glm::mat4 mv(1.0);
//   glGetFloatv(GL_PROJECTION_MATRIX, glm::value_ptr(mv));
//   glm::mat4 m(1.0f);
//   //translate and scale text so that it is infront of everything
//   m = glm::translate(m, glm::vec3(pos.x, pos.y, -5.5));
//   m = glm::scale(m, glm::vec3(.1, .1, .1));
//   glMultMatrixf(glm::value_ptr(m));
//   for (unsigned int i = 0; i < line.size(); i++) {
//      glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, line[i]);
//   }
//   glLoadMatrixf(glm::value_ptr(mv));
//
//}

//void ChOpenGLViewer::DisplayHUD() {
//
//   glDisable(GL_LIGHTING);
//   glDisable( GL_POINT_SMOOTH);
//   glColor3f(1.0, 1.0, 1.0);
//   glm::mat4 mv(1.0);
//   glGetFloatv(GL_PROJECTION_MATRIX, glm::value_ptr(mv));
//   glViewport(0, 0, window_size.x, window_size.y);
//   glm::mat4 projection = glm::ortho(0.f, float(window_size.x), 0.f, float(window_size.y), -10.0f, 10.f);
//   glm::mat4 view = glm::mat4(1.0f);
//   glm::mat4 model = glm::mat4(1.0f);
//   glm::mat4 MVP = projection * view * model;
//   glLoadMatrixf(glm::value_ptr(MVP));
//   DrawString("Hello", glm::vec3(0, 100, -5.5));
//
////   main_world.render_camera.GetMatricies(projection, view, model);
////   glm::mat4 mvp = projection * view * model;     //Compute the mvp matrix
////
////   glLoadMatrixf(glm::value_ptr(mvp));
//
//   glLoadMatrixf(glm::value_ptr(mv));
//
//   glEnable(GL_LIGHTING);
//   glEnable( GL_POINT_SMOOTH);
//   //glTranslated(0, -150, 0);
//}
