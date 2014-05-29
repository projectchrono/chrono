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
#include "FontData.h"
//using namespace std;
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
   pause_sim = 0;
   pause_vis = 0;
   render_mode = WIREFRAME;

}

ChOpenGLViewer::~ChOpenGLViewer() {
}

bool ChOpenGLViewer::Initialize() {

   if (FT_Init_FreeType(&ft)) {
      fprintf(stderr, "Could not init freetype library\n");
      return 0;
   }
   if (FT_New_Face(ft, "monaco.ttf", 0, &face)) {
      fprintf(stderr, "Could not open font %s\n", "monaco.ttf");
      return 0;
   }
   if (!font_shader.Initialize("text.vert", "text.frag")) {
      return 0;
   }

   ChOpenGLMaterial white(glm::vec3(.1, .1, .1), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
   ChOpenGLMaterial red(glm::vec3(.1, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
   if (!main_shader.Initialize("phong.vert", "phong.frag")) {
      return 0;
   }

   sphere.Initialize("sphere.obj", white, &main_shader);
   box.Initialize("box.obj", red, &main_shader);
   cylinder.Initialize("cylinder.obj", white, &main_shader);
   cone.Initialize("cone.obj", white, &main_shader);

   // Initialize vbo and vao for text
   glGenBuffers(1, &vbo);
   glGenVertexArrays(1, &vao);
   glGenTextures(1, &texture);
   glGenSamplers(1, &sampler);
   glSamplerParameteri(sampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
   glSamplerParameteri(sampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
   glSamplerParameteri(sampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glSamplerParameteri(sampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   //get the uniform location for the texture from shader
   texture_handle = font_shader.GetUniformLocation("tex");

}
void ChOpenGLViewer::Update() {
   if (pause_sim == true) {
      return;
   }
   physics_system->DoStepDynamics(physics_system->GetStep());

}
void ChOpenGLViewer::Render() {

   if (pause_vis == false) {

      render_camera.aspect = window_aspect;
      render_camera.window_width = window_size.x;
      render_camera.window_height = window_size.y;
      render_camera.Update();
      model, view, projection, modelview;
      render_camera.GetMatricies(projection, view, model);

      if (render_mode == WIREFRAME) {
         glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      } else {
         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }

      for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
         ChBody* abody = (ChBody*) physics_system->Get_bodylist()->at(i);
         DrawObject(abody);
      }
   }
   DisplayHUD();
}

void ChOpenGLViewer::DrawObject(
      ChBody * abody) {
   if (abody->GetAssets().size() == 0) {
      return;
   }

   const Vector pos = abody->GetPos();
   const Vector vel = abody->GetPos_dt();
   const Vector acc = abody->GetPos_dtdt();

   Quaternion rot = abody->GetRot();
   double angle;
   Vector axis;
   rot.Q_to_AngAxis(angle, axis);

   for (int i = 0; i < abody->GetAssets().size(); i++) {

      ChSharedPtr<ChAsset> asset = abody->GetAssets().at(i);

      ChVisualization* visual_asset = ((ChVisualization *) (asset.get_ptr()));
      Vector center = visual_asset->Pos;
      center = rot.Rotate(center);
      Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
      lrot = rot % lrot;
      lrot.Normalize();
      lrot.Q_to_AngAxis(angle, axis);

      if (asset.IsType<ChSphereShape>()) {
         ChSphereShape * sphere_shape = ((ChSphereShape *) (asset.get_ptr()));
         float radius = sphere_shape->GetSphereGeometry().rad;
         ChVector<> pos_final = pos + center;

         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(radius, radius, radius));
         sphere.Draw(projection, view * model);

      } else if (asset.IsType<ChEllipsoidShape>()) {

         ChEllipsoidShape * ellipsoid_shape = ((ChEllipsoidShape *) (asset.get_ptr()));
         Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
         sphere.Draw(projection, view * model);

      } else if (asset.IsType<ChBoxShape>()) {
         ChBoxShape * box_shape = ((ChBoxShape *) (asset.get_ptr()));
         ChVector<> pos_final = pos + center;
         Vector radius = box_shape->GetBoxGeometry().Size;

         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
         box.Draw(projection, view * model);

      } else if (asset.IsType<ChCylinderShape>()) {
         ChCylinderShape * cylinder_shape = ((ChCylinderShape *) (asset.get_ptr()));
         double rad = cylinder_shape->GetCylinderGeometry().rad;
         double height = cylinder_shape->GetCylinderGeometry().p1.y - cylinder_shape->GetCylinderGeometry().p2.y;
         Quaternion rott = chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X);
         Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
         lrot = lrot % rott;
         lrot = rot % lrot;

         lrot.Q_to_AngAxis(angle, axis);
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(rad, height, rad));
         cylinder.Draw(projection, view * model);

      } else if (asset.IsType<ChConeShape>()) {
         ChConeShape * cone_shape = ((ChConeShape *) (asset.get_ptr()));
         Vector rad = cone_shape->GetConeGeometry().rad;
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(rad.x, rad.y, rad.z));
         cylinder.Draw(projection, view * model);

      } else if (asset.IsType<ChTriangleMeshShape>()) {
         ChTriangleMeshShape * trimesh_shape = ((ChTriangleMeshShape *) (asset.get_ptr()));
         ChTriangleMeshConnected trimesh = trimesh_shape->GetMesh();
         glPushMatrix();
         glBegin(GL_TRIANGLES);
         for (int i = 0; i < trimesh.getNumTriangles(); i++) {
            ChTriangle temptri = trimesh.getTriangle(i);
            real3 A, B, C;
            A = R3(temptri.p1.x, temptri.p1.y, temptri.p1.z);
            B = R3(temptri.p2.x, temptri.p2.y, temptri.p2.z);
            C = R3(temptri.p3.x, temptri.p3.y, temptri.p3.z);
            real4 Q = R4(rot.e0, rot.e1, rot.e2, rot.e3);

            A = quatRotate(A, Q) + R3(pos.x, pos.y, pos.z);
            B = quatRotate(B, Q) + R3(pos.x, pos.y, pos.z);
            C = quatRotate(C, Q) + R3(pos.x, pos.y, pos.z);

            glVertex3f(A.x, A.y, A.z);
            glVertex3f(B.x, B.y, B.z);
            glVertex3f(C.x, C.y, C.z);
         }
         glEnd();
         glPopMatrix();
      }
   }
}

void ChOpenGLViewer::RenderText(
      const std::string &str,
      FT_Face face,
      float x,
      float y,
      float sx,
      float sy) {

   //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

   for (int i = 0; i < str.size(); i++) {

      texture_glyph_t *glyph = 0;
      for (int j = 0; j < font_data.glyphs_count; ++j) {
         if (font_data.glyphs[j].charcode == str[i]) {
            glyph = &font_data.glyphs[j];
            break;
         }
      }
      if (!glyph) {
         continue;
      }
      //float x_ = x + 24 * i;
      //float y_ = y;
      x += glyph->kerning[0].kerning;
      float x0 = (float) (x + glyph->offset_x * sx);
      float y0 = (float) (y + glyph->offset_y * sy);
      float x1 = (float) (x0 + glyph->width * sx);
      float y1 = (float) (y0 - glyph->height * sy);

      float s0 = glyph->s0;
      float t0 = glyph->t0;
      float s1 = glyph->s1;
      float t1 = glyph->t1;

      struct {
         float x, y, s, t;
      } data[6] = { { x0, y0, s0, t0 }, { x0, y1, s0, t1 }, { x1, y1, s1, t1 }, { x0, y0, s0, t0 }, { x1, y1, s1, t1 }, { x1, y0, s1, t0 } };

      glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), data, GL_DYNAMIC_DRAW);
      glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      x += (glyph->advance_x * sx);
   }

   // glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

}

void ChOpenGLViewer::DisplayHUD() {
   GLReturnedError("Start text");
   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, texture);
   glTexImage2D( GL_TEXTURE_2D, 0, GL_R8, font_data.tex_width, font_data.tex_height, 0, GL_RED, GL_UNSIGNED_BYTE, font_data.tex_data);
   glBindSampler(0, sampler);
   glBindVertexArray(vao);
   glEnableVertexAttribArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, vbo);
   font_shader.Use();
   glUniform1i(texture_handle, 0);

   float sx = 1. / window_size.x;
   float sy = 1. / window_size.y;

   FT_Set_Pixel_Sizes(face, 0, 20);
   char buffer[50];

   sprintf(buffer, "Time:  %04f", physics_system->GetChTime());
   RenderText(buffer, face, -1, -0.925, sx, sy);

   sprintf(buffer, "Step  :  %04f", physics_system->GetTimerStep());
   RenderText(buffer, face, -1, 0.925 - .06 * 0, sx, sy);
   sprintf(buffer, "Broad :  %04f", physics_system->GetTimerCollisionBroad());
   RenderText(buffer, face, -1, 0.925 - .06 * 1, sx, sy);
   sprintf(buffer, "Narrow:  %04f", physics_system->GetTimerCollisionNarrow());
   RenderText(buffer, face, -1, 0.925 - .06 * 2, sx, sy);
   sprintf(buffer, "Solver:  %04f", physics_system->GetTimerLcp());
   RenderText(buffer, face, -1, 0.925 - .06 * 3, sx, sy);
   sprintf(buffer, "Update:  %04f", physics_system->GetTimerUpdate());
   RenderText(buffer, face, -1, 0.925 - .06 * 4, sx, sy);

   vector<double> history = ((ChLcpIterativeSolver*) (physics_system->GetLcpSolverSpeed()))->GetViolationHistory();
   vector<double> dlambda = ((ChLcpIterativeSolver*) (physics_system->GetLcpSolverSpeed()))->GetDeltalambdaHistory();

   sprintf(buffer, "Iters   :  %04d", history.size());
   RenderText(buffer, face, .6, 0.925 - .06 * 0, sx, sy);
   sprintf(buffer, "Bodies  :  %04d", physics_system->GetNbodiesTotal());
   RenderText(buffer, face, .6, 0.925 - .06 * 1, sx, sy);
   sprintf(buffer, "Contacts:  %04d", physics_system->GetNcontacts());
   RenderText(buffer, face, .6, 0.925 - .06 * 2, sx, sy);

   sprintf(buffer, "Residual:  %04f", history[history.size() - 1]);
   RenderText(buffer, face, .6, 0.925 - .06 * 4, sx, sy);
   sprintf(buffer, "Correct :  %04f", dlambda[dlambda.size() - 1]);
   RenderText(buffer, face, .6, 0.925 - .06 * 5, sx, sy);


   glBindTexture(GL_TEXTURE_2D, 0);
   glUseProgram(0);
   GLReturnedError("End text");

}
