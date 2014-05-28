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

//using namespace glm;
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

   sphere.Initialize("sphere.obj", white, &main_shader);
   box.Initialize("box.obj", white, &main_shader);
   cylinder.Initialize("cylinder.obj", white, &main_shader);
   cone.Initialize("cone.obj", white, &main_shader);

}
void ChOpenGLViewer::Update() {
   render_camera.aspect = window_aspect;
   render_camera.window_width = window_size.x;
   render_camera.window_height = window_size.y;
   render_camera.Update();
   model, view, projection, modelview;
   render_camera.GetMatricies(projection, view, model);

   physics_system->DoStepDynamics(physics_system->GetStep());

   cout << physics_system->GetChTime() << endl;
}
void ChOpenGLViewer::Render() {
   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

   // DisplayHUD()

   for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
      ChBody* abody = (ChBody*) physics_system->Get_bodylist()->at(i);
      DrawObject(abody);
   }

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
