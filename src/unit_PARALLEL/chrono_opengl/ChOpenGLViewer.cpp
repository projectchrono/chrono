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
// OpenGL viewer, this class draws the system to the screen and handles input
// Authors: Hammad Mazhar
// =============================================================================
#include <omp.h>
#include "ChOpenGLViewer.h"
#include "FontData.h"

#include "resources/text_frag.h"
#include "resources/text_vert.h"
#include "resources/phong_frag.h"
#include "resources/phong_vert.h"
#include "resources/cloud_frag.h"
#include "resources/cloud_vert.h"

//using namespace std;
using namespace chrono;
using namespace chrono::opengl;

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
   single_step = 0;
   view_contacts = 0;
   view_help = 0;
   use_vsync = 0;
   render_mode = POINTS;
   old_time = current_time = 0;
   time_total = time_text = time_geometry = 0;

}

ChOpenGLViewer::~ChOpenGLViewer() {
}

void ChOpenGLViewer::TakeDown() {
   render_camera.TakeDown();
   main_shader.TakeDown();
   font_shader.TakeDown();
   sphere.TakeDown();
   box.TakeDown();
   cylinder.TakeDown();
   cone.TakeDown();
   cloud.TakeDown();

   for (map<string, ChOpenGLOBJ>::iterator iter = obj_files.begin(); iter != obj_files.end(); iter++) {
      (*iter).second.TakeDown();
   }

}

bool ChOpenGLViewer::Initialize() {

   if (!font_shader.InitializeStrings("text", text_vert, text_frag)) {
      return 0;
   }

   ChOpenGLMaterial white(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
   ChOpenGLMaterial red(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 1, 1));

   float ambient = .5;

   ChOpenGLMaterial slate(glm::vec3(85.0f, 98.0f, 112.0f) / 255.0f * ambient, glm::vec3(85.0f, 98.0f, 112.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial pacifica(glm::vec3(78.0f, 205.0f, 196.0f) / 255.0f * ambient, glm::vec3(78.0f, 205.0f, 196.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial apple(glm::vec3(199.0f, 244.0f, 100.0f) / 255.0f * ambient, glm::vec3(199.0f, 244.0f, 100.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial cherry(glm::vec3(255.0f, 107.0f, 107.0f) / 255.0f * ambient, glm::vec3(255.0f, 107.0f, 107.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial pillow(glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f * ambient, glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f, glm::vec3(1, 1, 1));

   ChOpenGLMaterial elated(glm::vec3(255.0f, 171.0f, 25.0f) / 255.0f * ambient, glm::vec3(255.0f, 171.0f, 25.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial greyslate(glm::vec3(158.0f, 158.0f, 158.0f) / 255.0f * ambient, glm::vec3(158.0f, 158.0f, 158.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial darkred(glm::vec3(193.0f, 21.0f, 21.0f) / 255.0f * ambient, glm::vec3(193.0f, 21.0f, 21.0f) / 255.0f, glm::vec3(1, 1, 1));

   ChOpenGLMaterial t1(glm::vec3(236.0f, 208.0f, 120.0f) / 255.0f * ambient, glm::vec3(236.0f, 208.0f, 120.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial t2(glm::vec3(217.0f, 91.0f, 67.0f) / 255.0f * ambient, glm::vec3(217.0f, 91.0f, 67.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial t3(glm::vec3(192.0f, 41.0f, 66.0f) / 255.0f * ambient, glm::vec3(192.0f, 41.0f, 66.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial t4(glm::vec3(84.0f, 36.0f, 55.0f) / 255.0f * ambient, glm::vec3(84.0f, 36.0f, 55.0f) / 255.0f, glm::vec3(1, 1, 1));
   ChOpenGLMaterial t5(glm::vec3(83.0f, 119.0f, 122.0f) / 255.0f * ambient, glm::vec3(83.0f, 119.0f, 122.0f) / 255.0f, glm::vec3(1, 1, 1));

   if (!main_shader.InitializeStrings("phong", phong_vert, phong_frag)) {
      return 0;
   }

   if (!cloud_shader.InitializeStrings("cloud", cloud_vert, cloud_frag)) {
      return 0;
   }

   sphere.Initialize("../resources/sphere.obj", slate, &main_shader);
   box.Initialize("../resources/box.obj", t3, &main_shader);
   cylinder.Initialize("../resources/cylinder.obj", apple, &main_shader);
   cone.Initialize("../resources/cone.obj", white, &main_shader);

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
   text_texture_handle = font_shader.GetUniformLocation("tex");
   text_color_handle = font_shader.GetUniformLocation("color");

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, texture);
   glTexImage2D( GL_TEXTURE_2D, 0, GL_R8, font_data.tex_width, font_data.tex_height, 0, GL_RED, GL_UNSIGNED_BYTE, font_data.tex_data);
   glBindTexture(GL_TEXTURE_2D, 0);
   cloud_data.resize(physics_system->Get_bodylist()->size());
   for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
      ChBody* abody = (ChBody*) physics_system->Get_bodylist()->at(i);

      ChVector<> pos = abody->GetPos();
      cloud_data[i] = glm::vec3(pos.x, pos.y, pos.z);
   }
   cloud.Initialize(cloud_data, white, &cloud_shader);

   contacts.Initialize(cloud_data, darkred, &cloud_shader);

   glPointSize(10);
   GenerateFontIndex();

}
bool ChOpenGLViewer::Update() {
   if (pause_sim == true && single_step == false) {
      return false;
   }

   physics_system->DoStepDynamics(physics_system->GetStep());
   single_step = false;
   return true;
}
void ChOpenGLViewer::Render() {
   render_timer.start();
   if (pause_vis == false) {
      geometry_timer.start();
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

      if (render_mode != POINTS) {
         model_box.clear();
         model_sphere.clear();
         model_cone.clear();
         model_cylinder.clear();
         model_obj.clear();
         for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
            ChBody* abody = (ChBody*) physics_system->Get_bodylist()->at(i);
            DrawObject(abody);
         }
         if (model_box.size() > 0) {
            box.Update(model_box);
            box.Draw(projection, view);
         }
         if (model_sphere.size() > 0) {
            sphere.Update(model_sphere);
            sphere.Draw(projection, view);
         }
         if (model_cone.size() > 0) {
            cone.Update(model_cone);
            cone.Draw(projection, view);
         }
         if (model_cylinder.size() > 0) {
            cylinder.Update(model_cylinder);
            cylinder.Draw(projection, view);
         }
         if (model_obj.size() > 0) {
            for (map<string, ChOpenGLOBJ>::iterator iter = obj_files.begin(); iter != obj_files.end(); iter++) {
               (*iter).second.Update(model_obj[(*iter).first]);
               (*iter).second.Draw(projection, view);

            }

         }

      } else {
         cloud_data.resize(physics_system->Get_bodylist()->size());
#pragma omp parallel for
         for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
            ChBody* abody = (ChBody*) physics_system->Get_bodylist()->at(i);
            ChVector<> pos = abody->GetPos();
            cloud_data[i] = glm::vec3(pos.x, pos.y, pos.z);
         }
      }

      if (render_mode == POINTS) {
         cloud.Update(cloud_data);
         glm::mat4 model(1);
         cloud.Draw(projection, view * model);
      }

      RenderContacts();

      geometry_timer.stop();
      time_geometry = .5 * geometry_timer() + .5 * time_geometry;
      text_timer.start();
      DisplayHUD();
      text_timer.stop();
      time_text = .5 * text_timer() + .5 * time_text;
   }
   render_timer.stop();
   time_total = .5 * render_timer() + .5 * time_total;
   current_time = time_total;
   current_time = current_time * 0.5 + old_time * 0.5;
   old_time = current_time;
   fps = 1.0 / current_time;

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
         model_sphere.push_back(model);

      } else if (asset.IsType<ChEllipsoidShape>()) {

         ChEllipsoidShape * ellipsoid_shape = ((ChEllipsoidShape *) (asset.get_ptr()));
         Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
         model_sphere.push_back(model);

      } else if (asset.IsType<ChBoxShape>()) {
         ChBoxShape * box_shape = ((ChBoxShape *) (asset.get_ptr()));
         ChVector<> pos_final = pos + center;
         Vector radius = box_shape->GetBoxGeometry().Size;

         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
         model_box.push_back(model);

      } else if (asset.IsType<ChCylinderShape>()) {
         ChCylinderShape * cylinder_shape = ((ChCylinderShape *) (asset.get_ptr()));
         double rad = cylinder_shape->GetCylinderGeometry().rad;
         double height = cylinder_shape->GetCylinderGeometry().p1.y - cylinder_shape->GetCylinderGeometry().p2.y;
         //Quaternion rott(1,0,0,0);
         Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
         //lrot = lrot % rott;
         lrot = rot % lrot;

         lrot.Q_to_AngAxis(angle, axis);
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(rad, height * .5, rad));
         model_cylinder.push_back(model);

      } else if (asset.IsType<ChConeShape>()) {
         ChConeShape * cone_shape = ((ChConeShape *) (asset.get_ptr()));
         Vector rad = cone_shape->GetConeGeometry().rad;
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(rad.x, rad.y, rad.z));
         model_cone.push_back(model);

      } else if (asset.IsType<ChRoundedBoxShape>()) {
         ChRoundedBoxShape * shape = ((ChRoundedBoxShape *) (asset.get_ptr()));
         Vector rad = shape->GetRoundedBoxGeometry().Size;
         double radsphere = shape->GetRoundedBoxGeometry().radsphere;
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::scale(model, glm::vec3(rad.x, rad.y, rad.z));
         model_box.push_back(model);

         glm::vec3 local = glm::rotate(glm::vec3(rad.x, rad.y, rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(rad.x, rad.y, -rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(-rad.x, rad.y, rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(-rad.x, rad.y, -rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(rad.x, -rad.y, rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(rad.x, -rad.y, -rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(-rad.x, -rad.y, rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

         local = glm::rotate(glm::vec3(-rad.x, -rad.y, -rad.z), float(angle), glm::vec3(axis.x, axis.y, axis.z));
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x + local.x, pos_final.y + local.y, pos_final.z + local.z));
         model = glm::scale(model, glm::vec3(radsphere));
         model_sphere.push_back(model);

      } else if (asset.IsType<ChTriangleMeshShape>()) {
         ChTriangleMeshShape * trimesh_shape = ((ChTriangleMeshShape *) (asset.get_ptr()));
         ChVector<> pos_final = pos + center;
         model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
         model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));

         if (obj_files.find(trimesh_shape->GetName()) == obj_files.end()) {
            ChOpenGLOBJ temp;
            ChOpenGLMaterial pillow(glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f * .5f, glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f, glm::vec3(1, 1, 1));
            cout << trimesh_shape->GetName() << endl;
            temp.Initialize(trimesh_shape->GetName(), pillow, &main_shader);
            obj_files[trimesh_shape->GetName()] = temp;
            model_obj[trimesh_shape->GetName()].push_back(model);
         } else {
            model_obj[trimesh_shape->GetName()].push_back(model);
         }
      }
   }
}
void ChOpenGLViewer::GenerateFontIndex() {
   string chars = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

   for (int i = 0; i < chars.size(); i++) {
      for (int j = 0; j < font_data.glyphs_count; ++j) {
         if (font_data.glyphs[j].charcode == chars[i]) {
            char_index[chars[i]] = j;
            break;
         }
      }
   }
}
void ChOpenGLViewer::RenderText(
                                const std::string &str,
                                float x,
                                float y,
                                float sx,
                                float sy) {
   for (int i = 0; i < str.size(); i++) {
      texture_glyph_t *glyph = 0;
      glyph = &font_data.glyphs[char_index[str[i]]];

      if (!glyph) {
         continue;
      }
      x += glyph->kerning[0].kerning;
      float x0 = (float) (x + glyph->offset_x * sx);
      float y0 = (float) (y + glyph->offset_y * sy);
      float x1 = (float) (x0 + glyph->width * sx);
      float y1 = (float) (y0 - glyph->height * sy);

      float s0 = glyph->s0;
      float t0 = glyph->t0;
      float s1 = glyph->s1;
      float t1 = glyph->t1;

      text_data.push_back(glm::vec4(x0, y0, s0, t0));
      text_data.push_back(glm::vec4(x0, y1, s0, t1));
      text_data.push_back(glm::vec4(x1, y1, s1, t1));
      text_data.push_back(glm::vec4(x0, y0, s0, t0));
      text_data.push_back(glm::vec4(x1, y1, s1, t1));
      text_data.push_back(glm::vec4(x1, y0, s1, t0));

      x += (glyph->advance_x * sx);
   }
}

void ChOpenGLViewer::DisplayHUD() {

   GLReturnedError("Start text");
   float sx = 2.0 / window_size.x;
   float sy = 2.0 / window_size.y;
   text_data.reserve(300);
   text_data.clear();

   char buffer[50];
   if (view_help) {
      RenderText("Press h to exit help", -.95, 0.925 - .06 * 0, sx, sy);
      RenderText("W: Forward", -.95, 0.925 - .06 * 1, sx, sy);
      RenderText("A: Strafe Left", -.95, 0.925 - .06 * 2, sx, sy);
      RenderText("S: Back", -.95, 0.925 - .06 * 3, sx, sy);
      RenderText("D: Strafe Right", -.95, 0.925 - .06 * 4, sx, sy);
      RenderText("Q: Down", -.95, 0.925 - .06 * 5, sx, sy);
      RenderText("E: Up", -.95, 0.925 - .06 * 6, sx, sy);

      RenderText("Mouse Look (Click and hold left mouse button)", -.95, 0.925 - .06 * 7, sx, sy);

      RenderText("1: Point Cloud (default)", -.95, 0.925 - .06 * 9, sx, sy);
      RenderText("2: Wireframe (slow)", -.95, 0.925 - .06 * 10, sx, sy);
      RenderText("3: Solid", -.95, 0.925 - .06 * 11, sx, sy);

      RenderText("C: Show/Hide Contacts (DVI only)", -.95, 0.925 - .06 * 13, sx, sy);

      RenderText("Space: Pause Simulation (not rendering)", -.95, 0.925 - .06 * 15, sx, sy);
      RenderText("P: Pause Rendering (not simulation)", -.95, 0.925 - .06 * 16, sx, sy);
      RenderText(".: Single Step ", -.95, 0.925 - .06 * 18, sx, sy);
      //RenderText("V: Enable/Disable Vsync ", -.95, 0.925 - .06 * 18, sx, sy);

      RenderText("Escape: Exit ", -.95, 0.925 - .06 * 20, sx, sy);

   } else {
      sprintf(buffer, "Press h for help");
      RenderText(buffer, 0, 0.925, sx, sy);

      sprintf(buffer, "Time:  %04f", physics_system->GetChTime());
      RenderText(buffer, -.95, 0.925, sx, sy);

      vector<double> history = ((ChLcpIterativeSolver*) (physics_system->GetLcpSolverSpeed()))->GetViolationHistory();
      vector<double> dlambda = ((ChLcpIterativeSolver*) (physics_system->GetLcpSolverSpeed()))->GetDeltalambdaHistory();

      sprintf(buffer, "Iters   :  %04d", history.size());
      RenderText(buffer, .6, 0.925 - .06 * 0, sx, sy);
      sprintf(buffer, "Bodies  :  %04d", physics_system->GetNbodiesTotal());
      RenderText(buffer, .6, 0.925 - .06 * 1, sx, sy);
      sprintf(buffer, "Contacts:  %04d", physics_system->GetNcontacts());
      RenderText(buffer, .6, 0.925 - .06 * 2, sx, sy);
      if (history.size() > 0) {
         sprintf(buffer, "Residual:  %04f", history[history.size() - 1]);
         RenderText(buffer, .6, 0.925 - .06 * 4, sx, sy);
         sprintf(buffer, "Correct :  %04f", dlambda[dlambda.size() - 1]);
         RenderText(buffer, .6, 0.925 - .06 * 5, sx, sy);
      }

      sprintf(buffer, "Step  :  %04f", physics_system->GetTimerStep());
      RenderText(buffer, .6, -0.925 + .06 * 9, sx, sy);
      sprintf(buffer, "Broad :  %04f", physics_system->GetTimerCollisionBroad());
      RenderText(buffer, .6, -0.925 + .06 * 8, sx, sy);
      sprintf(buffer, "Narrow:  %04f", physics_system->GetTimerCollisionNarrow());
      RenderText(buffer, .6, -0.925 + .06 * 7, sx, sy);
      sprintf(buffer, "Solver:  %04f", physics_system->GetTimerLcp());
      RenderText(buffer, .6, -0.925 + .06 * 6, sx, sy);
      sprintf(buffer, "Update:  %04f", physics_system->GetTimerUpdate());
      RenderText(buffer, .6, -0.925 + .06 * 5, sx, sy);

      sprintf(buffer, "FPS     : %04d", int(fps));
      RenderText(buffer, .6, -0.925 + .06 * 0, sx, sy);
      sprintf(buffer, "Total   : %04f", time_total);
      RenderText(buffer, .6, -0.925 + .06 * 1, sx, sy);
      sprintf(buffer, "Text    : %04f", time_text);
      RenderText(buffer, .6, -0.925 + .06 * 2, sx, sy);
      sprintf(buffer, "Geometry: %04f", time_geometry);
      RenderText(buffer, .6, -0.925 + .06 * 3, sx, sy);

      if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
         sprintf(buffer, "TimerA:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverA"));
         RenderText(buffer, -.95, -0.925 + .06 * 9, sx, sy);
         sprintf(buffer, "TimerB:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverB"));
         RenderText(buffer, -.95, -0.925 + .06 * 8, sx, sy);
         sprintf(buffer, "TimerC:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverC"));
         RenderText(buffer, -.95, -0.925 + .06 * 7, sx, sy);
         sprintf(buffer, "TimerD:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverD"));
         RenderText(buffer, -.95, -0.925 + .06 * 6, sx, sy);
         sprintf(buffer, "TimerE:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverE"));
         RenderText(buffer, -.95, -0.925 + .06 * 5, sx, sy);
         sprintf(buffer, "TimerF:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverF"));
         RenderText(buffer, -.95, -0.925 + .06 * 4, sx, sy);
         sprintf(buffer, "TimerG:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverG"));
         RenderText(buffer, -.95, -0.925 + .06 * 3, sx, sy);
         sprintf(buffer, "Shur A:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_shurA"));
         RenderText(buffer, -.95, -0.925 + .06 * 2, sx, sy);
         sprintf(buffer, "Shur B:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_shurB"));
         RenderText(buffer, -.95, -0.925 + .06 * 1, sx, sy);
         sprintf(buffer, "Proj  :  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_Project"));
         RenderText(buffer, -.95, -0.925 + .06 * 0, sx, sy);

      }
   }
   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, texture);

   ChOpenGLMaterial text(glm::vec3(0, 0, 0), glm::vec3(236.0f, 208.0f, 120.0f) / 255.0f, glm::vec3(1, 1, 1));

   glBindSampler(0, sampler);
   glBindVertexArray(vao);
   glEnableVertexAttribArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, vbo);
   font_shader.Use();
   glUniform1i(text_texture_handle, 0);
   glUniform3fv(text_color_handle, 1, glm::value_ptr(text.diffuse_color));
   glBufferData(GL_ARRAY_BUFFER, text_data.size() * sizeof(glm::vec4), &this->text_data[0], GL_STATIC_DRAW);
   glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
   glDrawArrays(GL_TRIANGLES, 0, text_data.size());
   glBindTexture(GL_TEXTURE_2D, 0);
   glUseProgram(0);
   GLReturnedError("End text");

}

void ChOpenGLViewer::RenderContacts() {
   if (view_contacts == false) {
      return;
   }

   if (ChSystemParallel* system = dynamic_cast<ChSystemParallel*>(physics_system)) {
      ChParallelDataManager * data_manager = system->data_manager;
      contact_data.resize(data_manager->num_contacts * 2);
      for (int i = 0; i < data_manager->num_contacts; i++) {
         int2 ID = data_manager->host_data.bids_rigid_rigid[i];

         real3 cpta = data_manager->host_data.cpta_rigid_rigid[i] + data_manager->host_data.pos_data[ID.x];
         real3 cptb = data_manager->host_data.cptb_rigid_rigid[i] + data_manager->host_data.pos_data[ID.y];

         contact_data[i] = glm::vec3(cpta.x, cpta.y, cpta.z);
         contact_data[i + data_manager->num_contacts] = glm::vec3(cptb.x, cptb.y, cptb.z);
      }

      contacts.Update(contact_data);
      glm::mat4 model(1);
      contacts.Draw(projection, view * model);
   } else {
      return;
   }

}

void ChOpenGLViewer::HandleInput(
                                 unsigned char key,
                                 int x,
                                 int y) {
//printf("%f,%f,%f\n", render_camera.camera_position.x, render_camera.camera_position.y, render_camera.camera_position.z);
   switch (key) {
      case 'W':
         render_camera.Move(FORWARD);
         break;
      case 'S':
         render_camera.Move(BACK);
         break;
      case 'D':
         render_camera.Move(RIGHT);
         break;
      case 'A':
         render_camera.Move(LEFT);
         break;
      case 'Q':
         render_camera.Move(DOWN);
         break;
      case 'E':
         render_camera.Move(UP);
         break;
      case GLFW_KEY_SPACE:
         pause_sim = !pause_sim;
         break;
      case 'P':
         pause_vis = !pause_vis;
         break;
      case '1':
         render_mode = POINTS;
         break;
      case '2':
         render_mode = WIREFRAME;
         break;
      case '3':
         render_mode = SOLID;
         break;
      case 'C':
         view_contacts = !view_contacts;
         break;
      case 'H':
         view_help = !view_help;
         break;
      case 'V':
//         use_vsync = !use_vsync;
//         if (use_vsync) {
//            glfwSwapInterval(1);
//         } else {
//            glfwSwapInterval(0);
//         }
         break;
      case GLFW_KEY_PERIOD:
         single_step = true;
         break;
      default:
         break;
   }
}
