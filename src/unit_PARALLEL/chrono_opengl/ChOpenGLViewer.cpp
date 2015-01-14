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
#include "chrono_parallel/ChApiParallel.h"
#include "chrono_opengl/ChOpenGLViewer.h"
#include "chrono_opengl/FontData.h"

#include "chrono_parallel/physics/ChNodeFluid.h"

#include "assets/ChBoxShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChConeShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChRoundedBoxShape.h"
#include "assets/ChRoundedConeShape.h"
#include "assets/ChRoundedCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "lcp/ChLcpIterativeSolver.h"

// Includes are generated at compile time!
#include "resources/text_frag.h"
#include "resources/text_vert.h"
#include "resources/phong_frag.h"
#include "resources/phong_vert.h"
#include "resources/cloud_frag.h"
#include "resources/cloud_vert.h"
#include "resources/dot_frag.h"
#include "resources/dot_vert.h"
#include "resources/sphere_frag.h"
#include "resources/sphere_vert.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace chrono;
using namespace chrono::opengl;

ChOpenGLViewer::ChOpenGLViewer(ChSystem* system) {
  physics_system = system;

  render_camera.SetMode(FREE);
  render_camera.SetPosition(glm::vec3(0, 0, -10));
  render_camera.SetLookAt(glm::vec3(0, 0, 0));
  render_camera.SetClipping(.1, 1000);
  render_camera.SetFOV(45);

  simulation_frame = 0;
  simulation_time = 0;
  simulation_h = 0;
  pause_sim = 0;
  pause_vis = 0;
  single_step = 0;
  view_contacts = 0;
  view_aabb = 0;
  view_help = 0;
  view_grid = 0;
  view_info = 0;
  use_vsync = 0;
  render_mode = POINTS;
  old_time = current_time = 0;
  time_total = time_text = time_geometry = 0;
}

ChOpenGLViewer::~ChOpenGLViewer() {}

void ChOpenGLViewer::TakeDown() {
  render_camera.TakeDown();
  main_shader.TakeDown();
  font_shader.TakeDown();
  cloud_shader.TakeDown();
  dot_shader.TakeDown();
  sphere_shader.TakeDown();
  sphere.TakeDown();
  box.TakeDown();
  cylinder.TakeDown();
  cone.TakeDown();
  cloud.TakeDown();
  contacts.TakeDown();
  fluid.TakeDown();
  grid.TakeDown();
  plots.TakeDown();
  for (std::map<std::string, ChOpenGLMesh>::iterator iter = obj_files.begin(); iter != obj_files.end(); iter++) {
    (*iter).second.TakeDown();
  }
}

bool ChOpenGLViewer::Initialize() {
  if (!font_shader.InitializeStrings("text", text_vert, text_frag)) {
    return 0;
  }

  ChOpenGLMaterial white(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
  ChOpenGLMaterial red(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 1, 1));
  ChOpenGLMaterial river(glm::vec3(0, 0, 0), glm::vec3(52.0 / 255.0, 152.0 / 255.0, 219.0 / 255.0), glm::vec3(1, 1, 1));
  ChOpenGLMaterial darkriver(glm::vec3(0, 0, 0), glm::vec3(77.0 / 255.0, 114.0 / 255.0, 130.0 / 255.0), glm::vec3(1, 1, 1));
  ChOpenGLMaterial brightriver(glm::vec3(0, 0, 0), glm::vec3(146.0 / 255.0, 214.0 / 255.0, 255.0 / 255.0), glm::vec3(1, 1, 1));

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
  if (!dot_shader.InitializeStrings("dot", dot_vert, dot_frag)) {
    return 0;
  }
  if (!sphere_shader.InitializeStrings("sphere", sphere_vert, sphere_frag)) {
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

  // get the uniform location for the texture from shader
  text_texture_handle = font_shader.GetUniformLocation("tex");
  text_color_handle = font_shader.GetUniformLocation("color");

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, font_data.tex_width, font_data.tex_height, 0, GL_RED, GL_UNSIGNED_BYTE, font_data.tex_data);
  glBindTexture(GL_TEXTURE_2D, 0);

  // Point cloud mode will draw the rigid bodies and any other physics items.
  // Currently only fluid is supported
  cloud_data.resize(physics_system->Get_bodylist()->size());
  fluid_data.resize(physics_system->Get_otherphysicslist()->size());
#pragma omp parallel for
  for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
    ChBody* abody = (ChBody*)physics_system->Get_bodylist()->at(i);

    ChVector<> pos = abody->GetPos();
    cloud_data[i] = glm::vec3(pos.x, pos.y, pos.z);
  }

  // Get the fluid point data
  for (int i = 0; i < physics_system->Get_otherphysicslist()->size(); i++) {
    if (ChNodeFluid* node = dynamic_cast<ChNodeFluid*>(physics_system->Get_otherphysicslist()->at(i))) {
      ChVector<> pos = node->GetPos();
      fluid_data[i] = glm::vec3(pos.x, pos.y, pos.z);
    }
  }

  cloud.Initialize(cloud_data, white, &cloud_shader);
  fluid.Initialize(fluid_data, river, &dot_shader);
  contacts.Initialize(cloud_data, darkred, &cloud_shader);
  grid.Initialize(grid_data, darkriver, &cloud_shader);
  plots.Initialize(grid_data, brightriver, &cloud_shader);
  contacts.SetPointSize(0.01);

  // glEnable(GL_MULTISAMPLE);
  glEnable(GL_POINT_SPRITE);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // glLineWidth(10);
  // glEnable(GL_LINE_SMOOTH);
  GenerateFontIndex();
}
bool ChOpenGLViewer::Update(double time_step) {
  if (pause_sim == true && single_step == false) {
    return false;
  }
  simulation_h = time_step;
  physics_system->DoStepDynamics(time_step);
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

    main_shader.SetViewport(window_size);
    cloud_shader.SetViewport(window_size);
    dot_shader.SetViewport(window_size);
    sphere_shader.SetViewport(window_size);

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
        ChBody* abody = (ChBody*)physics_system->Get_bodylist()->at(i);
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
        for (std::map<std::string, ChOpenGLMesh>::iterator iter = obj_files.begin(); iter != obj_files.end(); iter++) {
          (*iter).second.Update(model_obj[(*iter).first]);
          (*iter).second.Draw(projection, view);
        }
      }
      // Get the fluid point data
//      fluid_data.resize(physics_system->Get_otherphysicslist()->size());
//#pragma omp parallel for
//      for (int i = 0; i < physics_system->Get_otherphysicslist()->size(); i++) {
//        if (ChNodeFluid* node = dynamic_cast<ChNodeFluid*>(physics_system->Get_otherphysicslist()->at(i))) {
//          ChVector<> pos = node->GetPos();
//          fluid_data[i] = glm::vec3(pos.x, pos.y, pos.z);
//        }
//      }
//      fluid.AttachShader(&dot_shader);
//      if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
//        if (parallel_sys->data_manager->settings.fluid.fluid_is_rigid) {
//          fluid.SetPointSize(parallel_sys->data_manager->settings.fluid.kernel_radius * 2);
//        } else {
//          fluid.SetPointSize(parallel_sys->data_manager->settings.fluid.kernel_radius * 2 * .51);
//        }
//      }
    } else {
      cloud_data.resize(physics_system->Get_bodylist()->size());
#pragma omp parallel for
      for (int i = 0; i < physics_system->Get_bodylist()->size(); i++) {
        ChBody* abody = (ChBody*)physics_system->Get_bodylist()->at(i);
        ChVector<> pos = abody->GetPos();
        cloud_data[i] = glm::vec3(pos.x, pos.y, pos.z);
      }

      fluid_data.resize(physics_system->Get_otherphysicslist()->size());
// Get the fluid point data
#pragma omp parallel for
      for (int i = 0; i < physics_system->Get_otherphysicslist()->size(); i++) {
        if (ChNodeFluid* node = dynamic_cast<ChNodeFluid*>(physics_system->Get_otherphysicslist()->at(i))) {
          ChVector<> pos = node->GetPos();
          fluid_data[i] = glm::vec3(pos.x, pos.y, pos.z);
        }
      }
      fluid.AttachShader(&cloud_shader);
      fluid.SetPointSize(0.005);
    }
    if (physics_system->Get_otherphysicslist()->size() > 0) {
      fluid.Update(fluid_data);
      glm::mat4 model(1);
      fluid.Draw(projection, view * model);
    }
    if (render_mode == POINTS) {
      cloud.Update(cloud_data);
      glm::mat4 model(1);
      cloud.Draw(projection, view * model);
    }
    RenderGrid();
    RenderAABB();
    RenderPlots();
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

void ChOpenGLViewer::DrawObject(ChBody* abody) {
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

    if (!asset.IsType<ChVisualization>()) {
      continue;
    }

    ChVisualization* visual_asset = ((ChVisualization*)(asset.get_ptr()));
    Vector center = visual_asset->Pos;
    center = rot.Rotate(center);
    Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
    lrot = rot % lrot;
    lrot.Normalize();
    lrot.Q_to_AngAxis(angle, axis);

    if (asset.IsType<ChSphereShape>()) {
      ChSphereShape* sphere_shape = ((ChSphereShape*)(asset.get_ptr()));
      float radius = sphere_shape->GetSphereGeometry().rad;
      ChVector<> pos_final = pos + center;

      model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
      model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
      model = glm::scale(model, glm::vec3(radius, radius, radius));
      model_sphere.push_back(model);

    } else if (asset.IsType<ChEllipsoidShape>()) {
      ChEllipsoidShape* ellipsoid_shape = ((ChEllipsoidShape*)(asset.get_ptr()));
      Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
      ChVector<> pos_final = pos + center;
      model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
      model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
      model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
      model_sphere.push_back(model);

    } else if (asset.IsType<ChBoxShape>()) {
      ChBoxShape* box_shape = ((ChBoxShape*)(asset.get_ptr()));
      ChVector<> pos_final = pos + center;
      Vector radius = box_shape->GetBoxGeometry().Size;

      model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
      model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
      model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
      model_box.push_back(model);

    } else if (asset.IsType<ChCylinderShape>()) {
      ChCylinderShape* cylinder_shape = ((ChCylinderShape*)(asset.get_ptr()));
      double rad = cylinder_shape->GetCylinderGeometry().rad;
      double height = cylinder_shape->GetCylinderGeometry().p1.y - cylinder_shape->GetCylinderGeometry().p2.y;
      // Quaternion rott(1,0,0,0);
      Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
      // lrot = lrot % rott;
      lrot = rot % lrot;

      lrot.Q_to_AngAxis(angle, axis);
      ChVector<> pos_final = pos + center;
      model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
      model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
      model = glm::scale(model, glm::vec3(rad, height * .5, rad));
      model_cylinder.push_back(model);

    } else if (asset.IsType<ChConeShape>()) {
      ChConeShape* cone_shape = ((ChConeShape*)(asset.get_ptr()));
      Vector rad = cone_shape->GetConeGeometry().rad;
      ChVector<> pos_final = pos + center;
      model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
      model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));
      model = glm::scale(model, glm::vec3(rad.x, rad.y, rad.z));
      model_cone.push_back(model);

    } else if (asset.IsType<ChRoundedBoxShape>()) {
      ChRoundedBoxShape* shape = ((ChRoundedBoxShape*)(asset.get_ptr()));
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
      ChTriangleMeshShape* trimesh_shape = ((ChTriangleMeshShape*)(asset.get_ptr()));
      ChVector<> pos_final = pos + center;
      model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x, pos_final.y, pos_final.z));
      model = glm::rotate(model, float(angle), glm::vec3(axis.x, axis.y, axis.z));

      if (obj_files.find(trimesh_shape->GetName()) == obj_files.end()) {
        ChOpenGLMaterial pillow(glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f * .5f, glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f, glm::vec3(1, 1, 1));
        std::cout << trimesh_shape->GetName() << std::endl;
        obj_files[trimesh_shape->GetName()].Initialize(trimesh_shape, pillow);
        obj_files[trimesh_shape->GetName()].AttachShader(&main_shader);
        model_obj[trimesh_shape->GetName()].push_back(model);
      } else {
        model_obj[trimesh_shape->GetName()].push_back(model);
      }
    }
  }
}
void ChOpenGLViewer::GenerateFontIndex() {
  std::string chars =
      " !\"#$%&'()*+,-./"
      "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`"
      "abcdefghijklmnopqrstuvwxyz{|}~";

  for (int i = 0; i < chars.size(); i++) {
    for (int j = 0; j < font_data.glyphs_count; ++j) {
      if (font_data.glyphs[j].charcode == chars[i]) {
        char_index[chars[i]] = j;
        break;
      }
    }
  }
}
void ChOpenGLViewer::RenderText(const std::string& str, float x, float y, float sx, float sy) {
  for (int i = 0; i < str.size(); i++) {
    texture_glyph_t* glyph = 0;
    glyph = &font_data.glyphs[char_index[str[i]]];

    if (!glyph) {
      continue;
    }
    x += glyph->kerning[0].kerning;
    float x0 = (float)(x + glyph->offset_x * sx);
    float y0 = (float)(y + glyph->offset_y * sy);
    float x1 = (float)(x0 + glyph->width * sx);
    float y1 = (float)(y0 - glyph->height * sy);

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
  real spacing = 0.055;

  char buffer[50];
  if (view_help) {
    RenderText("Press h to exit help", -.95, 0.925 - spacing * 0, sx, sy);
    RenderText("W: Forward", -.95, 0.925 - spacing * 1, sx, sy);
    RenderText("A: Strafe Left", -.95, 0.925 - spacing * 2, sx, sy);
    RenderText("S: Back", -.95, 0.925 - spacing * 3, sx, sy);
    RenderText("D: Strafe Right", -.95, 0.925 - spacing * 4, sx, sy);
    RenderText("Q: Down", -.95, 0.925 - spacing * 5, sx, sy);
    RenderText("E: Up", -.95, 0.925 - spacing * 6, sx, sy);

    RenderText("Mouse Look (Click and hold left mouse button)", -.95, 0.925 - spacing * 7, sx, sy);

    RenderText("1: Point Cloud (default)", -.95, 0.925 - spacing * 9, sx, sy);
    RenderText("2: Wireframe (slow)", -.95, 0.925 - spacing * 10, sx, sy);
    RenderText("3: Solid", -.95, 0.925 - spacing * 11, sx, sy);

    RenderText("C: Show/Hide Contacts (DVI only)", -.95, 0.925 - spacing * 13, sx, sy);

    RenderText("Space: Pause Simulation (not rendering)", -.95, 0.925 - spacing * 15, sx, sy);
    RenderText("P: Pause Rendering (not simulating)", -.95, 0.925 - spacing * 16, sx, sy);
    RenderText(".: Single Step ", -.95, 0.925 - spacing * 18, sx, sy);
    RenderText("B: Enable/Disable AABB ", -.95, 0.925 - spacing * 20, sx, sy);

    RenderText("Escape: Exit ", -.95, 0.925 - spacing * 30, sx, sy);

  } else {
    sprintf(buffer, "Press h for help");
    RenderText(buffer, 0, 0.925, sx, sy);

    sprintf(buffer, "TIME:  %04f  | %04f", physics_system->GetChTime(), simulation_h);
    RenderText(buffer, -.95, 0.925, sx, sy);
    sprintf(buffer, "Camera Pos :  [%04f, %04f, %04f]", render_camera.camera_position.x, render_camera.camera_position.y, render_camera.camera_position.z);
    RenderText(buffer, -.95, 0.925 - spacing * 1, sx, sy);
    sprintf(buffer, "Camera Look:  [%04f, %04f, %04f]", render_camera.camera_look_at.x, render_camera.camera_look_at.y, render_camera.camera_look_at.z);
    RenderText(buffer, -.95, 0.925 - spacing * 2, sx, sy);
    sprintf(buffer, "Camera Up  :  [%04f, %04f, %04f]", render_camera.camera_up.x, render_camera.camera_up.y, render_camera.camera_up.z);
    RenderText(buffer, -.95, 0.925 - spacing * 3, sx, sy);

    if (((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetRecordViolation()) {
      std::vector<double> history(0);
      std::vector<double> dlambda(0);

      history = ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetViolationHistory();
      dlambda = ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetDeltalambdaHistory();

      if (history.size() > 0) {
        sprintf(buffer, "SOLVER INFO");
        RenderText(buffer, .6, 0.925 - spacing * 6, sx, sy);
        sprintf(buffer, "ITERS    %04d", history.size());
        RenderText(buffer, .6, 0.925 - spacing * 7, sx, sy);
        sprintf(buffer, "RESIDUAL %04f", history[history.size() - 1]);
        RenderText(buffer, .6, 0.925 - spacing * 8, sx, sy);
        sprintf(buffer, "CORRECT  %04f", dlambda[dlambda.size() - 1]);
        RenderText(buffer, .6, 0.925 - spacing * 9, sx, sy);
      }
    }
    int num_bodies = (physics_system->GetNbodiesTotal() + physics_system->GetNphysicsItems());
    int num_contacts = ((ChSystemParallel*)physics_system)->GetNcontacts();
    int average_contacts_per_body = 0;
    if (num_bodies > 0) {
      average_contacts_per_body = num_contacts / num_bodies;
    }
    sprintf(buffer, "MODEL INFO");
    RenderText(buffer, .6, 0.925 - spacing * 0, sx, sy);
    sprintf(buffer, "BODIES     %04d", num_bodies);
    RenderText(buffer, .6, 0.925 - spacing * 1, sx, sy);
    if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
      sprintf(buffer, "AABB       %04d", parallel_sys->data_manager->host_data.aabb_rigid.size() / 2);
      RenderText(buffer, .6, 0.925 - spacing * 2, sx, sy);
    }
    sprintf(buffer, "CONTACTS   %04d", num_contacts);
    RenderText(buffer, .6, 0.925 - spacing * 3, sx, sy);
    sprintf(buffer, "AVGCONPB   %04d", average_contacts_per_body);
    RenderText(buffer, .6, 0.925 - spacing * 4, sx, sy);

    sprintf(buffer, "TIMING INFO");
    RenderText(buffer, .6, -0.925 + spacing * 11, sx, sy);
    sprintf(buffer, "STEP     %04f", physics_system->GetTimerStep());
    RenderText(buffer, .6, -0.925 + spacing * 10, sx, sy);
    sprintf(buffer, "BROAD    %04f", physics_system->GetTimerCollisionBroad());
    RenderText(buffer, .6, -0.925 + spacing * 9, sx, sy);
    sprintf(buffer, "NARROW   %04f", physics_system->GetTimerCollisionNarrow());
    RenderText(buffer, .6, -0.925 + spacing * 8, sx, sy);
    sprintf(buffer, "SOLVE    %04f", physics_system->GetTimerLcp());
    RenderText(buffer, .6, -0.925 + spacing * 7, sx, sy);
    sprintf(buffer, "UPDATE   %04f", physics_system->GetTimerUpdate());
    RenderText(buffer, .6, -0.925 + spacing * 6, sx, sy);

    if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
      int3 grid_size = parallel_sys->data_manager->measures.collision.grid_size;
      real3 bin_size_vec = 1.0 / parallel_sys->data_manager->measures.collision.bin_size_vec;
      real3 min_pt = parallel_sys->data_manager->measures.collision.min_bounding_point;
      real3 max_pt = parallel_sys->data_manager->measures.collision.max_bounding_point;
      real3 center = (min_pt + max_pt) * .5;
      int max_aabb_per_bin = parallel_sys->data_manager->measures.collision.max_aabb_per_bin;
      sprintf(buffer, "COLLISION INFO");
      RenderText(buffer, .6, 0.925 - spacing * 11, sx, sy);
      sprintf(buffer, "DIMS  [%d,%d,%d]", grid_size.x, grid_size.y, grid_size.z);
      RenderText(buffer, .6, 0.925 - spacing * 12, sx, sy);
      sprintf(buffer, "MAX   %d", max_aabb_per_bin);
      RenderText(buffer, .6, 0.925 - spacing * 13, sx, sy);
      sprintf(buffer, "SX    %f", bin_size_vec.x);
      RenderText(buffer, .6, 0.925 - spacing * 14, sx, sy);
      sprintf(buffer, "SY    %f", bin_size_vec.y);
      RenderText(buffer, .6, 0.925 - spacing * 15, sx, sy);
      sprintf(buffer, "SZ    %f", bin_size_vec.z);
      RenderText(buffer, .6, 0.925 - spacing * 16, sx, sy);
      sprintf(buffer, "RIGID %d", parallel_sys->data_manager->num_contacts);
      RenderText(buffer, .6, 0.925 - spacing * 17, sx, sy);
//      sprintf(buffer, "BOUND %d", parallel_sys->data_manager->num_boundary_contacts);
//      RenderText(buffer, .6, 0.925 - spacing * 18, sx, sy);
//      sprintf(buffer, "FLUID %d", parallel_sys->data_manager->num_fluid_contacts);
//      RenderText(buffer, .6, 0.925 - spacing * 19, sx, sy);
    }

    sprintf(buffer, "RENDER INFO");
    RenderText(buffer, .6, -0.925 + spacing * 4, sx, sy);
    sprintf(buffer, "GEOMETRY %04f", time_geometry);
    RenderText(buffer, .6, -0.925 + spacing * 3, sx, sy);
    sprintf(buffer, "TEXT     %04f", time_text);
    RenderText(buffer, .6, -0.925 + spacing * 2, sx, sy);
    sprintf(buffer, "TOTAL    %04f", time_total);
    RenderText(buffer, .6, -0.925 + spacing * 1, sx, sy);
    sprintf(buffer, "FPS      %04d", int(fps));
    RenderText(buffer, .6, -0.925 + spacing * 0, sx, sy);

    if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
      sprintf(buffer, "TimerA:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverA"));
      RenderText(buffer, -.95, -0.925 + spacing * 9, sx, sy);
      sprintf(buffer, "TimerB:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverB"));
      RenderText(buffer, -.95, -0.925 + spacing * 8, sx, sy);
      sprintf(buffer, "TimerC:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverC"));
      RenderText(buffer, -.95, -0.925 + spacing * 7, sx, sy);
      sprintf(buffer, "TimerD:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverD"));
      RenderText(buffer, -.95, -0.925 + spacing * 6, sx, sy);
      sprintf(buffer, "TimerE:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverE"));
      RenderText(buffer, -.95, -0.925 + spacing * 5, sx, sy);
      sprintf(buffer, "TimerF:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverF"));
      RenderText(buffer, -.95, -0.925 + spacing * 4, sx, sy);
      sprintf(buffer, "TimerG:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverG"));
      RenderText(buffer, -.95, -0.925 + spacing * 3, sx, sy);
      sprintf(buffer, "Shur A:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_shurA"));
      RenderText(buffer, -.95, -0.925 + spacing * 2, sx, sy);
      sprintf(buffer, "Shur B:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_shurB"));
      RenderText(buffer, -.95, -0.925 + spacing * 1, sx, sy);
      sprintf(buffer, "Proj  :  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_Project"));
      RenderText(buffer, -.95, -0.925 + spacing * 0, sx, sy);
      float posx = -.6;
      sprintf(buffer, "B_Initial : %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_Init"));
      RenderText(buffer, posx, -0.925 + spacing * 9, sx, sy);
      sprintf(buffer, "B_AABBBINC: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_BIN_Count"));
      RenderText(buffer, posx, -0.925 + spacing * 8, sx, sy);
      sprintf(buffer, "B_AABBBINS: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_BIN_Store"));
      RenderText(buffer, posx, -0.925 + spacing * 7, sx, sy);
      sprintf(buffer, "B_SORT_RED: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_SortReduce"));
      RenderText(buffer, posx, -0.925 + spacing * 6, sx, sy);
      sprintf(buffer, "BAABBAABBC: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_AABB_Count"));
      RenderText(buffer, posx, -0.925 + spacing * 5, sx, sy);
      sprintf(buffer, "BAABBAABBS: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_AABB_Store"));
      RenderText(buffer, posx, -0.925 + spacing * 4, sx, sy);
      sprintf(buffer, "B_POST    : %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_Post"));
      RenderText(buffer, posx, -0.925 + spacing * 3, sx, sy);
      sprintf(buffer, "BROADPHASE: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase"));
      RenderText(buffer, posx, -0.925 + spacing * 2, sx, sy);

      posx = -.6 + .45;
      sprintf(buffer, "BuildD : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildD"));
      RenderText(buffer, posx, -0.925 + spacing * 9, sx, sy);
      sprintf(buffer, "BuildDA: %04f", parallel_sys->data_manager->system_timer.GetTime("BuildDAllocate"));
      RenderText(buffer, posx, -0.925 + spacing * 8, sx, sy);
      sprintf(buffer, "BuildDC: %04f", parallel_sys->data_manager->system_timer.GetTime("BuildDCompute"));
      RenderText(buffer, posx, -0.925 + spacing * 7, sx, sy);
      sprintf(buffer, "BuildE : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildE"));
      RenderText(buffer, posx, -0.925 + spacing * 6, sx, sy);
      sprintf(buffer, "BuildN : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildN"));
      RenderText(buffer, posx, -0.925 + spacing * 5, sx, sy);
      sprintf(buffer, "BuildM : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildM"));
      RenderText(buffer, posx, -0.925 + spacing * 4, sx, sy);
      sprintf(buffer, "Buildb : %04f", parallel_sys->data_manager->system_timer.GetTime("Buildb"));
      RenderText(buffer, posx, -0.925 + spacing * 3, sx, sy);
      sprintf(buffer, "SchurP : %04f", parallel_sys->data_manager->system_timer.GetTime("ShurProduct"));
      RenderText(buffer, posx, -0.925 + spacing * 2, sx, sy);
    }
  }
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture);

  ChOpenGLMaterial text(glm::vec3(0, 0, 0), glm::vec3(100.0f, 145.0f, 170.0f) / 255.0f, glm::vec3(1, 1, 1));

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
    ChParallelDataManager* data_manager = system->data_manager;
    int num_contacts = data_manager->num_contacts;
    contact_data.clear();
    contact_data.resize(num_contacts * 2);
#pragma omp parallel for
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
void ChOpenGLViewer::RenderAABB() {
  if (view_aabb == false || view_info) {
    return;
  }

  if (ChSystemParallel* system = dynamic_cast<ChSystemParallel*>(physics_system)) {
    ChParallelDataManager* data_manager = system->data_manager;
    model_box.clear();
    model_box.resize(data_manager->host_data.aabb_rigid.size() / 2);
#pragma omp parallel for
    for (int i = 0; i < data_manager->host_data.aabb_rigid.size() / 2; i++) {
      real3 min_p, max_p;
      min_p = data_manager->host_data.aabb_rigid[i] + data_manager->measures.collision.global_origin;
      max_p = data_manager->host_data.aabb_rigid[i + data_manager->host_data.aabb_rigid.size() / 2] + data_manager->measures.collision.global_origin;

      real3 radius = (max_p - min_p) * .5;
      real3 center = (min_p + max_p) * .5;

      glm::mat4 model = glm::translate(glm::mat4(1), glm::vec3(center.x, center.y, center.z));
      model = glm::scale(model, glm::vec3(radius.x, radius.y, radius.z));
      model_box[i] = (model);
    }
    if (model_box.size() > 0) {
      box.Update(model_box);
      box.Draw(projection, view);
    }
  }
}

void ChOpenGLViewer::RenderGrid() {
  if (view_grid == false || view_info) {
    return;
  }
  grid_data.clear();
  if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
    int3 grid_size = parallel_sys->data_manager->measures.collision.grid_size;
    real3 bin_size_vec = 1.0 / parallel_sys->data_manager->measures.collision.bin_size_vec;
    real3 min_pt = parallel_sys->data_manager->measures.collision.min_bounding_point;
    real3 max_pt = parallel_sys->data_manager->measures.collision.max_bounding_point;
    real3 center = (min_pt + max_pt) * .5;

    for (int i = 0; i <= grid_size.x; i++) {
      grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, center.y, min_pt.z));
      grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, center.y, max_pt.z));
    }
    for (int i = 0; i <= grid_size.z; i++) {
      grid_data.push_back(glm::vec3(min_pt.x, center.y, i * bin_size_vec.z + min_pt.z));
      grid_data.push_back(glm::vec3(max_pt.x, center.y, i * bin_size_vec.z + min_pt.z));
    }

    for (int i = 0; i <= grid_size.y; i++) {
      grid_data.push_back(glm::vec3(min_pt.x, i * bin_size_vec.y + min_pt.y, center.z));
      grid_data.push_back(glm::vec3(max_pt.x, i * bin_size_vec.y + min_pt.y, center.z));
    }
    for (int i = 0; i <= grid_size.y; i++) {
      grid_data.push_back(glm::vec3(center.x, i * bin_size_vec.y + min_pt.y, min_pt.z));
      grid_data.push_back(glm::vec3(center.x, i * bin_size_vec.y + min_pt.y, max_pt.z));
    }

    for (int i = 0; i <= grid_size.x; i++) {
      grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, min_pt.y, center.z));
      grid_data.push_back(glm::vec3(i * bin_size_vec.x + min_pt.x, max_pt.y, center.z));
    }
    for (int i = 0; i <= grid_size.z; i++) {
      grid_data.push_back(glm::vec3(center.x, min_pt.y, i * bin_size_vec.z + min_pt.z));
      grid_data.push_back(glm::vec3(center.x, max_pt.y, i * bin_size_vec.z + min_pt.z));
    }

    grid.Update(grid_data);
  }

  glm::mat4 model(1);
  grid.Draw(projection, view * model);
}

void ChOpenGLViewer::RenderPlots() {
  if (view_info == false || view_help) {
    return;
  }
  plot_data.clear();

  plot_data.push_back(glm::vec3(window_size.y * .1, window_size.y - window_size.y * .4, 0));
  plot_data.push_back(glm::vec3(window_size.y * .1, window_size.y - window_size.y * .1, 0));

  plot_data.push_back(glm::vec3(window_size.y * .1, window_size.y - window_size.y * .4, 0));
  plot_data.push_back(glm::vec3(window_size.y * .6, window_size.y - window_size.y * .4, 0));

  std::vector<double> history = ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetViolationHistory();
  std::vector<double> dlambda = ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetDeltalambdaHistory();

  real plot_h = (window_size.y * .4 - window_size.y * .1);
  if (history.size() > 1) {
    real max_res = *std::max_element(history.begin(), history.end());
    real min_res = *std::min_element(history.begin(), history.end());
    max_res = max_res + min_res;

    for (int i = 0; i < history.size() - 1; i++) {
      real value = (history[i] + min_res) / max_res * plot_h;
      real value_next = (history[i + 1] + min_res) / max_res * plot_h;

      real size_seg = (window_size.y * .6 - window_size.y * .1) / history.size();

      plot_data.push_back(glm::vec3(window_size.y * .1 + size_seg * i, window_size.y - window_size.y * .4 + value, 0));
      plot_data.push_back(glm::vec3(window_size.y * .1 + size_seg * (i + 1), window_size.y - window_size.y * .4 + value_next, 0));
    }
  }

  projection = glm::ortho(0.0f, float(window_size.x), 0.0f, float(window_size.y), -2.0f, 2.0f);
  modelview = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1));

  glm::mat4 mvp = projection * modelview;

  plots.Update(plot_data);
  plots.Draw(projection, modelview);
}

void ChOpenGLViewer::HandleInput(unsigned char key, int x, int y) {
  // printf("%f,%f,%f\n", render_camera.camera_position.x,
  // render_camera.camera_position.y, render_camera.camera_position.z);
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
    case 'B':
      view_aabb = !view_aabb;
      break;
    case 'G':
      view_grid = !view_grid;
      break;
    case 'H':
      view_help = !view_help;
      break;
    case 'I':
      view_info = !view_info;
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
