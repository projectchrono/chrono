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

#ifndef CHOPENGLVIEWER_H
#define CHOPENGLVIEWER_H

#include "chrono_opengl/core/ChApiOpenGL.h"
#include "chrono_opengl/core/ChOpenGLBase.h"
#include "chrono_opengl/ChOpenGLCamera.h"
#include "chrono_opengl/core/ChOpenGLShader.h"
#include "chrono_opengl/shapes/ChOpenGLCloud.h"
#include "chrono_opengl/shapes/ChOpenGLWires.h"
#include "chrono_opengl/shapes/ChOpenGLOBJ.h"
#include "physics/ChSystem.h"
#include "core/ChTimer.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include <glfw3.h>

namespace chrono {
namespace opengl {

enum RenderMode {
  POINTS,
  WIREFRAME,
  SOLID
};

class CH_OPENGL_API ChOpenGLViewer : public ChOpenGLBase {
 public:
  ChOpenGLViewer(ChSystem* system);
  ~ChOpenGLViewer();
  void TakeDown();
  bool Initialize();
  bool Update(double time_step);
  void Render();
  void DrawObject(ChBody* abody);
  void GenerateFontIndex();
  void RenderText(const std::string& str, float x, float y, float sx, float sy);
  void DisplayHUD();
  void RenderContacts();
  void RenderAABB();
  void RenderGrid();
  void RenderPlots();
  void HandleInput(unsigned char key, int x, int y);

  glm::ivec2 window_size;
  glm::ivec2 window_position;
  float window_aspect;
  int interval;

  ChOpenGLCamera render_camera, ortho_camera;
  ChSystem* physics_system;

  ChOpenGLShader main_shader;
  ChOpenGLShader cloud_shader;
  ChOpenGLShader dot_shader;
  ChOpenGLShader sphere_shader;
  ChOpenGLShader font_shader;

  std::map<char, int> char_index;

  ChOpenGLOBJ sphere;
  ChOpenGLOBJ box;
  ChOpenGLOBJ cylinder;
  ChOpenGLOBJ cone;

  ChOpenGLCloud cloud, contacts, fluid;
  ChOpenGLWires grid, plots;
  std::vector<glm::vec3> cloud_data, fluid_data;
  std::vector<glm::vec3> contact_data, grid_data, plot_data;
  int simulation_frame;     // The current frame number
  float simulation_h;       // The simulation step size
  float simulation_time;    // The current simulation time
  bool pause_sim;
  bool pause_vis;
  bool single_step;
  bool view_contacts, view_help, view_aabb, view_grid, view_info;
  bool use_vsync;
  RenderMode render_mode;

  glm::mat4 model, view, projection, modelview;

  GLuint vbo, vao;
  GLuint text_texture_handle, text_color_handle;
  GLuint texture, sampler;
  ChTimer<double> render_timer, text_timer, geometry_timer;
  float old_time, current_time;
  float time_geometry, time_text, time_total, fps;

  std::vector<glm::mat4> model_box;
  std::vector<glm::mat4> model_sphere;
  std::vector<glm::mat4> model_cylinder;
  std::vector<glm::mat4> model_cone;
  std::vector<glm::vec4> text_data;
  std::map<std::string, ChOpenGLMesh> obj_files;
  std::map<std::string, std::vector<glm::mat4> > model_obj;
};
}
}

#endif    // END of CHOPENGLVIEWER_H
