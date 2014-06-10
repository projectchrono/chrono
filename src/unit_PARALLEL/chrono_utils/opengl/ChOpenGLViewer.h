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

#include "chrono_utils/opengl/core/ChApiOpenGL.h"
#include "chrono_utils/opengl/core/ChOpenGLBase.h"
#include "chrono_utils/opengl/ChOpenGLCamera.h"
#include "chrono_utils/opengl/core/ChOpenGLShader.h"
#include "chrono_utils/opengl/shapes/ChOpenGLCloud.h"
#include "chrono_utils/opengl/shapes/ChOpenGLOBJ.h"
#include "physics/ChSystem.h"
#include "core/ChTimer.h"
#include "lcp/ChLcpIterativeSolver.h"
#include "assets/ChBoxShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChConeShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "chrono_parallel/ChSystemParallel.h"

namespace chrono {
namespace utils {

enum RenderMode {
   POINTS,
   WIREFRAME,
   SOLID
};

class CH_UTILS_OPENGL_API ChOpenGLViewer : public ChOpenGLBase {
 public:

   ChOpenGLViewer(
         ChSystem * system);
   ~ChOpenGLViewer();
   void TakeDown();
   bool Initialize();
   bool Update();
   void Render();
   void DrawObject(
         ChBody * abody);
   void RenderText(
         const std::string &str,
         float x,
         float y,
         float sx,
         float sy);
   void DisplayHUD();
   void HandleInput(
         unsigned char key,
         int x,
         int y);

   glm::ivec2 window_size;
   glm::ivec2 window_position;
   float window_aspect;
   int interval;

   ChOpenGLCamera render_camera;
   ChSystem * physics_system;

   ChOpenGLShader main_shader;
   ChOpenGLShader cloud_shader;
   ChOpenGLShader font_shader;

   ChOpenGLOBJ sphere;
   ChOpenGLOBJ box;
   ChOpenGLOBJ cylinder;
   ChOpenGLOBJ cone;

   ChOpenGLCloud cloud;
   vector<glm::vec3> cloud_data;
   int simulation_frame;  // The current frame number
   float simulation_h;  // The simulation step size
   float simulation_time;  // The current simulation time
   bool pause_sim;
   bool pause_vis;
   RenderMode render_mode;

   glm::mat4 model, view, projection, modelview;

   GLuint vbo, vao;
   GLuint texture_handle;
   GLuint texture, sampler;
   ChTimer<double> render_timer, text_timer, geometry_timer;
   float old_time, current_time;
   float time_geometry, time_text, time_total, fps;
};
}
}

#endif  // END of CHOPENGLVIEWER_H
