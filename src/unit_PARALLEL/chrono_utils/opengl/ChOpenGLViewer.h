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
#include "lcp/ChLcpIterativeSolver.h"
#include "assets/ChBoxShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChConeShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "chrono_parallel/ChSystemParallel.h"

#include <ft2build.h>
#include FT_FREETYPE_H

namespace chrono {
namespace utils {

enum RenderMode {
   POINTS,
   WIREFRAME,
   SOLID
};

struct point {
   GLfloat x;
   GLfloat y;
   GLfloat s;
   GLfloat t;
};

class CH_UTILS_OPENGL_API ChOpenGLViewer : public ChOpenGLBase {
 public:

   ChOpenGLViewer(
         ChSystem * system);
   ~ChOpenGLViewer();
   void TakeDown();
   bool Initialize();
   void Update();
   void Render();
   void DrawObject(
         ChBody * abody);

   void SetWindowSize(
         glm::ivec2 size) {
   }
//   void DrawString(
//         string line,
//         glm::vec3 pos);
   void RenderText(
         const std::string &str,
         FT_Face face,
         float x,
         float y,
         float sx,
         float sy);
   void DisplayHUD();
   void HandleInput(
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
         default:
            break;
      }
   }

   glm::ivec2 window_size;
   glm::ivec2 window_position;
   float window_aspect;
   int interval;

   ChOpenGLCamera render_camera;
   ChSystem * physics_system;

   ChOpenGLShader main_shader;
   ChOpenGLShader font_shader;

   ChOpenGLOBJ sphere;
   ChOpenGLOBJ box;
   ChOpenGLOBJ cylinder;
   ChOpenGLOBJ cone;

   ChOpenGLCloud cloud;

   int simulation_frame;  // The current frame number
   float simulation_h;  // The simulation step size
   float simulation_time;  // The current simulation time
   bool pause_sim;
   bool pause_vis;
   RenderMode render_mode;

   glm::mat4 model, view, projection, modelview;

   FT_Library ft;
   FT_Face face;
   GLuint vbo, vao;
   GLuint texture_handle;
   GLuint texture, sampler;

};
}
}

#endif  // END of CHOPENGLVIEWER_H
