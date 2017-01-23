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
// Authors: Hammad Mazhar
// =============================================================================
// OpenGL viewer, this class draws the system to the screen and handles input
// =============================================================================

#pragma once

#include "chrono_opengl/core/ChApiOpenGL.h"
#include "chrono_opengl/core/ChOpenGLBase.h"
#include "chrono_opengl/ChOpenGLCamera.h"
#include "chrono_opengl/core/ChOpenGLShader.h"
#include "chrono_opengl/shapes/ChOpenGLCloud.h"
#include "chrono_opengl/shapes/ChOpenGLWires.h"
#include "chrono_opengl/shapes/ChOpenGLText.h"
#include "chrono_opengl/shapes/ChOpenGLOBJ.h"
#include "chrono_opengl/UI/ChOpenGLContacts.h"
#include "chrono_opengl/UI/ChOpenGLHUD.h"
#include "chrono_opengl/UI/ChOpenGLGraphs.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChTimer.h"

//#include "chrono_parallel/physics/ChSystemParallel.h"
#include <glfw3.h>

namespace chrono {
namespace opengl {

/// @addtogroup opengl
/// @{

enum RenderMode { POINTS, WIREFRAME, SOLID };

/// OpenGL viewer, this class draws the system to the screen and handles input.
class CH_OPENGL_API ChOpenGLViewer : public ChOpenGLBase {
  public:
    ChOpenGLViewer(ChSystem* system);
    ~ChOpenGLViewer();
    void TakeDown();
    bool Initialize();
    bool Update(double time_step);
    void Render();
    void DrawObject(std::shared_ptr<ChBody> abody);
    void DisplayHUD();
    void RenderContacts();
    void RenderAABB();
    void RenderGrid();
    void RenderFluid();
    void RenderFEA();
    void RenderPlots();
    void HandleInput(unsigned char key, int x, int y);

    glm::ivec2 window_size;
    glm::ivec2 window_position;
    glm::ivec2 window_physical_size;
    double dpi;
    float window_aspect;
    int interval;

    ChOpenGLCamera render_camera, ortho_camera;
    ChSystem* physics_system;

    ChOpenGLShader main_shader;
    ChOpenGLShader cloud_shader;
    ChOpenGLShader dot_shader;
    ChOpenGLShader sphere_shader;

    ChOpenGLOBJ sphere;
    ChOpenGLOBJ box;
    ChOpenGLOBJ cylinder;
    ChOpenGLOBJ cone;

    ChOpenGLCloud cloud;
    ChOpenGLCloud fluid;
    ChOpenGLWires grid;

    ChOpenGLWires mpm_grid;
    ChOpenGLCloud mpm_node;

    ChOpenGLCloud fea_nodes;
    ChOpenGLWires fea_elements;

    ChOpenGLWires line_path;

    ChOpenGLContacts contact_renderer;
    ChOpenGLHUD HUD_renderer;
    ChOpenGLGraphs graph_renderer;

    std::vector<glm::vec3> cloud_data;
    std::vector<glm::vec3> fluid_data;
    std::vector<glm::vec3> fea_node_data;
    std::vector<glm::vec3> fea_element_data;
    std::vector<glm::vec3> grid_data;
    std::vector<glm::vec3> mpm_grid_data;
    std::vector<glm::vec3> mpm_node_data;
    std::vector<glm::vec3> line_path_data;

    int simulation_frame;    // The current frame number
    double simulation_h;     // The simulation step size
    double simulation_time;  // The current simulation time
    bool pause_sim;
    bool pause_vis;
    bool single_step;
    bool view_contacts, view_help, view_aabb, view_grid, view_info;
    bool use_vsync;
    RenderMode render_mode;

    ChTimer<> timer_render;
    ChTimer<> timer_text;
    ChTimer<> timer_geometry;

    glm::mat4 model, view, projection, modelview;

    double old_time, current_time, time_total, time_text, time_geometry;
    double fps;

    std::vector<glm::mat4> model_box;
    std::vector<glm::mat4> model_sphere;
    std::vector<glm::mat4> model_cylinder;
    std::vector<glm::mat4> model_cone;
    std::map<std::string, ChOpenGLMesh> obj_files;
    std::map<std::string, std::vector<glm::mat4> > model_obj;
};

/// @} opengl
}
}
