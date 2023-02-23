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

#include "chrono/core/ChTimer.h"

#include "chrono_opengl/core/ChApiOpenGL.h"
#include "chrono_opengl/core/ChOpenGLBase.h"
#include "chrono_opengl/ChOpenGLCamera.h"
#include "chrono_opengl/core/ChOpenGLShader.h"
#include "chrono_opengl/shapes/ChOpenGLCloud.h"
#include "chrono_opengl/shapes/ChOpenGLWires.h"
#include "chrono_opengl/shapes/ChOpenGLText.h"
#include "chrono_opengl/shapes/ChOpenGLOBJ.h"
#include "chrono_opengl/UI/ChOpenGLContacts.h"
#include "chrono_opengl/UI/ChOpenGLStats.h"
#include "chrono_opengl/UI/ChOpenGLGraphs.h"

#include <GLFW/glfw3.h>

namespace chrono {
namespace opengl {

class ChVisualSystemOpenGL;

/// @addtogroup opengl_module
/// @{

enum RenderMode { POINTS, WIREFRAME, SOLID };

/// OpenGL viewer, this class draws the system to the screen and handles input.
class CH_OPENGL_API ChOpenGLViewer : public ChOpenGLBase {
  public:
    ChOpenGLViewer(ChVisualSystemOpenGL* vis);
    ~ChOpenGLViewer();
    bool Initialize();
    void TakeDown();
    void Render(bool render_stats);
    void DrawVisualModel(std::shared_ptr<ChPhysicsItem> item);
    void RenderStats();
    void RenderContacts();
    void RenderAABB();
    void RenderGrid();
    void RenderFluid();
    void RenderParticles();
    void RenderFEA();
    void RenderPlots();
    void HandleInput(unsigned char key, int x, int y);

    ChVisualSystemOpenGL* m_vis;

    glm::ivec2 window_size;
    glm::ivec2 window_position;
    glm::ivec2 window_physical_size;
    double dpi;
    float window_aspect;
    int interval;

    ChOpenGLCamera render_camera;
    ChOpenGLCamera ortho_camera;

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

    ChOpenGLCloud particles;

    ChOpenGLWires mpm_grid;
    ChOpenGLCloud mpm_node;

    ChOpenGLCloud fea_nodes;
    ChOpenGLWires fea_elements;

    ChOpenGLWires line_path;

    ChOpenGLContacts contact_renderer;
    ChOpenGLGraphs graph_renderer;

    std::vector<glm::vec3> cloud_data;
    std::vector<glm::vec3> fluid_data;
    std::vector<glm::vec3> fea_node_data;
    std::vector<glm::vec3> fea_element_data;
    std::vector<glm::vec3> grid_data;
    std::vector<glm::vec3> mpm_grid_data;
    std::vector<glm::vec3> mpm_node_data;
    std::vector<glm::vec3> line_path_data;
    std::vector<glm::vec3> particle_data;

    bool view_contacts, view_help, view_aabb, view_grid;
    bool use_vsync;
    RenderMode render_mode;

    RenderMode particle_render_mode;
    float particle_radius;

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

/// @} opengl_module

}
}
