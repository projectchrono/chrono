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
// Class that renders the text and other UI elements
// =============================================================================

#include "chrono/solver/ChIterativeSolverVI.h"

#include "chrono/ChConfig.h"

#ifdef CHRONO_MULTICORE
    #include "chrono_multicore/physics/ChSystemMulticore.h"
    #include "chrono_multicore/ChDataManager.h"
    #include "chrono_multicore/physics/Ch3DOFContainer.h"
#endif

// Includes that are generated at compile time
#include "resources/text_frag.h"
#include "resources/text_vert.h"
#include "resources/bar_frag.h"
#include "resources/bar_vert.h"

#include "chrono_opengl/ChOpenGLMaterials.h"
#include "chrono_opengl/UI/ChOpenGLStats.h"

namespace chrono {
using namespace collision;
namespace opengl {
using namespace glm;

// --------------------------------------------------------------------------------------------------------------------

ChOpenGLStats::ChOpenGLStats() : time_total(0), time_text(0), time_geometry(0), fps(0) {
    screen.LEFT = -0.98f;
    screen.TOP = 0.95f;
    screen.BOTTOM = -0.95f;
    screen.RIGHT = 0.55f;
    screen.CENTER = 0.0f;
    screen.SCALE = 0.0007f;
}

bool ChOpenGLStats::Initialize(ChOpenGLCamera* camera) {
    if (this->GLReturnedError("ChOpenGLStatsDefault::Initialize - on entry"))
        return false;

    if (!font_shader.InitializeStrings("text", text_vert, text_frag)) {
        return 0;
    }
    if (!bar_shader.InitializeStrings("bar", bar_vert, bar_frag)) {
        return 0;
    }

    text.Initialize(text_mat, &font_shader);
    bars.Initialize(&bar_shader);

    render_camera = camera;

    return true;
}

void ChOpenGLStats::Update(const glm::ivec2& window_size,
                           const double& dpi,
                           const double& frame_per_sec,
                           const double& t_geometry,
                           const double& t_text,
                           const double& t_total) {
    int screen_width = window_size.x;
    int screen_height = window_size.y;

    float virtual_width = 1920;
    float virtual_height = 1080;

    float targetAspectRatio = virtual_width / virtual_height;

    // figure out the largest area that fits in this resolution at the desired aspect ratio
    int width = screen_width;
    int height = (int)(width / targetAspectRatio + 0.5f);

    if (height > screen_height) {
        // std::cout<<"It doesn't fit our height, we must switch to pillarbox then "<<height<<std::endl;;
        height = screen_height;
        width = (int)(height * targetAspectRatio + 0.5f);
    }

    // Set screen locations
    auto aspect = (1.0f * width) / height;

    screen.SX = screen.SCALE;
    screen.SY = screen.SX * aspect;
    screen.SPACING = screen.SY * 45.0f;

    text.Update();
    bars.Clear();

    time_geometry = t_geometry;
    time_text = t_text;
    time_total = t_total;
    fps = frame_per_sec;
}

void ChOpenGLStats::TakeDown() {
    font_shader.TakeDown();
    bar_shader.TakeDown();
    text.TakeDown();
}

void ChOpenGLStats::Render() {
    // bars.Update();
    text.Draw();
    // bars.Draw();
}

// --------------------------------------------------------------------------------------------------------------------

ChOpenGLStatsDefault::ChOpenGLStatsDefault() : ChOpenGLStats() {}

bool ChOpenGLStatsDefault::Initialize(ChOpenGLCamera* camera) {
    return ChOpenGLStats::Initialize(camera);
}

void ChOpenGLStatsDefault::GenerateStats(ChSystem& sys) {
    sprintf(buffer, "Press h for help");
    text.Render(buffer, screen.CENTER, screen.TOP, screen.SX, screen.SY);

    sprintf(buffer, "TIME:  %04f  [%04f]", sys.GetChTime(), sys.GetStep());
    text.Render(buffer, screen.LEFT, screen.TOP, screen.SX, screen.SY);

    GenerateCamera();
    GenerateSystem(sys);
    GenerateSolver(sys);
    GenerateCD(sys);
    GenerateRenderer();
}

void ChOpenGLStatsDefault::GenerateHelp() {
    text.Render("Press h to exit help", screen.LEFT, screen.TOP - screen.SPACING * 0, screen.SX, screen.SY);
    text.Render("W: Forward", screen.LEFT, screen.TOP - screen.SPACING * 1, screen.SX, screen.SY);
    text.Render("A: Strafe Left", screen.LEFT, screen.TOP - screen.SPACING * 2, screen.SX, screen.SY);
    text.Render("S: Back", screen.LEFT, screen.TOP - screen.SPACING * 3, screen.SX, screen.SY);
    text.Render("D: Strafe Right", screen.LEFT, screen.TOP - screen.SPACING * 4, screen.SX, screen.SY);
    text.Render("Q: Down", screen.LEFT, screen.TOP - screen.SPACING * 5, screen.SX, screen.SY);
    text.Render("E: Up", screen.LEFT, screen.TOP - screen.SPACING * 6, screen.SX, screen.SY);

    text.Render("Mouse Look (Click and hold left mouse button)", screen.LEFT, screen.TOP - screen.SPACING * 7,
                screen.SX, screen.SY);

    text.Render("1: Point Cloud (default)", screen.LEFT, screen.TOP - screen.SPACING * 9, screen.SX, screen.SY);
    text.Render("2: Wireframe (slow)", screen.LEFT, screen.TOP - screen.SPACING * 10, screen.SX, screen.SY);
    text.Render("3: Solid", screen.LEFT, screen.TOP - screen.SPACING * 11, screen.SX, screen.SY);

    text.Render("C: Show/Hide Contacts (NSC only)", screen.LEFT, screen.TOP - screen.SPACING * 13, screen.SX,
                screen.SY);

    text.Render("Space: Pause Simulation (not rendering)", screen.LEFT, screen.TOP - screen.SPACING * 15, screen.SX,
                screen.SY);
    text.Render("P: Pause Rendering (not simulating)", screen.LEFT, screen.TOP - screen.SPACING * 16, screen.SX,
                screen.SY);
    text.Render(".: Single Step ", screen.LEFT, screen.TOP - screen.SPACING * 18, screen.SX, screen.SY);
    text.Render("B: Enable/Disable AABB ", screen.LEFT, screen.TOP - screen.SPACING * 20, screen.SX, screen.SY);

    text.Render("Escape: Exit ", screen.LEFT, screen.TOP - screen.SPACING * 30, screen.SX, screen.SY);
}

void ChOpenGLStatsDefault::GenerateCamera() {
    sprintf(buffer, "CAM POS [%07.5f, %07.5f, %07.5f]", render_camera->camera_position.x,
            render_camera->camera_position.y, render_camera->camera_position.z);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 1, screen.SX, screen.SY);
    sprintf(buffer, "CAM EYE [%07.5f, %07.5f, %07.5f]", render_camera->camera_look_at.x,
            render_camera->camera_look_at.y, render_camera->camera_look_at.z);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 2, screen.SX, screen.SY);
    sprintf(buffer, "CAM UPV [%07.5f, %07.5f, %07.5f]", render_camera->camera_up.x, render_camera->camera_up.y,
            render_camera->camera_up.z);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 3, screen.SX, screen.SY);
    sprintf(buffer, "--------------------------------");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 4, screen.SX, screen.SY);
}

void ChOpenGLStatsDefault::GenerateSystem(ChSystem& sys) {
    int num_shapes = 0;
    int num_rigid_bodies = 0;
    int num_fluid_bodies = 0;
    int num_contacts = 0;
    int num_bilaterals = 0;
    double timer_step = sys.GetTimerStep();
    double timer_collision_broad = sys.GetTimerCollisionBroad();
    double timer_collision_narrow = sys.GetTimerCollisionNarrow();
    double timer_lcp = sys.GetTimerAdvance();
    double timer_update = sys.GetTimerUpdate();

#ifdef CHRONO_MULTICORE
    auto parallel_system = dynamic_cast<ChSystemMulticore*>(&sys);
    if (parallel_system) {
        num_shapes =
            parallel_system->data_manager->cd_data->num_rigid_shapes + parallel_system->data_manager->num_fluid_bodies;
        num_rigid_bodies = parallel_system->data_manager->num_rigid_bodies + parallel_system->GetNphysicsItems();
        num_fluid_bodies = parallel_system->data_manager->num_fluid_bodies;
        num_contacts = parallel_system->GetNcontacts();
        num_bilaterals = parallel_system->data_manager->num_bilaterals;
    }
    double left_b = screen.LEFT + screen.RIGHT;
    double right_b = -screen.LEFT;
    double thick = 0.05;
    double broad_v = glm::mix(left_b, right_b, timer_collision_broad / timer_step);
    double narrow_v = glm::mix(left_b, right_b, (timer_collision_broad + timer_collision_narrow) / timer_step);
    double lcp_v = glm::mix(left_b, right_b, (timer_collision_broad + timer_collision_narrow + timer_lcp) / timer_step);

    bars.AddBar(left_b, broad_v, screen.BOTTOM + thick, screen.BOTTOM, ColorConverter(0x5D9CEC));
    bars.AddBar(broad_v, narrow_v, screen.BOTTOM + thick, screen.BOTTOM, ColorConverter(0x48CFAD));
    bars.AddBar(narrow_v, lcp_v, screen.BOTTOM + thick, screen.BOTTOM, ColorConverter(0xA0D468));
    bars.AddBar(lcp_v, right_b, screen.BOTTOM + thick, screen.BOTTOM, ColorConverter(0xFFCE54));

    if (parallel_system) {
        real build_m = parallel_system->data_manager->system_timer.GetTime("ChIterativeSolverMulticore_M");
        real build_d = parallel_system->data_manager->system_timer.GetTime("ChIterativeSolverMulticore_D");
        real build_e = parallel_system->data_manager->system_timer.GetTime("ChIterativeSolverMulticore_E");
        real build_r = parallel_system->data_manager->system_timer.GetTime("ChIterativeSolverMulticore_R");
        real build_n = parallel_system->data_manager->system_timer.GetTime("ChIterativeSolverMulticore_N");
        real stab = parallel_system->data_manager->system_timer.GetTime("ChIterativeSolverMulticore_Stab");
        real shur = parallel_system->data_manager->system_timer.GetTime("ShurProduct");

        real number = build_m;
        real build_m_v = glm::mix(left_b, right_b, number / timer_lcp);

        number += build_d;
        real build_d_v = glm::mix(left_b, right_b, number / timer_lcp);

        number += build_e;
        real build_e_v = glm::mix(left_b, right_b, number / timer_lcp);

        number += build_r;
        real build_r_v = glm::mix(left_b, right_b, number / timer_lcp);

        number += build_n;
        real build_n_v = glm::mix(left_b, right_b, number / timer_lcp);

        number += stab;
        real stab_v = glm::mix(left_b, right_b, number / timer_lcp);

        number += shur;
        real shur_v = glm::mix(left_b, right_b, number / timer_lcp);

        bars.AddBar(left_b, build_m_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2, ColorConverter(0x5D9CEC));
        bars.AddBar(build_m_v, build_d_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2,
                    ColorConverter(0x48CFAD));
        bars.AddBar(build_d_v, build_e_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2,
                    ColorConverter(0xA0D468));
        bars.AddBar(build_e_v, build_r_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2,
                    ColorConverter(0xFFCE54));
        bars.AddBar(build_r_v, build_n_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2,
                    ColorConverter(0xFC6E51));
        bars.AddBar(build_n_v, stab_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2, ColorConverter(0xED5565));
        bars.AddBar(stab_v, shur_v, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2, ColorConverter(0xAC92EC));
        bars.AddBar(shur_v, right_b, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2, ColorConverter(0xEC87C0));
        // bars.AddBar(stab_v, right_b, screen.BOTTOM + thick * 3, screen.BOTTOM + thick * 2, normalize(glm::vec3(149,
        // 165, 166)));
    }
#endif

    sprintf(buffer, "MODEL INFO");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 5, screen.SX, screen.SY);
    sprintf(buffer, "BODIES R,F %04d, %04d", num_rigid_bodies, num_fluid_bodies);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 6, screen.SX, screen.SY);
    sprintf(buffer, "AABB       %04d", num_shapes);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 7, screen.SX, screen.SY);
    sprintf(buffer, "CONTACTS   %04d", num_contacts);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 8, screen.SX, screen.SY);
    sprintf(buffer, "BILATERALS %04d", num_bilaterals);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 9, screen.SX, screen.SY);

    sprintf(buffer, "--------------------------------");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 10, screen.SX, screen.SY);

    sprintf(buffer, "TIMING INFO");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 22, screen.SX, screen.SY);
    sprintf(buffer, "STEP     %04f", timer_step);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 23, screen.SX, screen.SY);
    sprintf(buffer, "BROAD    %04f", timer_collision_broad);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 24, screen.SX, screen.SY);
    sprintf(buffer, "NARROW   %04f", timer_collision_narrow);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 25, screen.SX, screen.SY);
    sprintf(buffer, "SOLVE    %04f", timer_lcp);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 26, screen.SX, screen.SY);
    sprintf(buffer, "UPDATE   %04f", timer_update);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 27, screen.SX, screen.SY);
    sprintf(buffer, "--------------------------------");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 28, screen.SX, screen.SY);
}

void ChOpenGLStatsDefault::GenerateSolver(ChSystem& sys) {
    double iters = std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver())->GetIterations();
    const std::vector<double>& vhist =
        std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver())->GetViolationHistory();
    const std::vector<double>& dhist =
        std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver())->GetDeltalambdaHistory();
    double residual = vhist.size() > 0 ? vhist.back() : 0.0;
    double dlambda = dhist.size() > 0 ? dhist.back() : 0.0;

    sprintf(buffer, "SOLVER INFO");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 11, screen.SX, screen.SY);
    sprintf(buffer, "ITERS    %04d", int(iters));
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 12, screen.SX, screen.SY);
    sprintf(buffer, "RESIDUAL %04f", residual);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 13, screen.SX, screen.SY);
    sprintf(buffer, "CORRECT  %04f", dlambda);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 14, screen.SX, screen.SY);
    sprintf(buffer, "--------------------------------");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 15, screen.SX, screen.SY);
}

void ChOpenGLStatsDefault::GenerateCD(ChSystem& sys) {
#ifdef CHRONO_MULTICORE
    if (ChSystemMulticore* parallel_sys = dynamic_cast<ChSystemMulticore*>(&sys)) {
        vec3 bins_per_axis = parallel_sys->data_manager->settings.collision.bins_per_axis;
        real3 bin_size_vec = 1.0 / parallel_sys->data_manager->measures.collision.bin_size;
        real3 min_pt = parallel_sys->data_manager->measures.collision.min_bounding_point;
        real3 max_pt = parallel_sys->data_manager->measures.collision.max_bounding_point;
        sprintf(buffer, "COLLISION INFO");
        text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 16, screen.SX, screen.SY);
        sprintf(buffer, "DIMS  [%d,%d,%d]", bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
        text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 17, screen.SX, screen.SY);
        sprintf(buffer, "SIZE  [%07.5f,%07.5f,%07.5f]", bin_size_vec.x, bin_size_vec.y, bin_size_vec.z);
        text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 18, screen.SX, screen.SY);

        sprintf(buffer, "R: %d B: %d F: %d", parallel_sys->data_manager->cd_data->num_rigid_contacts,
                parallel_sys->data_manager->cd_data->num_rigid_fluid_contacts,
                parallel_sys->data_manager->cd_data->num_fluid_contacts);
        text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 20, screen.SX, screen.SY);
        sprintf(buffer, "--------------------------------");
        text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 21, screen.SX, screen.SY);
    }
#endif
}

void ChOpenGLStatsDefault::GenerateRenderer() {
    sprintf(buffer, "RENDER INFO");
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 29, screen.SX, screen.SY);
    sprintf(buffer, "GEOMETRY %04f", time_geometry);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 30, screen.SX, screen.SY);
    sprintf(buffer, "TEXT     %04f", time_text);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 31, screen.SX, screen.SY);
    sprintf(buffer, "TOTAL    %04f", time_total);
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 32, screen.SX, screen.SY);
    sprintf(buffer, "FPS      %04d", int(fps));
    text.Render(buffer, screen.LEFT, screen.TOP - screen.SPACING * 33, screen.SX, screen.SY);
}

}  // namespace opengl
}  // namespace chrono
