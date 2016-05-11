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

#include "chrono_opengl/UI/ChOpenGLHUD.h"
#include "chrono_opengl/ChOpenGLMaterials.h"

#include "collision/ChCCollisionSystemBullet.h"
#include "lcp/ChLcpIterativeSolver.h"
// Includes that are generated at compile time
#include "resources/text_frag.h"
#include "resources/text_vert.h"

namespace chrono {
using namespace collision;
namespace opengl {
using namespace glm;

#define LEFT -.95f
#define TOP .95f
#define BOTTOM -.95f
#define RIGHT .55f
#define CENTER 0.0f
#define SPACING (sy * 35.0f)
#define SCALE .001f

ChOpenGLHUD::ChOpenGLHUD() : ChOpenGLBase() {
  time_total = time_text = time_geometry = 0;
  fps = 0;
}

bool ChOpenGLHUD::Initialize(ChOpenGLCamera* camera) {
  if (GLReturnedError("ChOpenGLHUD::Initialize - on entry"))
    return false;

  if (!font_shader.InitializeStrings("text", text_vert, text_frag)) {
    return 0;
  }

  text.Initialize(text_mat, &font_shader);
  render_camera = camera;
  return true;
}

void ChOpenGLHUD::Update(const glm::ivec2& window_size,
                         const float& dpi,
                         const float& frame_per_sec,
                         const float& t_geometry,
                         const float& t_text,
                         const float& t_total) {
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

  //  // set up the new viewport centered in the backbuffer
  //  int vp_x = (screen_width  / 2) - (width / 2);
  //  int vp_y = (screen_height / 2) - (height/ 2);

  aspect = float(width) / float(height);
  sx = SCALE;           // * (float)((float)(screen_width) / (float)virtual_width);
  sy = SCALE * aspect;  //(float)((float)(screen_height) / (float)virtual_height) ;

  //  z_x = vp_x/float(width)-1;
  //  z_y = vp_y/float(height)-1;
  //
  //
  //  std::cout<<window_size.x<<" "<<window_size.y<<" "<<vp_x<<" "<<vp_y<<" "<<z_x<<" "<<z_y<<" "<<height<<"
  //  "<<width<<std::endl;

  text.Update();

  time_geometry = t_geometry;
  time_text = t_text;
  time_total = t_total;
  fps = frame_per_sec;
}

void ChOpenGLHUD::TakeDown() {
  font_shader.TakeDown();
  text.TakeDown();
}

void ChOpenGLHUD::GenerateHelp() {
  text.Render("Press h to exit help", LEFT, TOP - SPACING * 0, sx, sy);
  text.Render("W: Forward", LEFT, TOP - SPACING * 1, sx, sy);
  text.Render("A: Strafe Left", LEFT, TOP - SPACING * 2, sx, sy);
  text.Render("S: Back", LEFT, TOP - SPACING * 3, sx, sy);
  text.Render("D: Strafe Right", LEFT, TOP - SPACING * 4, sx, sy);
  text.Render("Q: Down", LEFT, TOP - SPACING * 5, sx, sy);
  text.Render("E: Up", LEFT, TOP - SPACING * 6, sx, sy);

  text.Render("Mouse Look (Click and hold left mouse button)", LEFT, TOP - SPACING * 7, sx, sy);

  text.Render("1: Point Cloud (default)", LEFT, TOP - SPACING * 9, sx, sy);
  text.Render("2: Wireframe (slow)", LEFT, TOP - SPACING * 10, sx, sy);
  text.Render("3: Solid", LEFT, TOP - SPACING * 11, sx, sy);

  text.Render("C: Show/Hide Contacts (DVI only)", LEFT, TOP - SPACING * 13, sx, sy);

  text.Render("Space: Pause Simulation (not rendering)", LEFT, TOP - SPACING * 15, sx, sy);
  text.Render("P: Pause Rendering (not simulating)", LEFT, TOP - SPACING * 16, sx, sy);
  text.Render(".: Single Step ", LEFT, TOP - SPACING * 18, sx, sy);
  text.Render("B: Enable/Disable AABB ", LEFT, TOP - SPACING * 20, sx, sy);

  text.Render("Escape: Exit ", LEFT, TOP - SPACING * 30, sx, sy);
}

void ChOpenGLHUD::GenerateCamera() {
  sprintf(buffer, "Camera Pos :  [%04f, %04f, %04f]", render_camera->camera_position.x,
          render_camera->camera_position.y, render_camera->camera_position.z);
  text.Render(buffer, LEFT, TOP - SPACING * 1, sx, sy);
  sprintf(buffer, "Camera Look:  [%04f, %04f, %04f]", render_camera->camera_look_at.x, render_camera->camera_look_at.y,
          render_camera->camera_look_at.z);
  text.Render(buffer, LEFT, TOP - SPACING * 2, sx, sy);
  sprintf(buffer, "Camera Up  :  [%04f, %04f, %04f]", render_camera->camera_up.x, render_camera->camera_up.y,
          render_camera->camera_up.z);
  text.Render(buffer, LEFT, TOP - SPACING * 3, sx, sy);
}

void ChOpenGLHUD::GenerateSystem(ChSystem* physics_system) {
  int num_shapes = 0;
  int num_bodies = 0;
  int num_contacts = 0;
  int num_bilaterals = 0;
  double timer_step = physics_system->GetTimerStep();
  double timer_collision_broad = physics_system->GetTimerCollisionBroad();
  double timer_collision_narrow = physics_system->GetTimerCollisionNarrow();
  double timer_lcp = physics_system->GetTimerLcp();
  double timer_update = physics_system->GetTimerUpdate();
//  if (ChSystemParallel* parallel_system = dynamic_cast<ChSystemParallel*>(physics_system)) {
//    num_shapes = parallel_system->data_manager->num_rigid_shapes;
//    num_bodies = parallel_system->data_manager->num_rigid_bodies + parallel_system->GetNphysicsItems();
//    num_contacts = parallel_system->GetNumContacts();
//    num_bilaterals = parallel_system->data_manager->num_bilaterals;
//  } else
  {
    ChCollisionSystemBullet* collision_system = (ChCollisionSystemBullet*)physics_system->GetCollisionSystem();
    num_shapes = collision_system->GetBulletCollisionWorld()->getNumCollisionObjects();
    num_bodies = physics_system->GetNbodiesTotal() + physics_system->GetNphysicsItems();
    num_contacts = physics_system->GetNcontacts();
  }

  int average_contacts_per_body = num_bodies > 0 ? num_contacts / num_bodies : 0;

  sprintf(buffer, "MODEL INFO");
  text.Render(buffer, RIGHT, TOP - SPACING * 0, sx, sy);
  sprintf(buffer, "BODIES     %04d", num_bodies);
  text.Render(buffer, RIGHT, TOP - SPACING * 1, sx, sy);

  sprintf(buffer, "AABB       %04d", num_shapes);
  text.Render(buffer, RIGHT, TOP - SPACING * 2, sx, sy);

  sprintf(buffer, "CONTACTS   %04d", num_contacts);
  text.Render(buffer, RIGHT, TOP - SPACING * 3, sx, sy);
  sprintf(buffer, "BILATERALS %04d", num_bilaterals);
  text.Render(buffer, RIGHT, TOP - SPACING * 4, sx, sy);

  sprintf(buffer, "TIMING INFO");
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 11, sx, sy);
  sprintf(buffer, "STEP     %04f", timer_step);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 10, sx, sy);
  sprintf(buffer, "BROAD    %04f", timer_collision_broad);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 9, sx, sy);
  sprintf(buffer, "NARROW   %04f", timer_collision_narrow);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 8, sx, sy);
  sprintf(buffer, "SOLVE    %04f", timer_lcp);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 7, sx, sy);
  sprintf(buffer, "UPDATE   %04f", timer_update);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 6, sx, sy);
}

void ChOpenGLHUD::GenerateSolver(ChSystem* physics_system) {
    int iters = ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetTotalIterations();
    const std::vector<double>& vhist =
        ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetViolationHistory();
    const std::vector<double>& dhist =
        ((ChLcpIterativeSolver*)(physics_system->GetLcpSolverSpeed()))->GetDeltalambdaHistory();
    double residual = vhist.size() > 0 ? vhist.back() : 0.0;
    double dlambda = dhist.size() > 0 ? dhist.back() : 0.0;

    sprintf(buffer, "SOLVER INFO");
    text.Render(buffer, RIGHT, TOP - SPACING * 6, sx, sy);
    sprintf(buffer, "ITERS    %04d", iters);
    text.Render(buffer, RIGHT, TOP - SPACING * 7, sx, sy);
    sprintf(buffer, "RESIDUAL %04f", residual);
    text.Render(buffer, RIGHT, TOP - SPACING * 8, sx, sy);
    sprintf(buffer, "CORRECT  %04f", dlambda);
    text.Render(buffer, RIGHT, TOP - SPACING * 9, sx, sy);
}

void ChOpenGLHUD::GenerateCD(ChSystem* physics_system) {
//  if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
//    int3 bins_per_axis = parallel_sys->data_manager->settings.collision.bins_per_axis;
//    real3 bin_size_vec = 1.0 / parallel_sys->data_manager->measures.collision.bin_size_vec;
//    real3 min_pt = parallel_sys->data_manager->measures.collision.min_bounding_point;
//    real3 max_pt = parallel_sys->data_manager->measures.collision.max_bounding_point;
//    real3 center = (min_pt + max_pt) * .5;
//    sprintf(buffer, "COLLISION INFO");
//    text.Render(buffer, RIGHT, TOP - SPACING * 11, sx, sy);
//    sprintf(buffer, "DIMS  [%d,%d,%d]", bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
//    text.Render(buffer, RIGHT, TOP - SPACING * 12, sx, sy);
//    sprintf(buffer, "SX    %f", bin_size_vec.x);
//    text.Render(buffer, RIGHT, TOP - SPACING * 13, sx, sy);
//    sprintf(buffer, "SY    %f", bin_size_vec.y);
//    text.Render(buffer, RIGHT, TOP - SPACING * 14, sx, sy);
//    sprintf(buffer, "SZ    %f", bin_size_vec.z);
//    text.Render(buffer, RIGHT, TOP - SPACING * 15, sx, sy);
//    sprintf(buffer, "RIGID %d", parallel_sys->data_manager->num_rigid_contacts);
//    text.Render(buffer, RIGHT, TOP - SPACING * 16, sx, sy);
//  } else {
//    // ChCollisionSystemBullet* collision_system = (ChCollisionSystemBullet*)physics_system->GetCollisionSystem();
//    // btDbvtBroadphase * broadphase = (btDbvtBroadphase* )
//    // collision_system->GetBulletCollisionWorld()->getBroadphase();
//  }
}

void ChOpenGLHUD::GenerateRenderer() {
  sprintf(buffer, "RENDER INFO");
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 4, sx, sy);
  sprintf(buffer, "GEOMETRY %04f", time_geometry);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 3, sx, sy);
  sprintf(buffer, "TEXT     %04f", time_text);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 2, sx, sy);
  sprintf(buffer, "TOTAL    %04f", time_total);
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 1, sx, sy);
  sprintf(buffer, "FPS      %04d", int(fps));
  text.Render(buffer, RIGHT, BOTTOM + SPACING * 0, sx, sy);
}

void ChOpenGLHUD::GenerateStats(ChSystem* physics_system) {
  sprintf(buffer, "Press h for help");
  text.Render(buffer, CENTER, TOP, sx, sy);

  sprintf(buffer, "TIME:  %04f  | %04f", physics_system->GetChTime(), physics_system->GetStep());
  text.Render(buffer, LEFT, TOP, sx, sy);

  GenerateCamera();
  GenerateSystem(physics_system);
  GenerateSolver(physics_system);
  GenerateCD(physics_system);
  GenerateRenderer();
}
void ChOpenGLHUD::GenerateExtraStats(ChSystem* physics_system) {
//  if (ChSystemParallelDVI* parallel_sys = dynamic_cast<ChSystemParallelDVI*>(physics_system)) {
//    ChTimerParallel& system_timer = parallel_sys->data_manager->system_timer;
//
//    sprintf(buffer, "Compute N:  %04f", system_timer.GetTime("ChLcpSolverParallel_N"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 6, sx, sy);
//
//    sprintf(buffer, "Compute R:  %04f", system_timer.GetTime("ChLcpSolverParallel_R"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 5, sx, sy);
//
//    sprintf(buffer, "Compute E:  %04f", system_timer.GetTime("ChLcpSolverParallel_E"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 4, sx, sy);
//
//    sprintf(buffer, "Compute D:  %04f", system_timer.GetTime("ChLcpSolverParallel_D"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 3, sx, sy);
//
//    sprintf(buffer, "Solve:  %04f", system_timer.GetTime("ChSolverParallel_Solve"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 2, sx, sy);
//
//    sprintf(buffer, "ShurProduct:  %04f [%d]", system_timer.GetTime("ShurProduct"),
//            system_timer.GetRuns("ShurProduct"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 1, sx, sy);
//
//    sprintf(buffer, "Project:  %04f [%d]", system_timer.GetTime("ChSolverParallel_Project"),
//            system_timer.GetRuns("ChSolverParallel_Project"));
//    text.Render(buffer, LEFT, BOTTOM + SPACING * 0, sx, sy);
//
//    //    sprintf(buffer, "TimerA:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverA"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 9, sx, sy);
//    //    sprintf(buffer, "TimerB:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverB"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 8, sx, sy);
//    //    sprintf(buffer, "TimerC:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverC"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 7, sx, sy);
//    //    sprintf(buffer, "TimerD:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverD"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 6, sx, sy);
//    //    sprintf(buffer, "TimerE:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverE"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 5, sx, sy);
//    //    sprintf(buffer, "TimerF:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverF"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 4, sx, sy);
//    //    sprintf(buffer, "TimerG:  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_solverG"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 3, sx, sy);
//    //    sprintf(buffer, "Shur A:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_shurA"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 2, sx, sy);
//    //    sprintf(buffer, "Shur B:  %04f", parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_shurB"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 1, sx, sy);
//    //    sprintf(buffer, "Proj  :  %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("ChSolverParallel_Project"));
//    //    text.Render(buffer, -.95, -0.925 + SPACING * 0, sx, sy);
//    //    float posx = -.6;
//    //    sprintf(buffer, "B_Initial : %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_Init"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 9, sx, sy);
//    //    sprintf(buffer, "B_AABBBINC: %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_BIN_Count"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 8, sx, sy);
//    //    sprintf(buffer, "B_AABBBINS: %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_BIN_Store"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 7, sx, sy);
//    //    sprintf(buffer, "B_SORT_RED: %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("Broadphase_SortReduce"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 6, sx, sy);
//    //    sprintf(buffer, "BAABBAABBC: %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_AABB_Count"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 5, sx, sy);
//    //    sprintf(buffer, "BAABBAABBS: %04f",
//    //    parallel_sys->data_manager->system_timer.GetTime("Broadphase_AABB_AABB_Store"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 4, sx, sy);
//    //    sprintf(buffer, "B_POST    : %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase_Post"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 3, sx, sy);
//    //    sprintf(buffer, "BROADPHASE: %04f", parallel_sys->data_manager->system_timer.GetTime("Broadphase"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 2, sx, sy);
//    //
//    //    posx = -.6 + .45;
//    //    sprintf(buffer, "BuildD : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildD"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 9, sx, sy);
//    //    sprintf(buffer, "BuildDA: %04f", parallel_sys->data_manager->system_timer.GetTime("BuildDAllocate"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 8, sx, sy);
//    //    sprintf(buffer, "BuildDC: %04f", parallel_sys->data_manager->system_timer.GetTime("BuildDCompute"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 7, sx, sy);
//    //    sprintf(buffer, "BuildE : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildE"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 6, sx, sy);
//    //    sprintf(buffer, "BuildN : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildN"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 5, sx, sy);
//    //    sprintf(buffer, "BuildM : %04f", parallel_sys->data_manager->system_timer.GetTime("BuildM"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 4, sx, sy);
//    //    sprintf(buffer, "Buildb : %04f", parallel_sys->data_manager->system_timer.GetTime("Buildb"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 3, sx, sy);
//    //    sprintf(buffer, "SchurP : %04f", parallel_sys->data_manager->system_timer.GetTime("ShurProduct"));
//    //    text.Render(buffer, posx, -0.925 + SPACING * 2, sx, sy);
//  }
}
void ChOpenGLHUD::Draw() {
  text.Draw();
}
}
}
