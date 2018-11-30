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
// Authors: Asher Elmquist
// =============================================================================
//
// Base class for the Chrono_Godot application
//
// =============================================================================
#include <main/main.h>

#ifdef WINDOWS_ENABLED
//#undef CONNECT_DEFERRED
// extern HINSTANCE godot_hinstance;
//#include <platform/windows/os_windows.h>
// extern HINSTANCE godot_hinstance = NULL;

#else
#include <platform/x11/os_x11.h>
#endif

#include "chrono_godot/ChGodotApp.h"

namespace chrono {
namespace gd {

ChGodotApp::ChGodotApp(ChSystem* physical_system, int width, int height, bool displayWindow) {
#ifdef WINDOWS_ENABLED
    HINSTANCE g = NULL;
    m_os = std::make_shared<OS_Windows>(g);
#else
    m_os = std::make_shared<OS_X11>();
    std::cout << "Godot OS_X11 created\n";
#endif

    bool success = Initialize();

    std::cout << "Finished initializing Godot\n";

    if (success) {
        // save handle to the scene
        std::cout << "creating scene\n";
        m_scene = std::make_shared<ChGdScene>();
        // initialize the scene
        std::cout << "Initializing scene\n";
        m_scene->Initialize(m_os->get_main_loop(), physical_system);
        // save handle to the window
        // m_window = m_os->GetWindow();
        std::cout << "completed init scene\n";
    }

    m_system = physical_system;
}

ChGodotApp::~ChGodotApp() {
    // clean up the main loop
    m_os->get_main_loop()->finish();
    Main::cleanup();
}

bool ChGodotApp::Initialize() {
    std::cout << "Initializing Godot...\n";
    // set the location of the empty scene
    const std::string path_arg = "--path";

    // this scene is bare-bones so we can add whatever we want
    const std::string path_loc = GetChronoDataFile("/godot/empty_scene");
    std::cout << "Scene path: " << path_loc << std::endl;

    // use this scene to see if we can render correctly - it has example objects/materials
    // std::string path_loc = "/home/asher/godot/godot-demo-projects/3d/material_testers/";

    const std::string first = "ChGodotApp";  //"./demo_GD_collisionNSC";

    char char_path_arg[7];
    char char_path_loc[255];
    char char_first[11];

    strcpy(char_path_arg, path_arg.c_str());
    strcpy(char_path_loc, path_loc.c_str());
    strcpy(char_first, first.c_str());

    char* m_argv[] = {char_first, char_path_arg, char_path_loc};
    int m_argc = 3;

    // older version
    // std::string first = "ChGodotApp";  //"./demo_GD_collisionNSC";
    //
    // char char_path_arg[path_arg.length() + 1];
    // char char_path_loc[path_loc.length() + 1];
    // char char_first[first.length() + 1];
    //
    // strcpy(char_path_arg, path_arg.c_str());
    // strcpy(char_path_loc, path_loc.c_str());
    // strcpy(char_first, first.c_str());
    //
    // char* m_argv[] = {char_first, char_path_arg, char_path_loc};
    // int m_argc = 3;

    std::cout << "Setting up Main\n";
    Error err = Main::setup(m_argv[0], m_argc - 1, &m_argv[1], false);
    // Error err = Main::setup("ChGodotApp", m_argc - 1, &m_argv[1]);

    std::cout << "Setup1 complete\n";
    Error err2 = Main::setup2();
    std::cout << "Setup2 complete\n";

    if (err != OK) {
        // free(cwd);
        std::cout << "ERROR WHILE SETTING UP MAIN\n";
        return 255;
    }

    /*
       char* m_argv[] = {"./main", "--path", "../data/godot/empty_scene/"};
       int m_argc = 2;
       std::cout << "Setting up Main\n";

       Error err = Main::setup(m_argv[0], m_argc, &m_argv[1], false);
       if (err != OK) {
           // free(cwd);
           std::cout << "Error 1 setting up main\n";
           return 255;
       }
       Error err2 = Main::setup2();

       if (err2 != OK) {
           // free(cwd);
           std::cout << "Error 2 setting up main\n";
           return 255;
       }

       std::cout << "Starting Main\n";
       */

    std::cout << "Starting Main\n";
    Main::start();

    std::cout << "Initializing MainLoop\n";
    m_os->get_main_loop()->init();

    return true;
}

void ChGodotApp::DoStep(double step_size) {
    // check if we should draw based on framerate and time since last frame
    if (m_system->GetChTime() - m_lastDrawnTime > ((1.0 / m_fps) - (step_size / 2.0)) ||
        m_system->GetChTime() < step_size) {
        Draw();
        m_lastDrawnTime = m_system->GetChTime();
    }

    // update the window in case the user isn't doing it
    // should be done to make sure inputs are processed
    // m_window->WindowShouldClose();

    // take a step in dynamics regardless TODO: change if physics paused
    m_system->DoStepDynamics(step_size);
}

void ChGodotApp::Draw() {
    // make sure the MainLoop actually exists
    if (m_os->get_main_loop() == NULL) {
        return;
    }

    // update the positions of the bodies
    m_scene->UpdatePoses();
    m_scene->UpdateHUD();

    // perform an iteration step on the Godot MainLoop
    m_os->force_process_input();
    // std::cout << "Processed input\n";
    // m_os->run();
    m_should_close = Main::iteration();

    return;
}

void ChGodotApp::UpdateAssets() {
    m_scene->UpdateBodies(m_system);
}

void ChGodotApp::AddEnvironment() {
    m_scene->AddEnvironment();
}

void ChGodotApp::AddDirectionalLight(ChQuaternion<> q) {
    m_scene->AddDirectionalLight(q);
}

void ChGodotApp::AddPointLight(ChVector<> location) {
    m_scene->AddPointLight(location);
}

void ChGodotApp::AddInteractiveCamera(ChVector<> location, ChVector<> target_location, ChVector<> up, float FOV) {
    m_scene->AddInteractiveCamera(location, target_location, up, FOV);
}

void ChGodotApp::SetDisplayHUD(bool display) {
    m_scene->SetDisplayHUD(display);
}

void ChGodotApp::SetFPS(int fps) {
    if (fps > 0)
        m_fps = fps;
}

bool ChGodotApp::ShouldClose() {
    // return m_window->WindowShouldClose();
    m_os->force_process_input();
    // TODO: this needs to change to make sure the simulation knows it should be stopping rather than crashing when it
    // realizes it cannot draw
    return m_should_close;
}

void ChGodotApp::PrintGodotTree() {
    m_scene->PrintSceneTree();
}

}  // namespace gd
}  // end namespace chrono
