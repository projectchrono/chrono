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
#ifndef CHGODOTAPP_H
#define CHGODOTAPP_H

#ifdef WINDOWS_ENABLED
#undef CONNECT_DEFERRED

// class OS_Windows;
#include <platform/windows/os_windows.h>
//#include <thirdparty/rtaudio/RtAudio.h>
// extern HINSTANCE godot_hinstance = NULL;

//#include <platform/windows/os_windows.h>
#else
class OS_X11;
//#include <platform/x11/os_x11.h>
#endif

// Chrono Godot includes
#include "chrono_godot/ChGdScene.h"

namespace chrono {
namespace gd {

class ChGodotApp {
  public:
    ChGodotApp(ChSystem* system, int width = 1280, int height = 720, bool displayWindow = true);
    ~ChGodotApp();

    // TODO: implement these function
    void DoStep(double step_size);
    void Draw();
    void UpdateAssets();
    void AddEnvironment();
    void AddDirectionalLight(ChQuaternion<> q = {1, 0, 0, 0});
    void AddPointLight(ChVector<> location);
    void AddInteractiveCamera(ChVector<> location, ChVector<> target_location, ChVector<> up, float FOV);
    void SetDisplayHUD(bool display);
    void SetFPS(int fps);
    bool ShouldClose();

    void PrintGodotTree();

    // void AddCamera(ChVector<> pos, ChVector<> look_at, float FOV, ChVector<> up = {0, 0, 1}, std::string name =
    // "");
    //
    // bool AddMesh();
    // bool AddLight();
    //
    // std::shared_ptr<ChGodotScene> GetScene();
    //
    // bool Draw();

  private:
    bool Initialize();

    ChSystem* m_system;
    int m_fps = 60;
    int m_framenumber = 0;
    double m_lastDrawnTime = 0.0;
    bool m_should_close = false;

#ifdef WINDOWS_ENABLED
    std::shared_ptr<OS_Windows> m_os;
#else
    std::shared_ptr<OS_X11> m_os;
#endif
    std::shared_ptr<ChGdScene> m_scene;
};

}  // namespace gd
}  // end namespace chrono

#endif
