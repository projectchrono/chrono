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
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================
#ifndef CHGDSCENE_H
#define CHGDSCENE_H

//#include <iostream>
//#include <memory>

// godot includes
//#include <core/os/main_loop.h>
//#include <scene/main/scene_tree.h>

// chrono includes
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystem.h"

// ChGdIncludes
#include "chrono_godot/godot_utils/ChGdAssetManager.h"
#include "chrono_godot/nodes/ChGdBody.h"
#include "chrono_godot/nodes/ChGdHUD.h"
#include "chrono_godot/nodes/ChGdMainCamera.h"

namespace chrono {
namespace gd {

class ChGdScene {
  public:
    ChGdScene();
    ~ChGdScene();

    void Initialize(MainLoop* main_loop, ChSystem* chsystem);
    void PrintSceneTree();

    void AddEnvironment();
    void AddDirectionalLight(ChQuaternion<> q);
    void AddPointLight(ChVector<> location);
    void AddInteractiveCamera(ChVector<> location, ChVector<> target_location, ChVector<> up, float FOV);
    void SetDisplayHUD(bool display);
    void UpdateBodies(ChSystem* chsystem);
    void UpdatePoses();
    void UpdateHUD();

    SceneTree* GetSceneRoot();

    // void AddSphere();
    // void AddBox();
    // void AddCylinder();
    // void AddMesh();
    // void AddLight();
    // void AddSky();
    // void AddCamera(ChVector<> pos, ChVector<> look_at, float FOV, ChVector<> up, std::string name);

    // void DrawDebug();

  private:
    SceneTree* m_sceneTree;
    Spatial* m_spatialRoot;

    std::shared_ptr<ChGdAssetManager> m_assetManager;
    std::shared_ptr<ChGdHUD> m_hud;

    std::vector<Object*> m_resourceList;
    std::vector<std::shared_ptr<ChGdMainCamera>> m_cameraList;
    std::vector<std::shared_ptr<ChGdBody>> m_bodyList;
};

}  // namespace gd
}  // end namespace chrono

#endif
