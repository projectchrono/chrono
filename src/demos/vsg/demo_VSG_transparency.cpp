// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Tudor Miron
// =============================================================================
//
// Demo to test transparency rendering with shadows.
//
// Scene layout (Z-up):
//   - Large gray ground plane at Z=0
//   - Red opaque box at (-2, 0, 0.6)        -- casts shadow
//   - Blue transparent sphere at (0, 0, 1.0) -- also casts shadow
//   - Green opaque cylinder at (2, 0, 0.75)  -- reference shadow caster
//
// Shadows enabled via DirectionalLight.  After BindAll(), a visitor patches
// transparent pipelines: wraps them in vsg::DepthSorted (bin 10, after opaque),
// keeps depthWriteEnable = VK_TRUE so the shadow map sees them, and sets
// cullMode = VK_CULL_MODE_NONE for two-sided transparency.
//
// Expected result: all three objects cast distinct shadows on the ground.
// The global reference frame has Z up.
// =============================================================================

#include <iostream>
#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    std::cout << "=== Transparency + Shadow Demo ===" << std::endl;

    // Physics system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Ground plane (fixed, opaque gray)
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(12.0, 12.0, 0.1,  // large flat slab
                                                           1000, true, false);
    ground->SetFixed(true);
    ground->SetPos(ChVector3d(0, 0, -0.05));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.6f, 0.6f, 0.6f});
        mat->SetOpacity(1.0f);
        ground->GetVisualShape(0)->SetMaterial(0, mat);
    }
    sys.AddBody(ground);

    // Opaque red box
    auto opaqueBox = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.2, 500, true, false);
    opaqueBox->SetFixed(true);
    opaqueBox->SetPos(ChVector3d(-2.0, 0, 0.6));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.85f, 0.15f, 0.15f});
        mat->SetOpacity(1.0f);
        opaqueBox->GetVisualShape(0)->SetMaterial(0, mat);
    }
    sys.AddBody(opaqueBox);

    // Transparent blue sphere
    auto transSphere = chrono_types::make_shared<ChBodyEasySphere>(0.5, 500, true, false);
    transSphere->SetFixed(true);
    transSphere->SetPos(ChVector3d(0, 0, 1.0));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.2f, 0.4f, 0.9f});
        mat->SetOpacity(0.35f);  // 35% opaque -- clearly transparent
        transSphere->GetVisualShape(0)->SetMaterial(0, mat);
    }
    sys.AddBody(transSphere);

    // Opaque green cylinder (reference)
    auto opaqueCyl = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z, 0.4, 1.5, 500, true, false);
    opaqueCyl->SetFixed(true);
    opaqueCyl->SetPos(ChVector3d(2.0, 0, 0.75));
    {
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor({0.15f, 0.7f, 0.2f});
        mat->SetOpacity(1.0f);
        opaqueCyl->GetVisualShape(0)->SetMaterial(0, mat);
    }
    sys.AddBody(opaqueCyl);

    // VSG visualization
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(1400, 900);
    vis->SetWindowTitle("Chrono::VSG Transparency + Shadow Demo");

    // Camera: angled view to see shadows on ground
    vis->AddCamera(ChVector3d(6, -6, 5), ChVector3d(0, 0, 0.5));

    // Enable shadows -- must be called before Initialize()
    vis->EnableShadows(true);

    // Light direction: low angle to cast long visible shadows
    vis->SetLightDirection(1.2, 0.5);  // azimuth ~70deg, elevation ~30deg
    vis->SetLightIntensity(0.9f);

    vis->Initialize();

    std::cout << "Scene ready. Look for shadows on the ground plane." << std::endl;
    std::cout << "  Red box (opaque)        -- should cast shadow" << std::endl;
    std::cout << "  Blue sphere (35% alpha) -- should ALSO cast shadow" << std::endl;
    std::cout << "  Green cylinder (opaque) -- reference shadow" << std::endl;
    std::cout << std::endl;
    std::cout << "Close the window or press ESC to exit." << std::endl;

    // Main loop (static scene, no dynamics needed)
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    }

    return 0;
}
