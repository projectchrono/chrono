// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
//  Utilities to create a run-time visualization for FEA demos
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::fea;

std::shared_ptr<ChVisualSystem> CreateVisualizationSystem(ChVisualSystem::Type vis_type,
                                                          CameraVerticalDir vertical,
                                                          ChSystem& sys,
                                                          const std::string& title,
                                                          const ChVector3d& cam_pos,
                                                          const ChVector3d& cam_target = VNULL) {
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(vertical);
            vis_irr->SetWindowSize(1280, 720);
            vis_irr->SetWindowTitle(title);
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(cam_pos, cam_target);
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(1.0, 25.0, -5.0), ChVector3d(0, 0, 0), 35, 0.2, 35, 35, 512,
                                        ChColor(0.6f, 0.8f, 1.0f));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(vertical);
            vis_vsg->SetWindowSize(ChVector2i(1280, 720));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetWindowTitle(title);
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->AddCamera(2.0 * cam_pos, cam_target);
            vis_vsg->SetCameraAngleDeg(50);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    return vis;
}
