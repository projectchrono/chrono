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
// Authors: Rainer Gericke
// =============================================================================
//
// Chrono::vsg3d test program.
//
// Template for the new chrono graphics representation
//
// The global reference frame has Z up.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2022 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto bin = chrono_types::make_shared<ChBody>();
    sys.AddBody(bin);

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(ChVector2<int>(800, 600));
    vis->SetWindowPosition(ChVector2<int>(100, 100));
    vis->SetWindowTitle("VSG Test Lab");
    vis->SetClearColor(ChColor(0.8,0.85,0.9));
    vis->SetUseSkyBox(true); // use built-in path
    vis->SetCameraVertical(chrono::vsg3d::CameraVerticalDir::Z);
    vis->AddCamera(ChVector<>(-2, 3, -4));
    vis->Initialize();

    while(vis->Run()) {
        if(vis->GetFrameNumber() == 10) {
            std::string imageFileName = "picture.png"; // allowed formats png, bmp, jpg, tga
            vis->WriteImageToFile(imageFileName); // does not work with frame == 0!
        }
        vis->Render();
    }
    return 0;
}
