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
// Authors: Radu Serban
// =============================================================================
//
// Demosntration of the Chrono::VSG run-time visualization system
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    ChSystemNSC sys;

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(ChVector2<int>(1200, 900));
    vis->SetWindowPosition(ChVector2<int>(100, 300));
    vis->SetWindowTitle("Chrono VSG Assets");
    vis->SetUseSkyBox(false);
    vis->AddCamera(ChVector<>(0.4, -1, 0.5), ChVector<>(0, 0, 0.4));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);

    {
        auto shape = chrono_types::make_shared<ChVisualShapeModelFile>();
        shape->SetFilename(GetChronoDataFile("models/FlightHelmet/FlightHelmet.gltf"));
        vis->AddVisualModel(shape, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    }

    vis->Initialize();

    while (vis->Run()) {
        vis->Render();
        sys.DoStepDynamics(0.01);
    }

    return 0;
}
