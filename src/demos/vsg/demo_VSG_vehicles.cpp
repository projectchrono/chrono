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
#include "chrono/assets/ChObjFileShape.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

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
    vis->AddCamera(ChVector<>(10, 10, 4));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);

    {
        std::vector<ChVector<>> hmmwv_wpos = {ChVector<>(1.64, 0.910, -0.026), ChVector<>(1.64, -0.910, -0.026),
                                              ChVector<>(-1.64, 0.910, -0.026), ChVector<>(-1.64, -0.910, -0.026)};
        auto hmmwv = chrono_types::make_shared<ChVisualModel>();
        auto hmmwv_chassis = chrono_types::make_shared<ChObjFileShape>();
        auto hmmwv_wheel = chrono_types::make_shared<ChObjFileShape>();
        auto hmmwv_tireL = chrono_types::make_shared<ChObjFileShape>();
        auto hmmwv_tireR = chrono_types::make_shared<ChObjFileShape>();
        hmmwv_chassis->SetFilename(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"));
        hmmwv_wheel->SetFilename(GetChronoDataFile("vehicle/hmmwv/hmmwv_rim.obj"));
        hmmwv_tireL->SetFilename(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_left.obj"));
        hmmwv_tireR->SetFilename(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_right.obj"));
        hmmwv->AddShape(hmmwv_chassis);
        for (int i = 0; i < 4; i++) {
            hmmwv->AddShape(hmmwv_wheel, ChFrame<>(hmmwv_wpos[i], Q_from_AngZ(CH_C_PI * i)));
        }
        hmmwv->AddShape(hmmwv_tireL, ChFrame<>(hmmwv_wpos[0], Q_from_AngZ(CH_C_PI * 0)));
        hmmwv->AddShape(hmmwv_tireR, ChFrame<>(hmmwv_wpos[1], Q_from_AngZ(CH_C_PI * 1)));
        hmmwv->AddShape(hmmwv_tireL, ChFrame<>(hmmwv_wpos[2], Q_from_AngZ(CH_C_PI * 2)));
        hmmwv->AddShape(hmmwv_tireR, ChFrame<>(hmmwv_wpos[3], Q_from_AngZ(CH_C_PI * 3)));
        vis->AddVisualModel(hmmwv, ChFrame<>(ChVector<>(0, -4.5, 0), QUNIT));
    }

    {
        std::vector<ChVector<>> gator_wpos = {ChVector<>(0.97, 0.56, -0.02), ChVector<>(0.97, -0.56, -0.02),
                                              ChVector<>(-0.97, 0.62, 0), ChVector<>(-0.97, -0.62, 0)};
        auto gator = chrono_types::make_shared<ChVisualModel>();
        auto gator_chassis = chrono_types::make_shared<ChObjFileShape>();
        auto gator_wheelF = chrono_types::make_shared<ChObjFileShape>();
        auto gator_wheelR = chrono_types::make_shared<ChObjFileShape>();
        gator_chassis->SetFilename(GetChronoDataFile("vehicle/gator/gator_chassis.obj"));
        gator_wheelF->SetFilename(GetChronoDataFile("vehicle/gator/gator_wheel_FL.obj"));
        gator_wheelR->SetFilename(GetChronoDataFile("vehicle/gator/gator_wheel_RL.obj"));
        gator->AddShape(gator_chassis);
        gator->AddShape(gator_wheelF, ChFrame<>(gator_wpos[0], Q_from_AngZ(CH_C_PI * 0)));
        gator->AddShape(gator_wheelF, ChFrame<>(gator_wpos[1], Q_from_AngZ(CH_C_PI * 1)));
        gator->AddShape(gator_wheelR, ChFrame<>(gator_wpos[2], Q_from_AngZ(CH_C_PI * 2)));
        gator->AddShape(gator_wheelR, ChFrame<>(gator_wpos[3], Q_from_AngZ(CH_C_PI * 3)));
        vis->AddVisualModel(gator, ChFrame<>(ChVector<>(0, -1.5, 0), QUNIT));
    }

    {
        auto audi_chassis = chrono_types::make_shared<ChObjFileShape>();
        audi_chassis->SetFilename(GetChronoDataFile("vehicle/audi/audi_chassis.obj"));
        // vis->AddVisualModel(audi_chassis, ChFrame<>(ChVector<>(0, -1.5, 0), QUNIT));
    }

    {
        std::vector<ChVector<>> uaz_wpos = {ChVector<>(0, 0.733, 0), ChVector<>(0, -0.733, 0),
                                            ChVector<>(-2.3, 0.733, 0), ChVector<>(-2.3, -0.733, 0)};
        auto uaz = chrono_types::make_shared<ChVisualModel>();
        auto uaz_chassis = chrono_types::make_shared<ChObjFileShape>();
        auto uaz_wheel = chrono_types::make_shared<ChObjFileShape>();
        auto uaz_tire = chrono_types::make_shared<ChObjFileShape>();
        uaz_chassis->SetFilename(GetChronoDataFile("vehicle/uaz/uazbus_chassis.obj"));
        uaz_wheel->SetFilename(GetChronoDataFile("vehicle/uaz/uaz_rim.obj"));
        uaz_tire->SetFilename(GetChronoDataFile("vehicle/uaz/uaz_tire.obj"));
        uaz->AddShape(uaz_chassis);
        for (int i = 0; i < 4; i++) {
            uaz->AddShape(uaz_wheel, ChFrame<>(uaz_wpos[i], Q_from_AngZ(CH_C_PI * i)));
            uaz->AddShape(uaz_tire, ChFrame<>(uaz_wpos[i], Q_from_AngZ(CH_C_PI * i)));
        }
        vis->AddVisualModel(uaz, ChFrame<>(ChVector<>(0, 1.5, 0), QUNIT));

        std::vector<ChVector<>> suv_wpos = {ChVector<>(0, 0.960, 0.1), ChVector<>(0, -0.960, 0.1),
                                            ChVector<>(-3.336, 1.010, 0.05), ChVector<>(-3.336, -1.010, 0.05)};
        auto suv = chrono_types::make_shared<ChVisualModel>();
        auto suv_chassis = chrono_types::make_shared<ChObjFileShape>();
        auto suv_wheel = chrono_types::make_shared<ChObjFileShape>();
        auto suv_tire = chrono_types::make_shared<ChObjFileShape>();
        suv_chassis->SetFilename(GetChronoDataFile("vehicle/Nissan_Patrol/suv_chassis.obj"));
        suv_wheel->SetFilename(GetChronoDataFile("vehicle/Nissan_Patrol/suv_rim.obj"));
        suv_tire->SetFilename(GetChronoDataFile("vehicle/Nissan_Patrol/suv_tire.obj"));
        suv->AddShape(suv_chassis);
        for (int i = 0; i < 4; i++) {
            suv->AddShape(suv_wheel, ChFrame<>(suv_wpos[i], Q_from_AngZ(CH_C_PI * i)));
            suv->AddShape(suv_tire, ChFrame<>(suv_wpos[i], Q_from_AngZ(CH_C_PI * i)));
        }
        vis->AddVisualModel(suv, ChFrame<>(ChVector<>(0, 4.5, 0), QUNIT));
    }

    vis->Initialize();

    while (vis->Run()) {
        vis->Render();
        sys.DoStepDynamics(0.01);
    }

    return 0;
}
