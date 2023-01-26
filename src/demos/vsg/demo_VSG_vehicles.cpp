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
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::vsg3d;

std::shared_ptr<ChTriangleMeshShape> CreateMeshShape(const std::string& filename) {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(filesystem::path(filename).stem());
    trimesh_shape->SetMutable(false);

    return trimesh_shape;
}

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
        auto hmmwv_chassis = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"));
        auto hmmwv_wheel = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_rim.obj"));
        auto hmmwv_tireL = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_left.obj"));
        auto hmmwv_tireR = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_right.obj"));
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
        auto gator_chassis = CreateMeshShape(GetChronoDataFile("vehicle/gator/gator_chassis.obj"));
        auto gator_wheelF = CreateMeshShape(GetChronoDataFile("vehicle/gator/gator_wheel_FL.obj"));
        auto gator_wheelR = CreateMeshShape(GetChronoDataFile("vehicle/gator/gator_wheel_RL.obj"));
        gator->AddShape(gator_chassis);
        gator->AddShape(gator_wheelF, ChFrame<>(gator_wpos[0], Q_from_AngZ(CH_C_PI * 0)));
        gator->AddShape(gator_wheelF, ChFrame<>(gator_wpos[1], Q_from_AngZ(CH_C_PI * 1)));
        gator->AddShape(gator_wheelR, ChFrame<>(gator_wpos[2], Q_from_AngZ(CH_C_PI * 2)));
        gator->AddShape(gator_wheelR, ChFrame<>(gator_wpos[3], Q_from_AngZ(CH_C_PI * 3)));
        vis->AddVisualModel(gator, ChFrame<>(ChVector<>(0, -1.5, 0), QUNIT));
    }
    /*
    {
        auto audi = chrono_types::make_shared<ChVisualModel>();
        auto audi_chassis = CreateMeshShape(GetChronoDataFile("vehicle/audi/audi_chassis.obj"));
        audi->AddShape(audi_chassis);
        vis->AddVisualModel(audi, ChFrame<>(ChVector<>(0, -1.5, 0), QUNIT));
    }
    */
    {
        std::vector<ChVector<>> uaz_wpos = {ChVector<>(0, 0.733, 0), ChVector<>(0, -0.733, 0),
                                            ChVector<>(-2.3, 0.733, 0), ChVector<>(-2.3, -0.733, 0)};
        auto uaz = chrono_types::make_shared<ChVisualModel>();
        auto uaz_chassis = CreateMeshShape(GetChronoDataFile("vehicle/uaz/uazbus_chassis.obj"));
        auto uaz_wheel = CreateMeshShape(GetChronoDataFile("vehicle/uaz/uaz_rim.obj"));
        auto uaz_tire = CreateMeshShape(GetChronoDataFile("vehicle/uaz/uaz_tire.obj"));
        uaz->AddShape(uaz_chassis);
        for (int i = 0; i < 4; i++) {
            uaz->AddShape(uaz_wheel, ChFrame<>(uaz_wpos[i], Q_from_AngZ(CH_C_PI * i)));
            uaz->AddShape(uaz_tire, ChFrame<>(uaz_wpos[i], Q_from_AngZ(CH_C_PI * i)));
        }
        vis->AddVisualModel(uaz, ChFrame<>(ChVector<>(0, 1.5, 0), QUNIT));

        std::vector<ChVector<>> suv_wpos = {ChVector<>(0, 0.960, 0.1), ChVector<>(0, -0.960, 0.1),
                                            ChVector<>(-3.336, 1.010, 0.05), ChVector<>(-3.336, -1.010, 0.05)};
        auto suv = chrono_types::make_shared<ChVisualModel>();
        auto suv_chassis = CreateMeshShape(GetChronoDataFile("vehicle/Nissan_Patrol/suv_chassis.obj"));
        auto suv_wheel = CreateMeshShape(GetChronoDataFile("vehicle/Nissan_Patrol/suv_rim.obj"));
        auto suv_tire = CreateMeshShape(GetChronoDataFile("vehicle/Nissan_Patrol/suv_tire.obj"));
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
