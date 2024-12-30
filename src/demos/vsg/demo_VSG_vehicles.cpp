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
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::vsg3d;

std::shared_ptr<ChVisualShapeTriangleMesh> CreateMeshShape(const std::string& filename) {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
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
    vis->SetWindowSize(ChVector2i(1200, 800));
    vis->SetWindowPosition(ChVector2i(100, 300));
    vis->SetWindowTitle("Chrono VSG Assets");
    vis->SetUseSkyBox(false);
    vis->AddCamera(ChVector3d(8.0, 12.3, 3.0), ChVector3d(-0.1, 1.0, 0.4));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(2.0f);
    vis->SetLightDirection(CH_PI_2, CH_PI_4);

    ChVector3d bus_pos(3, -4.5, 0);
    ChVector3d polaris_pos(-3, -1.5, 0);
    ChVector3d suv_pos(-3, +1.5, 0);
    ChVector3d uaz_pos(-3, +4.5, 0);
    ChVector3d hmmwv_pos(3, -1.5, 0);
    ChVector3d audi_pos(3, +1.5, 0);
    ChVector3d gator_pos(3, +4.5, 0);

    {
        std::vector<ChVector3d> bus_wpos = {ChVector3d(0, 1.128, 0.5), ChVector3d(0, -1.128, 0.5),
                                            ChVector3d(-7.184, 0.96, 0.5), ChVector3d(-7.184, -0.96, 0.5)};
        auto bus = chrono_types::make_shared<ChVisualModel>();
        auto bus_chassis = CreateMeshShape(GetChronoDataFile("vehicle/citybus/CityBus_Vis.obj"));
        auto bus_wheel = CreateMeshShape(GetChronoDataFile("vehicle/citybus/CityBusRim.obj"));
        auto bus_tire = CreateMeshShape(GetChronoDataFile("vehicle/citybus/CityBusTire.obj"));
        bus->AddShape(bus_chassis);
        for (int i = 0; i < 4; i++) {
            bus->AddShape(bus_wheel, ChFrame<>(bus_wpos[i], QuatFromAngleZ(CH_PI * i)));
            bus->AddShape(bus_tire, ChFrame<>(bus_wpos[i], QuatFromAngleZ(CH_PI * i)));
        }
        vis->AddVisualModel(bus, ChFrame<>(bus_pos, QUNIT));
    }

    {
        std::vector<ChVector3d> hmmwv_wpos = {ChVector3d(1.64, 0.910, -0.026), ChVector3d(1.64, -0.910, -0.026),
                                              ChVector3d(-1.64, 0.910, -0.026), ChVector3d(-1.64, -0.910, -0.026)};
        auto hmmwv = chrono_types::make_shared<ChVisualModel>();
        auto hmmwv_chassis = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"));
        auto hmmwv_wheel = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_rim.obj"));
        auto hmmwv_tireL = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_left.obj"));
        auto hmmwv_tireR = CreateMeshShape(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_right.obj"));
        hmmwv->AddShape(hmmwv_chassis);
        for (int i = 0; i < 4; i++) {
            hmmwv->AddShape(hmmwv_wheel, ChFrame<>(hmmwv_wpos[i], QuatFromAngleZ(CH_PI * i)));
        }
        hmmwv->AddShape(hmmwv_tireL, ChFrame<>(hmmwv_wpos[0], QuatFromAngleZ(CH_PI * 0)));
        hmmwv->AddShape(hmmwv_tireR, ChFrame<>(hmmwv_wpos[1], QuatFromAngleZ(CH_PI * 1)));
        hmmwv->AddShape(hmmwv_tireL, ChFrame<>(hmmwv_wpos[2], QuatFromAngleZ(CH_PI * 2)));
        hmmwv->AddShape(hmmwv_tireR, ChFrame<>(hmmwv_wpos[3], QuatFromAngleZ(CH_PI * 3)));
        vis->AddVisualModel(hmmwv, ChFrame<>(hmmwv_pos, QUNIT));
    }

    {
        std::vector<ChVector3d> gator_wpos = {ChVector3d(0.97, 0.56, -0.02), ChVector3d(0.97, -0.56, -0.02),
                                              ChVector3d(-0.97, 0.62, 0), ChVector3d(-0.97, -0.62, 0)};
        auto gator = chrono_types::make_shared<ChVisualModel>();
        auto gator_chassis = CreateMeshShape(GetChronoDataFile("vehicle/gator/gator_chassis.obj"));
        auto gator_wheelF = CreateMeshShape(GetChronoDataFile("vehicle/gator/gator_wheel_FL.obj"));
        auto gator_wheelR = CreateMeshShape(GetChronoDataFile("vehicle/gator/gator_wheel_RL.obj"));
        gator->AddShape(gator_chassis);
        gator->AddShape(gator_wheelF, ChFrame<>(gator_wpos[0], QuatFromAngleZ(CH_PI * 0)));
        gator->AddShape(gator_wheelF, ChFrame<>(gator_wpos[1], QuatFromAngleZ(CH_PI * 1)));
        gator->AddShape(gator_wheelR, ChFrame<>(gator_wpos[2], QuatFromAngleZ(CH_PI * 2)));
        gator->AddShape(gator_wheelR, ChFrame<>(gator_wpos[3], QuatFromAngleZ(CH_PI * 3)));
        vis->AddVisualModel(gator, ChFrame<>(gator_pos, QUNIT));
    }

    {
        std::vector<ChVector3d> audi_wpos = {ChVector3d(1.44, 0.8, 0.12), ChVector3d(1.44, -0.8, 0.12),
                                             ChVector3d(-1.48, 0.8, 0.12), ChVector3d(-1.48, -0.8, 0.12)};
        auto audi = chrono_types::make_shared<ChVisualModel>();
        auto audi_chassis = CreateMeshShape(GetChronoDataFile("vehicle/audi/audi_chassis_white.obj"));
        auto audi_wheel = CreateMeshShape(GetChronoDataFile("vehicle/audi/audi_rim.obj"));
        auto audi_tire = CreateMeshShape(GetChronoDataFile("vehicle/audi/audi_tire.obj"));
        audi->AddShape(audi_chassis);
        for (int i = 0; i < 4; i++) {
            audi->AddShape(audi_wheel, ChFrame<>(audi_wpos[i], QuatFromAngleZ(CH_PI * i)));
            audi->AddShape(audi_tire, ChFrame<>(audi_wpos[i], QuatFromAngleZ(CH_PI * i)));
        }
        vis->AddVisualModel(audi, ChFrame<>(audi_pos, QUNIT));
    }

    {
        std::vector<ChVector3d> uaz_wpos = {ChVector3d(0, 0.733, 0), ChVector3d(0, -0.733, 0),
                                            ChVector3d(-2.3, 0.733, 0), ChVector3d(-2.3, -0.733, 0)};
        auto uaz = chrono_types::make_shared<ChVisualModel>();
        auto uaz_chassis = CreateMeshShape(GetChronoDataFile("vehicle/uaz/uazbus_chassis.obj"));
        auto uaz_wheel = CreateMeshShape(GetChronoDataFile("vehicle/uaz/uaz_rim.obj"));
        auto uaz_tire = CreateMeshShape(GetChronoDataFile("vehicle/uaz/uaz_tire.obj"));
        uaz->AddShape(uaz_chassis);
        for (int i = 0; i < 4; i++) {
            uaz->AddShape(uaz_wheel, ChFrame<>(uaz_wpos[i], QuatFromAngleZ(CH_PI * i)));
            uaz->AddShape(uaz_tire, ChFrame<>(uaz_wpos[i], QuatFromAngleZ(CH_PI * i)));
        }
        vis->AddVisualModel(uaz, ChFrame<>(uaz_pos, QUNIT));
    }

    {
        std::vector<ChVector3d> suv_wpos = {ChVector3d(0, 0.960, 0.1), ChVector3d(0, -0.960, 0.1),
                                            ChVector3d(-3.336, 1.010, 0.05), ChVector3d(-3.336, -1.010, 0.05)};
        auto suv = chrono_types::make_shared<ChVisualModel>();
        auto suv_chassis = CreateMeshShape(GetChronoDataFile("vehicle/Nissan_Patrol/suv_chassis.obj"));
        auto suv_wheel = CreateMeshShape(GetChronoDataFile("vehicle/Nissan_Patrol/suv_rim.obj"));
        auto suv_tire = CreateMeshShape(GetChronoDataFile("vehicle/Nissan_Patrol/suv_tire.obj"));
        suv->AddShape(suv_chassis);
        for (int i = 0; i < 4; i++) {
            suv->AddShape(suv_wheel, ChFrame<>(suv_wpos[i], QuatFromAngleZ(CH_PI * i)));
            suv->AddShape(suv_tire, ChFrame<>(suv_wpos[i], QuatFromAngleZ(CH_PI * i)));
        }
        vis->AddVisualModel(suv, ChFrame<>(suv_pos, QUNIT));
    }

    {
        std::vector<ChVector3d> polaris_wpos = {ChVector3d(0, 0.616, 0.397), ChVector3d(0, -0.616, 0.397),
                                                ChVector3d(-2.715, 0.616, 0.405), ChVector3d(-2.715, -0.616, 0.405)};
        auto polaris = chrono_types::make_shared<ChVisualModel>();
        auto polaris_chassis = CreateMeshShape(GetChronoDataFile("vehicle/Polaris/meshes/Polaris_chassis.obj"));
        auto polaris_wheel = CreateMeshShape(GetChronoDataFile("vehicle/Polaris/meshes/Polaris_wheel.obj"));
        auto polaris_tire = CreateMeshShape(GetChronoDataFile("vehicle/Polaris/meshes/Polaris_tire.obj"));
        polaris->AddShape(polaris_chassis);
        for (int i = 0; i < 4; i++) {
            polaris->AddShape(polaris_wheel, ChFrame<>(polaris_wpos[i], QuatFromAngleZ(CH_PI * i)));
            polaris->AddShape(polaris_tire, ChFrame<>(polaris_wpos[i], QuatFromAngleZ(CH_PI * i)));
        }
        vis->AddVisualModel(polaris, ChFrame<>(polaris_pos, QUNIT));
    }

    vis->SetLogoVisible(true);
    vis->Initialize();

    while (vis->Run()) {
        vis->Render();
        sys.DoStepDynamics(0.01);
    }

    return 0;
}
