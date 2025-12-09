// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Writing and reading checkpoint files
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/input_output/ChCheckpointASCII.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vsg3d;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

void ConstructModel(ChSystem& sys, int id) {
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto crank = chrono_types::make_shared<ChBody>();
    crank->SetName("crank_" + std::to_string(id));
    crank->SetPos(ChVector3d(1, 0, 0));
    crank->SetMass(2);
    auto crank_shape = chrono_types::make_shared<ChVisualShapeCylinder>(0.03, 2);
    crank_shape->SetColor(id == 1 ? ChColor(0, 1, 0) : ChColor(0, 0, 1));
    crank->AddVisualShape(crank_shape, ChFramed(VNULL, QuatFromAngleY(CH_PI_2)));
    sys.AddBody(crank);

    auto rod = chrono_types::make_shared<ChBody>();
    rod->SetName("rod_" + std::to_string(id));
    rod->SetPos(ChVector3d(4, 0, 0));
    rod->SetMass(3);
    auto rod_shape = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, 4);
    rod_shape->SetColor(id == 1 ? ChColor(0, 1, 0) : ChColor(0, 0, 1));
    rod->AddVisualShape(rod_shape, ChFramed(VNULL, QuatFromAngleY(CH_PI_2)));
    sys.AddBody(rod);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(crank, rod, ChFrame<>(ChVector3d(2, 0, 0)));
    sys.AddLink(revolute);

    auto point_line = chrono_types::make_shared<ChLinkLockPointLine>();
    point_line->Initialize(rod, ground, ChFrame<>(ChVector3d(6, 0, 0)));
    sys.AddLink(point_line);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(ground, crank, ChFrame<>(ChVector3d(0, 0, 0)));
    sys.AddLink(motor);
    auto my_speed_function = chrono_types::make_shared<ChFunctionConst>(CH_PI);
    motor->SetSpeedFunction(my_speed_function);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "DEMO_CHECKPOINT";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Construct systems
    ChSystemNSC sys1;
    ChSystemNSC sys2;
    ConstructModel(sys1, 1);
    ConstructModel(sys2, 2);

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys1);
    vis->AttachSystem(&sys2);
    vis->SetWindowTitle("Slider-crank checkpointing");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->AddCamera(ChVector3d(2, 0, 6), ChVector3d(2, 0, 0));
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->Initialize();

    // Checkpoint setup
    std::string cp_filename = out_dir + "/checkpoint.txt";
    double cp_time = 1.5;
    bool cp_created = false;

    // Simulation loop
    double h = 1e-3;
    double t = 0;

    while (true) {
        if (!vis->Run())
            break;

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (t < cp_time) {
            // Simulate 1st system
            sys1.DoStepDynamics(h);
        } else {
            // Checkpoint 1st system and initialize 2nd system
            if (!cp_created) {
                {
                    ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
                    cp.WriteState(&sys1);
                    cp.WriteFile(cp_filename);
                }
                {
                    ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
                    cp.OpenFile(cp_filename);
                    cp.ReadState(&sys2);
                }
                cp_created = true;
            }
            // Simulate 2nd system
            sys2.DoStepDynamics(h);
        }

        t += h;
    }

    return 0;
}
