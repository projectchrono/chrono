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
// Chrono::Multicore test program using NSC method for frictional contact.
//
// The model simulated here consists of a number of spherical objects falling
// in a fixed container.
//
// The global reference frame has Z up.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 1 * CH_PI / 20;

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
int count_X = 2;
int count_Y = 2;

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> AddContainer(ChSystemMulticoreNSC* sys) {
    // Create a common material
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    // Create the containing bin (4 x 4 x 1)
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetMass(1);
    bin->SetPos(ChVector3d(0, 0, 0));
    bin->SetRot(QuatFromAngleY(tilt_angle));
    bin->EnableCollision(true);
    bin->SetFixed(true);

    utils::AddBoxContainer(bin, mat,                                 //
                           ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT),  //
                           ChVector3d(4, 4, 1), 0.2,                 //
                           ChVector3i(2, 2, -1));

    sys->AddBody(bin);

    return bin;
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChContactMaterialNSC>();
    ballMat->SetFriction(0.4f);

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);

    for (int ix = -count_X; ix <= count_X; ix++) {
        for (int iy = -count_Y; iy <= count_Y; iy++) {
            ChVector3d pos(0.4 * ix, 0.4 * iy, 1);

            auto ball = chrono_types::make_shared<ChBody>();
            ball->SetMass(mass);
            ball->SetInertiaXX(inertia);
            ball->SetPos(pos);
            ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
            ball->SetFixed(false);
            ball->EnableCollision(true);

            utils::AddSphereGeometry(ball.get(), ballMat, radius);

            sys->AddBody(ball);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;

    uint max_iteration = 300;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemMulticoreNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set number of threads
    sys.SetNumThreads(8);

    // Set gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));

    // Set solver parameters
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
    sys.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
    sys.GetSettings()->solver.tolerance = tolerance;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.contact_recovery_speed = 10000;
    sys.ChangeSolverType(SolverType::APGDREF);
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    sys.GetSettings()->collision.collision_envelope = 0.01;
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create the fixed and moving bodies
    // ----------------------------------

    auto container = AddContainer(&sys);
    AddFallingBalls(&sys);

#ifdef CHRONO_VSG
    // Create run-time visualization
    // -----------------------------

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Balls NSC");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(2, -4, 4), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyTexture(SkyMode::BOX);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();
#else
    double time_end = 10;
#endif

    // Set output
    // ----------

    const std::string out_dir = GetChronoOutputPath() + "MCORE_BALLS_NSC";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::string out_file = out_dir + "/force.txt";

    // Perform the simulation
    // ----------------------

    double time = 0;
    double time_report = 2.5;
    ChWriterCSV csv(" ");

    while (true) {
#ifdef CHRONO_VSG
        if (!vis->Run())
            break;
        vis->Render();
#else
        if (time > time_end)
            break;
#endif

        sys.DoStepDynamics(time_step);
        time += time_step;

        sys.CalculateContactForces();
        real3 frc = sys.GetBodyContactForce(container);
        csv << time << frc.x << frc.y << frc.z << std::endl;

        if (time >= time_report && time < time_report + time_step)
            std::cout << "Time = " << time << "  Vertical reaction force = " << frc.z << std::endl;
    }

    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    // Plot results
    // ------------

    postprocess::ChGnuPlot gplot(out_dir + "/force.gpl");
    gplot.SetGrid();
    std::string speed_title = "Container reaction force";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("force (N)");
    gplot.Plot(out_file, 1, 4, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif

    return 0;
}
