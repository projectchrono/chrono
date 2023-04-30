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
// Chrono::Multicore test program using SMC method for frictional contact.
//
// The model simulated here consists of a number of spherical objects falling
// in a fixed container.
//
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 1 * CH_C_PI / 20;

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
int count_X = 2;
int count_Y = 2;

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemMulticoreSMC* sys) {
    // IDs for the two bodies
    int binId = -200;

    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    // Create the containing bin (4 x 4 x 1)
    auto bin = std::shared_ptr<ChBody>(sys->NewBody());
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxContainer(bin, mat,                                 //
                           ChFrame<>(ChVector<>(0, 0, 0.5), QUNIT),  //
                           ChVector<>(4, 4, 1), 0.2,                 //
                           ChVector<int>(2, 2, -1));
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    int ballId = 0;
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    for (int ix = -count_X; ix <= count_X; ix++) {
        for (int iy = -count_Y; iy <= count_Y; iy++) {
            ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

            auto ball = std::shared_ptr<ChBody>(sys->NewBody());
            ball->SetIdentifier(ballId++);
            ball->SetMass(mass);
            ball->SetInertiaXX(inertia);
            ball->SetPos(pos);
            ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
            ball->SetBodyFixed(false);
            ball->SetCollide(true);

            ball->GetCollisionModel()->ClearModel();
            utils::AddSphereGeometry(ball.get(), ballMat, radius);
            ball->GetCollisionModel()->BuildModel();

            sys->AddBody(ball);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;

    uint max_iteration = 100;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemMulticoreSMC sys;

    // Set number of threads
    sys.SetNumThreads(8);

    // Set gravitational acceleration
    sys.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    sys.GetSettings()->solver.tolerance = tolerance;

    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Create the fixed and moving bodies
    // ----------------------------------
    AddContainer(&sys);
    AddFallingBalls(&sys);

// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Balls SMC");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, -6, 0), ChVector<>(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    while (true) {
        if (vis.Run()) {
            sys.DoStepDynamics(time_step);
            vis.Render();
            // Print cumulative contact force on container bin.
            real3 frc = sys.GetBodyContactForce(0);
            std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;
        } else {
            break;
        }
    }
#else
    // Run simulation for specified time
    double time_end = 2;
    int num_steps = (int)std::ceil(time_end / time_step);
    double time = 0;

    for (int i = 0; i < num_steps; i++) {
        sys.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
