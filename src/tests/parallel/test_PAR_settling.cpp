// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// ChronoParallel test program for settling process of granular material.
//
// The global reference frame has Z up.
// All units SI (CGS, i.e., centimeter - gram - second)
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <valarray>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;

// --------------------------------------------------------------------------

void TimingHeader() {
    printf("    TIME    |");
    printf("    STEP |");
    printf("   BROAD |");
    printf("  NARROW |");
    printf("  SOLVER |");
    printf("  UPDATE |");
    printf("# BODIES |");
    printf("# CONTACT|");
    printf(" # ITERS |");
    printf("   RESID |");
    printf("\n\n");
}

void TimingOutput(chrono::ChSystem* mSys) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double SOLVER = mSys->GetTimerSolver();
    double UPDT = mSys->GetTimerUpdate();
    double RESID = 0;
    int REQ_ITS = 0;
    int BODS = mSys->GetNbodies();
    int CNTC = mSys->GetNcontacts();
    if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(mSys)) {
        RESID = ((chrono::ChIterativeSolverParallel*)(mSys->GetSolverSpeed()))->GetResidual();
        REQ_ITS = ((chrono::ChIterativeSolverParallel*)(mSys->GetSolverSpeed()))->GetTotalIterations();
        BODS = parallel_sys->GetNbodies();
        CNTC = parallel_sys->GetNcontacts();
    }

    printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f |\n", TIME, STEP, BROD, NARR, SOLVER,
        UPDT, BODS, CNTC, REQ_ITS, RESID);
}

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    int num_threads = 4;
    ChMaterialSurfaceBase::ContactMethod method = ChMaterialSurfaceBase::DEM;
    bool use_mat_properties = true;
    bool render = false;

    // Get number of threads from arguments (if specified)
    if (argc > 1) {
        num_threads = std::stoi(argv[1]);
    }

    std::cout << "Requested number of threads: " << num_threads << std::endl;

    // ----------------
    // Model parameters
    // ----------------

    // Container dimensions
    double hdimX = 5.0;
    double hdimY = 0.25;
    double hdimZ = 0.5;
    double hthick = 0.25;

    // Granular material properties
    double radius_g = 0.006;
    int Id_g = 10000;
    double rho_g = 2500;
    double vol_g = (4.0 / 3) * CH_C_PI * radius_g * radius_g * radius_g;
    double mass_g = rho_g * vol_g;
    ChVector<> inertia_g = 0.4 * mass_g * radius_g * radius_g * ChVector<>(1, 1, 1);
    int num_layers = 10;

    // Terrain contact properties
    float friction_terrain = 0.9f;
    float restitution_terrain = 0.0f;
    float Y_terrain = 8e5f;
    float nu_terrain = 0.3f;
    float kn_terrain = 1.0e7f;
    float gn_terrain = 1.0e3f;
    float kt_terrain = 2.86e6f;
    float gt_terrain = 1.0e3f;

    // Estimates for number of bins for broad-phase
    int factor = 2;
    int binsX = (int)std::ceil(hdimX / radius_g) / factor;
    int binsY = (int)std::ceil(hdimY / radius_g) / factor;
    int binsZ = 1;
    std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

    // --------------------------
    // Create the parallel system
    // --------------------------

    // Create system and set method-specific solver settings
    chrono::ChSystemParallel* system;

    switch (method) {
        case ChMaterialSurfaceBase::DEM: {
            ChSystemParallelDEM* sys = new ChSystemParallelDEM;
            sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hooke;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = use_mat_properties;
            system = sys;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            ChSystemParallelDVI* sys = new ChSystemParallelDVI;
            sys->GetSettings()->solver.solver_mode = SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.1 * radius_g;
            sys->ChangeSolverType(APGD);
            system = sys;

            break;
        }
    }

    system->Set_G_acc(ChVector<>(0, 0, -9.81));
    system->GetSettings()->perform_thread_tuning = false;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = 0.1;
    system->GetSettings()->solver.max_iteration_bilateral = 100;
    system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    system->GetSettings()->collision.bins_per_axis = I3(binsX, binsY, binsZ);

    // Set number of threads
    system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    // Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
    { std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

    // ---------------------
    // Create terrain bodies
    // ---------------------

    // Create contact material for terrain
    std::shared_ptr<ChMaterialSurfaceBase> material_terrain;

    switch (method) {
        case ChMaterialSurfaceBase::DEM: {
            auto mat_ter = std::make_shared<ChMaterialSurfaceDEM>();
            mat_ter->SetFriction(friction_terrain);
            mat_ter->SetRestitution(restitution_terrain);
            mat_ter->SetYoungModulus(Y_terrain);
            mat_ter->SetPoissonRatio(nu_terrain);
            mat_ter->SetAdhesion(100.0f);  // TODO
            mat_ter->SetKn(kn_terrain);
            mat_ter->SetGn(gn_terrain);
            mat_ter->SetKt(kt_terrain);
            mat_ter->SetGt(gt_terrain);

            material_terrain = mat_ter;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            auto mat_ter = std::make_shared<ChMaterialSurface>();
            mat_ter->SetFriction(friction_terrain);
            mat_ter->SetRestitution(restitution_terrain);

            material_terrain = mat_ter;

            break;
        }
    }

    // Create container body
    auto container = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);
    container->SetMaterialSurface(material_terrain);

    container->GetCollisionModel()->ClearModel();
    // Bottom box
    utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    // Front box
    utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Rear box
    utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Left box
    utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Right box
    utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    container->GetCollisionModel()->BuildModel();

    // ----------------
    // Create particles
    // ----------------

    // Create a particle generator and a mixture entirely made out of spheres
    utils::Generator gen(system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(material_terrain);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(radius_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(Id_g);

    // Create particles in layers until reaching the desired number of particles
    double r = 1.01 * radius_g;
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, 2 * r);

    for (int il = 0; il < num_layers; il++) {
        gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
        center.z += 2 * r;
    }

    unsigned int num_particles = gen.getTotalNumBodies();
    std::cout << "Generated particles:  " << num_particles << std::endl;

// -------------------------------
// Create the visualization window
// -------------------------------

#ifdef CHRONO_OPENGL
    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "Settling test", system);
        gl_window.SetCamera(ChVector<>(0, -1, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // ---------------
    // Simulate system
    // ---------------

    ChTimer<double> timer;
    double cumm_sim_time = 0;

    double time_end = 0.4;
    double time_step = 1e-4;

    TimingHeader();
    while (system->GetChTime() < time_end) {
        ////timer.reset();
        ////timer.start();
        system->DoStepDynamics(time_step);
        TimingOutput(system);
        ////timer.stop();
        ////cumm_sim_time += timer();
        ////std::cout << std::fixed << std::setprecision(6) << system->GetChTime() << "  [" << timer.GetTimeSeconds() << "]"
        ////          << std::endl;
#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active()) {
                gl_window.Render();
            } else {
                return 1;
            }
        }
#endif
    }

    return 0;
}