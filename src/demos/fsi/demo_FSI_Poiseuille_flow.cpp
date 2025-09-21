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
// Author: Milad Rakhsha, Wei Hu, Luning Bakke
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// -----------------------------------------------------------------

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Poiseuille_Flow";

// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim = 0.2;
double byDim = 0.1;
double bzDim = 0.2;

// Final simulation time
double t_end = 10.0;

// Enable/disable run-time visualization
bool render = true;
bool snapshots = false;
float render_fps = 100;

// =============================================================================

int main(int argc, char* argv[]) {
    double initial_spacing = 0.01;

    // Create a physics system and an SPH system
    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(true);
    auto sysFSI = fsi.GetFsiSystemSPH();
    auto sysSPH = fsi.GetFluidSystemSPH();

    double step_size = 2e-3;
    double density = 1000;
    double viscosity = 1.0;
    ChVector3d body_force(0.01, 0, 0);  // body force to drive the flow

    sysSPH->SetBodyForce(body_force);

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, 0.0);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = density;
    fluid_props.viscosity = viscosity;
    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 0.1;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.density_reinit_steps = 10000;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.use_delta_sph = false;
    sph_params.eos_type = EosType::ISOTHERMAL;
    sph_params.use_consistent_gradient_discretization = true;  // consistent discretization only for laminar viscosity
    sph_params.use_consistent_laplacian_discretization = true;
    fsi.SetSPHParameters(sph_params);

    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize,                              // length x width x depth
                  ChVector3d(0, 0, initial_spacing),  // position of bottom origin
                  BoxSide::Z_NEG | BoxSide::Z_POS     // bottom and top boundaries
    );

    // Explicitly set computational domain
    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -5 * initial_spacing);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + 5 * initial_spacing);
    fsi.SetComputationalDomain(ChAABB(c_min, c_max), BC_ALL_PERIODIC);

    // Complete construction of the fluid system
    fsi.Initialize();

    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH->GetPhysicsProblemString() + "_" + sysSPH->GetSphIntegrationSchemeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
        std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
        return 1;
    }

    // Create a run-tme visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 0.04);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Poiseuille Flow");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -5 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.5 * bzDim));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Save data of the simulation
        if (output && time >= out_frame / out_fps) {
            std::cout << " -- Output frame " << out_frame << " at t = " << time << std::endl;
            sysSPH->SaveParticleData(out_dir + "/particles");

            out_frame++;
        }

        // Render FSI system
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif

        // Call the FSI solver
        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
