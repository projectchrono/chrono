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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Luning Bakke
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsGenerators.h"

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
std::string out_dir = GetChronoOutputPath() + "FSI_Compressibility";

// Output frequency
bool output = true;
double output_fps = 20;

// Size of the box
double smalldis = 1.0e-5;
double bxDim = 1.0 + smalldis;
double byDim = 1.0 + smalldis;
double bzDim = 1.4 + smalldis;

// Size of the fluid domain
double fxDim = bxDim;
double fyDim = byDim;
double fzDim = 1.0 + smalldis;

// Final simulation time
double t_end = 2.0;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 1000;

// =============================================================================

int main(int argc, char* argv[]) {
    // Create 
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET); 

    double initial_spacing = 0.1; // WCSPH
    double step_size = 1e-4;


    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    auto sysFSI = fsi.GetFsiSystemSPH();


    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -1.0);
    fsi.SetGravitationalAcceleration(gravity);
    
    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 0.1;
    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.num_bce_layers = 5;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 1.0;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.density_reinit_steps = 2;

    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size); // do i need this? 



    ChVector3d cMin = ChVector3d(-bxDim / 2, -byDim / 2, -bzDim / 2) - ChVector3d(initial_spacing * 20);
    ChVector3d cMax = ChVector3d(+bxDim / 2, +byDim / 2, bzDim) + ChVector3d(initial_spacing * 10);
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), BC_ALL_PERIODIC);

    ChVector3d fsize = ChVector3d(fxDim, fyDim, fzDim);
    fsi.Construct(fsize,                          // length x width x depth
                  ChVector3d(0, 0, 0),            // position of bottom origin
                  BoxSide::ALL & ~BoxSide::Z_POS  // all boundaries except top
    );


    // Complete construction of the FSI system
    fsi.Initialize();

    auto sysSPH = fsi.GetFluidSystemSPH();
    int numPart = sysSPH->GetNumFluidMarkers();


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

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
    }

    // Create a run-tme visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 5.0);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Compressibility Test");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -5 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.5 * bzDim));

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    std::ofstream outf;
    std::string delim = ",";
    outf.open((out_dir + "/Analysis.txt"), std::ios::trunc);
    outf << "Time" << delim << "Rho_fluid" << delim << "k_fluid" << std::endl;

    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();


    while (time < t_end) {
        std::cout << "step: " << sim_frame << "  time: " << time << std::endl;

        // Save data of the simulation
        if (output && time >= out_frame / output_fps) {
            std::cout << "------- OUTPUT" << std::endl;
            fsi.SaveOutputData(time, out_dir + "/particles", "");
            out_frame++;
        }

        // Render FSI system
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
#endif

        fsi.DoStepDynamics(step_size);

        auto rhoPresMu = sysSPH->GetParticleFluidProperties();
        auto vel = sysSPH->GetParticleVelocities();

        double KE = 0;
        double Rho = 0;
        for (int i = 0; i < numPart; i++) {
            KE += 0.5 * vel[i].Length();
            Rho += rhoPresMu[i].x();
        }

        outf << time << delim << Rho / numPart << delim << sysSPH->GetParticleMass() * KE / numPart << std::endl;

        time += step_size;
        sim_frame++;
    }

    outf.close();
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
