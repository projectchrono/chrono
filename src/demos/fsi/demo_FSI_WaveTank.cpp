// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualizationSPH.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::utils;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Container dimensions
ChVector3d csize(5.0, 0.4, 0.8);

// Beach start point
double x_start = csize.x() / 2;

// Fluid depth
double depth = 0.4;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualizationSPH::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};

// -----------------------------------------------------------------------------

class WaveFunction : public ChFunction {
  public:
    WaveFunction() : delay(0), a2(0), omega(0) {}
    WaveFunction(double delay, double amplitude, double frequency)
        : delay(delay), a2(amplitude / 2), omega(CH_2PI * frequency) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        if (t <= delay)
            return 0;
        double tt = t - delay;
        return a2 * (1 - std::cos(omega * tt));
    }

  private:
    double delay;
    double a2;
    double omega;
};

class WaveFunctionDecay : public ChFunction {
  public:
    // stroke s0, period T, with an exponential decay
    WaveFunctionDecay() : s0(0.1), T(1) {}
    WaveFunctionDecay(double s0, double period) : s0(s0 / 2), T(period) {}

    virtual WaveFunction* Clone() const override { return new WaveFunction(); }

    virtual double GetVal(double t) const override {
        return 0.5 * s0 * (1 - std::exp(-t / T)) * std::sin(2. * CH_PI / T * t);
    }

  private:
    double s0;  // stroke
    double T;   // period
};

// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& render,
                     double& render_fps,
                     bool& snapshots,
                     int& ps_freq,
                     std::string& boundary_type,
                     std::string& viscosity_type) {
    ChCLI cli(argv[0], "Wave Tank FSI demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("Output", "output", "Enable collection of output files");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    cli.AddOption<bool>("Visualization", "snapshots", "Enable writing snapshot image files");

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    cli.AddOption<std::string>("Physics", "boundary_type", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_type",
                               "Viscosity type (laminar/artificial_unilateral/artificial_bilateral)",
                               "artificial_unilateral");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    t_end = cli.GetAsType<double>("t_end");

    verbose = !cli.GetAsType<bool>("quiet");
    output = cli.GetAsType<bool>("output");
    render = !cli.GetAsType<bool>("no_vis");
    snapshots = cli.GetAsType<bool>("snapshots");

    output_fps = cli.GetAsType<double>("output_fps");
    render_fps = cli.GetAsType<double>("render_fps");

    ps_freq = cli.GetAsType<int>("ps_freq");

    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");

    return true;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.025;
    double step_size = 1e-4;

    // Parse command line arguments
    double t_end = 12.0;
    bool verbose = true;
    bool output = true;
    double output_fps = 20;
    bool render = true;
    double render_fps = 400;
    bool snapshots = true;
    int ps_freq = 1;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_unilateral";
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, render, render_fps, snapshots, ps_freq,
                         boundary_type, viscosity_type)) {
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 4.0;  // maximum velocity should be 2*sqrt(grav * fluid_height)
    // sph_params.shifting_method = ShiftingMethod::XSPH;
    // sph_params.shifting_xsph_eps = 0.5;

    sph_params.shifting_method = ShiftingMethod::DIFFUSION;
    sph_params.shifting_diffusion_A = 1.;
    sph_params.shifting_diffusion_AFSM = 3.;
    sph_params.shifting_diffusion_AFST = 2.;

    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.artificial_viscosity = 0.02;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.eos_type = EosType::TAIT;

    // set boundary and viscosity types
    if (boundary_type == "holmes") {
        sph_params.boundary_type = BoundaryType::HOLMES;
    } else {
        sph_params.boundary_type = BoundaryType::ADAMI;
    }

    if (viscosity_type == "laminar") {
        sph_params.viscosity_type = ViscosityType::LAMINAR;
    } else if (viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    }

    fsi.SetSPHParameters(sph_params);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(depth));

    // Create wavemaker actuation function
    ////auto fun = chrono_types::make_shared<WaveFunctionDecay>(0.2, 1.4);
    auto fun = chrono_types::make_shared<WaveFunction>(0.25, 0.2, 1);

    // Create wave tank with wavemaker mechanism
    ////auto body = fsi.ConstructWaveTank(ChFsiProblemSPH::WavemakerType::PISTON,                                    //
    ////                                  ChVector3d(0, 0, 0), csize, depth,                                         //
    ////                                  fun);                                                                      //
    auto body = fsi.ConstructWaveTank(ChFsiProblemSPH::WavemakerType::PISTON,                              //
                                      ChVector3d(0, 0, 0), csize, depth,                                   //
                                      fun,                                                                 //
                                      chrono_types::make_shared<WaveTankRampBeach>(x_start, 0.2), true);  //
    ////auto body = fsi.ConstructWaveTank(ChFsiProblemSPH::WavemakerType::PISTON,                                    //
    ////                                  ChVector3d(0, 0, 0), csize, depth,                                         //
    ////                                  fun,                                                                       //
    ////                                  chrono_types::make_shared<WaveTankParabolicBeach>(x_start, 0.32), false);  //

    fsi.Initialize();

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Wave_Tank/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + fsi.GetSphMethodTypeString() + "_" + viscosity_type + "_" + boundary_type + "_ps" +
              std::to_string(ps_freq);
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            cerr << "Error creating directory " << out_dir + "/particles" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            cerr << "Error creating directory " << out_dir + "/fsi" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            cerr << "Error creating directory " << out_dir + "/vtk" << endl;
            return 1;
        }
    }

    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
            return 1;
        }
    }

// Create a run-time visualizer
#ifndef CHRONO_VSG
    render = false;
#endif

    std::shared_ptr<ChFsiVisualizationSPH> visFSI;
    if (render) {
#ifdef CHRONO_VSG
        visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif

        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(
            ChColor(1, 0, 0), ChColor(0.14f, 0.44f, 0.7f), -1000, 3900);

        visFSI->SetTitle("Chrono::FSI Wave Tank");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, -12 * csize.y(), depth / 2), ChVector3d(0, 0, depth / 2));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->SetLightIntensity(0.9);
        visFSI->SetLightDirection(-CH_PI_2, CH_PI / 6);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualizationSPH::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualizationSPH::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Write results to a txt file
    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Extract FSI force on piston body
        auto force_body = fsi.GetFsiBodyForce(body).x();
        ofile << time << "\t" << force_body << "\n";

        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");
            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Call the FSI solver
        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/results.gpl");
    gplot.SetGrid();
    std::string speed_title = "Piston FSI force";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("force (N)");
    gplot.Plot(out_file, 1, 2, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}