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
#include <fstream>

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_fsi/ChFsiProblem.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Sphere_Bounce";

// Container dimensions
ChVector3d csize(0.8, 0.8, 1.6);

// Dimensions of fluid domain
ChVector3d fsize(0.8, 0.8, 1.2);

// Sphere density
double density = 500;

// Sphere initial height
double initial_height = 0.95;

// Final simulation time
double t_end = 3.0;

// Output frequency
bool output = false;
double output_fps = 20;

// Enable/disable run-time visualization
bool render = true;
double render_fps = 400;

// Enable saving snapshots
bool snapshots = false;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = false;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

class PositionVisibilityCallback : public ChParticleCloud::VisibilityCallback {
  public:
    PositionVisibilityCallback() {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        auto p = cloud.GetParticlePos(n);
        return p.x() < 0 || p.y() < 0;
    };
};

// -----------------------------------------------------------------------------
bool GetProblemSpecs(int argc,
                     char** argv,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& render,
                     double& render_fps,
                     bool& snapshots,
                     int& ps_freq) {
    ChCLI cli(argv[0], "FSI Sphere Bounce demo");

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("Output", "output", "Enable collection of output files");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    cli.AddOption<bool>("Visualization", "snapshots", "Enable writing snapshot image files");

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    verbose = !cli.GetAsType<bool>("quiet");
    output = cli.GetAsType<bool>("output");
    render = !cli.GetAsType<bool>("no_vis");
    snapshots = cli.GetAsType<bool>("snapshots");

    output_fps = cli.GetAsType<double>("output_fps");
    render_fps = cli.GetAsType<double>("render_fps");

    ps_freq = cli.GetAsType<int>("ps_freq");

    return true;
}

int main(int argc, char* argv[]) {
    double initial_spacing = 0.025;
    double step_size = 1e-4;
    bool verbose = true;
    int ps_freq = 1;

    if (!GetProblemSpecs(argc, argv, verbose, output, output_fps, render, render_fps, snapshots, ps_freq)) {
        return 1;
    }
    out_dir = out_dir + std::to_string(ps_freq);

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChSystemFsi& sysFSI = fsi.GetSystemFSI();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set CFD fluid properties
    ChSystemFsi::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;

    sysFSI.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChSystemFsi::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.num_bce_layers = 4;
    sph_params.kernel_h = initial_spacing;
    sph_params.initial_spacing = initial_spacing;
    sph_params.max_velocity = 1.0;
    sph_params.xsph_coefficient = 0.5;
    sph_params.shifting_coefficient = 0.0;
    sph_params.density_reinit_steps = 1000;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = ps_freq;

    sysFSI.SetSPHParameters(sph_params);
    sysFSI.SetStepSize(step_size);

    // Create a rigid body
    double radius = 0.12;
    auto mass = density * ChSphere::GetVolume(radius);
    auto inertia = mass * ChSphere::GetGyration(radius);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("ball");
    body->SetPos(ChVector3d(0, 0, initial_height));
    body->SetRot(QUNIT);
    body->SetMass(mass);
    body->SetInertia(inertia);
    body->SetFixed(false);
    body->EnableCollision(false);
    sysMBS.AddBody(body);

    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, radius, 0));
    if (show_rigid)
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Add as an FSI body (create BCE markers on a grid)
    fsi.AddRigidBody(body, geometry, true, true);

    // Enable height-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(
        chrono_types::make_shared<DepthPressurePropertiesCallback>(sysFSI, fsize.z()));

    // Create SPH material (do not create boundary BCEs)
    fsi.Construct(fsize,                // length x width x depth
                  ChVector3d(0, 0, 0),  // position of bottom origin
                  false,                // bottom wall?
                  false                 // side walls?
    );

    // Add box container (with bottom, side, and top walls)
    fsi.AddBoxContainer(csize, ChVector3d(0, 0, 0), true, true, true);

    fsi.Initialize();
    // Output file for plotting the sphere vertical position using an external tool

    if (output || snapshots) {
        if (output) {
            // Create oputput directories
            if (!filesystem::create_directory(filesystem::path(out_dir))) {
                cerr << "Error creating directory " << out_dir << endl;
                return 1;
            }
            out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();
            if (!filesystem::create_directory(filesystem::path(out_dir))) {
                cerr << "Error creating directory " << out_dir << endl;
                return 1;
            }
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
            if (!output) {
                // Create output directories if it does not exist
                if (!filesystem::create_directory(filesystem::path(out_dir))) {
                    cerr << "Error creating directory " << out_dir << endl;
                    return 1;
                }
                out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();
                if (!filesystem::create_directory(filesystem::path(out_dir))) {
                    cerr << "Error creating directory " << out_dir << endl;
                    return 1;
                }
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
                return 1;
            }
        }
    }

    ////fsi.SaveInitialMarkers(out_dir);

    // Create a run-time visualizer
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        auto col_callback = chrono_types::make_shared<VelocityColorCallback>(0, 1.0);
        auto vis_callback = chrono_types::make_shared<PositionVisibilityCallback>();

        visFSI->SetTitle("Chrono::FSI Sphere Bounce Test");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(2.5 * fsize.x(), 2.5 * fsize.y(), 1.5 * fsize.z()),
                          ChVector3d(0, 0, 0.5 * fsize.z()));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(vis_callback);
        visFSI->SetBCEVisibilityCallback(vis_callback);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Record sphere height
    ChFunctionInterp height_recorder;

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile;
    if (output) {
        ofile.open(out_file, std::ios::trunc);
    }

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);

            auto body_height = body->GetPos().z();
            ofile << time << "\t" << body_height << "\n";

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

        auto body_height = body->GetPos().z();
        height_recorder.AddPoint(time, body_height);
        ////cout << "step: " << sim_frame << "\ttime: " << time << "\tRTF: " << sysFSI.GetRTF()
        ////     << "\tbody z: " << body_height << endl;

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    if (output)
        ofile.close();

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string speed_title = "Sphere height";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(height_recorder, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}
