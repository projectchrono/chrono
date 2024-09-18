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
// Author: Huzaifa Unjhawala, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChFsiProblem.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------

// Run-time visualization system (VSG, OpenGL, or NONE)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Baffle_Flow";

// Output frequency
bool output = false;
unsigned int output_fps = 100;

// Container dimensions
ChVector3d csize(1.6, 1.4, 0.16);

// Size of the baffles
ChVector3d bsize(0.1, 0.1, 0.16);

// Baffle locations
ChVector3d bloc1(0, 0.3, 0);
ChVector3d bloc2(0, -0.3, 0);
ChVector3d bloc3(0.4, 0, 0);

// Initial size of SPH material
ChVector3d fsize(0.2, 0.8, 0.14);

// Final simulation time
double t_end = 0.7;

// Enable/disable run-time visualization
bool render = true;
double render_fps = 300;

// Enable saving snapshots
bool snapshots = false;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = false;
bool show_particles_sph = true;

// ----------------------------------------------------------------------------
// Callback for setting initial SPH particle properties
class SPHPropertiesCallback : public ChFsiProblem::ParticlePropertiesCallback {
  public:
    SPHPropertiesCallback(const ChSystemFsi& sysFSI, double zero_height, const ChVector3d& init_velocity)
        : ParticlePropertiesCallback(sysFSI), zero_height(zero_height), init_velocity(init_velocity) {
        gz = std::abs(sysFSI.GetGravitationalAcceleration().z());
        c2 = sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed();
    }

    virtual void set(const ChVector3d& pos) override {
        p0 = sysFSI.GetDensity() * gz * (zero_height - pos.z());
        rho0 = sysFSI.GetDensity() + p0 / c2;
        mu0 = sysFSI.GetViscosity();
        v0 = init_velocity;
    }

    double zero_height;
    ChVector3d init_velocity;
    double gz;
    double c2;
};

// ----------------------------------------------------------------------------

void CreateBaffles(ChFsiProblem& fsi) {
    ChSystem& sysMBS = fsi.GetSystyemMBS();

    // Common contact material and geometry
    ChContactMaterialData cmat;
    cmat.Y = 1e8f;
    cmat.mu = 0.2f;
    cmat.cr = 0.05f;

    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(cmat);
    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0.5 * bsize.z()), QUNIT, bsize, 0));

    auto baffle1 = chrono_types::make_shared<ChBody>();
    baffle1->SetPos(bloc1);
    baffle1->SetRot(QUNIT);
    baffle1->SetFixed(true);
    sysMBS.AddBody(baffle1);
    if (show_rigid)
        geometry.CreateVisualizationAssets(baffle1, VisualizationType::COLLISION);
    fsi.AddRigidBody(baffle1, geometry, false);

    auto baffle2 = chrono_types::make_shared<ChBody>();
    baffle2->SetPos(bloc2);
    baffle2->SetRot(QUNIT);
    baffle2->SetFixed(true);
    sysMBS.AddBody(baffle2);
    if (show_rigid)
        geometry.CreateVisualizationAssets(baffle2, VisualizationType::COLLISION);
    fsi.AddRigidBody(baffle2, geometry, false);

    auto baffle3 = chrono_types::make_shared<ChBody>();
    baffle3->SetPos(bloc3);
    baffle3->SetRot(QUNIT);
    baffle3->SetFixed(true);
    sysMBS.AddBody(baffle3);
    if (show_rigid)
        geometry.CreateVisualizationAssets(baffle3, VisualizationType::COLLISION);
    fsi.AddRigidBody(baffle3, geometry, false);
}

// ----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     bool& verbose,
                     bool& output,
                     unsigned int& output_fps,
                     bool& render,
                     double& render_fps,
                     bool& snapshots,
                     int& ps_freq) {
    ChCLI cli(argv[0], "Baffle Flow FSI demo");

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("Output", "output", "Enable collection of output files");
    cli.AddOption<unsigned int>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));
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
    output_fps = cli.GetAsType<unsigned int>("output_fps");
    render_fps = cli.GetAsType<double>("render_fps");
    ps_freq = cli.GetAsType<int>("ps_freq");

    return true;
}

// ----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.01;
    double step_size = 1e-4;
    bool verbose = true;
    int ps_freq = 1;  // Frequency of Proximity Search
    if (!GetProblemSpecs(argc, argv, verbose, output, output_fps, render, render_fps, snapshots, ps_freq)) {
        return 1;
    }

    out_dir = out_dir + std::to_string(ps_freq) + "/";

    // Create the Chrono system and associated collision system
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChSystemFsi& sysFSI = fsi.GetSystemFSI();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set soil propertiees
    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.density = 1800;
    mat_props.Young_modulus = 2e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.05;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.0614;
    mat_props.friction_angle = CH_PI / 10;  // default
    mat_props.dilation_angle = CH_PI / 10;  // default
    mat_props.cohesion_coeff = 0;           // default

    sysFSI.SetElasticSPH(mat_props);

    // Set SPH solution parameters
    ChSystemFsi::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.kernel_h = 0.012;
    sph_params.initial_spacing = initial_spacing;
    sph_params.xsph_coefficient = 0.5;
    sph_params.shifting_coefficient = 1.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.num_proximity_search_steps = ps_freq;

    sysFSI.SetSPHParameters(sph_params);

    // Create rigid bodies
    CreateBaffles(fsi);

    // Enable height-based initial pressure for SPH particles
    ChVector3 v0(2, 0, 0);
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<SPHPropertiesCallback>(sysFSI, fsize.z(), v0));

    // Create SPH material (do not create boundary BCEs)
    fsi.Construct(fsize,                                                                          // box dimensions
                  ChVector3d(bloc1.x() - bsize.x() / 2 - fsize.x() / 2 - initial_spacing, 0, 0),  // reference location
                  false,                                                                          // bottom wall?
                  false                                                                           // side walls?
    );

    // Create container
    bool side_walls = false;
    fsi.AddBoxContainer(csize,                // length x width x height
                        ChVector3d(0, 0, 0),  // reference location
                        true,                 // bottom wall
                        side_walls,           // side walls
                        false                 // top wall
    );

    if (show_rigid) {
        ChVector3d ground_box_size(csize.x(), csize.y(), 0.02);
        ChVector3d ground_box_loc(0, 0, -initial_spacing - 0.01);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(ground_box_size);
        fsi.GetGroundBody()->AddVisualShape(vis_shape, ChFramed(ground_box_loc, QUNIT));
    }

    fsi.Initialize();
    if (output || snapshots) {
        if (output) {
            // Create output directories
            if (!filesystem::create_directory(filesystem::path(out_dir))) {
                std::cerr << "Error creating directory " << out_dir << std::endl;
                return 1;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return 1;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
                std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
                return 1;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
                std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
                return 1;
            }
        }
        if (snapshots) {
            if (!output) {
                // Create output directories
                if (!filesystem::create_directory(filesystem::path(out_dir))) {
                    std::cerr << "Error creating directory " << out_dir << std::endl;
                    return 1;
                }
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

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

        auto col_callback = chrono_types::make_shared<VelocityColorCallback>(0, v0.Length());

        visFSI->SetTitle("Chrono::FSI baffle flow");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(1.5, -1.5, 0.5), ChVector3d(0, 0, 0));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render SPH particles
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        if (sim_frame % 1000 == 0) {
            std::cout << "step: " << sim_frame << "\ttime: " << time << "\tRTF: " << sysFSI.GetRTF() << std::endl;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}