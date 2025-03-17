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
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------

// Container dimensions
ChVector3d csize(1.6, 1.4, 0.5);

// Size of the baffles
ChVector3d bsize(0.1, 0.1, 0.16);

// Baffle locations
ChVector3d bloc1(0, 0.3, 0);
ChVector3d bloc2(0, -0.3, 0);
ChVector3d bloc3(0.4, 0, 0);

// Initial size of SPH material
ChVector3d fsize(0.2, 0.8, 0.14);

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = false;
bool show_particles_sph = true;

// ----------------------------------------------------------------------------

// Callback for setting initial SPH particle properties
class SPHPropertiesCallback : public ChFsiProblemSPH::ParticlePropertiesCallback {
  public:
    SPHPropertiesCallback(double zero_height, const ChVector3d& init_velocity)
        : ParticlePropertiesCallback(), zero_height(zero_height), init_velocity(init_velocity) {}

    virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override {
        double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
        double c2 = sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed();
        p0 = sysSPH.GetDensity() * gz * (zero_height - pos.z());
        rho0 = sysSPH.GetDensity() + p0 / c2;
        mu0 = sysSPH.GetViscosity();
        v0 = init_velocity;
    }

    double zero_height;
    ChVector3d init_velocity;
};

// ----------------------------------------------------------------------------

void CreateBaffles(ChFsiProblemSPH& fsi) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

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
    ChCLI cli(argv[0], "Baffle Flow FSI demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("Output", "output", "Enable collection of output files");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    cli.AddOption<bool>("Visualization", "snapshots", "Enable writing snapshot image files");

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    // options for boundary condition and viscosity type
    cli.AddOption<std::string>("Physics", "boundary_type", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", "artificial_unilateral");

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

// ----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.01;
    double step_size = 1e-4;

    // Parse command line arguments
    double t_end = 0.7;
    bool verbose = true;
    bool output = false;
    double output_fps = 100;
    bool render = true;
    double render_fps = 300;
    bool snapshots = false;
    int ps_freq = 1;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_unilateral";
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, render, render_fps, snapshots, ps_freq,
                         boundary_type, viscosity_type)) {
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemSMC sysMBS;
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

    // Set soil propertiees
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1800;
    mat_props.Young_modulus = 2e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.0614;
    mat_props.cohesion_coeff = 0;  // default

    fsi.SetElasticSPH(mat_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.artificial_viscosity = 0.1;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.num_proximity_search_steps = ps_freq;

    // Set boundary type
    if (boundary_type == "holmes") {
        sph_params.boundary_type = BoundaryType::HOLMES;
    } else {
        sph_params.boundary_type = BoundaryType::ADAMI;
    }

    // Set viscosity type
    if (viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    }

    fsi.SetSPHParameters(sph_params);

    // Create rigid bodies
    CreateBaffles(fsi);

    // Enable depth-based initial pressure for SPH particles
    ChVector3d v0(1, 0, 0);
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<SPHPropertiesCallback>(fsize.z(), v0));

    // Create SPH material (do not create boundary BCEs)
    fsi.Construct(fsize,                                                                          // box dimensions
                  ChVector3d(bloc1.x() - bsize.x() / 2 - fsize.x() / 2 - initial_spacing, 0, 0),  // reference location
                  BoxSide::NONE                                                                   // no boundary BCEs
    );

    // Create container
    fsi.AddBoxContainer(csize,                // length x width x height
                        ChVector3d(0, 0, 0),  // reference location
                        BoxSide::Z_NEG        // creater only bottom boundary
    );

    // Explicitly set computational domain (necessary if no side walls)
    ChAABB aabb(ChVector3d(-csize.x() / 2, -csize.y() / 2, -0.1),
                ChVector3d(+csize.x() / 2, +csize.y() / 2, +0.1 + csize.z()));
    fsi.SetComputationalDomain(aabb, PeriodicSide::NONE);

    if (show_rigid) {
        ChVector3d ground_box_size(csize.x(), csize.y(), 0.02);
        ChVector3d ground_box_loc(0, 0, -initial_spacing - 0.01);
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(ground_box_size);
        fsi.GetGroundBody()->AddVisualShape(vis_shape, ChFramed(ground_box_loc, QUNIT));
    }

    fsi.Initialize();

    // Output directories
    std::string out_dir;
    if (output || snapshots) {
        out_dir = GetChronoOutputPath() + "FSI_Baffle_Flow/";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        out_dir = out_dir + viscosity_type + "_" + boundary_type + std::to_string(ps_freq);
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (output) {
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
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, v0.Length());

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Baffle Flow");
        visVSG->SetWindowSize(1280, 720);
        visVSG->SetWindowPosition(400, 400);
        visVSG->AddCamera(ChVector3d(1.5, -1.5, 0.5), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
        visVSG->SetWireFrameMode(false);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");
            out_frame++;
        }

        // Render SPH particles
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        if (sim_frame % 1000 == 0) {
            std::cout << "step: " << sim_frame << "\ttime: " << time << "\tRTF_fluid: " << fsi.GetRtfCFD()
                      << "\tRTF_solid: " << fsi.GetRtfMBD() << std::endl;
        }

        // Call the FSI solver
        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}