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

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
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

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Container dimensions
ChVector3d csize(0.8, 0.8, 1.6);

// Dimensions of fluid domain
ChVector3d fsize(0.8, 0.8, 1.2);

// Object type
enum class ObjectType {SPHERE, CYLINDER};
ObjectType object_type = ObjectType::CYLINDER;

// Object density
double density = 500;

// Object initial height above floor (as a ratio of fluid height)
double initial_height = 1.05;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override {
        auto p = pos[n];
        return p.x < 0 || p.y < 0;
    }
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
    ChCLI cli(argv[0], "FSI object drop demo");

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
    double t_end = 3.0;
    bool output = false;
    double output_fps = 20;
    bool render = true;
    double render_fps = 400;
    bool verbose = true;
    bool snapshots = false;
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
    ChFluidSystemSPH& sysSPH = fsi.GetFluidSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set integration step size
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    // Set CFD fluid properties
    ChFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;

    sysSPH.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.num_bce_layers = 4;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 8.0;
    sph_params.xsph_coefficient = 0.5;
    sph_params.shifting_coefficient = 0.0;
    sph_params.artificial_viscosity = 0.03;
    sph_params.eos_type = EosType::TAIT;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;

    // Set boundary and viscosity types
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

    sysSPH.SetSPHParameters(sph_params);

    // Create the rigid body
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());

    auto body = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(body);
    body->SetName("object");
    body->SetFixed(false);
    body->EnableCollision(false);

    switch (object_type) {
        case ObjectType::SPHERE: {
            double radius = 0.12;
            auto mass = density * ChSphere::GetVolume(radius);
            auto inertia = mass * ChSphere::GetGyration(radius);

            body->SetPos(ChVector3d(0, 0, initial_height * fsize.z() + radius));
            body->SetRot(QUNIT);
            body->SetMass(mass);
            body->SetInertia(inertia);

            geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, radius, 0));

            break;
        }
        case ObjectType::CYLINDER: {
            double length = 0.20;
            double radius = 0.12;

            double mass = density * ChCylinder::GetVolume(radius, length);
            auto inertia = mass * ChCylinder::GetGyration(radius, length / 2);

            body->SetPos(ChVector3d(0, 0, initial_height * fsize.z() + radius));
            body->SetRot(QUNIT);
            body->SetMass(mass);
            body->SetInertia(inertia);

            geometry.coll_cylinders.push_back(
                utils::ChBodyGeometry::CylinderShape(VNULL, Q_ROTATE_Z_TO_X, radius, length));

            break;
        }
    }

    if (show_rigid)
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Add as an FSI body (create BCE markers on a grid)
    fsi.AddRigidBody(body, geometry, true, true);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(
        chrono_types::make_shared<DepthPressurePropertiesCallback>(sysSPH, fsize.z()));

    // Create SPH material and boundaries
    fsi.Construct(fsize,                          // length x width x depth
                  ChVector3d(0, 0, 0),            // position of bottom origin
                  BoxSide::ALL & ~BoxSide::Z_POS  // all boundaries except top
    );

    // Initialize FSI problem
    fsi.Initialize();

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Object_Drop/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + sysSPH.GetSphMethodTypeString() + "_" + viscosity_type + "_" + boundary_type + "_ps" +
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

        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 1.0);
        ////auto col_callback = chrono_types::make_shared<ParticleDensityColorCallback>(995, 1005);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(
            ChColor(1, 0, 0), ChColor(0.14f, 0.44f, 0.7f), -1000, 12000);

        visFSI->SetTitle("Chrono::FSI object drop");
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
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    double timer_CFD = 0;
    double timer_MBS = 0;
    double timer_FSI = 0;
    double timer_step = 0;

    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        auto body_height = body->GetPos().z();
        ofile << time << "\t" << body_height << "\n";

        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;

            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);

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
        sysFSI.DoStepDynamics(step_size);

        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBS += sysFSI.GetTimerMBS();
        timer_FSI += sysFSI.GetTimerFSI();
        timer_step += sysFSI.GetTimerStep();
        if (verbose && sim_frame == 2000) {
            cout << "Cummulative timers at time: " << time << endl;
            cout << "   timer CFD:  " << timer_CFD << endl;
            cout << "   timer MBS:  " << timer_MBS << endl;
            cout << "   timer FSI:  " << timer_FSI << endl;
            cout << "   timer step: " << timer_step << endl;
        }

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string speed_title = "Object height";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(out_file, 1, 2, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}