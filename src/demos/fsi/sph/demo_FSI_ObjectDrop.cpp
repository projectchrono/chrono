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
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Dimensions of fluid domain
ChVector3d fsize(0.8, 0.8, 1.2);

// Object type
enum class ObjectShape { BOX_PRIMITIVE, SPHERE_PRIMITIVE, CYLINDER_PRIMITIVE, MESH };
ObjectShape object_shape = ObjectShape::CYLINDER_PRIMITIVE;

// Mesh specification (for object_shape = ObjectShape::MESH)
std::string mesh_obj_filename = GetChronoDataFile("models/semicapsule.obj");
double mesh_scale = 1;
double mesh_bottom_offset = 0.1;
////std::string mesh_obj_filename = GetChronoDataFile("models/sphere.obj");
////double mesh_scale = 0.12;
////double mesh_bottom_offset = 0.12;

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
                     bool& use_variable_time_step,
                     std::string& boundary_method,
                     std::string& viscosity_method) {
    ChCLI cli(argv[0], "FSI object drop demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    std::string output_particle_data_str = output ? "true" : "false";
    cli.AddOption<std::string>("Output", "output_particle_data", "Enable collection of output files",
                               output_particle_data_str);
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    std::string snapshots_str = snapshots ? "true" : "false";
    cli.AddOption<std::string>("Visualization", "snapshots", "Enable writing snapshot image files", snapshots_str);

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    // options for boundary condition and viscosity type
    cli.AddOption<std::string>("Physics", "boundary_method", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_method",
                               "Viscosity type (laminar/artificial_unilateral/artificial_bilateral)",
                               "artificial_unilateral");
    std::string use_variable_time_step_str = use_variable_time_step ? "true" : "false";
    cli.AddOption<std::string>("Physics", "use_variable_time_step", "true/false to use variable time step",
                               use_variable_time_step_str);

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    t_end = cli.GetAsType<double>("t_end");

    verbose = !cli.GetAsType<bool>("quiet");
    output = parse_bool(cli.GetAsType<std::string>("output_particle_data"));
    render = !cli.GetAsType<bool>("no_vis");
    snapshots = parse_bool(cli.GetAsType<std::string>("snapshots"));

    output_fps = cli.GetAsType<double>("output_fps");
    render_fps = cli.GetAsType<double>("render_fps");

    ps_freq = cli.GetAsType<int>("ps_freq");

    boundary_method = cli.GetAsType<std::string>("boundary_method");
    viscosity_method = cli.GetAsType<std::string>("viscosity_method");
    use_variable_time_step = parse_bool(cli.GetAsType<std::string>("use_variable_time_step"));
    return true;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.025;
    // If variable time step is enabled, this step size is only used for the first time step
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
    bool use_variable_time_step = true;
    std::string boundary_method = "adami";
    std::string viscosity_method = "artificial_unilateral";
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, render, render_fps, snapshots, ps_freq,
                         use_variable_time_step, boundary_method, viscosity_method)) {
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

     // Meta-step (communication interval)
    double meta_time_step = 5 * step_size;

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;

    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    int num_bce_layers = 4;
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = num_bce_layers;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    // Compute max velocity as root of 2*g*h
    sph_params.max_velocity = 4.538;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.artificial_viscosity = 0.03;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.use_variable_time_step = use_variable_time_step;

    // Set boundary and viscosity types
    if (boundary_method == "holmes") {
        sph_params.boundary_method = BoundaryMethod::HOLMES;
    } else {
        sph_params.boundary_method = BoundaryMethod::ADAMI;
    }

    if (viscosity_method == "laminar") {
        sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    } else if (viscosity_method == "artificial_bilateral") {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    }

    fsi.SetSPHParameters(sph_params);

    // Set surface reconstruction parameters
    ChFsiFluidSystemSPH::SplashsurfParameters splashsurf_params;
    splashsurf_params.smoothing_length = 2.0;
    splashsurf_params.cube_size = 0.3;
    splashsurf_params.surface_threshold = 0.6;

    fsi.SetSplashsurfParameters(splashsurf_params);

    // Create the rigid body
    double bottom_offset = 0;
    double mass = 0;
    ChMatrix33d inertia;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    switch (object_shape) {
        case ObjectShape::BOX_PRIMITIVE: {
            ChVector3d size(0.20, 0.20, 0.10);
            bottom_offset = size.z() / 2;
            ChBox box(size);
            mass = density * box.GetVolume();
            inertia = mass * box.GetGyration();
            geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, box));
            break;
        }
        case ObjectShape::SPHERE_PRIMITIVE: {
            double radius = 0.12;
            bottom_offset = radius;
            ChSphere sphere(radius);
            mass = density * sphere.GetVolume();
            inertia = mass * sphere.GetGyration();
            geometry->coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere, 0));
            break;
        }
        case ObjectShape::CYLINDER_PRIMITIVE: {
            double radius = 0.12;
            double length = 0.20;
            bottom_offset = radius;
            ChCylinder cylinder(radius, length);
            mass = density * cylinder.GetVolume();
            inertia = mass * cylinder.GetGyration();
            geometry->coll_cylinders.push_back(
                utils::ChBodyGeometry::CylinderShape(VNULL, Q_ROTATE_Z_TO_X, cylinder, 0));
            break;
        }
        case ObjectShape::MESH: {
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_obj_filename, true, true);
            ChVector3d com;
            trimesh->ComputeMassProperties(true, mass, com, inertia, mesh_scale);
            mass *= density;
            inertia *= density;
            bottom_offset = mesh_bottom_offset;
            geometry->coll_meshes.push_back(
                utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, mesh_obj_filename, VNULL, mesh_scale, 0.01, 0));
            break;
        }
    }

    auto body = chrono_types::make_shared<ChBody>();
    double height = initial_height * fsize.z() + bottom_offset;
    body->SetName("object");
    body->SetPos(ChVector3d(0, 0, height));
    body->SetRot(QUNIT);
    body->SetMass(mass);
    body->SetInertia(inertia);
    body->SetFixed(false);
    body->EnableCollision(false);
    sysMBS.AddBody(body);

    if (show_rigid)
        geometry->CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Add as an FSI body
    fsi.AddRigidBody(body, geometry, true, true);

    cout << "FSI body: " << endl;
    cout << "   initial height = " << height << endl;
    cout << "   mass = " << mass << endl;
    cout << "   inertia\n" << inertia << endl;

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(fsize.z()));

    // Create SPH material and boundaries
    fsi.Construct(fsize,                          // length x width x depth
                  ChVector3d(0, 0, 0),            // position of bottom origin
                  BoxSide::ALL & ~BoxSide::Z_POS  // all boundaries except top
    );

    // Initialize FSI problem
    fsi.Initialize();

    auto domain_aabb = fsi.GetComputationalDomain();
    cout << "Computational domain: " << endl;
    cout << "   min: " << domain_aabb.min << endl;
    cout << "   max: " << domain_aabb.max << endl;

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Object_Drop/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + fsi.GetSphIntegrationSchemeString() + "_" + viscosity_method + "_" + boundary_method + "_ps" +
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

    if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
        cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/meshes"))) {
        cerr << "Error creating directory " << out_dir + "/meshes" << endl;
        return 1;
    }

    ////fsi.SaveInitialMarkers(out_dir);

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 1.0);
        ////auto col_callback = chrono_types::make_shared<ParticleDensityColorCallback>(995, 1005);
        auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(-1000, 12000, true);

        std::vector<MarkerPlanesVisibilityCallback::Plane> planes = {
            {VNULL, ChVector3d(1, 0, 0)},
            {VNULL, ChVector3d(0, 1, 0)},
        };
        auto mode = MarkerPlanesVisibilityCallback::Mode::ALL;
        auto vis_callback_SPH = chrono_types::make_shared<MarkerPlanesVisibilityCallback>(planes, mode);
        auto vis_callback_BCE = chrono_types::make_shared<MarkerPlanesVisibilityCallback>(planes, mode);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::RED_BLUE);
        visFSI->SetSPHVisibilityCallback(vis_callback_SPH);
        visFSI->SetBCEVisibilityCallback(vis_callback_BCE);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Object Drop");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(2.5 * fsize.x(), 2.5 * fsize.y(), 1.5 * fsize.z()),
                          ChVector3d(0, 0, 0.5 * fsize.z()));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

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

    std::string out_file;
    if (use_variable_time_step) {
        out_file = out_dir + "/results_variable_time_step.txt";
    } else {
        out_file = out_dir + "/results_fixed_time_step.txt";
    }
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        auto body_height = body->GetPos().z();
        ofile << time << "\t" << body_height << "\n";

        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");

            out_frame++;
        }

        // Render FSI system
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            if (render_frame >= 70 && render_frame < 80) {
                std::ostringstream meshname;
                meshname << "mesh_" << std::setw(5) << std::setfill('0') << render_frame + 1;
                fsi.WriteReconstructedSurface(out_dir + "/meshes", meshname.str(), true);
            }

            render_frame++;
        }
#endif

        // Advance dynamics of the FSI system to next communication time
        fsi.DoStepDynamics(meta_time_step);

        time += meta_time_step;
        sim_frame++;
    }
    timer.stop();
    ofile.close();
    std::cout << "End Time: " << t_end << std::endl;
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    // Write RTF file
    std::ofstream rtf_file;
    if (use_variable_time_step) {
        rtf_file.open(out_dir + "/rtf_variable_time_step.txt", std::ios::trunc);
    } else {
        rtf_file.open(out_dir + "/rtf_fixed_time_step.txt", std::ios::trunc);
    }
    rtf_file << "time (s)"
             << "\t"
             << "wall clock time (s)"
             << "\t"
             << "RTF" << endl;
    double rtf = timer() / t_end;
    rtf_file << t_end << "\t" << timer() << "\t" << rtf << endl;
    rtf_file.close();

    if (use_variable_time_step) {
        fsi.PrintStats();
        fsi.PrintTimeSteps(out_dir + "/time_steps.txt");
    }

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