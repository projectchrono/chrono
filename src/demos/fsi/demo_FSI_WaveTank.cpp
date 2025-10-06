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

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChElementShellANCF_3423.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#else
    #include "chrono/solver/ChIterativeSolverLS.h"
#endif

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fea;
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

// Create FSI flexible solids
bool create_rigid_post = false;
bool create_flex_cable = false;
bool create_flex_plate = false;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

// Color coding
enum ColorCode { VELOCITY, PRESSURE };
ColorCode color_code = ColorCode::VELOCITY;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

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
                     bool& use_variable_time_step,
                     std::string& boundary_method,
                     std::string& viscosity_method) {
    ChCLI cli(argv[0], "Wave Tank FSI demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    std::string output_str = output ? "true" : "false";
    cli.AddOption<std::string>("Output", "output_particle_data", "Enable collection of output files", output_str);
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    std::string snapshots_str = snapshots ? "true" : "false";
    cli.AddOption<std::string>("Visualization", "snapshots", "Enable writing snapshot image files", snapshots_str);

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    cli.AddOption<std::string>("Physics", "boundary_method", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_method",
                               "Viscosity type (laminar/artificial_unilateral/artificial_bilateral)",
                               "artificial_unilateral");

    // Set the default
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

std::shared_ptr<ChBody> CreateRigidPost(ChSystem& sysMBS, const ChVector3d& loc, double length) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("Post");
    body->SetPos(loc + ChVector3d(0, 0, length / 2));
    body->SetFixed(true);
    body->EnableCollision(false);

    sysMBS.AddBody(body);

    return body;
}

std::shared_ptr<ChMesh> CreateFlexibleCable(ChSystem& sysMBS, const ChVector3d& loc, double length, double radius) {
    // Cable properties
    double element_length = 0.05;
    int num_elements = std::round(length / element_length);
    double density = 8000;
    double E = 1e9;
    double Rayleigh_damping = 0.2;

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Create beam section and construct beam
    auto section_cable = chrono_types::make_shared<ChBeamSectionCable>();
    section_cable->SetDiameter(2 * radius);
    section_cable->SetYoungModulus(E);
    section_cable->SetDensity(density);
    section_cable->SetRayleighDamping(Rayleigh_damping);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    mesh->SetName("Cable");
    std::vector<std::vector<int>> node_indices;
    std::vector<std::vector<int>> node_nbrs;
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh,                            // FEA mesh with nodes and elements
                      section_cable,                   // section material for cable elements
                      num_elements,                    // number of elements in the segment
                      loc,                             // beam start point
                      loc + ChVector3d(0, 0, length),  // beam end point
                      node_indices,                    // node indices
                      node_nbrs                        // neighbor node indices
    );

    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(builder.GetLastBeamNodes().front());
    auto pos_const = chrono_types::make_shared<ChLinkNodeFrame>();
    pos_const->Initialize(node, ground);
    sysMBS.Add(pos_const);

    auto dir_const = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    dir_const->Initialize(node, ground);
    dir_const->SetDirectionInAbsoluteCoords(node->GetSlope1());
    sysMBS.Add(dir_const);

    // Add FEA visualization
    auto vis_cable = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_cable->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_cable->SetColormapRange(-0.4, 0.4);
    vis_cable->SetSmoothFaces(true);
    vis_cable->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_cable);

    sysMBS.Add(mesh);

    return mesh;
}

std::shared_ptr<ChMesh> CreateFlexiblePlate(ChSystem& sysMBS, const ChVector3d& loc, double length, double width) {
    // Plate properties
    double thickness = 0.02;

    double rho = 8000;
    double E = 2e7;
    double nu = 0.3;

    int numDiv_y = 4;
    int numDiv_z = 15;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    ChVector3d center_plate = loc + ChVector3d(0, 0, length / 2);

    // For uniform mesh
    double dy = width / numDiv_y;
    double dz = length / numDiv_z;

    // Create an FEA mesh representing a cantilever plate modeled with ANCF shell elements
    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    mesh->SetName("Plate");

    std::vector<std::shared_ptr<ChNodeFEAbase>> collision_nodes;

    // Create and add the nodes
    ChVector3d pos;
    ChVector3d dir(1, 0, 0);
    for (int k = 0; k < N_z; k++) {
        for (int j = 0; j < N_y; j++) {
            pos.x() = center_plate.x();
            pos.y() = j * dy - width / 2 + center_plate.y();
            pos.z() = k * dz - length / 2 + center_plate.z();

            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(pos, dir);
            node->SetMass(0);

            // Set fixed nodes (no collision) and collect nodes with collision
            if (k == 0)
                node->SetFixed(true);
            else
                collision_nodes.push_back(node);

            mesh->AddNode(node);
        }
    }

    // Create an isotropic material; all layers for all elements share the same material
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create the elements
    int num_elem = 0;
    for (int k = 0; k < numDiv_z; k++) {
        for (int j = 0; j < numDiv_y; j++) {
            int node0 = (j + 0) + N_y * (k + 0);
            int node1 = (j + 1) + N_y * (k + 0);
            int node2 = (j + 1) + N_y * (k + 1);
            int node3 = (j + 0) + N_y * (k + 1);

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

            // Set element dimensions
            element->SetDimensions(dy, dz);

            // Add a single layers with a fiber angle of 0 degrees
            element->AddLayer(thickness, 0, mat);

            // Set structural damping for this element
            element->SetAlphaDamp(0.05);

            // Add element to mesh
            mesh->AddElement(element);
            ChVector3d center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                        element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
            ////cout << "Add element " << num_elem << " with center:  " << center << endl;
            num_elem++;
        }
    }

    // Create the FEA contact surface
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    auto contact_surface = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_surface->AddNodesFromNodeSet(collision_nodes, 0.01);
    mesh->AddContactSurface(contact_surface);

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    // Add FEA visualization
    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColormapRange(0.0, 3.0);
    vis_mesh->SetShrinkElements(true, 0.85);
    vis_mesh->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.025;
    // If variable time step is enabled, this step size is only used for the first time step
    double step_size_CFD = 1e-4;
    double step_size_MBD = (create_flex_cable || create_flex_plate) ? 1e-5 : 1e-4;

     // Meta-step (communication interval)
    double meta_time_step = 5 * std::max(step_size_CFD, step_size_MBD);

    // Parse command line arguments
    double t_end = 12.0;
    bool verbose = true;
    bool output = true;
    double output_fps = 20;
    bool render = true;
    double render_fps = 400;
    bool snapshots = false;
    int ps_freq = 1;
    std::string boundary_method = "adami";
    bool use_variable_time_step = true;
    std::string viscosity_method = "artificial_unilateral";
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, render, render_fps, snapshots, ps_freq,
                         use_variable_time_step, boundary_method, viscosity_method)) {
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemWavetank fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size_CFD);
    fsi.SetStepsizeMBD(step_size_MBD);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 4.0;  // maximum velocity should be 2*sqrt(grav * fluid_height)
    // sph_params.shifting_method = ShiftingMethod::XSPH;
    // sph_params.shifting_xsph_eps = 0.5;

    sph_params.shifting_method = ShiftingMethod::DIFFUSION;
    sph_params.shifting_diffusion_A = 1.;
    sph_params.shifting_diffusion_AFSM = 3.;
    sph_params.shifting_diffusion_AFST = 2.;

    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.artificial_viscosity = 0.02;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_variable_time_step = use_variable_time_step;

    // set boundary and viscosity types
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

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(depth));

    // Create wavemaker actuation function
    ////auto fun = chrono_types::make_shared<WaveFunctionDecay>(0.2, 1.4);
    auto fun = chrono_types::make_shared<WaveFunction>(0.25, 0.2, 1);

    // Create tank bottom profile (flat bottom if none specified)
    fsi.SetProfile(chrono_types::make_shared<WaveTankRampBeach>(x_start, 0.2), true);
    ////fsi.SetProfile(chrono_types::make_shared<WaveTankParabolicBeach>(x_start, 0.32), false);

    // Optionally, replace lateral walls with period BC
    ////fsi.SetLateralPeriodicBC(true);

    // Create wave tank with wavemaker mechanism
    auto body = fsi.ConstructWaveTank(ChFsiProblemWavetank::WavemakerType::PISTON,  //
                                      ChVector3d(0, 0, 0), csize, depth,            //
                                      fun);                                         //

    // Create solid objects
    if (create_rigid_post) {
        double length = 0.8;
        double radius = 0.02;
        auto body = CreateRigidPost(fsi.GetMultibodySystem(), ChVector3d(-csize.x() / 4, 0, 0), length);
        auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, QUNIT, radius, length));
        geometry->CreateVisualizationAssets(body, VisualizationType::COLLISION);
        fsi.AddRigidBody(body, geometry, true);
    } else if (create_flex_cable) {
        double length = 0.8;
        double radius = 0.01;
        auto mesh = CreateFlexibleCable(fsi.GetMultibodySystem(), ChVector3d(-csize.x() / 4, 0, 0), length, radius);
        fsi.SetBcePattern1D(BcePatternMesh1D::FULL, false);
        fsi.AddFeaMesh(mesh, true);
    } else if (create_flex_plate) {
        double length = 0.8;
        double width = csize.y() / 2;
        auto mesh = CreateFlexiblePlate(fsi.GetMultibodySystem(), ChVector3d(-csize.x() / 4, 0, 0), length, width);
        fsi.SetBcePattern2D(BcePatternMesh2D::CENTERED, false);
        fsi.AddFeaMesh(mesh, true);
    }

    // Complete construction of the FSI problem
    fsi.Initialize();

    // Output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Wave_Tank/";
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

    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
            return 1;
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        std::shared_ptr<ChSphVisualizationVSG::ParticleColorCallback> col_callback;
        ChColormap::Type col_map;

        switch (color_code) {
            case ColorCode::VELOCITY:
                col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0);
                col_map = ChColormap::Type::FAST;
                break;
            default:
            case ColorCode::PRESSURE:
                col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(-1000, 3900, true);
                col_map = ChColormap::Type::RED_BLUE;
                break;
        }

        // FSI plugin
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, col_map);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Wave Tank");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -12 * csize.y(), depth / 2), ChVector3d(0, 0, depth / 2));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // SetMBS threads
    int num_threads_chrono = std::min(4, ChOMP::GetNumProcs() / 2);
    int num_threads_pardiso = std::min(4, ChOMP::GetNumProcs() / 2);
    int num_threads_collision = 1;
    int num_threads_eigen = 1;
    sysMBS.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    cout << "Set num. threads" << endl;
    cout << "   num_threads_chrono:    " << num_threads_chrono << endl;
    cout << "   num_threads_pardiso:   " << num_threads_pardiso << endl;
    cout << "   num_threads_collision: " << num_threads_collision << endl;
    cout << "   num_threads_eigen:     " << num_threads_eigen << endl;

    // Set MBS solver
    if (create_flex_cable || create_flex_plate) {
#ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(num_threads_pardiso);
        mkl_solver->LockSparsityPattern(true);
        sysMBS.SetSolver(mkl_solver);
#else
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        sysMBS.SetSolver(solver);
        solver->SetMaxIterations(2000);
        solver->SetTolerance(1e-12);
        solver->EnableDiagonalPreconditioner(true);
        solver->SetVerbose(false);
#endif
    }

    // Create output file
    std::string out_file;
    if (use_variable_time_step) {
        out_file = out_dir + "/results_variable_time_step.txt";
    } else {
        out_file = out_dir + "/results_fixed_time_step.txt";
    }
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

    // Write an RTF file
    std::ofstream rtf_file;
    if (use_variable_time_step) {
        rtf_file.open(out_dir + "/rtf_variable_time_step.txt", std::ios::trunc);
    } else {
        rtf_file.open(out_dir + "/rtf_fixed_time_step.txt", std::ios::trunc);
    }
    // Write header
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