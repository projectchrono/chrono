// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Wei Hu, Pei Li, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Physics problem type
PhysicsProblem problem_type = PhysicsProblem::CRM;

// Dimension of the domain
double cxDim = 3.0;
double cyDim = 0.2;
double czDim = 2.0;

// Create additional solids
bool create_flex_cable2 = false;
bool create_cylinder_post = true;
bool create_cylinder_free = true;

// Use nodal directions
NodeDirectionsMode FEA_node_directions_mode = NodeDirectionsMode::NONE;

// Visibility flags
bool show_rigid_bce = false;
bool show_mesh_bce = false;
bool show_boundary_bce = false;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiProblemSPH& fsi);
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
                     std::string& boundary_method,
                     std::string& viscosity_method);

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Parse command line arguments
    double t_end = 10.0;
    bool verbose = true;
    bool output = false;
    double output_fps = 20;
    bool render = true;
    double render_fps = 400;
    bool snapshots = false;
    int ps_freq = 1;
    std::string boundary_method = "adami";
    std::string viscosity_method =
        (problem_type == PhysicsProblem::CFD) ? "artificial_unilateral" : "artificial_bilateral";
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, render, render_fps, snapshots, ps_freq,
                         boundary_method, viscosity_method)) {
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    double initial_spacing = (problem_type == PhysicsProblem::CFD) ? 0.02 : 0.01;

    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.81);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    double step_size = (problem_type == PhysicsProblem::CFD) ? 2e-5 : 2.5e-4;
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set fluid phase properties
    switch (problem_type) {
        case PhysicsProblem::CFD: {
            ChFsiFluidSystemSPH::FluidProperties fluid_props;
            fluid_props.density = 1000;
            fluid_props.viscosity = 5.0;

            fsi.SetCfdSPH(fluid_props);

            break;
        }
        case PhysicsProblem::CRM: {
            ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
            mat_props.density = 1700;
            mat_props.Young_modulus = 1e6;
            mat_props.Poisson_ratio = 0.3;
            mat_props.mu_I0 = 0.03;
            mat_props.mu_fric_s = 0.5;
            mat_props.mu_fric_2 = 0.5;
            mat_props.average_diam = 0.005;
            mat_props.cohesion_coeff = 0;

            fsi.SetElasticSPH(mat_props);

            break;
        }
    }

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    // Enable/disable use of node directions for FSI flexible meshes
    fsi.UseNodeDirections(FEA_node_directions_mode);

    switch (problem_type) {
        case PhysicsProblem::CFD:
            sph_params.integration_scheme = IntegrationScheme::RK2;
            sph_params.initial_spacing = initial_spacing;
            sph_params.d0_multiplier = 1.0;
            sph_params.max_velocity = 10;
            sph_params.shifting_method = ShiftingMethod::XSPH;
            sph_params.shifting_xsph_eps = 0.5;
            sph_params.free_surface_threshold = 0.8;
            sph_params.artificial_viscosity = 0.02;
            sph_params.use_delta_sph = true;
            sph_params.delta_sph_coefficient = 0.1;
            sph_params.num_proximity_search_steps = ps_freq;

            break;

        case PhysicsProblem::CRM:
            sph_params.integration_scheme = IntegrationScheme::RK2;
            sph_params.initial_spacing = initial_spacing;
            sph_params.d0_multiplier = 1.0;
            sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
            sph_params.shifting_xsph_eps = 0.25;
            sph_params.shifting_ppst_pull = 1.0;
            sph_params.shifting_ppst_push = 3.0;
            sph_params.free_surface_threshold = 0.8;
            sph_params.artificial_viscosity = 0.5;
            sph_params.num_proximity_search_steps = ps_freq;

            break;
    }

    if (boundary_method == "holmes")
        sph_params.boundary_method = BoundaryMethod::HOLMES;
    else
        sph_params.boundary_method = BoundaryMethod::ADAMI;

    if (viscosity_method == "laminar")
        sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    else if (viscosity_method == "artificial_bilateral")
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    else
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;

    fsi.SetSPHParameters(sph_params);

    // Create FSI solid bodies
    auto mesh = CreateSolidPhase(fsi);

    // Dimension of the fluid domain
    double fxDim = 1.0;
    double fyDim = 0.2;
    double fzDim = 1.4;

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(fzDim));

    // Create SPH material (do not create any boundary BCEs)
    fsi.Construct({fxDim, fyDim, fzDim},           // box dimensions
                  {-cxDim / 2 + fxDim / 2, 0, 0},  // reference location
                  BoxSide::NONE                    // no boundary BCEs
    );

    // Create container (with bottom and left/right boundaries)
    fsi.AddBoxContainer({cxDim, cyDim, czDim},                            // length x width x height
                        ChVector3d(0, 0, 0),                              // reference location
                        BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS  // bottom and left/right walls
    );

    // Explicitly set computational domain (necessary if no side walls)
    ChVector3d cMin = ChVector3d(-5 * cxDim, -cyDim / 2 - initial_spacing / 2, -5 * czDim);
    ChVector3d cMax = ChVector3d(+5 * cxDim, +cyDim / 2 + initial_spacing / 2, +5 * czDim);
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    // Initialize FSI problem
    fsi.Initialize();

    // Create oputput directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Cable/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + fsi.GetPhysicsProblemString() + "_" + fsi.GetSphIntegrationSchemeString() + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + viscosity_method + "_" + boundary_method + "_ps" + std::to_string(ps_freq);
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

    // Create a run-tme visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.5);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->EnableFlexBodyMarkers(show_mesh_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::FAST);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Flexible Cable");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetCOMFrameScale(0.3);
        visVSG->AddCamera(ChVector3d(2.2, -1.6, 1.0), ChVector3d(0.1, 0.2, 0.2));
        ////visVSG->AddCamera(ChVector3d(-0.3, -1.5, 0.0), ChVector3d(-0.3, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        ////visVSG->AddGuiColorbar("Mz (Nm)", {-0.01, 0.01}, ChColormap::Type::JET);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

// Set MBS solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
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

    // Simulation loop
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    // Initial position of top most node
    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(0));
    std::cout << "Initial position of top node: " << node->GetPos().x() << " " << node->GetPos().y() << " "
              << node->GetPos().z() << std::endl;
    ChVector3d init_pos = node->GetPos();

    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");

            std::ostringstream filename;
            filename << out_dir << "/vtk/flex_body." << std::setw(5) << std::setfill('0') << out_frame + 1 << ".vtk";
            fea::ChMeshExporter::WriteFrame(mesh, out_dir + "/Flex_MESH.vtk", filename.str());

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

        ChVector3d pos = node->GetPos();
        double displacement = (pos - init_pos).Length();
        ofile << time << "\t" << pos.x() << "\t" << pos.y() << "\t" << pos.z() << "\t" << displacement << "\n";

        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string speed_title = "Displacement of top node in cable";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("displacement (m)");
    gplot.Plot(out_file, 1, 5, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<ChMesh> CreateFlexibleCable(ChSystem& sysMBS,
                                            double loc_x,
                                            double E,
                                            int num_elements,
                                            std::shared_ptr<ChBody> ground) {
    double length_cable = 0.8;

    // Material Properties
    double density = 8000;
    double rayleigh_damping = 0.02;

    auto section_cable = chrono_types::make_shared<ChBeamSectionCable>();
    section_cable->SetDiameter(0.02);
    section_cable->SetYoungModulus(E);
    section_cable->SetDensity(density);
    section_cable->SetRayleighDamping(rayleigh_damping);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    std::vector<std::vector<int>> node_indices;
    std::vector<std::vector<int>> node_nbrs;
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh,                                  // FEA mesh with nodes and elements
                      section_cable,                         // section material for cable elements
                      num_elements,                          // number of elements in the segment
                      ChVector3d(loc_x, 0.0, length_cable),  // beam start point
                      ChVector3d(loc_x, 0.0, 0.005),         // beam end point
                      node_indices,                          // node indices
                      node_nbrs                              // neighbor node indices
    );

    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(builder.GetLastBeamNodes().back());
    auto pos_const = chrono_types::make_shared<ChLinkNodeFrame>();
    pos_const->Initialize(node, ground);
    sysMBS.Add(pos_const);

    auto dir_const = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    dir_const->Initialize(node, ground);
    dir_const->SetDirectionInAbsoluteCoords(node->GetSlope1());
    sysMBS.Add(dir_const);

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh->SetColormapRange(-0.4, 0.4);
    vis_mesh->SetSmoothFaces(true);
    vis_mesh->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiProblemSPH& fsi) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Downstream locations
    double cable1_x = -0.3;
    double post_x = +0.6;
    double cable2_x = +0.8;
    double cyl_x = 1.0;

    // Contact material (default properties)
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    // Create ground body with collision boxes
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(cxDim, cyDim, 0.1), ChVector3d(0, 0, -0.05));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, cyDim, czDim + 0.2),
                          ChVector3d(+cxDim / 2 + 0.05, 0, czDim / 2));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, cyDim, czDim + 0.2),
                          ChVector3d(-cxDim / 2 - 0.05, 0, czDim / 2));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(cxDim + 0.2, 0.1, czDim + 0.2),
                          ChVector3d(0, +cyDim / 2 + 0.05, czDim / 2), QUNIT, false);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(cxDim + 0.2, 0.1, czDim + 0.2),
                          ChVector3d(0, -cyDim / 2 - 0.05, czDim / 2), QUNIT, false);
    sysMBS.AddBody(ground);

    // Create a fixed cylindrical post
    if (create_cylinder_post) {
        double length = 0.8;
        double radius = 0.02;

        auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        geometry->materials.push_back(contact_material_info);
        geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, QUNIT, radius, length));

        auto cylinder = chrono_types::make_shared<ChBody>();
        cylinder->SetName("CylinderPost");
        cylinder->SetPos(ChVector3d(post_x, 0, length / 2));
        cylinder->SetFixed(true);
        cylinder->EnableCollision(false);
        sysMBS.AddBody(cylinder);

        geometry->CreateVisualizationAssets(cylinder, VisualizationType::COLLISION);

        fsi.AddRigidBody(cylinder, geometry, false);
    }

    // Create a free cylindrical rigid body
    if (create_cylinder_free) {
        double length = 0.1;
        double radius = 0.05;
        double density = 200;
        double volume = ChCylinder::GetVolume(radius, length);
        double mass = density * volume;
        auto gyration = ChCylinder::GetGyration(radius, length).diagonal();

        auto cylinder = chrono_types::make_shared<ChBody>();
        cylinder->SetName("CylinderFree");
        cylinder->SetMass(mass);
        cylinder->SetInertiaXX(mass * gyration);
        cylinder->SetPos(ChVector3d(cyl_x, 0, 0.1 + radius));
        cylinder->SetFixed(false);
        cylinder->EnableCollision(true);
        sysMBS.AddBody(cylinder);

        auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        geometry->materials.push_back(contact_material_info);
        geometry->coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(VNULL, Q_ROTATE_Y_TO_Z, radius, length, 0));
        geometry->CreateVisualizationAssets(cylinder, VisualizationType::COLLISION);
        geometry->CreateCollisionShapes(cylinder, 1, sysMBS.GetContactMethod());

        fsi.AddRigidBody(cylinder, geometry, false);
    }

    fsi.SetBcePattern1D(BcePatternMesh1D::STAR, false);

    // Create the first flexible cable and add to FSI system
    auto mesh1 = CreateFlexibleCable(sysMBS, cable1_x, 6e8, 8, ground);
    mesh1->SetName("Cable1");
    fsi.AddFeaMesh(mesh1, false);

    // Create second flexible cable
    if (create_flex_cable2) {
        auto mesh2 = CreateFlexibleCable(sysMBS, cable2_x, 5e8, 15, ground);
        mesh2->SetName("Cable2");
        fsi.AddFeaMesh(mesh2, false);
    }

    return mesh1;
}

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
                     std::string& boundary_method,
                     std::string& viscosity_method) {
    ChCLI cli(argv[0], "Flexible cable FSI demo");

    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("Output", "output", "Enable collection of output files");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    cli.AddOption<bool>("Visualization", "snapshots", "Enable writing snapshot image files");

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    cli.AddOption<std::string>("Physics", "boundary_method", "Boundary condition type (holmes/adami)", boundary_method);
    cli.AddOption<std::string>("Physics", "viscosity_method",
                               "Viscosity type (laminar/artificial_unilateral/artificial_bilateral)", viscosity_method);

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

    boundary_method = cli.GetAsType<std::string>("boundary_method");
    viscosity_method = cli.GetAsType<std::string>("viscosity_method");

    return true;
}