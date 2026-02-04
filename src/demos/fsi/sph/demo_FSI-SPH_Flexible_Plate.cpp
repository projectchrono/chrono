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

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3443.h"
#include "chrono/fea/ChElementShellANCF_3833.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

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

// Dimensions of the boundary and fluid domains
double cxDim = 3;
double cyDim = 0.2;
double czDim = 1.0;

// Create additional solids
bool create_flex_plate2 = false;
bool create_cylinder_post = false;

// FEA shell element type
enum class ANCFShellElementType { ANCF_3423, ANCF_3443, ANCF_3833 };
ANCFShellElementType shell_type = ANCFShellElementType::ANCF_3833;

// Use nodal directions
NodeDirectionsMode FEA_node_directions_mode = NodeDirectionsMode::NONE;

// Visibility flags
bool show_rigid_bce = false;
bool show_mesh_bce = false;
bool show_boundary_bce = false;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiProblemSPH& fsi, bool verbose);
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

int main(int argc, char* argv[]) {
    // Parse command line arguments
    double t_end = 4.0;
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

    // Create a physics system and an FSI system
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
            sph_params.d0_multiplier = 1.2;
            sph_params.max_velocity = 10;
            sph_params.free_surface_threshold = 2.0;
            sph_params.shifting_method = ShiftingMethod::XSPH;
            sph_params.shifting_xsph_eps = 0.5;
            sph_params.artificial_viscosity = 0.2;
            sph_params.use_delta_sph = true;
            sph_params.delta_sph_coefficient = 0.1;
            sph_params.num_proximity_search_steps = ps_freq;

            break;

        case PhysicsProblem::CRM:
            sph_params.integration_scheme = IntegrationScheme::RK2;
            sph_params.initial_spacing = initial_spacing;
            sph_params.d0_multiplier = 1.2;
            sph_params.shifting_xsph_eps = 0.5;
            sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
            sph_params.shifting_ppst_pull = 1.0;
            sph_params.shifting_ppst_push = 3.0;
            sph_params.free_surface_threshold = 2.0;
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

    // Create solids
    auto mesh = CreateSolidPhase(fsi, verbose);

    // Dimension of the fluid domain
    double fxDim = 1.0;
    double fyDim = 0.2;
    double fzDim = (problem_type == PhysicsProblem::CFD) ? 0.75 : 1.0;

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

    ChVector3d cMin =
        ChVector3d(-cxDim - 3 * initial_spacing, -cyDim / 2 - initial_spacing / 2, -czDim - 3 * initial_spacing);
    ChVector3d cMax = ChVector3d(cxDim + 3 * initial_spacing, +cyDim / 2 + initial_spacing / 2, czDim);
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    // Initialize FSI problem
    fsi.Initialize();

    // Create output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Plate/";
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
        std::string title = "Flexible Plate ";
        switch (shell_type) {
            case ANCFShellElementType::ANCF_3423:
                title += "ANCF-3423";
                break;
            case ANCFShellElementType::ANCF_3443:
                title += "ANCF-3443";
                break;
            case ANCFShellElementType::ANCF_3833:
                title += "ANCF-3833";
                break;
        }

        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.5);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->EnableFlexBodyMarkers(show_mesh_bce);
        visFSI->SetSPHColorCallback(col_callback);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle(title);
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(1.5, -1.5, 0.5), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

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

    // Initial position of top node at the (approx) middle of the plate in x and y
    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(15));
    std::cout << "Initial position of node: " << node->GetPos().x() << " " << node->GetPos().y() << " "
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
    std::string speed_title = "Displacement of a node in the top row of the plate";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("displacement (m)");
    gplot.Plot(out_file, 1, 5, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}

std::shared_ptr<fea::ChMesh> CreatePlate_3423(ChSystem& sysMBS, double loc, double E) {
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    // Geometry of the plate
    double thickness = 0.02;
    double width = 0.2;
    double height = 0.75;

    ChVector3d center(loc, 0.0, height / 2);

    // Specification of the mesh
    int numDiv_y = 4;
    int numDiv_z = 5;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // For uniform mesh
    double dy = width / numDiv_y;
    double dz = height / numDiv_z;

    // Create an isotropic material
    double rho = 8000;
    double nu = 0.3;
    auto shell_material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    std::vector<std::shared_ptr<ChNodeFEAbase>> collision_nodes;

    // Create and add the nodes
    ChVector3d n_dirZ(1, 0, 0);
    for (int k = 0; k < N_z; k++) {
        for (int j = 0; j < N_y; j++) {
            auto n_loc = center + ChVector3d(0, j * dy - width / 2, k * dz - height / 2);
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(n_loc, n_dirZ);
            node->SetMass(0);
            // Set fixed nodes (no collision) and collect nodes with collision
            if (k == 0)                           //
                node->SetFixed(true);             // set fixed nodes at bottom
            else                                  //
                collision_nodes.push_back(node);  // collect nodes with collision

            mesh->AddNode(node);
        }
    }

    // Create and add the elements
    for (int k = 0; k < numDiv_z; k++) {
        for (int j = 0; j < numDiv_y; j++) {
            int node0 = (j + 0) + N_y * (k + 0);
            int node1 = (j + 1) + N_y * (k + 0);
            int node2 = (j + 1) + N_y * (k + 1);
            int node3 = (j + 0) + N_y * (k + 1);

            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));
            element->SetDimensions(dy, dz);
            element->AddLayer(thickness, 0, shell_material);
            element->SetAlphaDamp(0.0);

            mesh->AddElement(element);
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

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColormapRange(0.0, 3.0);
    vis_mesh->SetShrinkElements(true, 0.85);
    ////vis_mesh->SetShellResolution(4);
    vis_mesh->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

std::shared_ptr<fea::ChMesh> CreatePlate_3443(ChSystem& sysMBS, double loc, double E) {
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    // Geometry of the plate
    double thickness = 0.02;
    double width = 0.2;
    double height = 0.75;

    ChVector3d center(loc, 0.0, height / 2);

    // Specification of the mesh
    int numDiv_y = 4;
    int numDiv_z = 5;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // For uniform mesh
    double dy = width / numDiv_y;
    double dz = height / numDiv_z;

    // Create an isotropic material
    double rho = 8000;
    double nu = 0.3;
    auto shell_material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    std::vector<std::shared_ptr<ChNodeFEAbase>> collision_nodes;

    // Create and add the nodes
    ChVector3d n_dirX(0, 0, -1);
    ChVector3d n_dirY(0, 1, 0);
    ChVector3d n_dirZ(1, 0, 0);
    for (int k = 0; k < N_z; k++) {
        for (int j = 0; j < N_y; j++) {
            auto n_loc = center + ChVector3d(0, j * dy - width / 2, k * dz - height / 2);
            auto node = chrono_types::make_shared<ChNodeFEAxyzDDD>(n_loc, n_dirX, n_dirY, n_dirZ);
            node->SetMass(0);
            if (k == 0)                           //
                node->SetFixed(true);             // set fixed nodes at bottom
            else                                  //
                collision_nodes.push_back(node);  // collect nodes with collision

            mesh->AddNode(node);
        }
    }

    // Create and add the elements
    for (int k = 0; k < numDiv_z; k++) {
        for (int j = 0; j < numDiv_y; j++) {
            int node0 = (j + 0) + N_y * (k + 0);
            int node1 = (j + 1) + N_y * (k + 0);
            int node2 = (j + 1) + N_y * (k + 1);
            int node3 = (j + 0) + N_y * (k + 1);

            auto element = chrono_types::make_shared<ChElementShellANCF_3443>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node3)));
            element->SetDimensions(dy, dz);
            element->AddLayer(thickness, 0, shell_material);
            element->SetAlphaDamp(0.0);

            mesh->AddElement(element);
        }
    }

    // Create the FEA contact surface
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    auto contact_surface = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_surface->AddNodesFromNodeSet(collision_nodes, 0.01);
    mesh->AddContactSurface(contact_surface);

    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColormapRange(0.0, 3.0);
    vis_mesh->SetShrinkElements(true, 0.85);
    ////vis_mesh->SetShellResolution(4);
    vis_mesh->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

std::shared_ptr<fea::ChMesh> CreatePlate_3833(ChSystem& sysMBS, double loc, double E) {
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    // Geometry of the plate
    double Lx = 0.02;  // thickness
    double Ly = 0.2;   // width
    double Lz = 0.75;  // height

    double hLy = Ly / 2;

    // Specification of the mesh
    int num_elements = 5;
    double dz = Lz / num_elements;

    // Create an isotropic material
    double rho = 8000;
    double nu = 0.3;
    auto shell_material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    std::vector<std::shared_ptr<ChNodeFEAbase>> collision_nodes;

    //       z
    //       ^
    //       |
    //   B---F---C
    //   |   |   |
    //   E---+---G
    //   |   |   |
    //   A---H---D----> y

    // Setup shell normals to initially align with the global x direction with no curvature
    // Attention to the sign of the normal (A, B, C, D nodes muct be in counter-clockwise order)
    ChVector3d n_normal(-1, 0, 0);    // node normal direction
    ChVector3d n_curvature(0, 0, 0);  // no curvature

    std::shared_ptr<ChNodeFEAxyzDD> nodeA;
    std::shared_ptr<ChNodeFEAxyzDD> nodeB;
    std::shared_ptr<ChNodeFEAxyzDD> nodeC;
    std::shared_ptr<ChNodeFEAxyzDD> nodeD;
    std::shared_ptr<ChNodeFEAxyzDD> nodeE;
    std::shared_ptr<ChNodeFEAxyzDD> nodeF;
    std::shared_ptr<ChNodeFEAxyzDD> nodeG;
    std::shared_ptr<ChNodeFEAxyzDD> nodeH;

    // Create the first nodes and fix them completely to ground (Cantilever constraint)
    nodeA = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, -hLy, 0.0), n_normal, n_curvature);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    nodeH = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, 0, 0), n_normal, n_curvature);
    nodeH->SetFixed(true);
    mesh->AddNode(nodeH);

    nodeD = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, hLy, 0), n_normal, n_curvature);
    nodeD->SetFixed(true);
    mesh->AddNode(nodeD);

    // Generate the rest of the nodes as well as all of the elements
    for (int i = 1; i <= num_elements; i++) {
        nodeE = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, -hLy, (i - 0.5) * dz), n_normal, n_curvature);
        mesh->AddNode(nodeE);
        nodeG = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, +hLy, (i - 0.5) * dz), n_normal, n_curvature);
        mesh->AddNode(nodeG);
        nodeB = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, -hLy, i * dz), n_normal, n_curvature);
        mesh->AddNode(nodeB);
        nodeF = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, 0, i * dz), n_normal, n_curvature);
        mesh->AddNode(nodeF);
        nodeC = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(loc, +hLy, i * dz), n_normal, n_curvature);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementShellANCF_3833>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH);
        element->SetDimensions(Ly, dz);
        element->SetAlphaDamp(0.001);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(Lx, 0, shell_material);

        // Collect collision nodes
        collision_nodes.push_back(nodeB);
        collision_nodes.push_back(nodeC);
        collision_nodes.push_back(nodeE);
        collision_nodes.push_back(nodeF);
        collision_nodes.push_back(nodeG);

        mesh->AddElement(element);

        nodeA = nodeB;
        nodeH = nodeF;
        nodeD = nodeC;
    }

    // Create the FEA contact surface
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    auto contact_surface = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_surface->AddNodesFromNodeSet(collision_nodes, 0.01);
    mesh->AddContactSurface(contact_surface);

    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColormapRange(0.0, 3.0);
    vis_mesh->SetShrinkElements(true, 0.85);
    vis_mesh->SetShellResolution(4);
    vis_mesh->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiProblemSPH& fsi, bool verbose) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Downstream locations
    double plate1_x = -0.1;
    double post_x = +0.7;
    double plate2_x = +1.2;

    // Contact material (default properties)
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    // Create ground body with a collision box
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(cxDim, cyDim + 0.02, 0.1),
                          ChVector3d(0, 0, -0.05));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, cyDim + 0.02, czDim),
                          ChVector3d(cxDim / 2 + 0.05, 0, czDim / 2));
    sysMBS.AddBody(ground);

    // Create a fixed cylindrical post
    if (create_cylinder_post) {
        double height = 0.75;
        double radius = 0.02;

        auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        geometry->materials.push_back(contact_material_info);
        geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, QUNIT, radius, height));

        auto cylinder = chrono_types::make_shared<ChBody>();
        cylinder->SetPos(ChVector3d(post_x, 0, height / 2));
        cylinder->SetFixed(true);
        cylinder->EnableCollision(false);
        sysMBS.AddBody(cylinder);

        geometry->CreateVisualizationAssets(cylinder, VisualizationType::COLLISION);

        fsi.AddRigidBody(cylinder, geometry, false);
    }

    fsi.SetBcePattern2D(BcePatternMesh2D::INWARD);

    // Create the first flexible plate
    std::shared_ptr<ChMesh> mesh1;
    switch (shell_type) {
        case ANCFShellElementType::ANCF_3423:
            mesh1 = CreatePlate_3423(sysMBS, plate1_x, 2e7);
            break;
        case ANCFShellElementType::ANCF_3443:
            mesh1 = CreatePlate_3443(sysMBS, plate1_x, 2e7);
            break;
        case ANCFShellElementType::ANCF_3833:
            mesh1 = CreatePlate_3833(sysMBS, plate1_x, 2e7);
            break;
    }
    fsi.AddFeaMesh(mesh1, false);

    // Create second flexible plate
    if (create_flex_plate2) {
        std::shared_ptr<ChMesh> mesh2;
        switch (shell_type) {
            case ANCFShellElementType::ANCF_3423:
                mesh2 = CreatePlate_3423(sysMBS, plate2_x, 5e7);
                break;
            case ANCFShellElementType::ANCF_3443:
                mesh2 = CreatePlate_3443(sysMBS, plate2_x, 5e7);
                break;
            case ANCFShellElementType::ANCF_3833:
                mesh2 = CreatePlate_3833(sysMBS, plate2_x, 5e7);
                break;
        }
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
    ChCLI cli(argv[0], "Flexible flat plate FSI demo");

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