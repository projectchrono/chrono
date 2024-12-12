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

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

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

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Dimensions of the boundary and fluid domains
double Lx_bndry = 3;
double Ly_bndry = 0.2;
double Lz_bndry = 3.0;

double Lx_fluid = 1.0;
double Ly_fluid = Ly_bndry;
double Lz_fluid = 0.75;

// Create additional solids
bool create_flex_plate2 = true;
bool create_cylinder_post = true;
bool create_cylinder_free = false;
// -----------------------------------------------------------------------------

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiProblemSPH& fsi, bool verbose);
bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& inputJSON,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& render,
                     double& render_fps,
                     bool& snapshots,
                     int& ps_freq,
                     std::string& boundary_type,
                     std::string& viscosity_type);

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Parse command line arguments
    PhysicsProblem problem_type = PhysicsProblem::CFD;
    std::string inputJSON = GetChronoDataFile("fsi/input_json/demo_FSI_Flexible_Flat_Plate_Explicit.json");
    double t_end = 4.0;
    bool verbose = true;
    bool output = false;
    double output_fps = 20;
    bool render = true;
    double render_fps = 400;
    bool snapshots = false;
    int ps_freq = 1;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_unilateral";
    if (!GetProblemSpecs(argc, argv, inputJSON, t_end, verbose, output, output_fps, render, render_fps, snapshots,
                         ps_freq, boundary_type, viscosity_type)) {
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    double initial_spacing = (problem_type == PhysicsProblem::CFD) ? 0.02 : 0.01;

    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();

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
            ChFluidSystemSPH::FluidProperties fluid_props;
            fluid_props.density = 1000;
            fluid_props.viscosity = 5.0;

            fsi.SetCfdSPH(fluid_props);

            break;
        }
        case PhysicsProblem::CRM: {
            ChFluidSystemSPH::ElasticMaterialProperties mat_props;
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
    ChFluidSystemSPH::SPHParameters sph_params;

    switch (problem_type) {
        case PhysicsProblem::CFD:
            sph_params.sph_method = SPHMethod::WCSPH;
            sph_params.initial_spacing = initial_spacing;
            sph_params.d0_multiplier = 1.2;
            sph_params.max_velocity = 10;
            sph_params.kernel_threshold = 0.8;
            sph_params.xsph_coefficient = 0.5;
            sph_params.shifting_coefficient = 0.0;
            sph_params.artificial_viscosity = 0.2;
            sph_params.use_delta_sph = true;
            sph_params.delta_sph_coefficient = 0.1;
            sph_params.num_proximity_search_steps = ps_freq;

            break;

        case PhysicsProblem::CRM:
            sph_params.sph_method = SPHMethod::WCSPH;
            sph_params.initial_spacing = initial_spacing;
            sph_params.d0_multiplier = 1.2;
            sph_params.xsph_coefficient = 0.5;
            sph_params.shifting_coefficient = 1.0;
            sph_params.kernel_threshold = 0.8;
            sph_params.artificial_viscosity = 0.5;
            sph_params.num_proximity_search_steps = ps_freq;

            break;
    }

    if (boundary_type == "holmes")
        sph_params.boundary_type = BoundaryType::HOLMES;
    else
        sph_params.boundary_type = BoundaryType::ADAMI;

    if (viscosity_type == "laminar")
        sph_params.viscosity_type = ViscosityType::LAMINAR;
    else if (viscosity_type == "artificial_bilateral")
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    else
        sph_params.viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;

    // Create solids
    auto mesh = CreateSolidPhase(fsi, verbose);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(Lz_fluid));
    // Create SPH material (do not create any boundary BCEs)
    fsi.Construct({Lx_fluid, Ly_fluid, Lz_fluid},        // box dimensions
                  {-Lx_bndry / 2 + Lx_fluid / 2, 0, 0},  // reference location
                  BoxSide::NONE                          // no boundary BCEs
    );

    // Create container (with bottom and left/right boundaries)
    fsi.AddBoxContainer({Lx_bndry, Ly_bndry, Lz_bndry},                   // length x width x height
                        ChVector3d(0, 0, 0),                              // reference location
                        BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS  // bottom and left/right walls
    );

    ChVector3d cMin = ChVector3d(-5 * Lx_bndry, -Ly_bndry / 2 - initial_spacing / 2, -5 * Lz_bndry);
    ChVector3d cMax = ChVector3d(+5 * Lx_bndry, +Ly_bndry / 2 + initial_spacing / 2, +5 * Lz_bndry);
    fsi.SetComputationalDomainSize(ChAABB(cMin, cMax));

    // Initialize FSI problem
    fsi.Initialize();

    // Create output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Plate/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + fsi.GetPhysicsProblemString() + "_" + fsi.GetSphMethodTypeString() + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + viscosity_type + "_" + boundary_type + "_ps" + std::to_string(ps_freq);
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
                visFSI->AddCamera(ChVector3d(0, -2, 0.75), ChVector3d(0, 0, 0.75));
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
                visFSI->AddCamera(ChVector3d(0.75, -3, 0.75), ChVector3d(0, 0, 0.75));
#endif
                break;
            }
        }

        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.5);

        visFSI->SetTitle("Chrono::FSI flexible plate");
        visFSI->SetSize(1280, 720);
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->SetLightIntensity(0.9);
        visFSI->SetLightDirection(-CH_PI_2, CH_PI / 6);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableFlexBodyMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetColorFlexBodyMarkers(ChColor(1, 1, 1));
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

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

    double timer_CFD = 0;
    double timer_MBD = 0;
    double timer_FSI = 0;
    double timer_step = 0;

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

        ChVector3d pos = node->GetPos();
        double displacement = (pos - init_pos).Length();
        ofile << time << "\t" << pos.x() << "\t" << pos.y() << "\t" << pos.z() << "\t" << displacement << "\n";

        fsi.DoStepDynamics(step_size);

        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBD += sysFSI.GetTimerMBD();
        timer_FSI += sysFSI.GetTimerFSI();
        timer_step += sysFSI.GetTimerStep();
        if (verbose && sim_frame == 2000) {
            cout << "Cummulative timers at time: " << time << endl;
            cout << "   timer CFD:  " << timer_CFD << endl;
            cout << "   timer MBD:  " << timer_MBD << endl;
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
    std::string speed_title = "Displacement of a node in the top row of the plate";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("displacement (m)");
    gplot.Plot(out_file, 1, 5, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}
std::shared_ptr<fea::ChMesh> CreateFlexiblePlate(ChSystem& sysMBS,
                                                 double plate_x,
                                                 double E,
                                                 double initSpace0,
                                                 std::shared_ptr<ChBody> ground,
                                                 bool verbose) {
    // Create an FEA mesh representing a cantilever plate modeled with ANCF shell elements
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    // Geometry of the plate
    double x_plate = 0.02;
    double y_plate = Ly_fluid;
    double z_plate = Lz_fluid;

    ChVector3d center_plate(plate_x, 0.0, z_plate / 2);

    // Specification of the mesh
    int numDiv_x = 1;
    int numDiv_y = 5;
    int numDiv_z = 20;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // For uniform mesh
    double dx = x_plate / numDiv_x;
    double dy = y_plate / numDiv_y;
    double dz = z_plate / numDiv_z;

    // Create and add the nodes
    ChVector3d loc;
    ChVector3d dir(1, 0, 0);
    for (int k = 0; k < N_z; k++) {
        for (int j = 0; j < N_y; j++) {
            loc.x() = center_plate.x();
            loc.y() = j * dy - y_plate / 2 + center_plate.y();
            loc.z() = k * dz - z_plate / 2 + center_plate.z();

            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
            node->SetMass(0);

            // Fix nodes connected to the ground
            if (k == 0)
                node->SetFixed(true);

            mesh->AddNode(node);
        }
    }

    // Create an isotropic material; all layers for all elements share the same material
    double rho = 8000;
    double nu = 0.3;
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
            element->AddLayer(dx, 0 * CH_DEG_TO_RAD, mat);

            // Set structural damping for this element
            element->SetAlphaDamp(0.05);

            // Add element to mesh
            mesh->AddElement(element);
            ChVector3d center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                        element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
            if (verbose)
                cout << "Adding element " << num_elem << " with center:  " << center.x() << " " << center.y() << " "
                     << center.z() << endl;
            num_elem++;
        }
    }

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColorscaleMinMax(0.0, 5.50);
    vis_mesh->SetShrinkElements(true, 0.85);
    vis_mesh->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiProblemSPH& fsi, bool verbose) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Downstream locations
    double plate1_x = 0.0;
    double post_x = +0.6;
    double plate2_x = +0.8;
    double cyl_x = 1.2;

    double initSpace0 = fsi.GetSystemFSI().GetFluidSystemSPH().GetInitialSpacing();
    // Contact material (default properties)
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    // Create ground body with a collision box
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(Lx_bndry, Ly_bndry, 0.1), ChVector3d(0, 0, -0.05));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, Ly_bndry, Lz_bndry + 0.2),
                          ChVector3d(+Lx_bndry / 2 + 0.05, 0, Lz_bndry / 2));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, Ly_bndry, Lz_bndry + 0.2),
                          ChVector3d(-Lx_bndry / 2 - 0.05, 0, Lz_bndry / 2));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(Lx_bndry + 0.2, 0.1, Lz_bndry + 0.2),
                          ChVector3d(0, +Ly_bndry / 2 + 0.05, Lz_bndry / 2), QUNIT, false);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(Lx_bndry + 0.2, 0.1, Lz_bndry + 0.2),
                          ChVector3d(0, -Ly_bndry / 2 - 0.05, Lz_bndry / 2), QUNIT, false);
    sysMBS.AddBody(ground);

    // Create a fixed cylindrical post
    if (create_cylinder_post) {
        double length = 0.8;
        double radius = 0.02;

        utils::ChBodyGeometry geometry;
        geometry.materials.push_back(contact_material_info);
        geometry.coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, QUNIT, radius, length));

        auto cylinder = chrono_types::make_shared<ChBody>();
        cylinder->SetPos(ChVector3d(post_x, 0, length / 2));
        cylinder->SetFixed(true);
        cylinder->EnableCollision(false);
        sysMBS.AddBody(cylinder);

        geometry.CreateVisualizationAssets(cylinder, VisualizationType::COLLISION);

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

        utils::ChBodyGeometry geometry;
        geometry.materials.push_back(contact_material_info);
        geometry.coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, Q_ROTATE_Y_TO_Z, radius, length));

        auto cylinder = chrono_types::make_shared<ChBody>();
        cylinder->SetMass(mass);
        cylinder->SetInertiaXX(mass * gyration);
        cylinder->SetPos(ChVector3d(cyl_x, 0, 0.1 + radius));
        cylinder->SetFixed(false);
        cylinder->EnableCollision(true);
        sysMBS.AddBody(cylinder);

        geometry.CreateVisualizationAssets(cylinder, VisualizationType::COLLISION);

        fsi.AddRigidBody(cylinder, geometry, false);
    }

    // Create the first flexible cable and add to FSI system
    auto mesh1 = CreateFlexiblePlate(sysMBS, plate1_x, 2e7, initSpace0, ground, verbose);
    fsi.AddFeaMesh(mesh1, false);

    // Create second flexible cable
    if (create_flex_plate2) {
        auto mesh2 = CreateFlexiblePlate(sysMBS, plate2_x, 5e8, initSpace0, ground, verbose);
        fsi.AddFeaMesh(mesh2, false);
    }

    return mesh1;
}

// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& inputJSON,
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
    ChCLI cli(argv[0], "Flexible flat plate FSI demo");

    cli.AddOption<std::string>("Input", "inputJSON", "Problem specification file [JSON format]", inputJSON);
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

    inputJSON = cli.Get("inputJSON").as<std::string>();
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