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

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

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

// -----------------------------------------------------------------------------

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiSystemSPH& sysFSI, bool verbose);
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
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysFSI.SetVerbose(verbose);

    // Use the specified input JSON file
    sysSPH.ReadParametersFromFile(inputJSON);

    // Set boundary type
    if (boundary_type == "holmes") {
        sysSPH.SetBoundaryType(BoundaryType::HOLMES);
    } else {
        sysSPH.SetBoundaryType(BoundaryType::ADAMI);
    }

    // Set viscosity type
    if (viscosity_type == "laminar") {
        sysSPH.SetViscosityType(ViscosityType::LAMINAR);
    } else if (viscosity_type == "artificial_bilateral") {
        sysSPH.SetViscosityType(ViscosityType::ARTIFICIAL_BILATERAL);
        sysSPH.SetArtificialViscosityCoefficient(0.2);
    } else {
        sysSPH.SetViscosityType(ViscosityType::ARTIFICIAL_UNILATERAL);
        sysSPH.SetArtificialViscosityCoefficient(0.2);
    }

    // Set frequency of proximity search
    sysSPH.SetNumProximitySearchSteps(ps_freq);

    // Set simulation domain
    sysSPH.SetContainerDim(ChVector3d(Lx_bndry, Ly_bndry, Lz_bndry));

    auto initSpace0 = sysSPH.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-5 * Lx_bndry, -Ly_bndry / 2 - initSpace0 / 2, -5 * Lz_bndry);
    ChVector3d cMax = ChVector3d(+5 * Lx_bndry, +Ly_bndry / 2 + initSpace0 / 2, +5 * Lz_bndry);
    sysSPH.SetBoundaries(cMin, cMax);

    // Set SPH discretization type, consistent or inconsistent
    sysSPH.SetConsistentDerivativeDiscretization(false, false);

    // Create SPH particles of fluid region
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(-Lx_bndry / 2 + Lx_fluid / 2, 0, Lz_fluid / 2);
    ChVector3d boxHalfDim(Lx_fluid / 2 - initSpace0, Ly_fluid / 2, Lz_fluid / 2 - initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysSPH.AddSPHParticle(points[i]);
    }

    // Create solids
    auto mesh = CreateSolidPhase(sysFSI, verbose);

    // Initialize FSI system
    sysFSI.Initialize();

    // Create output directories
    std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Plate/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir = out_dir + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphMethodTypeString() + "/";
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
    double dT = sysFSI.GetStepSizeCFD();
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    double timer_CFD = 0;
    double timer_MBS = 0;
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

            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);

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

        sysFSI.DoStepDynamics(dT);

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

        time += dT;
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

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<fea::ChMesh> CreateSolidPhase(ChFsiSystemSPH& sysFSI, bool verbose) {
    ChFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();
    ChSystem& sysMBS = sysFSI.GetMultibodySystem();

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto initSpace0 = sysSPH.GetInitialSpacing();

    // Contact material (default properties)
    auto contact_material_info = ChContactMaterialData();
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    // Create ground body with a collision box
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(Lx_bndry, Ly_bndry + 2 * initSpace0, 0.1),
                          ChVector3d(0, 0, -0.05));
    sysMBS.AddBody(ground);

    // FSI representation of walls
    sysSPH.AddBoxContainerBCE(ground,                                            //
                              ChFrame<>(ChVector3d(0, 0, Lz_bndry / 2), QUNIT),  //
                              ChVector3d(Lx_bndry, Ly_bndry, Lz_bndry),          //
                              ChVector3i(2, 0, -1));


    // Create an FEA mesh representing a cantilever plate modeled with ANCF shell elements
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    // Geometry of the plate
    double x_plate = 0.02;
    double y_plate = Ly_fluid;
    double z_plate = Lz_fluid;

    ChVector3d center_plate(0.0, 0.0, z_plate / 2 + initSpace0);
    ////ChVector3d center_plate(-0.25, 0.0, z_plate / 2 + initSpace0);

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
    double E = 2e7;
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

    // Create the FEA contact surface
    auto contact_surface = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_surface->AddAllNodes(*mesh, sysSPH.GetInitialSpacing());
    mesh->AddContactSurface(contact_surface);

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColorscaleMinMax(0.0, 5.50);
    vis_mesh->SetShrinkElements(true, 0.85);
    vis_mesh->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh);

    // Add the mesh to the FSI system (only these meshes interact with the fluid)
    sysFSI.AddFsiMesh(mesh);

    return mesh;
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