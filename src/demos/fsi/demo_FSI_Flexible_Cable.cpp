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

#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_fsi/ChSystemFsi.h"

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

// Set the output directory
std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Cable";

// Dimension of the domain
double smalldis = 1.0e-9;
double bxDim = 3.0 + smalldis;
double byDim = 0.2 + smalldis;
double bzDim = 2.0 + smalldis;

// Dimension of the fluid domain
double fxDim = 1.0 + smalldis;
double fyDim = 0.2 + smalldis;
double fzDim = 1.0 + smalldis;

// Dimension of the cable
double length_cable = 0.8 + smalldis;
double loc_x = -0.3;
int num_cable_element = 15;

// Material Properties
double E = 8e9;
double density = 8000;
double BeamRayleighDamping = 0.02;

// -----------------------------------------------------------------------------

std::shared_ptr<fea::ChMesh> Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI);
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
                     int& ps_freq);

// -----------------------------------------------------------------------------

class PositionVisibilityCallback : public ChParticleCloud::VisibilityCallback {
  public:
    PositionVisibilityCallback() {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        auto p = cloud.GetParticlePos(n);
        return p.y() > 0;
    };
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string inputJSON = GetChronoDataFile("fsi/input_json/demo_FSI_Flexible_Cable_Explicit.json");
    double t_end = 10.0;
    bool verbose = true;
    bool output = false;
    double output_fps = 20;
    bool render = true;
    double render_fps = 400;
    bool snapshots = false;
    int ps_freq = 1;
    if (!GetProblemSpecs(argc, argv, inputJSON, t_end, verbose, output, output_fps, render, render_fps, snapshots,
                         ps_freq)) {
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysFSI.SetVerbose(verbose);

    // Use the specified input JSON file
    sysFSI.ReadParametersFromFile(inputJSON);

    // Set simulation domain
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-5 * bxDim, -byDim / 2.0 - initSpace0 / 2.0, -5 * bzDim);
    ChVector3d cMax = ChVector3d(5 * bxDim, byDim / 2.0 + initSpace0 / 2.0, 5 * bzDim);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Create SPH particles of fluid region
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - initSpace0, fyDim / 2, fzDim / 2 - initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysFSI.AddSPHParticle(points[i]);
    }

    // Create solids
    auto mesh = Create_MB_FE(sysMBS, sysFSI);

    sysFSI.SetNumProximitySearchSteps(ps_freq);

    // Initialize FSI system
    sysFSI.Initialize();

    out_dir = out_dir + std::to_string(ps_freq);

    // Create oputput directories
    if (output || snapshots) {
        if (output) {
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
                visFSI->AddCamera(ChVector3d(0, -2, 0.25), ChVector3d(0, 0, 0.25));
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
                visFSI->AddCamera(ChVector3d(0, -4, 0.25), ChVector3d(0, 0, 0.25));
#endif
                break;
            }
        }

        visFSI->SetTitle("Chrono::FSI flexible cable");
        visFSI->SetVerbose(verbose);
        visFSI->SetSize(1920, 1200);
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableFlexBodyMarkers(true);
        visFSI->SetColorFlexBodyMarkers(ChColor(1, 1, 1));
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, 2.5));
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<PositionVisibilityCallback>());
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
    double dT = sysFSI.GetStepSize();
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    // Initial position of top most node
    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(0));
    std::cout << "Initial position of top node: " << node->GetPos().x() << " " << node->GetPos().y() << " "
              << node->GetPos().z() << std::endl;
    ChVector3d init_pos = node->GetPos();

    // Record Top node displacement
    ChFunctionInterp displacement_recorder;
    double displacement = 0;
    ChTimer timer;
    timer.start();

    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile;
    if (output) {
        ofile.open(out_file, std::ios::trunc);
    }

    while (time < t_end) {
        if (verbose)
            cout << sim_frame << " time: " << time << endl;

        if (output && time >= out_frame / output_fps) {
            if (verbose)
                cout << " -- Output frame " << out_frame << " at t = " << time << endl;

            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);

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
        displacement =
            sqrt(pow(pos.x() - init_pos.x(), 2) + pow(pos.y() - init_pos.y(), 2) + pow(pos.z() - init_pos.z(), 2));

        displacement_recorder.AddPoint(time, displacement);

        if (output) {
            ofile << time << "\t" << pos.x() << "\t" << pos.y() << "\t" << pos.z() << "\n";
        }

        sysFSI.DoStepDynamics_FSI();

        time += dT;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    return 0;

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string speed_title = "Displacement of top node in cable";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("displacement (m)");
    gplot.Plot(displacement_recorder, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<fea::ChMesh> Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sysMBS.AddBody(ground);

    // FSI representation of walls
    sysFSI.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 0, -1));

    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Create an FEA mesh representing a cantilever beam modeled with ANCF cable elements
    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    std::vector<std::vector<int>> _1D_elementsNodes_mesh;

    auto msection_cable = chrono_types::make_shared<ChBeamSectionCable>();
    msection_cable->SetDiameter(initSpace0);
    msection_cable->SetYoungModulus(E);
    msection_cable->SetDensity(density);
    msection_cable->SetRayleighDamping(BeamRayleighDamping);

    ChBuilderCableANCF builder;
    std::vector<std::vector<int>> node_nbrs;
    builder.BuildBeam(mesh,                                  // FEA mesh with nodes and elements
                      msection_cable,                        // section material for cable elements
                      num_cable_element,                     // number of elements in the segment
                      ChVector3d(loc_x, 0.0, length_cable),  // beam start point
                      ChVector3d(loc_x, 0.0, initSpace0),    // beam end point
                      _1D_elementsNodes_mesh,                // node indices
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

    // Add the mesh to the FSI system (only these meshes interact with the fluid)
    sysFSI.SetBcePattern1D(BcePatternMesh1D::STAR, false);
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
                     int& ps_freq) {
    ChCLI cli(argv[0], "Flexible cable FSI demo");

    cli.AddOption<std::string>("Input", "inputJSON", "Problem specification file [JSON format]", inputJSON);
    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

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

    inputJSON = cli.Get("inputJSON").as<std::string>();
    t_end = cli.GetAsType<double>("t_end");

    verbose = !cli.GetAsType<bool>("quiet");
    output = cli.GetAsType<bool>("output");
    render = !cli.GetAsType<bool>("no_vis");
    snapshots = cli.GetAsType<bool>("snapshots");

    output_fps = cli.GetAsType<double>("output_fps");
    render_fps = cli.GetAsType<double>("render_fps");
    ps_freq = cli.GetAsType<int>("ps_freq");

    return true;
}
