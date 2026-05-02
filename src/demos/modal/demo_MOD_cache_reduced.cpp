// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban, Dario Mangoni
// =============================================================================
//
// Show how to save and restore the reduced model of a ChModalAssembly
// This is not a full Archiving, so the source file that builds the model is
// required to restore the simulation.
// The model is first built and stored on file; then is simulated.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"

#include "chrono/serialization/ChArchiveJSON.h"

#include "chrono_modal/ChModalAssembly.h"
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include <filesystem>

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "MODAL_REDUCTION";

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;
int n_elements = 8;

// damping coefficients
double damping_alpha = 0.0001;
double damping_beta = 0.01;

double step_size = 0.005;

unsigned int num_modes = 12;

std::string reduced_model_file = "reduced_model_file.json";

void RunModel(bool load_from_file) {
    // CREATE THE MODEL

    ChSystemNSC sys;

    // no gravity used here
    sys.SetGravitationalAcceleration(VNULL);  // enable it later

    // CREATE THE ASSEMBLY.
    //
    // The ChModalAssembly is the most important item when doing modal analysis.
    // You must add finite elements, bodies and constraints into this assembly in order
    // to compute the modal frequencies etc.; objects not added into this won't be counted.
    auto modal_assembly = chrono_types::make_shared<ChModalAssembly>();
    sys.Add(modal_assembly);

    modal_assembly->SetInternalNodesUpdate(true);
    modal_assembly->SetUseLinearInertialTerm(true);
    modal_assembly->SetUseStaticCorrection(true);
    modal_assembly->SetModalAutomaticGravity(true);

    // Specify the modal reduction method used:
    // 1. HERTING: free-free modes are used as the modal basis;
    // 2. CRAIG_BAMPTON: clamped-clamped modes are used as the modal basis.
    modal_assembly->SetReductionType(ChModalAssembly::ReductionType::HERTING);

    // Now populate the assembly to analyze.
    // In this demo, make a cantilever with fixed end

    // Create two FEM meshes: one for nodes that will be removed in modal reduction,
    // the other for the nodes that will remain after modal reduction.

    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddInternal(mesh_internal);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()

    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    modal_assembly->Add(mesh_boundary);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()

    mesh_internal->SetAutomaticGravity(false);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    section->SetDensity(beam_density);
    section->SetYoungModulus(beam_Young);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(damping_beta);
    section->SetRayleighDampingAlpha(damping_alpha);
    section->SetAsRectangularSection(beam_wy, beam_wz);

    ChBuilderBeamEuler builder;

    // The first node is a boundary node: add it to mesh_boundary
    auto my_node_A_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>();
    my_node_A_boundary->SetMass(0);
    my_node_A_boundary->GetInertia().setZero();
    mesh_boundary->AddNode(my_node_A_boundary);

    // The last node is a boundary node: add it to mesh_boundary
    auto my_node_B_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(beam_L, 0, 0)));
    my_node_B_boundary->SetMass(0);
    my_node_B_boundary->GetInertia().setZero();
    mesh_boundary->AddNode(my_node_B_boundary);

    my_node_B_boundary->SetForce(ChVector3d(0, -3, 0));  // to trigger some vibration at the free end

    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                      section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      n_elements,          // the number of ChElementBeamEuler to create
                      my_node_A_boundary,  // the 'A' point in space (beginning of beam)
                      my_node_B_boundary,  // ChVector3d(beam_L, 0, 0), // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    // set gravity
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -0.5));  // -Z axis

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    my_body_A->SetFixed(true);
    my_body_A->SetPos(ChVector3d(-0.5, 0, 0));
    modal_assembly->Add(my_body_A);

    auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root->Initialize(my_node_A_boundary, my_body_A, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));
    modal_assembly->Add(my_root);

    if (!load_from_file) {
        auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
        ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_solver(num_modes, 1e-5, true, false,
                                                                                  eigen_solver);
        ChModalDampingRayleigh modal_damping(damping_alpha, damping_beta);
        modal_assembly->DoModalReduction(modal_solver, modal_damping);
        sys.Setup();
        sys.Update(UpdateFlags::UPDATE_ALL);

        std::ofstream file_out(reduced_model_file);

        file_out << std::setprecision(12);

        if (!file_out.is_open()) {
            std::cerr << "Cannot save reduced model file: " << reduced_model_file << std::endl;
            return;
        }
        {
            ChArchiveOutJSON archive_out(file_out);

            modal_assembly->SaveReducedModel(archive_out);
        }
        return;
    } else {
        std::ifstream file_in(reduced_model_file);

        if (!file_in.is_open()) {
            std::cerr << "Cannot load reduced model file: " << reduced_model_file << std::endl;
            return;
        }

        ChArchiveInJSON archive_in(file_in);
        modal_assembly->LoadReducedModel(archive_in);
        sys.Setup();
        sys.Update(UpdateFlags::UPDATE_ALL);
    }

    // VISUALIZATION ASSETS:

    auto visualizeInternalA = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizeInternalA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MY);
    visualizeInternalA->SetColormapRange(-600, 600);
    visualizeInternalA->SetSmoothFaces(true);
    visualizeInternalA->SetWireframe(false);
    mesh_internal->AddVisualShapeFEA(visualizeInternalA);

    // VISUALIZATION

    // Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&sys);
    vis.SetWindowSize(1024, 768);
    vis.SetWindowTitle("Archive Reduced Model");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera(ChVector3d(1, 1.3, 6), ChVector3d(3, 0, 0));
    vis.AddTypicalLights();

    // Set linear solver
// #undef CHRONO_PARDISO_MKL
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);
#endif

    vis.BindAll();

    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    if (hht_stepper) {
        hht_stepper->SetVerbose(false);
        hht_stepper->SetStepControl(false);
        hht_stepper->SetAlpha(-0.2);
        hht_stepper->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_STEP);
    }

    sys.Setup();
    sys.Update(UpdateFlags::UPDATE_ALL);

    while (vis.Run()) {
        vis.BeginScene();
        vis.Render();

        sys.DoStepDynamics(step_size);

        vis.EndScene();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::filesystem::create_directory(std::filesystem::path(out_dir));
    // Directory for output data
    if (!std::filesystem::exists(std::filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    RunModel(false);
    RunModel(true);

    return 0;
}
