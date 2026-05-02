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
// Test load and save of reduced model (no archive)
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

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include <filesystem>

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

const double error_threshold = 1e-5;

// Output directory
static const std::string out_dir = GetChronoTestOutputPath() + "/archive_reduced_model/";

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;
int n_elements = 8;

// damping coefficients
double damping_alpha = 0.001;
double damping_beta = 0.1;

double step_size = 0.005;

unsigned int num_modes = 12;
bool USE_STATIC_CORRECTION = true;
bool UPDATE_INTERNAL_NODES = true;
bool USE_LINEAR_INERTIAL_TERM = true;
bool USE_GRAVITY = true;

// static stuff for GUI:
bool FIX_SUBASSEMBLY = true;
bool ADD_INTERNAL_BODY = true;
bool ADD_BOUNDARY_BODY = true;
bool ADD_FORCE = true;
bool ADD_OTHER_ASSEMBLY = true;

std::string reduced_model_filename = "reduced_model_file.json";

class LoadScaling : public ChStaticNonLinearAnalysisIncremental::LoadIncrementCallback {
  public:
    LoadScaling(std::shared_ptr<ChNodeFEAxyzrot> node, double nominal_load)
        : m_node(node), m_nominal_load(nominal_load) {}

    virtual void OnLoadScaling(const double load_scaling,
                               const int iteration_n,
                               ChStaticNonLinearAnalysisIncremental* analysis) override {
        m_node->SetForce(ChVector3d(0, 0, load_scaling * m_nominal_load));
    }

  private:
    std::shared_ptr<ChNodeFEAxyzrot> m_node;
    double m_nominal_load;
};

ChVector3d RunModel(bool load_from_file) {
    // CREATE THE MODEL

    // Create a Chrono physical system
    ChSystemNSC sys;

    // auto myb = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    // myb->SetFixed(true);
    // myb->SetPos(ChVector3d(-0.5, 0, 0));
    // sys.Add(myb);

    // no gravity used here
    sys.SetGravitationalAcceleration(VNULL);

    // Set linear solver
// #undef CHRONO_PARDISO_MKL
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);
#endif

    // CREATE THE ASSEMBLY.
    //
    // The ChModalAssembly is the most important item when doing modal analysis.
    // You must add finite elements, bodies and constraints into this assembly in order
    // to compute the modal frequencies etc.; objects not added into this won't be counted.
    auto modal_assembly = chrono_types::make_shared<ChModalAssembly>();
    sys.Add(modal_assembly);

    modal_assembly->SetInternalNodesUpdate(UPDATE_INTERNAL_NODES);
    modal_assembly->SetUseLinearInertialTerm(USE_LINEAR_INERTIAL_TERM);
    modal_assembly->SetUseStaticCorrection(USE_STATIC_CORRECTION);
    modal_assembly->SetModalAutomaticGravity(USE_GRAVITY);

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

    mesh_internal->SetAutomaticGravity(USE_GRAVITY);

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

    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                      section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      n_elements,          // the number of ChElementBeamEuler to create
                      my_node_A_boundary,  // the 'A' point in space (beginning of beam)
                      my_node_B_boundary,  // ChVector3d(beam_L, 0, 0), // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();

    if (FIX_SUBASSEMBLY) {
        // BODY: the base:
        auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
        my_body_A->SetFixed(true);
        my_body_A->SetPos(ChVector3d(-0.5, 0, 0));
        modal_assembly->Add(my_body_A);

        // my_node_A_boundary->SetFixed(true); // NO - issues with bookeeping in modal_Hblock ***TO FIX***, for the
        // moment: Constraint the boundary node to truss
        my_root->Initialize(my_node_A_boundary, my_body_A, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));
        modal_assembly->Add(my_root);
    } else {
    }

    if (ADD_INTERNAL_BODY) {
        // BODY: in the middle, as internal
        auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(0.9, 1.4, 1.2, 100);
        my_body_B->SetPos(ChVector3d(beam_L * 0.5, 0, 0));
        modal_assembly->AddInternal(my_body_B);

        auto my_mid_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_mid_constr->Initialize(builder.GetLastBeamNodes()[n_elements / 2], my_body_B,
                                  ChFrame<>(ChVector3d(beam_L * 0.5, 0, 0), QUNIT));
        modal_assembly->AddInternal(my_mid_constr);
    }

    if (ADD_BOUNDARY_BODY) {
        // BODY: in the end, as boundary
        auto my_body_C = chrono_types::make_shared<ChBodyEasyBox>(0.8, 0.8, 0.8, 200);
        my_body_C->SetPos(ChVector3d(beam_L, 0, 0));
        modal_assembly->Add(my_body_C);

        auto my_end_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr->Initialize(builder.GetLastBeamNodes().back(), my_body_C,
                                  ChFrame<>(ChVector3d(beam_L, 0, 0), QUNIT));
        modal_assembly->Add(my_end_constr);
    }

    if (ADD_FORCE) {
        // Add a force to boundary nodes in the same way as in a full-state assembly
        my_node_B_boundary->SetForce(ChVector3d(0, -3, 0));  // to trigger some vibration at the free end

        // Add a force to internal nodes in the same way as in a full-state assembly
        auto my_node_internal =
            std::dynamic_pointer_cast<ChNodeFEAxyzrot>(modal_assembly->GetMeshesInternal().front()->GetNodes().back());
        my_node_internal->SetForce(ChVector3d(0, -1, 0));
        my_node_internal->SetTorque(ChVector3d(0, 2, 0));
    }

    if (ADD_OTHER_ASSEMBLY) {
        auto my_body_D = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.4, 0.4, 200);
        my_body_D->SetPos(ChVector3d(beam_L * 1.1, 0, 0));
        sys.Add(my_body_D);

        auto my_end_constr2 = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr2->Initialize(builder.GetLastBeamNodes().back(), my_body_D,
                                   ChFrame<>(ChVector3d(beam_L, 0, 0), QUNIT));
        sys.Add(my_end_constr2);

        // example if putting additional items in a second assembly (just a simple rotating blade)
        auto assembly0 = chrono_types::make_shared<ChAssembly>();
        sys.Add(assembly0);

        auto my_body_blade = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.6, 0.2, 150);
        my_body_blade->SetPos(ChVector3d(beam_L * 1.15, 0.3, 0));
        assembly0->Add(my_body_blade);

        auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        rotmotor1->Initialize(my_body_blade,                                           // slave
                              my_body_D,                                               // master
                              ChFrame<>(my_body_D->GetPos(), QuatFromAngleY(CH_PI_2))  // motor frame, in abs. coords
        );
        auto mwspeed =
            chrono_types::make_shared<ChFunctionConst>(CH_2PI);  // constant angular speed, in [rad/s], 2PI/s =360�/s
        rotmotor1->SetSpeedFunction(mwspeed);
        assembly0->Add(rotmotor1);
    }

    // set gravity
    if (USE_GRAVITY) {
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -0.25));  // -Z axis
    } else
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    if (!load_from_file) {
        auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
        ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_solver(num_modes, 1e-5, true, false,
                                                                                  eigen_solver);
        ChModalDampingRayleigh modal_damping(damping_alpha, damping_beta);
        modal_assembly->DoModalReduction(modal_solver, modal_damping);
        sys.Setup();
        sys.Update(UpdateFlags::UPDATE_ALL);

        std::ofstream file_out(reduced_model_filename);

        file_out << std::setprecision(12);

        if (!file_out.is_open()) {
            std::cerr << "Cannot save reduced model file: " << reduced_model_filename << std::endl;
            return false;
        }

        ChArchiveOutJSON archive_out(file_out);

        modal_assembly->SaveReducedModel(archive_out);
    } else {
        std::ifstream file_in(reduced_model_filename);

        if (!file_in.is_open()) {
            std::cerr << "Cannot load reduced model file: " << reduced_model_filename << std::endl;
            return false;
        }

        ChArchiveInJSON archive_in(file_in);
        modal_assembly->LoadReducedModel(archive_in);
        sys.Setup();
        sys.Update(UpdateFlags::UPDATE_ALL);
    }

    // Use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    if (hht_stepper) {
        hht_stepper->SetVerbose(false);
        hht_stepper->SetStepControl(false);
        hht_stepper->SetAlpha(-0.2);
        hht_stepper->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_STEP);
    }

    // Create a load scaling callback object
    double Pz = 3;
    auto load_scaling = chrono_types::make_shared<LoadScaling>(my_node_B_boundary, Pz);

    // Static analysis (incremental)
    ChStaticNonLinearAnalysisIncremental static_analysis;
    static_analysis.SetLoadIncrementCallback(load_scaling);
    static_analysis.SetMaxIterationsNewton(100);
    static_analysis.SetCorrectionTolerance(1e-4, 1e-8);
    static_analysis.SetVerbose(false);

    sys.DoStaticAnalysis(static_analysis);

    return my_node_B_boundary->GetPos();
}

int main(int argc, char* argv[]) {
    std::filesystem::create_directory(std::filesystem::path(out_dir));
    // Directory for output data
    if (!std::filesystem::exists(std::filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    bool passed = true;

    ChVector3d res_ref = RunModel(false);
    ChVector3d res_cmp = RunModel(true);

    passed &= (abs(res_cmp.x() - res_ref.x()) < error_threshold);
    passed &= (abs(res_cmp.y() - res_ref.y()) < error_threshold);
    passed &= (abs(res_cmp.z() - res_ref.z()) < error_threshold);

    if (!passed) {
        std::cout << "Reference: " << res_ref << std::endl;
        std::cout << "Result: " << res_cmp << std::endl;
    }

    return !passed;
}
