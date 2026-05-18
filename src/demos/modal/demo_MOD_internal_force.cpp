// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hu Wang
// =============================================================================
//
// Demo for internal force extraction from modal assembly.
//
// This demo shows the method to get the internal load of the modal assembly and
// results' accuracy. the assembly is loaded with gravity and top node force
//
// The model is like a wind turbine support structure, which contains some beams.
//
// The simulation results from modal redution are compared against the results from
// the Chrono corotational formulation.
//
// =============================================================================

#include <iomanip>
#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_modal/ChModalAssembly.h"
#include "chrono_modal/ChModalSolverUndamped.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

// -----------------------------------------------------------------------------

// Supported direct sparse linear solver types: PardisoMKL and SparseLU
ChSolver::Type solver_type = ChSolver::Type::SPARSE_LU;

// -----------------------------------------------------------------------------

void RunInternalForce(bool do_modal_reduction,
                      ChModalAssembly::ReductionType reduction_type,
                      ChMatrixDynamic<>& res,
                      bool verbose) {
    double time_step = 0.002;
    double time_step_prt = 1.0;
    double time_length = 50;

    // Create a Chrono physical system
    ChSystemNSC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double beam_L = 100.0;
    int n_parts = 1;
    int n_totalelements = n_parts * 30;

    // Damping coefficients
    double damping_alpha = 0;
    double damping_beta = 0.005;

    // Beam parameters
    double density = 8000.0;
    double elastic_modulus = 200000000000.0;
    double shear_modulus = 0.5 * elastic_modulus;
    double D = 5.0;
    double Thickness = 0.05;
    double Area = CH_PI * (std::pow(D / 2.0, 2.0) - std::pow(D / 2.0 - Thickness, 2.0));
    double Ibend = CH_PI / 4.0 * (std::pow(D / 2.0, 4.0) - std::pow(D / 2.0 - Thickness, 4.0));
    double Itors = Ibend * 2.0;

    double EA = elastic_modulus * Area;
    double GJxx = shear_modulus * Area;
    double EIyy = elastic_modulus * Ibend;
    double EIzz = elastic_modulus * Ibend;
    double mass_per_unit_length = Area * density;

    // Parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(ChVector3d(0, 0, 0));
    my_ground->SetFixed(true);
    sys.AddBody(my_ground);

    // Beam section
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvancedGeneric>();
    section->SetAxialRigidity(EA);
    section->SetTorsionRigidityX(GJxx);
    section->SetBendingRigidityY(EIyy);
    section->SetBendingRigidityZ(EIzz);
    section->SetMassPerUnitLength(mass_per_unit_length);
    section->SetInertiaJxxPerUnitLengthInMassReference(Itors * density);
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetArtificialJyyJzzFactor(1.0 / 500.0);
    section->SetRayleighDampingBeta(damping_beta);
    section->SetRayleighDampingAlpha(damping_alpha);

    // All beams pointer to get internal force
    std::vector<std::shared_ptr<ChElementBeamEuler>> beam_list;
    std::vector<int> beams_modal_index;

    int modal_index = 0;

    // A function to make a single modal assembly
    auto MakeSingleModalAssembly = [&](std::shared_ptr<ChModalAssembly> mod_assem, int mn_ele, double mstart_x,
                                       double mend_x) {
        // Settings
        // Switch off the InternalNodesUpdate and StaticCorrection
        // call UpdateInternalStateWithStaticEquilibrium manually instead.
        // It have better performance and accuracy
        mod_assem->SetInternalNodesUpdate(false);
        mod_assem->SetUseLinearInertialTerm(true);
        mod_assem->SetUseStaticCorrection(false);
        mod_assem->SetModalAutomaticGravity(true);

        mod_assem->SetReductionType(reduction_type);

        sys.Add(mod_assem);

        // Initialize mesh
        auto mesh_internal = chrono_types::make_shared<ChMesh>();
        mod_assem->AddInternalMesh(mesh_internal);
        auto mesh_boundary = chrono_types::make_shared<ChMesh>();
        mod_assem->AddMesh(mesh_boundary);
        mesh_internal->SetAutomaticGravity(true);
        mesh_boundary->SetAutomaticGravity(true);

        // Build boundary nodes
        auto my_node_bottom = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 0, mstart_x), QUNIT));
        my_node_bottom->SetMass(0);
        my_node_bottom->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_bottom);

        auto my_node_top = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 0, mend_x), QUNIT));
        my_node_top->SetMass(0);
        my_node_top->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_top);

        // Use attached style floating frame for better numerical convergence
        mod_assem->AddAttachedFrame(my_node_bottom, 0.5);
        mod_assem->AddAttachedFrame(my_node_top, 0.5);

        ChBuilderBeamEuler builder;
        // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
        builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                          section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                          mn_ele,              // the number of ChElementBeamEuler to create
                          my_node_bottom,      // the 'A' point in space (beginning of beam)
                          my_node_top,         // the 'B' point in space (end of beam)
                          ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
        );

        // Store beams and modals index for updating corotational quaternion
        for (const auto& beam : builder.GetLastBeamElements()) {
            beam_list.push_back(beam);
            beams_modal_index.push_back(modal_index);
        }
        modal_index++;
    };

    // Mesh the beam with several seperate modal assemblies
    std::vector<std::shared_ptr<ChModalAssembly>> modal_assembly_list;
    double delta_x = beam_L / n_parts;
    for (int i_part = 0; i_part < n_parts; i_part++) {
        auto my_assembly = chrono_types::make_shared<ChModalAssembly>();
        modal_assembly_list.push_back(my_assembly);
        MakeSingleModalAssembly(my_assembly, n_totalelements / n_parts, delta_x * i_part, delta_x * (i_part + 1));
    }

    // Build the fix links to connect modal assemblies
    if (n_parts > 1)
        for (int i_link = 0; i_link < n_parts - 1; i_link++) {
            auto node_L = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                modal_assembly_list.at(i_link)->GetMeshes().front()->GetNodes().back());
            auto node_R = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                modal_assembly_list.at(i_link + 1)->GetMeshes().front()->GetNodes().front());
            auto my_link = chrono_types::make_shared<ChLinkMateFix>();
            my_link->Initialize(node_L, node_R);
            sys.AddLink(my_link);
        }

    // Root link
    auto root_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_list.front()->GetMeshes().front()->GetNodes().front());
    auto my_rootlink = chrono_types::make_shared<ChLinkMateFix>();
    my_rootlink->Initialize(root_node, my_ground);
    sys.AddLink(my_rootlink);

    // Retrieve the node
    auto top_node =
        std::dynamic_pointer_cast<ChNodeFEAxyzrot>(modal_assembly_list.back()->GetMeshes().front()->GetNodes().back());
    auto bot_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_list.front()->GetMeshes().front()->GetNodes().front());

    auto my_top_body = chrono_types::make_shared<ChBody>();
    my_top_body->SetPos(top_node->GetPos());
    my_top_body->SetRot(QUNIT);
    my_top_body->SetFixed(false);
    my_top_body->SetMass(1e5);
    my_top_body->SetInertiaXX({1.1e5, 1.2e5, 1.3e5});
    sys.AddBody(my_top_body);

    auto top_link = chrono_types::make_shared<ChLinkMateFix>();
    top_link->Initialize(top_node, my_top_body);
    sys.AddLink(top_link);

    // Set gravity 9.8
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Set linear solver
#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::PARDISO_MKL)
        solver_type = ChSolver::Type::SPARSE_LU;
#endif

    switch (solver_type) {
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            std::cout << "Using PardisoMKL linear solver" << std::endl;
            auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            sys.SetSolver(mkl_solver);
#endif
            break;
        }
        default:
        case ChSolver::Type::SPARSE_LU: {
            std::cout << "Using SPARSE_LU linear solver" << std::endl;
            auto lu_solver = chrono_types::make_shared<ChSolverSparseLU>();
            sys.SetSolver(lu_solver);
            break;
        }
    }

    sys.Setup();
    sys.Update(UpdateFlags::UPDATE_ALL);

    // Do modal reduction for all modal assemblies
    if (do_modal_reduction) {
        auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();

        // The success of eigen solve is sensitive to the frequency shift (1e-4). If the eigen solver fails, try to
        // tune the shift value.
        ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_solver(12, 1e-4, true, false, eigen_solver);
        auto damping_beam = ChModalDampingRayleigh(damping_alpha, damping_beta);

        for (int i_part = 0; i_part < n_parts; i_part++) {
            modal_assembly_list.at(i_part)->DoModalReduction(modal_solver, damping_beam);
        }
    }

    // use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    if (hht_stepper != nullptr) {
        hht_stepper->SetStepControl(false);
    }

    double T = 15.0;

    int Nframes = (int)(time_length / time_step);
    int itv_frame = (int)(time_step_prt / time_step);
    int frame = 0;

    res.resize(Nframes, 8);
    res.setZero();

    double step_timer = 0;

    if (verbose) {
        std::cout
            << "|       Time       | Top input Fx[Nm] |    Bot My[Nm]    |    Bot Fz[N]     |     Top position[m]   \n"
            << "|------------------|------------------|------------------|------------------|--------------------------"
               "----|"
            << std::endl;
    }

    hht_stepper->SetVerbose(false);

    while (frame < Nframes) {
        double tao = sys.GetChTime() / T;

        double kf = 1e5;
        double km = 1e6;
        double F = kf * ((std::cos(CH_2PI * tao) - 1.0) + 5.0);
        double M = km * ((std::sin(CH_2PI * tao)) + 5.0);

        // Set external load
        ChVector3<> force_input{2.0 * F, 0.5 * F, 0.5 * F};
        ChVector3<> torque_input{1.0 * M, 1.5 * M, 0.5 * M};
        my_top_body->AddAccumulator();
        my_top_body->EmptyAccumulator(0);
        my_top_body->AccumulateForce(0, force_input, {0, 0, 0}, true);
        my_top_body->AccumulateTorque(0, torque_input, true);

        if (frame == 0) {
            sys.DoStaticLinear();
            sys.DoStaticLinear();
            sys.DoStaticLinear();
            sys.DoStaticLinear();
            sys.DoStaticLinear();
        }

        sys.DoStepDynamics(time_step);
        step_timer += sys.GetTimerStep();

        // Improve internal load accuracy by call UpdateInternalStateWithStaticEquilibrium and UpdateRotation
        // todo: try to improve code design, may need to modified fea code
        if (do_modal_reduction) {
            for (auto& modal : modal_assembly_list) {
                modal->UpdateInternalStateWithStaticEquilibrium();
            }
            int beam_index = 0;
            for (auto& beam : beam_list) {
                beam->UpdateRotation(
                    modal_assembly_list[beams_modal_index[beam_index++]]->GetFloatingFrameOfReference().GetRot());
            }
        }

        res(frame, 0) = sys.GetChTime();
        res(frame, 1) = top_node->GetPos().x();

        ChVectorDynamic<> Fres;
        Fres.resize(12);
        beam_list.front()->ComputeInternalForces(Fres);
        ChVector3<> force{Fres(0), Fres(1), Fres(2)};
        ChVector3<> torque{-Fres(5), Fres(4), Fres(3)};  // trans to global (z_to_x)
        res.block(frame, 2, 1, 6) = Fres.transpose().head(6);

        auto top_pos = top_node->GetPos();

        if (verbose && (frame % itv_frame == 0)) {
            std::cout << "| " << std::setw(16) << sys.GetChTime() - sys.GetStep() << " | ";
            std::cout << std::setw(16) << force_input.x() << " | ";
            std::cout << std::setw(16) << torque.y() << " | " << std::setw(16) << force.z() << " | "
                      << std::setprecision(5) << std::setw(8) << top_pos.x() << "  " << std::setprecision(5)
                      << std::setw(8) << top_pos.y() << "  " << std::setprecision(5) << std::setw(8) << top_pos.z()
                      << " |" << std::endl;
        }
        frame++;
    }

    std::cout << "Simulation time = " << step_timer << "s" << std::endl;
}

bool CompareResults(const ChMatrixDynamic<>& ref_mat, const ChMatrixDynamic<>& my_mat) {
    bool test_passed = true;

    ChVectorDynamic<> diff_force_x = ref_mat.col(2) - my_mat.col(2);
    ChVectorDynamic<> diff_force_y = ref_mat.col(3) - my_mat.col(3);
    ChVectorDynamic<> diff_force_z = ref_mat.col(4) - my_mat.col(4);
    ChVectorDynamic<> diff_moment_z = ref_mat.col(5) - my_mat.col(5);
    ChVectorDynamic<> diff_moment_y = ref_mat.col(6) - my_mat.col(6);
    ChVectorDynamic<> diff_moment_x = ref_mat.col(7) - my_mat.col(7);

    double diff_ratio_x = diff_force_x.norm() / ref_mat.col(2).norm();
    double diff_ratio_y = diff_force_y.norm() / ref_mat.col(3).norm();
    double diff_ratio_z = diff_force_z.norm() / ref_mat.col(4).norm();
    double diff_ratio_rz = diff_moment_z.norm() / ref_mat.col(5).norm();
    double diff_ratio_ry = diff_moment_y.norm() / ref_mat.col(6).norm();
    double diff_ratio_rx = diff_moment_x.norm() / ref_mat.col(7).norm();

    // check every internal force difference < 2%
    double err_threshold = 0.02;

    std::cout << "\nCheck internal force result with error threshold " << err_threshold * 100 << "%\n";

    std::cout << "Force  x norm diff: " << diff_ratio_x * 100.0 << "%" << std::endl;
    std::cout << "Force  y norm diff: " << diff_ratio_y * 100.0 << "%" << std::endl;
    std::cout << "Force  z norm diff: " << diff_ratio_z * 100.0 << "%" << std::endl;
    std::cout << "Moment x norm diff: " << diff_ratio_rx * 100.0 << "%" << std::endl;
    std::cout << "Moment y norm diff: " << diff_ratio_ry * 100.0 << "%" << std::endl;
    std::cout << "Moment z norm diff: " << diff_ratio_rz * 100.0 << "%" << std::endl;

    test_passed &= diff_ratio_x < err_threshold;   // shear force x
    test_passed &= diff_ratio_y < err_threshold;   // shear force y
    test_passed &= diff_ratio_z < err_threshold;   // axial force z
    test_passed &= diff_ratio_rx < err_threshold;  // bending moment x
    test_passed &= diff_ratio_ry < err_threshold;  // bending moment y
    test_passed &= diff_ratio_rz < err_threshold;  // torsion moment z

    // std::ofstream file_ref("modal_internal_ref.csv");
    // file_ref << std::setprecision(12) << std::scientific;
    // StreamOut(ref_mat, file_ref);

    // std::ofstream file_res("modal_internal_res.csv");
    // file_res << std::setprecision(12) << std::scientific;
    // StreamOut(my_mat, file_res);

    return test_passed;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    bool verbose = true;

    std::cout << "1. Run full corotational beam model:\n";
    // ChModalAssembly should be able to run successfully in the full state
    ChMatrixDynamic<> res_corot;
    RunInternalForce(false, ChModalAssembly::ReductionType::CRAIG_BAMPTON, res_corot, verbose);

    std::cout << "\n\n2. Run modal reduction model (Craig Bampton method):\n";
    ChMatrixDynamic<> res_modal_CraigBampton;
    RunInternalForce(true, ChModalAssembly::ReductionType::CRAIG_BAMPTON, res_modal_CraigBampton, verbose);
    bool check_CraigBampton = CompareResults(res_corot, res_modal_CraigBampton);
    std::cout << "\nCraig-Bampton reduced model check: " << (check_CraigBampton ? "PASSED" : "FAILED") << std::endl;

    return 0;
}
