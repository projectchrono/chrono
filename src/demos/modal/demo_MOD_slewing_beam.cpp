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
// Authors: Chao Peng, Alessandro Tasora
// =============================================================================
//
// Demo for modal assembly using the "slewing beam" model.
//
// This benchmark model "slewing beam" is originally proposed in:
// 1. Wu, S.C., Haug, E.J.: Geometric non-linear substructuring for dynamics of
// flexible mechanical elements. Int.J.Numer.Methods Eng.26, 2211–2226(1988).
// and then used in the research paper:
// 2. Sonneville, V., Scapolan, M., Shan, M. et al. Modal reduction procedures
// for flexible multibody dynamics. Multibody Syst Dyn 51, 377–418 (2021).
//
// The simulation results from modal redution are compared against the results from
// the Chrono corotational formulation.
//
// This demo is used to validate: the modal assembly's internal force, inertia,
// and numerical integration implementations.
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

// Supported direct sparse linear solver types: PardisoMKL and SparseQR
ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;

// -----------------------------------------------------------------------------

void RunSlewingBeam(bool do_modal_reduction,
                    ChModalAssembly::ReductionType reduction_type,
                    ChMatrixDynamic<>& res,
                    bool verbose) {
    double time_step = 0.002;
    double time_step_prt = 0.5;
    double time_length = 50;

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double beam_L = 8.0;
    int n_parts = 1;
    int n_totalelements = n_parts * 10;

    // damping coefficients
    double damping_alpha = 0;
    double damping_beta = 0.005;

    double EA = 5.03e6;
    double GJxx = 6.047e5;
    double EIyy = 1.654 * 20;  // increase the bending stiffness by 20 times, otherwise the simulation tends to
                               // divergent because of the extremely low flexibility.
    double EIzz = 566.6;
    double mass_per_unit_length = 0.2019;
    double Jxx = 2.28e-5;

    // Parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(ChVector3d(0, 0, 0));
    my_ground->SetFixed(true);
    sys.AddBody(my_ground);

    auto my_truss = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 10);
    my_truss->SetPos(ChVector3d(0, 0, 0));
    sys.AddBody(my_truss);

    // Build the driving joint
    auto my_link_truss = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()), QUNIT));
    auto driving_fun = chrono_types::make_shared<ChFunctionSetpoint>();
    driving_fun->SetSetpoint(0, 0);
    my_link_truss->SetAngleFunction(driving_fun);
    sys.AddLink(my_link_truss);

    // Beam section
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvancedGeneric>();
    section->SetAxialRigidity(EA);
    section->SetTorsionRigidityX(GJxx);
    section->SetBendingRigidityY(EIyy);
    section->SetBendingRigidityZ(EIzz);
    section->SetMassPerUnitLength(mass_per_unit_length);
    section->SetInertiaJxxPerUnitLength(Jxx);
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetArtificialJyyJzzFactor(1.0 / 500.0);
    section->SetRayleighDampingBeta(damping_beta);
    section->SetRayleighDampingAlpha(damping_alpha);

    // A function to make a single modal assembly
    auto MakeSingleModalAssembly = [&](std::shared_ptr<ChModalAssembly> mod_assem, int mn_ele, double mstart_x,
                                       double mend_x) {
        // Settings
        mod_assem->SetInternalNodesUpdate(true);
        mod_assem->SetUseLinearInertialTerm(true);
        mod_assem->SetUseStaticCorrection(false);
        mod_assem->SetModalAutomaticGravity(false);  // no gravity

        mod_assem->SetReductionType(reduction_type);

        sys.Add(mod_assem);

        // Initialize mesh
        auto mesh_internal = chrono_types::make_shared<ChMesh>();
        mod_assem->AddInternalMesh(mesh_internal);
        auto mesh_boundary = chrono_types::make_shared<ChMesh>();
        mod_assem->AddMesh(mesh_boundary);
        mesh_internal->SetAutomaticGravity(false);  // no gravity
        mesh_boundary->SetAutomaticGravity(false);  // no gravity

        // Build boundary nodes
        auto my_node_L = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(mstart_x, 0, 0), QUNIT));
        my_node_L->SetMass(0);
        my_node_L->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_L);

        auto my_node_R = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(mend_x, 0, 0), QUNIT));
        my_node_R->SetMass(0);
        my_node_R->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_R);

        ChBuilderBeamEuler builder;
        // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
        builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                          section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                          mn_ele,              // the number of ChElementBeamEuler to create
                          my_node_L,           // the 'A' point in space (beginning of beam)
                          my_node_R,           // the 'B' point in space (end of beam)
                          ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
        );
    };

    // Mesh the slewing beam with several seperate modal assemblies
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
    my_rootlink->Initialize(root_node, my_truss);
    sys.AddLink(my_rootlink);

    // Retrieve the tip node
    auto tip_node =
        std::dynamic_pointer_cast<ChNodeFEAxyzrot>(modal_assembly_list.back()->GetMeshes().front()->GetNodes().back());

    // No gravity
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Set linear solver
#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::PARDISO_MKL)
        solver_type = ChSolver::Type::SPARSE_QR;
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
        case ChSolver::Type::SPARSE_QR: {
            std::cout << "Using SparseQR linear solver" << std::endl;
            auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
            sys.SetSolver(qr_solver);
            break;
        }
    }

    sys.Setup();
    sys.Update();

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
        // hht_stepper->SetVerbose(false);
        hht_stepper->SetStepControl(false);
        // hht_stepper->SetRelTolerance(1e-4);
        // hht_stepper->SetAbsTolerances(1e-6);
        // hht_stepper->SetAlpha(-0.2);
        // hht_stepper->SetModifiedNewton(false);
        // hht_stepper->SetMaxiters(10);
    }

    double omega = 4.0;
    double T = 15.0;

    int Nframes = (int)(time_length / time_step);
    int itv_frame = (int)(time_step_prt / time_step);
    int frame = 0;

    res.resize(Nframes, do_modal_reduction ? 20 : 8);
    res.setZero();

    double step_timer = 0;

    if (verbose){
        std::cout << "|       Time       | Rot.Speed[rad/s] |    Rel.Def.X     |    Rel.Def.Y     |    Rel.Def.Z     |\n" <<
                     "|------------------|------------------|------------------|------------------|------------------|" << std::endl;
    }

    while (frame < Nframes) {
        double tao = sys.GetChTime() / T;

        double rot_angle = 0;
        if (tao < 1.0)
            rot_angle = omega * T * (tao * tao / 2.0 + (std::cos(CH_2PI * tao) - 1.0) / std::pow(CH_2PI, 2.0));
        else
            rot_angle = omega * T * (tao - 0.5);

        driving_fun->SetSetpoint(rot_angle, sys.GetChTime());

        sys.DoStepDynamics(time_step);
        step_timer += sys.GetTimerStep();

        res(frame, 0) = sys.GetChTime();
        res(frame, 1) = root_node->GetAngVelLocal().z();  // rotational angular speed, rad/s

        // beam tip deflection with respect to root
        ChFrameMoving<> relative_defl = root_node->TransformParentToLocal(tip_node->Frame());
        res(frame, 2) = relative_defl.GetPos().x() - beam_L;
        res(frame, 3) = relative_defl.GetPos().y();
        res(frame, 4) = relative_defl.GetPos().z();
        res(frame, 5) = relative_defl.GetRot().GetRotVec().x();
        res(frame, 6) = relative_defl.GetRot().GetRotVec().y();
        res(frame, 7) = relative_defl.GetRot().GetRotVec().z();

        if (do_modal_reduction) {
            // floating frame of reference: F
            ChFrameMoving<> floating_frame_F = modal_assembly_list.back()->GetFloatingFrameOfReference();

            ChFrameMoving<> relative_frame_F = root_node->TransformParentToLocal(floating_frame_F);
            res(frame, 8) = relative_frame_F.GetPos().x();
            res(frame, 9) = relative_frame_F.GetPos().y();
            res(frame, 10) = relative_frame_F.GetPos().z();
            res(frame, 11) = relative_frame_F.GetRot().GetRotVec().x();
            res(frame, 12) = relative_frame_F.GetRot().GetRotVec().y();
            res(frame, 13) = relative_frame_F.GetRot().GetRotVec().z();

            // residual of constraint equations on the configuration of the floating frame F.
            // It is expected to be zero.
            res.block(frame, 14, 1, 6) =
                modal_assembly_list.back()->GetConstraintsResidualF().transpose().segment(0, 6);
        }

        if (verbose && (frame % itv_frame == 0)) {
            std::cout << "| " << std::setw(16) << sys.GetChTime() << " | ";
            std::cout << std::setw(16) << res(frame, 1) << " | ";
            std::cout << std::setw(16) << relative_defl.GetPos().x() - beam_L << " | " << std::setw(16)
                      << relative_defl.GetPos().y() << " | " << std::setw(16) << relative_defl.GetPos().z() << " | "
                      << std::endl;
        }
        frame++;
    }

    std::cout << "Simulation time = " << step_timer << std::endl;
}

bool CompareResults(const ChMatrixDynamic<>& ref_mat, const ChMatrixDynamic<>& my_mat) {
    bool test_passed = true;

    // relative deflection of beam tip node w.r.t. root node.
    // used to reflect the numerical accuracy of the modal reduction method
    double tol_defl = 1e-12;
    ChVectorDynamic<> diff_defl_x = ref_mat.col(2) - my_mat.col(2);
    ChVectorDynamic<> diff_defl_y = ref_mat.col(3) - my_mat.col(3);
    ChVectorDynamic<> diff_defl_z = ref_mat.col(4) - my_mat.col(4);
    ChVectorDynamic<> diff_defl_rotx = ref_mat.col(5) - my_mat.col(5);
    ChVectorDynamic<> diff_defl_roty = ref_mat.col(6) - my_mat.col(6);
    ChVectorDynamic<> diff_defl_rotz = ref_mat.col(7) - my_mat.col(7);
    test_passed &= diff_defl_x.norm() < 0.2;       // axial
    test_passed &= diff_defl_y.norm() < 1.0;       // transverse deflection in the rotating plane
    test_passed &= diff_defl_z.norm() < tol_defl;  // lateral deflection out of the rotating plane, expected to be zero
    test_passed &= diff_defl_rotx.norm() < tol_defl;  // torsional angle, expected to be zero
    test_passed &= diff_defl_roty.norm() < tol_defl;  // rotational angle out of the rotating plane, expected to be zero
    test_passed &= diff_defl_rotz.norm() < 0.5;       // rotational angle in the rotating plane

    // relative position of the floating frame F w.r.t. beam node.
    // used to reflect the numerical stability of the modal reduction method
    double tol_relative_F = 1e-12;
    // ChVectorDynamic<> relative_F_x = my_mat.col(8);
    // ChVectorDynamic<> relative_F_y = my_mat.col(9);
    ChVectorDynamic<> relative_F_z = my_mat.col(10);
    ChVectorDynamic<> relative_F_rotx = my_mat.col(11);
    ChVectorDynamic<> relative_F_roty = my_mat.col(12);
    // ChVectorDynamic<> relative_F_rotz = my_mat.col(13);
    test_passed &= relative_F_z.norm() < tol_relative_F;  // displacement out of the rotating plane, expected to be zero
    test_passed &= relative_F_rotx.norm() < tol_relative_F;  // torsional angle, expected to be zero
    test_passed &=
        relative_F_roty.norm() < tol_relative_F;  // rotational angle out of the rotating plane, expected to be zero

    // residual of the six constraint equations on the floating frame F.
    // used to reflect the numerical stability of the modal reduction method
    ChVectorDynamic<> residual_F_x = my_mat.col(14);
    ChVectorDynamic<> residual_F_y = my_mat.col(15);
    ChVectorDynamic<> residual_F_z = my_mat.col(16);
    ChVectorDynamic<> residual_F_rotx = my_mat.col(17);
    ChVectorDynamic<> residual_F_roty = my_mat.col(18);
    ChVectorDynamic<> residual_F_rotz = my_mat.col(19);
    double tol_residual = 1e-10;
    test_passed &= residual_F_x.norm() < tol_residual;
    test_passed &= residual_F_y.norm() < tol_residual;
    test_passed &= residual_F_z.norm() < tol_residual;
    test_passed &= residual_F_rotx.norm() < tol_residual;
    test_passed &= residual_F_roty.norm() < tol_residual;
    test_passed &= residual_F_rotz.norm() < tol_residual;

    return test_passed;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    bool verbose = true;

    std::cout << "1. Run full corotational beam model:\n";
    // ChModalAssembly should be able to run successfully in the full state
    ChMatrixDynamic<> res_corot;
    RunSlewingBeam(false, ChModalAssembly::ReductionType::CRAIG_BAMPTON, res_corot, verbose);

    std::cout << "\n\n2. Run modal reduction model (Craig Bampton method):\n";
    ChMatrixDynamic<> res_modal_CraigBampton;
    RunSlewingBeam(true, ChModalAssembly::ReductionType::CRAIG_BAMPTON, res_modal_CraigBampton, verbose);
    bool check_CraigBampton = CompareResults(res_corot, res_modal_CraigBampton);
    std::cout << "\nCraig-Bampton reduced model check: " << (check_CraigBampton ? "PASSED" : "FAILED") << std::endl;

    std::cout << "\n\n3. Run modal reduction model (Herting method):\n";
    ChMatrixDynamic<> res_modal_Herting;
    RunSlewingBeam(true, ChModalAssembly::ReductionType::HERTING, res_modal_Herting, verbose);
    bool check_Herting = CompareResults(res_corot, res_modal_Herting);
    std::cout << "\nHerting reduced model check: " << (check_Herting ? "PASSED" : "FAILED") << std::endl;

    return 0;
}
