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
// Authors: Alessandro Tasora, Chao Peng
// =============================================================================
//
// Show how to use the ChAssembly to manage the hierarchy of the entire system
// containing substructures.
//
// =============================================================================

#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_modal/ChModalAssembly.h"

// #include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace filesystem;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "LINKS";

constexpr double time_step = 0.002;
constexpr double time_step_prt = 0.2;
constexpr double time_length = 10;

constexpr bool RUN_ORIGIN = false;
constexpr bool RUN_MODAL = true;
constexpr bool ROTATING_BEAM = false;
constexpr bool APPLY_FORCE_BOUNDARY = false;
constexpr bool APPLY_FORCE_INTERNAL = true;
constexpr bool USE_HERTING = true;

constexpr bool ADD_BB = true;
constexpr bool ADD_BI = true;
constexpr bool ADD_II = true;

constexpr bool USE_STATIC_CORRECTION = true;
constexpr bool UPDATE_INTERNAL_NODES = true;
constexpr bool USE_LINEAR_INERTIAL_TERM = true;
constexpr bool USE_GRAVITY = false;

void MakeAndRunDemo_Links(bool do_modal_reduction, ChMatrixDynamic<>& res) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double beam_L = 8.0;
    int n_elements = 10;

    // damping coefficients
    double damping_alpha = 0;
    double damping_beta = 0.005;

    double EA = 5.03e6;
    double GJxx = 6.047e5;
    double EIyy = 1.654 * 200;  // increase the stiffness for the sake of higher numerical stability
    double EIzz = 566.6;
    double mass_per_unit_length = 0.2019;
    double Jxx = 2.28e-5;

    // Parent system:
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
    my_link_truss->SetName("Driving link with rot angles");
    auto driving_fun = chrono_types::make_shared<ChFunctionSetpoint>();
    driving_fun->SetSetpoint(0, 0);
    my_link_truss->SetAngleFunction(driving_fun);
    sys.AddLink(my_link_truss);

    // Beam section:
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

    auto modal_assembly = chrono_types::make_shared<ChModalAssembly>();
    modal_assembly->SetInternalNodesUpdate(UPDATE_INTERNAL_NODES);
    modal_assembly->SetUseLinearInertialTerm(USE_LINEAR_INERTIAL_TERM);
    modal_assembly->SetUseStaticCorrection(USE_STATIC_CORRECTION);
    if (USE_HERTING)
        modal_assembly->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::HERTING);
    else
        modal_assembly->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::CRAIG_BAMPTON);

    modal_assembly->SetModalAutomaticGravity(USE_GRAVITY);
    sys.Add(modal_assembly);

    // Initialize mesh
    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddInternalMesh(mesh_internal);
    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddMesh(mesh_boundary);
    mesh_internal->SetAutomaticGravity(USE_GRAVITY);
    mesh_boundary->SetAutomaticGravity(USE_GRAVITY);

    // Build boundary nodes
    auto my_root_node = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    my_root_node->SetMass(0);
    my_root_node->GetInertia().setZero();
    mesh_boundary->AddNode(my_root_node);

    auto my_tip_node = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(beam_L, 0, 0), QUNIT));
    my_tip_node->SetMass(0);
    my_tip_node->GetInertia().setZero();
    mesh_boundary->AddNode(my_tip_node);

    ChBuilderBeamEuler builder;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                      section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      n_elements,          // the number of ChElementBeamEuler to create
                      my_root_node,        // the 'A' point in space (beginning of beam)
                      my_tip_node,         // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    // Root link
    auto my_rootlink = chrono_types::make_shared<ChLinkMateFix>();
    my_rootlink->Initialize(my_root_node, my_truss);
    my_rootlink->SetName("RootLink");
    sys.AddLink(my_rootlink);

    // link: internal - internal
    auto mid_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().at(n_elements / 2));
    auto my_body_II = chrono_types::make_shared<ChBodyEasyBox>(0.02, 0.05, 0.03, 2000);
    my_body_II->SetMass(0.2222);
    my_body_II->SetPos(ChVector3d(beam_L * 0.65, -0.2, beam_L * 0.01));
    // my_body_II->SetPos(mid_node->GetPos());
    if (ADD_II)
        modal_assembly->AddInternal(my_body_II);
    auto my_constr_II = chrono_types::make_shared<ChLinkMateFix>();
    my_constr_II->Initialize(mid_node, my_body_II);
    my_constr_II->SetName("Link_II");
    if (ADD_II)
        modal_assembly->AddInternal(my_constr_II);

    // link: boundary - boundary
    auto my_body_BB = chrono_types::make_shared<ChBodyEasyBox>(0.05, 0.05, 0.05, 2000);
    my_body_BB->SetMass(0.3333);
    my_body_BB->SetPos(ChVector3d(beam_L * 1.13, 0.14, beam_L * 0.02));
    // my_body_BB->SetPos(my_tip_node->GetPos());
    if (ADD_BB)
        modal_assembly->Add(my_body_BB);
    auto my_constr_BB = chrono_types::make_shared<ChLinkMateFix>();
    my_constr_BB->Initialize(my_tip_node, my_body_BB);
    my_constr_BB->SetName("Link_BB");
    if (ADD_BB)
        modal_assembly->Add(my_constr_BB);

    // link: boundary - internal
    auto node_int_b = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().at(n_elements / 2 + 2));
    auto my_body_BI = chrono_types::make_shared<ChBodyEasyBox>(0.05, 0.01, 0.03, 2000);
    my_body_BI->SetMass(0.4444);
    my_body_BI->SetPos(ChVector3d(beam_L * 0.85, 0.06, -beam_L * 0.02));
    // my_body_BI->SetPos(node_int_b->GetPos());
    if (ADD_BI)
        modal_assembly->Add(my_body_BI);
    auto my_constr_BI = chrono_types::make_shared<ChLinkMateFix>();
    my_constr_BI->Initialize(node_int_b, my_body_BI);
    my_constr_BI->SetName("Link_BI");
    if (ADD_BI)
        modal_assembly->AddInternal(my_constr_BI);
    // modal_assembly->Add(my_constr_BI);//should trigger error

    // set gravity
    if (USE_GRAVITY)
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));  // -Z axis
    else
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

        // Set linear solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);
#endif

    sys.Setup();
    sys.Update();

    sys.WriteSystemMatrices(true, true, true, true, (out_dir + "/sys"));

    // Do modal reduction for all modal assemblies
    if (do_modal_reduction) {
        ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        // ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        // The success of eigen solve is sensitive to the frequency shift (1e-4). If the eigen solver fails, try to
        // tune the shift value.
        auto modes_settings = ChModalSolveUndamped(13, 1e-4, 500, 1e-10, false, eigen_solver);
        auto damping_beam = ChModalDampingRayleigh(damping_alpha, damping_beta);

        // modal_assembly->SetVerbose(true);
        modal_assembly->WriteSubassemblyMatrices(true, true, true, true, out_dir + "/modal_assembly_full");
        modal_assembly->DoModalReduction(modes_settings, damping_beam);
        modal_assembly->WriteSubassemblyMatrices(true, true, true, true, out_dir + "/modal_assembly_reduced");
    }

    // Do dynamics simulation

    // use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    if (hht_stepper != nullptr) {
        // hht_stepper->SetVerbose(false);
        hht_stepper->SetStepControl(false);
        // hht_stepper->SetRelTolerance(1e-6);
        // hht_stepper->SetAbsTolerances(1e-12);
        // hht_stepper->SetAlpha(-0.2);
        // hht_stepper->SetModifiedNewton(false);
        // hht_stepper->SetMaxIters(20);
    }

    double omega = 1.0;
    double T = 15.0;

    int Nframes = (int)(time_length / time_step);
    int itv_frame = (int)(time_step_prt / time_step);
    int frame = 0;

    res.resize(Nframes, 23);
    res.setZero();
    while (frame < Nframes) {
        double tao = sys.GetChTime() / T;

        if (ROTATING_BEAM) {
            double rot_angle = 0;
            if (tao < 1.0)
                rot_angle = omega * T * (tao * tao / 2.0 + (cos(CH_2PI * tao) - 1.0) / pow(CH_2PI, 2.0));
            else
                rot_angle = omega * T * (tao - 0.5);

            driving_fun->SetSetpoint(rot_angle, sys.GetChTime());
        }

        // Add a force to generate vibration
        if (APPLY_FORCE_BOUNDARY) {
            if (sys.GetChTime() < 5.5) {
                my_tip_node->SetForce(my_tip_node->GetRot().Rotate(ChVector3d(0, 0, 0.3)));
                my_tip_node->SetTorque(ChVector3d(0, 0, 1.2));
            } else {
                my_tip_node->SetForce(ChVector3d(0, 0, 0));
                my_tip_node->SetTorque(ChVector3d(0, 0, 0));
            }
        }
        if (APPLY_FORCE_INTERNAL) {
            if (sys.GetChTime() < 100) {
                double t0 = sys.GetChTime();
                auto node_int1 =
                    std::dynamic_pointer_cast<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().at(n_elements / 2 - 1));
                node_int1->SetForce(
                    node_int1->GetRot().Rotate(ChVector3d(0, 0, 0.47 * sin(t0) * (1.0 + cos(3.8 * t0)))));
                node_int1->SetTorque(ChVector3d(0, 0, 1.23 * cos(t0) * (1.3 - sin(2 * t0))));
                // node_int1->SetForce(node_int1->GetRot().Rotate(ChVector3d(0, 0, 0.46*sin(t0))));
                // node_int1->SetTorque(ChVector3d(0, 0, 0.79*cos(t0)));
            } else {
                for (auto& node_int : builder.GetLastBeamNodes()) {
                    node_int->SetForce(ChVector3d(0, 0, 0));
                    node_int->SetTorque(ChVector3d(0, 0, 0));
                }
            }
        }

        sys.DoStepDynamics(time_step);

        res(frame, 0) = sys.GetChTime();
        // res(frame, 1) = root_node->GetRot().GetRotVec().z() * CH_RAD_TO_DEG;  // rotational angles, deg
        res(frame, 1) = my_root_node->GetAngVelLocal().z();  // rotational angular speed, rad/s

        // reaction forces and torques of constraints in the modal assembly
        if (ADD_BB) {
            res(frame, 2) = my_constr_BB->GetReaction2().force.x();
            res(frame, 3) = my_constr_BB->GetReaction2().force.y();
            res(frame, 4) = my_constr_BB->GetReaction2().force.z();
            res(frame, 5) = my_constr_BB->GetReaction2().torque.x();
            res(frame, 6) = my_constr_BB->GetReaction2().torque.y();
            res(frame, 7) = my_constr_BB->GetReaction2().torque.z();
        }
        if (ADD_BI) {
            res(frame, 8) = my_constr_BI->GetReaction2().force.x();
            res(frame, 9) = my_constr_BI->GetReaction2().force.y();
            res(frame, 10) = my_constr_BI->GetReaction2().force.z();
            res(frame, 11) = my_constr_BI->GetReaction2().torque.x();
            res(frame, 12) = my_constr_BI->GetReaction2().torque.y();
            res(frame, 13) = my_constr_BI->GetReaction2().torque.z();
        }
        if (ADD_II) {
            res(frame, 14) = my_constr_II->GetReaction2().force.x();
            res(frame, 15) = my_constr_II->GetReaction2().force.y();
            res(frame, 16) = my_constr_II->GetReaction2().force.z();
            res(frame, 17) = my_constr_II->GetReaction2().torque.x();
            res(frame, 18) = my_constr_II->GetReaction2().torque.y();
            res(frame, 19) = my_constr_II->GetReaction2().torque.z();
        }

        // positions of bodies connected with links
        /*res(frame, 2) = my_body_BB->GetPos().x();
        res(frame, 3) = my_body_BB->GetPos().y();
        res(frame, 4) = my_body_BB->GetPos().z();
        res(frame, 5) = my_body_BB->GetRot().GetRotVec().x();
        res(frame, 6) = my_body_BB->GetRot().GetRotVec().y();
        res(frame, 7) = my_body_BB->GetRot().GetRotVec().z();

        res(frame, 8) = my_body_BI->GetPos().x();
        res(frame, 9) = my_body_BI->GetPos().y();
        res(frame, 10) = my_body_BI->GetPos().z();
        res(frame, 11) = my_body_BI->GetRot().GetRotVec().x();
        res(frame, 12) = my_body_BI->GetRot().GetRotVec().y();
        res(frame, 13) = my_body_BI->GetRot().GetRotVec().z();

        res(frame, 14) = my_body_II->GetPos().x();
        res(frame, 15) = my_body_II->GetPos().y();
        res(frame, 16) = my_body_II->GetPos().z();
        res(frame, 17) = my_body_II->GetRot().GetRotVec().x();
        res(frame, 18) = my_body_II->GetRot().GetRotVec().y();
        res(frame, 19) = my_body_II->GetRot().GetRotVec().z();*/

        // residual of constraints
        res(frame, 20) = my_constr_BB->GetConstraintViolation().norm();
        res(frame, 21) = my_constr_BI->GetConstraintViolation().norm();
        res(frame, 22) = my_constr_II->GetConstraintViolation().norm();

        if (frame % itv_frame == 0) {
            std::cout << "t: " << sys.GetChTime() << "\t";
            std::cout << "Rot. Speed (rad/s): " << res(frame, 1) << "\t";
            std::cout << "Res. Constr: " << res(frame, 20) << " " << res(frame, 21) << " " << res(frame, 22) << "\t";
            ChFrameMoving<> tip_defl = my_root_node->TransformParentToLocal(my_tip_node->Frame());
            std::cout << "Tip pos[m]: " << tip_defl.GetPos().eigen().transpose()
                      << " rot[deg]: " << tip_defl.GetRot().GetRotVec().eigen().transpose() * CH_RAD_TO_DEG << "\n";
        }
        frame++;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (create_directory(path(out_dir))) {
        ChTimer m_timer_computation;
        double time_corot = 0;
        double time_modal = 0;

        if (RUN_ORIGIN) {
            std::cout << "1. Run corotational beam model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> res_link_reactions;
            MakeAndRunDemo_Links(false, res_link_reactions);
            m_timer_computation.stop();

            time_corot = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of corotational beam model: t_corot = \t" << time_corot << " seconds\n";
            m_timer_computation.reset();

            std::ofstream file_link_reactions((out_dir + "/link_reactions_corot.dat").c_str());
            file_link_reactions << std::setprecision(12) << std::scientific;
            StreamOut(res_link_reactions, file_link_reactions);
        }

        if (RUN_MODAL) {
            std::cout << "\n\n2. Run modal reduction model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> res_link_reactions;
            MakeAndRunDemo_Links(true, res_link_reactions);
            m_timer_computation.stop();

            time_modal = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of modal reduction model: t_modal = \t" << time_modal << " seconds\n";
            m_timer_computation.reset();

            std::ofstream file_link_reactions((out_dir + "/link_reactions_modal.dat").c_str());
            file_link_reactions << std::setprecision(12) << std::scientific;
            StreamOut(res_link_reactions, file_link_reactions);
        }

        if (time_corot && time_modal)
            std::cout << "\n\n3. Ratio of computation time: t_modal / t_corot = \t" << time_modal / time_corot << "\n";
    } else {
        std::cout << "  ...Error creating subdirectories\n";
    }

    return 0;
}
