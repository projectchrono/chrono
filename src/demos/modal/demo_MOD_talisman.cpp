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

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_modal/ChModalAssembly.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace filesystem;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "TALISMAN";

constexpr double time_step = 0.01;
constexpr double time_step_prt = 0.01;
constexpr double time_length = 5;

constexpr bool RUN_ORIGIN = true;
constexpr bool RUN_MODAL = false;
constexpr bool USE_HERTING = false;
constexpr bool DO_DYNAMICS = true;

constexpr bool USE_LINEAR_INERTIAL_TERM = true;

bool VISUALIZATION = false;

void MakeAndRunDemo_SlewingBeam(bool do_modal_reduction, ChMatrixDynamic<>& mdeflection) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;
    sys.Clear();
    sys.SetChTime(0);

    ChVisualSystemIrrlicht vis;
    if (VISUALIZATION) {
        vis.AttachSystem(&sys);
        vis.SetWindowSize(1024, 768);
        vis.SetWindowTitle("Modal reduction");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddCamera(ChVector3d(1, 1.3, 6), ChVector3d(3, 0, 0));
        vis.AddLightWithShadow(ChVector3d(20, 20, 20), ChVector3d(0, 0, 0), 50, 5, 50, 55);
        vis.AddLight(ChVector3d(-20, -20, 0), 6, ChColor(0.6f, 1.0f, 1.0f));
        vis.AddLight(ChVector3d(0, -20, -20), 6, ChColor(0.6f, 1.0f, 1.0f));
    }

    // Parameters
    int n_elements = 8;
    double w = 1.0;
    double h = 1.0;
    double d = 1.0;

    double EA = 10.51e6;
    double GJxx = 81.85;
    double EIyy = 126.1;
    double EIzz = 126.1;
    //double GAyy = 3.35e6;
    //double GAzz = 3.35e6;
    double mass_per_unit_length = 0.386;
    double Jyy = 4.63e-6;
    double Jzz = 4.63e-6;
    double Jxx = Jyy + Jzz;

    // Parent system:
    std::vector<std::shared_ptr<ChBody>> root_body_list;
    auto BuildDumbBody = [&](const ChVector3d& mpos) {
        auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1);
        my_ground->SetPos(mpos);
        my_ground->SetFixed(true);
        sys.AddBody(my_ground);
        root_body_list.push_back(my_ground);
    };
    BuildDumbBody(ChVector3d(0, 0, 0));  // ground, A0
    BuildDumbBody(ChVector3d(0, 0, h));  // ground, B0
    BuildDumbBody(ChVector3d(0, w, h));  // ground, C0
    BuildDumbBody(ChVector3d(0, w, 0));  // ground, D0

    // Settings
    std::shared_ptr<ChModalAssembly> modal_assembly = chrono_types::make_shared<ChModalAssembly>();
    // Settings
    modal_assembly->SetUseLinearInertialTerm(USE_LINEAR_INERTIAL_TERM);
    if (USE_HERTING)
        modal_assembly->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::HERTING);
    else
        modal_assembly->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::CRAIG_BAMPTON);
    sys.Add(modal_assembly);

    // Initialize mesh
    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddInternalMesh(mesh_internal);
    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddMesh(mesh_boundary);
    mesh_internal->SetAutomaticGravity(false);
    mesh_boundary->SetAutomaticGravity(false);

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> root_node_list;  // A0,B0,C0,D0
    auto BuildNode_Root = [&](const ChVector3d& mpos) {
        auto my_node = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mpos, QUNIT));
        my_node->SetMass(0);
        my_node->GetInertia().setZero();
        mesh_boundary->AddNode(my_node);

        root_node_list.push_back(my_node);
    };
    BuildNode_Root(ChVector3d(0, 0, 0));  // A0
    BuildNode_Root(ChVector3d(0, 0, h));  // B0
    BuildNode_Root(ChVector3d(0, w, h));  // C0
    BuildNode_Root(ChVector3d(0, w, 0));  // D0

    assert(root_body_list.size() == root_node_list.size());
    for (int i = 0; i < root_body_list.size(); i++) {
        auto my_link = chrono_types::make_shared<ChLinkMateFix>();
        my_link->Initialize(root_body_list[i], root_node_list[i]);
        sys.AddLink(my_link);
    }

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
    section->SetRayleighDampingBeta(0.005);
    section->SetRayleighDampingAlpha(0.000);
    section->SetDrawCircularRadius(0.01);

    ChBuilderBeamEuler builder;
    auto node_A0 = root_node_list[0];
    auto node_B0 = root_node_list[1];
    auto node_C0 = root_node_list[2];
    auto node_D0 = root_node_list[3];
    builder.BuildBeam(mesh_internal, section, n_elements, node_A0, node_B0, VECT_X);
    builder.BuildBeam(mesh_internal, section, n_elements, node_B0, node_C0, VECT_X);
    builder.BuildBeam(mesh_internal, section, n_elements, node_C0, node_D0, VECT_X);
    builder.BuildBeam(mesh_internal, section, n_elements, node_D0, node_A0, VECT_X);

    // A function to make a single modal assembly
    auto MakeSingleUnit = [&](const std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& mnode_list_left,
                              std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& mnode_list_right) {
        auto mnode_A0 = mnode_list_left[0];
        auto mnode_B0 = mnode_list_left[1];
        auto mnode_C0 = mnode_list_left[2];
        auto mnode_D0 = mnode_list_left[3];

        auto mnode_A1 =
            chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mnode_A0->GetPos() + ChVector3d(2 * d, 0, 0), QUNIT));
        mnode_A1->SetMass(0);
        mnode_A1->GetInertia().setZero();
        auto mnode_B1 =
            chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mnode_A0->GetPos() + ChVector3d(2 * d, 0, h), QUNIT));
        mnode_B1->SetMass(0);
        mnode_B1->GetInertia().setZero();
        auto mnode_C1 =
            chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mnode_A0->GetPos() + ChVector3d(2 * d, w, h), QUNIT));
        mnode_C1->SetMass(0);
        mnode_C1->GetInertia().setZero();
        auto mnode_D1 =
            chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mnode_A0->GetPos() + ChVector3d(2 * d, w, 0), QUNIT));
        mnode_D1->SetMass(0);
        mnode_D1->GetInertia().setZero();

        auto mnode_E0 =
            chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mnode_A0->GetPos() + ChVector3d(d, 0, 0), QUNIT));
        mnode_E0->SetMass(0);
        mnode_E0->GetInertia().setZero();
        auto mnode_F0 =
            chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(mnode_A0->GetPos() + ChVector3d(d, w, h), QUNIT));
        mnode_F0->SetMass(0);
        mnode_F0->GetInertia().setZero();

        if (mnode_A1->GetPos().x() > 11.9) {
            mesh_boundary->AddNode(mnode_A1);  // A6
            mesh_boundary->AddNode(mnode_B1);  // B6
            mesh_boundary->AddNode(mnode_C1);  // C6
            mesh_boundary->AddNode(mnode_D1);  // D6
        } else {
            mesh_internal->AddNode(mnode_A1);
            mesh_internal->AddNode(mnode_B1);
            mesh_internal->AddNode(mnode_C1);
            mesh_internal->AddNode(mnode_D1);
        }

        mesh_internal->AddNode(mnode_E0);
        mesh_internal->AddNode(mnode_F0);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_A0, mnode_E0, VECT_Y);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_E0, mnode_A1, VECT_Y);
        builder.BuildBeam(mesh_internal, section, n_elements * 2, mnode_B0, mnode_B1, VECT_Y);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_C0, mnode_F0, VECT_Y);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_F0, mnode_C1, VECT_Y);
        builder.BuildBeam(mesh_internal, section, n_elements * 2, mnode_D0, mnode_D1, VECT_Y);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_B0, mnode_E0, VECT_Y);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_B1, mnode_E0, VECT_Y);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_D0, mnode_F0, VECT_Y);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_D1, mnode_F0, VECT_Y);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_B0, mnode_F0, VECT_Z);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_B1, mnode_F0, VECT_Z);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_D0, mnode_E0, VECT_Z);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_D1, mnode_E0, VECT_Z);

        builder.BuildBeam(mesh_internal, section, n_elements, mnode_A1, mnode_B1, VECT_X);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_B1, mnode_C1, VECT_X);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_C1, mnode_D1, VECT_X);
        builder.BuildBeam(mesh_internal, section, n_elements, mnode_D1, mnode_A1, VECT_X);

        mnode_list_right.clear();
        mnode_list_right.push_back(mnode_A1);
        mnode_list_right.push_back(mnode_B1);
        mnode_list_right.push_back(mnode_C1);
        mnode_list_right.push_back(mnode_D1);
    };

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> node_list_left;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> node_list_right;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> node_list_tip;  // A6,B6,C6,D6
    node_list_left = root_node_list;
    for (int i_unit = 0; i_unit < 6; i_unit++) {
        MakeSingleUnit(node_list_left, node_list_right);
        if (i_unit == 5) {
            node_list_tip = node_list_right;
        } else {
            node_list_left = node_list_right;
            node_list_right.clear();
        }
    }

    auto node_A6 = node_list_tip[0];  // A6
    auto node_B6 = node_list_tip[1];  // B6
    auto node_C6 = node_list_tip[2];  // C6
    auto node_D6 = node_list_tip[3];  // D6

    // Retrieve the all beam nodes
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> all_beam_nodes;
    for (auto node : modal_assembly->GetMeshes().front()->GetNodes())
        all_beam_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node));
    for (auto node : modal_assembly->GetMeshesInternal().front()->GetNodes())
        all_beam_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node));
    std::cout << "There are " << all_beam_nodes.size() << " nodes\n";

    std::vector<std::shared_ptr<ChElementBeamEuler>> all_beam_elements;
    for (auto ele : modal_assembly->GetMeshes().front()->GetElements())
        all_beam_elements.push_back(std::dynamic_pointer_cast<ChElementBeamEuler>(ele));
    for (auto ele : modal_assembly->GetMeshesInternal().front()->GetElements())
        all_beam_elements.push_back(std::dynamic_pointer_cast<ChElementBeamEuler>(ele));
    std::cout << "There are " << all_beam_elements.size() << " elements\n";

    // gravity is zero
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    if (VISUALIZATION) {
        // VISUALIZATION ASSETS:
        auto visualizeInternalA = chrono_types::make_shared<ChVisualShapeFEA>(mesh_internal);
        visualizeInternalA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_DISP_Z);
        visualizeInternalA->SetColorscaleMinMax(-600, 600);
        visualizeInternalA->SetSmoothFaces(true);
        visualizeInternalA->SetWireframe(false);
        mesh_internal->AddVisualShapeFEA(visualizeInternalA);

        auto visualizeInternalB = chrono_types::make_shared<ChVisualShapeFEA>(mesh_internal);
        visualizeInternalB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        visualizeInternalB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        visualizeInternalB->SetSymbolsThickness(0.2);
        visualizeInternalB->SetSymbolsScale(0.1);
        visualizeInternalB->SetZbufferHide(false);
        mesh_internal->AddVisualShapeFEA(visualizeInternalB);

        auto visualizeBoundaryB = chrono_types::make_shared<ChVisualShapeFEA>(mesh_boundary);
        visualizeBoundaryB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        visualizeBoundaryB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        visualizeBoundaryB->SetSymbolsThickness(0.4);
        visualizeBoundaryB->SetSymbolsScale(4);
        visualizeBoundaryB->SetZbufferHide(false);
        mesh_boundary->AddVisualShapeFEA(visualizeBoundaryB);

        // This is needed if you want to see things in Irrlicht
        vis.BindAll();
    }

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
        // ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        auto modes_settings = ChModalSolveUndamped(12, 1e-5, 500, 1e-10, false, eigen_solver);

        auto damping_beam = ChModalDampingReductionR(*modal_assembly);
        // modal_assembly->verbose = true;
        modal_assembly->DoModalReduction(modes_settings, damping_beam);
        modal_assembly->WriteSubassemblyMatrices(true, true, true, true, (out_dir + "/modal_assembly"));
    }

    // Do dynamics simulation
    if (DO_DYNAMICS) {
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
            // hht_stepper->SetMaxiters(20);
        }

        double P0 = 75.0;
        double P1 = 0;

        int Nframes = (int)(time_length / time_step);
        int itv_frame = (int)(time_step_prt / time_step);
        int frame = 0;

        mdeflection.resize(Nframes, 20);
        mdeflection.setZero();
        while (frame < Nframes) {
            double t = sys.GetChTime();

            if (t < 1.0 || (t > 2.0 && t < 3.0))
                P1 = P0 * (1 - cos(CH_2PI * t)) / 2.0;
            else
                P1 = 0;

            node_B6->SetForce(ChVector3d(0, 0, P1));
            node_C6->SetForce(ChVector3d(0, 0, P1));

            if (VISUALIZATION) {
                if (!vis.Run())
                    break;
                else {
                    vis.BeginScene();
                    vis.Render();
                    vis.EndScene();
                    sys.DoStepDynamics(time_step);
                }
            } else
                sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame_tip = node_A0->TransformParentToLocal(node_A6->Frame());

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = P1;  // applied forces

            // displacement
            mdeflection(frame, 2) = relative_frame_tip.GetPos().x();
            mdeflection(frame, 3) = relative_frame_tip.GetPos().y();
            mdeflection(frame, 4) = relative_frame_tip.GetPos().z();
            mdeflection(frame, 5) = relative_frame_tip.GetRot().GetRotVec().x();
            mdeflection(frame, 6) = relative_frame_tip.GetRot().GetRotVec().y();
            mdeflection(frame, 7) = relative_frame_tip.GetRot().GetRotVec().z();

            if (do_modal_reduction) {
                ChFrameMoving<> floating_frame_F = modal_assembly->GetFloatingFrameOfReference();
                mdeflection(frame, 8) = floating_frame_F.GetPos().x();
                mdeflection(frame, 9) = floating_frame_F.GetPos().y();
                mdeflection(frame, 10) = floating_frame_F.GetPos().z();
                mdeflection(frame, 11) = floating_frame_F.GetRot().GetRotVec().x();
                mdeflection(frame, 12) = floating_frame_F.GetRot().GetRotVec().y();
                mdeflection(frame, 13) = floating_frame_F.GetRot().GetRotVec().z();

                // residual of constraint equations on configuration of F
                mdeflection.block(frame, 14, 1, 6) =
                    modal_assembly->GetConstraintsResidualF().transpose().segment(0, 6);
            }

            if (frame % itv_frame == 0) {
                std::cout << "t: " << sys.GetChTime() << "\t";
                std::cout << "Tip force: " << mdeflection(frame, 1) << "\t";
                std::cout << "Tip. Def.:\t" << relative_frame_tip.GetPos().x() << "\t" << relative_frame_tip.GetPos().y()
                         << "\t" << relative_frame_tip.GetPos().z() << "\n";
            }
            frame++;
        }
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
            ChMatrixDynamic<> rel_deflection_corot;
            MakeAndRunDemo_SlewingBeam(false, rel_deflection_corot);
            m_timer_computation.stop();

            time_corot = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of corotational beam model: t_corot = \t" << time_corot << " seconds\n";
            m_timer_computation.reset();

            std::ofstream file_defl_corot((out_dir + "/tip_deflection_corot.dat").c_str());
            file_defl_corot<< std::setprecision(12) << std::scientific;
            StreamOut(rel_deflection_corot, file_defl_corot);
        }

        if (RUN_MODAL) {
            std::cout << "\n\n2. Run modal reduction model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> rel_deflection_modal;
            MakeAndRunDemo_SlewingBeam(true, rel_deflection_modal);
            m_timer_computation.stop();

            time_modal = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of modal reduction model: t_modal = \t" << time_modal << " seconds\n";
            m_timer_computation.reset();

            std::ofstream file_defl_modal((out_dir + "/tip_deflection_modal.dat").c_str());
            file_defl_modal<< std::setprecision(12) << std::scientific;
            StreamOut(rel_deflection_modal, file_defl_modal);
        }

        if (time_corot && time_modal)
            std::cout << "\n\n3. Ratio of computation time: t_modal / t_corot = \t" << time_modal / time_corot << "\n";
    } else {
        std::cout << "  ...Error creating subdirectories\n";
    }

    return 0;
}
