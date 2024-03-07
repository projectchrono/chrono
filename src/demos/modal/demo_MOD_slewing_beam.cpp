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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_modal/ChModalAssembly.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace filesystem;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "SLEWING_BEAM";

constexpr double time_step = 0.002;
constexpr double time_length = 50;

constexpr bool RUN_ORIGIN = false;
constexpr bool RUN_MODAL = true;
constexpr bool ROTATING_BEAM = true;
constexpr bool APPLY_FORCE = !ROTATING_BEAM;
constexpr bool USE_HERTING = true;
constexpr bool DO_DYNAMICS = true;

constexpr bool USE_LINEAR_INERTIAL_TERM = true;

void MakeAndRunDemo_SlewingBeam(bool do_modal_reduction, ChMatrixDynamic<>& mdeflection) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double beam_L = 8.0;
    int n_parts = 2;
    int n_totalelements = n_parts * 10;

    double EA = 5.03e6;
    double GJxx = 6.047e5;
    double EIyy = 1.654;
    double EIzz = 566.6;
    double mass_per_unit_length = 0.2019;
    double Jxx = 2.28e-5;

    // Parent system:
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(ChVector<>(0, 0, 0));
    my_ground->SetBodyFixed(true);
    sys.AddBody(my_ground);

    auto my_truss = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 10);
    my_truss->SetPos(ChVector<>(0, 0, 0));
    sys.AddBody(my_truss);

    // Build the driving joint
    auto my_link_truss = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()), QUNIT));
    auto driving_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    driving_fun->SetSetpoint(0, 0);
    my_link_truss->SetAngleFunction(driving_fun);
    sys.AddLink(my_link_truss);

    // Beam section:
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvancedGeneric>();
    section->SetAxialRigidity(EA);
    section->SetXtorsionRigidity(GJxx);
    section->SetYbendingRigidity(EIyy);
    section->SetZbendingRigidity(EIzz);
    section->SetMassPerUnitLength(mass_per_unit_length);
    section->SetInertiaJxxPerUnitLength(Jxx);
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetArtificialJyyJzzFactor(1.0 / 500.0);
    section->SetBeamRaleyghDampingBeta(0.002);
    section->SetBeamRaleyghDampingAlpha(0.000);

    // A function to make a single modal assembly
    auto MakeSingleModalAssembly = [&](std::shared_ptr<ChModalAssembly> mmodal_assembly, int mn_ele, double mstart_x,
                                       double mend_x) {
        // Settings
        mmodal_assembly->use_linear_inertial_term = USE_LINEAR_INERTIAL_TERM;
        if (USE_HERTING)
            mmodal_assembly->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Herting;
        else
            mmodal_assembly->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Craig_Bampton;

        sys.Add(mmodal_assembly);

        // Initialize mesh
        auto mesh_internal = chrono_types::make_shared<ChMesh>();
        mmodal_assembly->AddInternalMesh(mesh_internal);
        auto mesh_boundary = chrono_types::make_shared<ChMesh>();
        mmodal_assembly->AddMesh(mesh_boundary);
        mesh_internal->SetAutomaticGravity(false);
        mesh_boundary->SetAutomaticGravity(false);

        // Build boundary nodes
        auto my_node_L = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(mstart_x, 0, 0), QUNIT));
        my_node_L->SetMass(0);
        my_node_L->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_L);

        auto my_node_R = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(mend_x, 0, 0), QUNIT));
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
                          ChVector<>(0, 1, 0)  // the 'Y' up direction of the section for the beam
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
                modal_assembly_list.at(i_link)->Get_meshlist().front()->GetNodes().back());
            auto node_R = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                modal_assembly_list.at(i_link + 1)->Get_meshlist().front()->GetNodes().front());
            auto my_link = chrono_types::make_shared<ChLinkMateFix>();
            my_link->Initialize(node_L, node_R);
            sys.AddLink(my_link);
        }

    // Root link
    auto root_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_list.front()->Get_meshlist().front()->GetNodes().front());
    auto my_rootlink = chrono_types::make_shared<ChLinkMateFix>();
    my_rootlink->Initialize(root_node, my_truss);
    sys.AddLink(my_rootlink);

    // Retrieve the tip node
    auto tip_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_list.back()->Get_meshlist().front()->GetNodes().back());

    // Retrieve the middle node
    std::shared_ptr<ChNodeFEAxyzrot> mid_node;
    if (n_parts % 2) {  // odd
        auto beam_nodes = modal_assembly_list.at(n_parts / 2)->Get_internal_meshlist().front()->GetNodes();
        mid_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(beam_nodes.at(beam_nodes.size() / 2));
    } else {  // even
        auto beam_nodes = modal_assembly_list.at(n_parts / 2)->Get_meshlist().front()->GetNodes();
        mid_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(beam_nodes.front());
    }

    // Retrieve the all beam nodes
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> all_beam_nodes;
    for (auto massembly : modal_assembly_list) {
        for (auto node : massembly->Get_meshlist().front()->GetNodes())
            all_beam_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node));
        for (auto node : massembly->Get_internal_meshlist().front()->GetNodes())
            all_beam_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node));
    }

    // gravity is zero
    sys.Set_G_acc(ChVector<>(0, 0, 0));

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

    // Do modal reduction for all modal assemblies
    if (do_modal_reduction) {
        // ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        auto modes_settings = ChModalSolveUndamped(6, 1e-5, 500, 1e-10, false, eigen_solver);

        for (int i_part = 0; i_part < n_parts; i_part++) {
            auto damping_beam = ChModalDampingReductionR(*modal_assembly_list.at(i_part));
            // modal_assembly_list.at(i_part)->verbose = true;
            modal_assembly_list.at(i_part)->SwitchModalReductionON(modes_settings, damping_beam);
            modal_assembly_list.at(i_part)->DumpSubassemblyMatrices(
                true, true, true, true, (out_dir + "/modal_assembly_" + std::to_string(i_part)).c_str());
        }
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

        double omega = 4.0;
        double T = 15.0;

        int Nframes = (int)(time_length / time_step);
        int frame = 0;

        mdeflection.resize(Nframes, 20);
        mdeflection.setZero();
        while (frame < Nframes) {
            double tao = sys.GetChTime() / T;

            if (ROTATING_BEAM) {
                double rot_angle = 0;
                if (tao < 1.0)
                    rot_angle = omega * T * (tao * tao / 2.0 + (cos(CH_C_2PI * tao) - 1.0) / pow(CH_C_2PI, 2.0));
                else
                    rot_angle = omega * T * (tao - 0.5);

                driving_fun->SetSetpoint(rot_angle, sys.GetChTime());
            }

            // Add a force to generate vibration
            if (APPLY_FORCE) {
                if (sys.GetChTime() < 5.5) {
                    tip_node->SetForce(ChVector<>(0, 0, 0.02));
                    tip_node->SetTorque(ChVector<>(0, 0, 1.0));
                } else {
                    tip_node->SetForce(ChVector<>(0, 0, 0));
                    tip_node->SetTorque(ChVector<>(0, 0, 0));
                }
            }

            sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame_tip;  // The tip node
            root_node->TransformParentToLocal(tip_node->Frame(), relative_frame_tip);

            ChFrameMoving<> relative_frame_mid;  // The middle node
            root_node->TransformParentToLocal(mid_node->Frame(), relative_frame_mid);

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = root_node->GetRot().Q_to_Rotv().z() * CH_C_RAD_TO_DEG;
            // displacement
            mdeflection(frame, 2) = relative_frame_tip.GetPos().x();
            mdeflection(frame, 3) = relative_frame_tip.GetPos().y();
            mdeflection(frame, 4) = relative_frame_tip.GetPos().z();
            mdeflection(frame, 5) = relative_frame_tip.GetRot().Q_to_Rotv().x();
            mdeflection(frame, 6) = relative_frame_tip.GetRot().Q_to_Rotv().y();
            mdeflection(frame, 7) = relative_frame_tip.GetRot().Q_to_Rotv().z();
            mdeflection(frame, 8) = relative_frame_mid.GetPos().x();
            mdeflection(frame, 9) = relative_frame_mid.GetPos().y();
            mdeflection(frame, 10) = relative_frame_mid.GetPos().z();
            mdeflection(frame, 11) = relative_frame_mid.GetRot().Q_to_Rotv().x();
            mdeflection(frame, 12) = relative_frame_mid.GetRot().Q_to_Rotv().y();
            mdeflection(frame, 13) = relative_frame_mid.GetRot().Q_to_Rotv().z();
            if (do_modal_reduction) {
                ChFrameMoving<> relative_frame_F;
                root_node->TransformParentToLocal(modal_assembly_list.back()->GetFloatingFrameOfReference(),
                                                  relative_frame_F);
                mdeflection(frame, 14) = relative_frame_F.GetPos().x();
                mdeflection(frame, 15) = relative_frame_F.GetPos().y();
                mdeflection(frame, 16) = relative_frame_F.GetPos().z();
                mdeflection(frame, 17) = relative_frame_F.GetRot().Q_to_Rotv().x();
                mdeflection(frame, 18) = relative_frame_F.GetRot().Q_to_Rotv().y();
                mdeflection(frame, 19) = relative_frame_F.GetRot().Q_to_Rotv().z();
            }

            // output the beam configuration to check the deformed shape
            double time_instant = 28.436;
            if (do_modal_reduction && frame == (int)(time_instant / time_step)) {
                ChMatrixDynamic<> beam_shape;
                beam_shape.setZero(all_beam_nodes.size(), 6);
                int i_node = 0;
                for (auto node : all_beam_nodes) {
                    ChFrameMoving<> relative_frame_i;
                    root_node->TransformParentToLocal(node->Frame(), relative_frame_i);
                    beam_shape(i_node, 0) = relative_frame_i.GetPos().x();
                    beam_shape(i_node, 1) = relative_frame_i.GetPos().y();
                    beam_shape(i_node, 2) = relative_frame_i.GetPos().z();
                    beam_shape(i_node, 3) = relative_frame_i.GetRot().Q_to_Rotv().x();
                    beam_shape(i_node, 4) = relative_frame_i.GetRot().Q_to_Rotv().y();
                    beam_shape(i_node, 5) = relative_frame_i.GetRot().Q_to_Rotv().z();
                    i_node++;
                }
                ChStreamOutAsciiFile file_beam_shape((out_dir + "/beam_shape_modal.dat").c_str());
                file_beam_shape.SetNumFormat("%.12g");
                StreamOutDenseMatlabFormat(beam_shape, file_beam_shape);
            }

            if (frame % 100 == 0) {
                GetLog() << "t: " << sys.GetChTime() << "\t";
                GetLog() << "Rot Angle (deg): " << mdeflection(frame, 1) << "\t";
                GetLog() << "Rel. Def.:\t" << relative_frame_tip.GetPos().x() << "\t" << relative_frame_tip.GetPos().y()
                         << "\t" << relative_frame_tip.GetPos().z() << "\n";
            }
            frame++;
        }

    } else {  // static analysis

        for (int i_part = 0; i_part < n_parts; i_part++) {
            modal_assembly_list.at(i_part)->verbose = true;
        }

        if (APPLY_FORCE) {
            class MyCallback : public ChModalAssembly::CustomForceFullCallback {
              public:
                MyCallback(){};
                virtual void evaluate(
                    ChVectorDynamic<>& computed_custom_F_full,  //< compute F here, size= n_boundary_coords_w +
                    // n_internal_coords_w
                    const ChModalAssembly& link  ///< associated modal assembly
                ) {
                    // remember! assume F vector is already properly sized, but not zeroed!
                    computed_custom_F_full.setZero();
                    // This is an exact linear force acting on the flexible modalassembly.
                    // If a list of forces are applied, the response might be different due to the discrepacy on
                    // the nonlinearity under moderate deflections. There is no similar difference for applied
                    // torque. todo: to remedy this difference for applied forces on internal nodes.
                    // computed_custom_F_full[1 + 6] = 4.0;//boundary node B: Fy=4
                    computed_custom_F_full[5 + 6] = 20.71;  // boundary node B: Mz=30.7
                }
            };
            auto my_callback = chrono_types::make_shared<MyCallback>();
            modal_assembly_list.back()->RegisterCallback_CustomForceFull(my_callback);
        }

        // Perform static analysis
        sys.DoStaticNonlinear(100, false);

        ChFrameMoving<> relative_frame_tip;  // The tip node
        root_node->TransformParentToLocal(tip_node->Frame(), relative_frame_tip);
        GetLog() << "Rel. Def.:\t" << relative_frame_tip.GetPos().x() << "\t" << relative_frame_tip.GetPos().y() << "\t"
                 << relative_frame_tip.GetPos().z() << "\n";
        GetLog() << "\tRoot Pos:\t" << root_node->GetPos().x() << "\t" << root_node->GetPos().y() << "\t"
                 << root_node->GetPos().z() << "\t";
        GetLog() << "Root Rot:\t" << root_node->GetRot().Q_to_Rotv().x() << "\t" << root_node->GetRot().Q_to_Rotv().y()
                 << "\t" << root_node->GetRot().Q_to_Rotv().z() << "\n";
        GetLog() << "Tip Pos:\t" << tip_node->GetPos().x() << "\t" << tip_node->GetPos().y() << "\t"
                 << tip_node->GetPos().z() << "\t";
        GetLog() << "Tip Rot:\t" << tip_node->GetRot().Q_to_Rotv().x() << "\t" << tip_node->GetRot().Q_to_Rotv().y()
                 << "\t" << tip_node->GetRot().Q_to_Rotv().z() << "\n";
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

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
            GetLog() << "1. Run corotational beam model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> rel_deflection_corot;
            MakeAndRunDemo_SlewingBeam(false, rel_deflection_corot);
            m_timer_computation.stop();

            time_corot = m_timer_computation.GetTimeSeconds();
            GetLog() << "Computation time of corotational beam model: t_corot = \t" << time_corot << " seconds\n";
            m_timer_computation.reset();

            ChStreamOutAsciiFile file_defl_corot((out_dir + "/tip_deflection_corot.dat").c_str());
            file_defl_corot.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_corot, file_defl_corot);
        }

        if (RUN_MODAL) {
            GetLog() << "\n\n2. Run modal reduction model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> rel_deflection_modal;
            MakeAndRunDemo_SlewingBeam(true, rel_deflection_modal);
            m_timer_computation.stop();

            time_modal = m_timer_computation.GetTimeSeconds();
            GetLog() << "Computation time of modal reduction model: t_modal = \t" << time_modal << " seconds\n";
            m_timer_computation.reset();

            ChStreamOutAsciiFile file_defl_modal((out_dir + "/tip_deflection_modal.dat").c_str());
            file_defl_modal.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_modal, file_defl_modal);
        }

        if (time_corot && time_modal)
            GetLog() << "\n\n3. Ratio of computation time: t_modal / t_corot = \t" << time_modal / time_corot << "\n";
    } else {
        GetLog() << "  ...Error creating subdirectories\n";
    }

    return 0;
}
