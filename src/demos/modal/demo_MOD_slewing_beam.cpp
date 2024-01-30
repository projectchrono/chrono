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
// Authors: Alessandro Tasora, Radu Serban
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
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"

#include "chrono_modal/ChModalAssembly.h"
// #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace filesystem;
// using namespace chrono::irrlicht;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "SLEWING_BEAM";

constexpr double time_step = 0.002;
constexpr double time_length = 50;

constexpr bool RUN_ORIGIN = false;
constexpr bool RUN_MODAL = true;
constexpr bool ROTATING_BEAM = true;
constexpr bool APPLY_FORCE = !ROTATING_BEAM;
constexpr bool USE_HERTING = false;
constexpr bool DO_DYNAMICS = true;

constexpr bool USE_GEOMETRIC_STIFFNESS = true;
constexpr bool USE_INERTIAL_STIFFNESS = false;
constexpr bool USE_INERTIAL_DAMPING = true;
constexpr bool USE_QUADRATIC_VELOCITY_TERM = true;

void MakeAndRunDemo_SlewingBeam(bool do_modal_reduction, ChMatrixDynamic<>& mdeflection) {
    GetLog() << "\n\nRUN TEST\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Prepare for mesh
    double beam_L = 8.0;
    int n_elements = 10;
    double EA = 5.03e6;
    double GJxx = 6.047e5;
    double EIyy = 1.654;  // Too small value results in divergece.
    double EIzz = 566.6;
    double mass_per_unit_length = 0.2019;
    double Jxx = 2.28e-5;
    double offset_coeff = 0.0;

    // parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(ChVector<>(0, 0, 0));
    my_ground->SetBodyFixed(true);
    sys.AddBody(my_ground);

    auto my_truss = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 10);
    my_truss->SetPos(ChVector<>(0, 0, 0));
    sys.AddBody(my_truss);

     auto my_link_truss = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    //auto my_link_truss = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()), QUNIT));
    auto driving_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    driving_fun->SetSetpoint(0, 0);
     my_link_truss->SetAngleFunction(driving_fun);
    //my_link_truss->SetSpeedFunction(driving_fun);
    // my_link_truss->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_truss);

    // auto my_link_truss = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    // my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()),
    // QUNIT));
    //// my_link_truss->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    // sys.AddLink(my_link_truss);

    auto assembly = chrono_types::make_shared<ChModalAssembly>();
    assembly->use_geometric_stiffness = USE_GEOMETRIC_STIFFNESS;
    assembly->use_inertial_stiffness = USE_INERTIAL_STIFFNESS;
    assembly->use_inertial_damping = USE_INERTIAL_DAMPING;
    assembly->use_quadratic_velocity_term = USE_QUADRATIC_VELOCITY_TERM;
    if (USE_HERTING)
        assembly->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Herting;
    else
        assembly->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Craig_Bampton;
    sys.Add(assembly);

    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    assembly->AddInternal(mesh_internal);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()

    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    assembly->Add(mesh_boundary);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()

    mesh_internal->SetAutomaticGravity(false);
    mesh_boundary->SetAutomaticGravity(false);

    auto my_node_A =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_truss->GetPos() + ChVector<>(0, 0, 0), QUNIT));
    my_node_A->SetMass(0);
    my_node_A->GetInertia().setZero();
    mesh_boundary->AddNode(my_node_A);

    auto my_node_B =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_node_A->GetPos() + ChVector<>(beam_L, 0, 0), QUNIT));
    my_node_B->SetMass(0);
    my_node_B->GetInertia().setZero();
    mesh_boundary->AddNode(my_node_B);  // If added to mesh_boundary, the time stepper will diverge.

    // Beam section:
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvancedGeneric>();
    section->SetAxialRigidity(EA);
    section->SetXtorsionRigidity(GJxx);
    section->SetYbendingRigidity(EIyy);
    section->SetZbendingRigidity(EIzz);
    section->SetMassPerUnitLength(mass_per_unit_length);
    section->SetInertiaJxxPerUnitLength(Jxx);
    section->SetSectionRotation(1.0e-4 * offset_coeff);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(-0.3e-4 * offset_coeff);
    section->SetCentroidZ(1.2e-5 * offset_coeff);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetArtificialJyyJzzFactor(1.0 / 500.0);
    section->SetBeamRaleyghDampingBeta(0.002);
    section->SetBeamRaleyghDampingAlpha(0.000);

    ChBuilderBeamEuler builder;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                      section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      n_elements,          // the number of ChElementBeamEuler to create
                      my_node_A,           // the 'A' point in space (beginning of beam)
                      my_node_B,           // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    auto my_link_root = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_root->Initialize(my_node_A, my_truss, ChFrame<>(0.5 * (my_node_A->GetPos() + my_truss->GetPos()), QUNIT));
    my_link_root->SetConstrainedCoords(true, true, true, true, true, true);
    sys.AddLink(my_link_root);

    //{
    //    auto my_tip_mass = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 0.1);
    //    my_tip_mass->SetMass(mass_per_unit_length * beam_L * 0.05);
    //    my_tip_mass->SetPos(my_node_B->GetPos());
    //    sys.AddBody(my_tip_mass);

    //    auto my_link_tip = chrono_types::make_shared<ChLinkMateGeneric>();
    //    my_link_tip->Initialize(my_node_B, my_tip_mass,
    //                            ChFrame<>(0.5 * (my_node_B->GetPos() + my_tip_mass->GetPos()), QUNIT));
    //    my_link_tip->SetConstrainedCoords(true, true, true, true, true, true);
    //    sys.AddLink(my_link_tip);
    //}

    // gravity along -Y
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

    sys.DumpSystemMatrices(true, true, true, true, (out_dir + "/sys_dump").c_str());
    assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/assembly_dump").c_str());

    if (do_modal_reduction) {
        // ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        auto modes_settings = ChModalSolveUndamped(12, 1e-5, 500, 1e-10, false, eigen_solver);
        // auto damping_model = ChModalDampingRayleigh(0.000, 0.002);
        auto damping_model = ChModalDampingReductionR(*assembly);

        assembly->SwitchModalReductionON(modes_settings, damping_model);

        assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/assembly_dump_reduced").c_str());
    }

    // Do dynamics simulation
    if (DO_DYNAMICS) {
        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            // hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
             //hht_stepper->SetRelTolerance(1e-6);
             //hht_stepper->SetAbsTolerances(1e-12);
             //hht_stepper->SetAlpha(-0.3);
             //hht_stepper->SetModifiedNewton(false);
             //hht_stepper->SetMaxiters(20);
        }

        double omega = 4;
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

                /*double rot_speed = 0;
                if (tao < 1.0)
                    rot_speed = tao * omega;
                else if (tao < 2.0)
                    rot_speed = (2.0 - tao) * omega;
                else if (tao < 5.0)
                    rot_speed = 0;
                else if (tao < 6.0)
                    rot_speed = (tao - 5.0) * omega;
                else if (tao < 7.0)
                    rot_speed = (7.0 - tao) * omega;
                else if (tao < 10.0)
                    rot_speed = 0;
                else if (tao < 11.0)
                    rot_speed = (tao - 10.0) * omega;
                else if (tao < 12.0)
                    rot_speed = (12.0 - tao) * omega;
                else if (tao < 15.0)
                    rot_speed = 0;
                else
                    rot_speed = 0;

                driving_fun->SetSetpoint(rot_speed, sys.GetChTime());*/
            }

            // Add a force to generate vibration
            if (APPLY_FORCE) {
                if (sys.GetChTime() < 5.5)
                    my_node_B->SetTorque(ChVector<>(0, 0, 10.0));
                else
                    my_node_B->SetTorque(ChVector<>(0, 0, 0));
            }

            // if (APPLY_FORCE) {
            //     class MyCallback : public ChModalAssembly::CustomForceFullCallback {
            //       public:
            //         MyCallback(){};
            //         virtual void evaluate(
            //             ChVectorDynamic<>& computed_custom_F_full,  //< compute F here, size= n_boundary_coords_w +
            //             // n_internal_coords_w
            //             const ChModalAssembly& link  ///< associated modal assembly
            //         ) {
            //             // remember! assume F vector is already properly sized, but not zeroed!
            //             computed_custom_F_full.setZero();
            //             // This is an exact linear force acting on the flexible modalassembly.
            //             // If a list of forces are applied, the response might be different due to the discrepacy on
            //             the
            //             // nonlinearity under moderate deflections. There is no similar difference for applied
            //             torque.
            //             // todo: to remedy this difference for applied forces on internal nodes.
            //             // computed_custom_F_full[1 + 6] = 4.0;//boundary node B: Fy=4
            //             computed_custom_F_full[5 + 6] = 10;  // boundary node B: Mz=10
            //         }
            //     };
            //     auto my_callback = chrono_types::make_shared<MyCallback>();
            //     assembly->RegisterCallback_CustomForceFull(my_callback);
            // }

            if (sys.GetChTime() > time_length * 0.98)
                assembly->verbose = true;
            else
                assembly->verbose = false;

            sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame_tip;  // The tip node
            my_node_A->TransformParentToLocal(my_node_B->Frame(), relative_frame_tip);

            ChFrameMoving<> relative_frame_2;  // The middle node
            my_node_A->TransformParentToLocal(builder.GetLastBeamNodes()[(int)(n_elements / 2.0)]->Frame(),
                                              relative_frame_2);

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = my_node_A->GetRot().Q_to_Rotv().z() * CH_C_RAD_TO_DEG;
            // displacement
            mdeflection(frame, 2) = relative_frame_tip.GetPos().x();
            mdeflection(frame, 3) = relative_frame_tip.GetPos().y();
            mdeflection(frame, 4) = relative_frame_tip.GetPos().z();
            mdeflection(frame, 5) = relative_frame_tip.GetRot().Q_to_Rotv().x();
            mdeflection(frame, 6) = relative_frame_tip.GetRot().Q_to_Rotv().y();
            mdeflection(frame, 7) = relative_frame_tip.GetRot().Q_to_Rotv().z();
            mdeflection(frame, 8) = relative_frame_2.GetPos().x();
            mdeflection(frame, 9) = relative_frame_2.GetPos().y();
            mdeflection(frame, 10) = relative_frame_2.GetPos().z();
            mdeflection(frame, 11) = relative_frame_2.GetRot().Q_to_Rotv().x();
            mdeflection(frame, 12) = relative_frame_2.GetRot().Q_to_Rotv().y();
            mdeflection(frame, 13) = relative_frame_2.GetRot().Q_to_Rotv().z();
            if (do_modal_reduction) {
                // ChFrameMoving<> relative_frame_F = assembly->GetFloatingFrameOfReference();
                ChFrameMoving<> relative_frame_F;
                my_node_A->TransformParentToLocal(assembly->GetFloatingFrameOfReference(), relative_frame_F);
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
                beam_shape.setZero(builder.GetLastBeamNodes().size(), 6);
                int i_node = 0;
                for (auto node : builder.GetLastBeamNodes()) {
                    ChFrameMoving<> relative_frame_i;  // The middle node
                    my_node_A->TransformParentToLocal(node->Frame(), relative_frame_i);
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
                // GetLog() << "Root Pos:\t" << my_node_A->GetPos().x() << "\t" << my_node_A->GetPos().y() << "\t"
                //          << my_node_A->GetPos().z() << "\t";
                // GetLog() << "Root Rot:\t" << my_node_A->GetRot().Q_to_Rotv().x() << "\t"
                //          << my_node_A->GetRot().Q_to_Rotv().y() << "\t" << my_node_A->GetRot().Q_to_Rotv().z() <<
                //          "\n";
                // GetLog() << "Tip Pos:\t" << my_node_B->GetPos().x() << "\t" << my_node_B->GetPos().y() << "\t"
                //          << my_node_B->GetPos().z() << "\t";
                // GetLog() << "Tip Rot:\t" << my_node_B->GetRot().Q_to_Rotv().x() << "\t"
                //          << my_node_B->GetRot().Q_to_Rotv().y() << "\t" << my_node_B->GetRot().Q_to_Rotv().z() <<
                //          "\n";
            }
            frame++;
        }

    } else {  // static analysis

        assembly->verbose = true;

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
            assembly->RegisterCallback_CustomForceFull(my_callback);
        }

        // Perform static analysis
        sys.DoStaticNonlinear(100, false);
        // sys.DoStaticLinear();

        // play with parameters of static analysis
        // ChStaticNonLinearAnalysis static_analysis;
        // static_analysis.SetVerbose(true);
        // static_analysis.SetMaxIterations(100);
        // static_analysis.SetResidualTolerance(1e-20);
        // static_analysis.SetIncrementalSteps(20);
        // sys.DoStaticAnalysis(static_analysis);

        ChFrameMoving<> relative_frame_tip;  // The tip node
        my_node_A->TransformParentToLocal(my_node_B->Frame(), relative_frame_tip);
        GetLog() << "Rel. Def.:\t" << relative_frame_tip.GetPos().x() << "\t" << relative_frame_tip.GetPos().y() << "\t"
                 << relative_frame_tip.GetPos().z() << "\n";
        GetLog() << "\tRoot Pos:\t" << my_node_A->GetPos().x() << "\t" << my_node_A->GetPos().y() << "\t"
                 << my_node_A->GetPos().z() << "\t";
        GetLog() << "Root Rot:\t" << my_node_A->GetRot().Q_to_Rotv().x() << "\t" << my_node_A->GetRot().Q_to_Rotv().y()
                 << "\t" << my_node_A->GetRot().Q_to_Rotv().z() << "\n";
        GetLog() << "Tip Pos:\t" << my_node_B->GetPos().x() << "\t" << my_node_B->GetPos().y() << "\t"
                 << my_node_B->GetPos().z() << "\t";
        GetLog() << "Tip Rot:\t" << my_node_B->GetRot().Q_to_Rotv().x() << "\t" << my_node_B->GetRot().Q_to_Rotv().y()
                 << "\t" << my_node_B->GetRot().Q_to_Rotv().z() << "\n";
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
        if (RUN_ORIGIN) {
            ChMatrixDynamic<> rel_deflection_corot;
            MakeAndRunDemo_SlewingBeam(false, rel_deflection_corot);

            ChStreamOutAsciiFile file_defl_corot((out_dir + "/tip_deflection_corot.dat").c_str());
            file_defl_corot.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_corot, file_defl_corot);
        }

        if (RUN_MODAL) {
            ChMatrixDynamic<> rel_deflection_modal;
            MakeAndRunDemo_SlewingBeam(true, rel_deflection_modal);

            ChStreamOutAsciiFile file_defl_modal((out_dir + "/tip_deflection_modal.dat").c_str());
            file_defl_modal.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_modal, file_defl_modal);
        }
    } else {
        GetLog() << "  ...Error creating subdirectories\n";
    }

    return 0;
}
