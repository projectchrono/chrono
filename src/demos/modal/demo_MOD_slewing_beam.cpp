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

double time_step = 0.002;
double time_length = 2;
 //bool use_tangent_stiffness_Kc = true;

bool RUN_ORIGIN = false;
bool RUN_MODAL = true;
bool ROTATING_BEAM = true;

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

    // parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(ChVector<>(0, 0, 0));
    my_ground->SetBodyFixed(true);
    sys.AddBody(my_ground);

    auto my_truss = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 10);
    my_truss->SetPos(ChVector<>(3.111, 1.777, 0));
    sys.AddBody(my_truss);

    auto my_link_truss = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()), QUNIT));
    auto driving_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    driving_fun->SetSetpoint(0, 0);
    my_link_truss->SetAngleFunction(driving_fun);
     //my_link_truss->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_truss);

    // auto my_link_truss = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    // my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()),
    // QUNIT));
    //// my_link_truss->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    // sys.AddLink(my_link_truss);

    auto assembly = chrono_types::make_shared<ChModalAssembly>();
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
    mesh_boundary->AddNode(my_node_B);

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
    section->SetArtificialJyyJzzFactor(1.0 / 500);
    section->SetBeamRaleyghDampingBeta(0.000);
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
    // my_link_root->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_root);

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
    // Finally, log damped eigenvalue analysis to see the effect of the modal damping (0= search ALL damped modes)

    if (do_modal_reduction) {
        assembly->SwitchModalReductionON(
            6,  // The number of modes to retain from modal reduction, or a ChModalSolveUndamped with more settings
            ChModalDampingRayleigh(0.000,
                                   0.00)  // The damping model - Optional parameter: default is ChModalDampingNone().
        );

        assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/assembly_dump_reduced").c_str());
    }

    // Do dynamics simulation
    {
         //sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
         //auto euler_stepper = std::dynamic_pointer_cast<ChTimestepperEulerImplicitLinearized>(sys.GetTimestepper());
         //if (euler_stepper != nullptr) {
         //    euler_stepper->SetVerbose(false);
         //}

        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            //hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
            // hht_stepper->SetRelTolerance(1e-9);
            // hht_stepper->SetAbsTolerances(1e-16);
            //hht_stepper->SetAlpha(-0.2);
            // hht_stepper->SetModifiedNewton(true);
             //hht_stepper->SetMaxiters(10);
        }

        double omega = 4.0;
        double T = 15.0;

        int Nframes = (int)(time_length / time_step);
        int frame = 0;

        mdeflection.resize(Nframes, 8);
        mdeflection.setZero();
        while (frame < Nframes) {
            // if (sys.GetChTime() > 3.5)
            //     if (hht_stepper != nullptr) {
            //         hht_stepper->SetVerbose(true);
            //         hht_stepper->SetMaxiters(20);
            //     }

            double tao = sys.GetChTime() / T;

            if (ROTATING_BEAM) {
                double rot_angle = 0;
                if (tao < 1.0)
                    rot_angle = omega * T * (tao * tao / 2.0 + (cos(CH_C_2PI * tao) - 1.0) / pow(CH_C_2PI, 2.0));
                else
                    rot_angle = omega * T * (tao - 0.5);

                driving_fun->SetSetpoint(rot_angle, sys.GetChTime());
            }

            //// Add a force to generate vibration
            //if (sys.GetChTime() < 3.5)
            //    my_node_B->SetForce(ChVector<>(0, 5.0 * tao, 0));
            //else
            //    my_node_B->SetForce(ChVector<>(0, 0, 0));

            sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame_tip;  // The tip node
            my_node_A->TransformParentToLocal(my_node_B->Frame(), relative_frame_tip);

            ChFrameMoving<> relative_frame_2;  // The middle node
            my_node_A->TransformParentToLocal(builder.GetLastBeamNodes()[n_elements/2]->Frame(), relative_frame_2);

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = my_node_A->GetRot().Q_to_Rotv().z() * CH_C_RAD_TO_DEG;
            mdeflection(frame, 2) = relative_frame_tip.GetPos().x();
            mdeflection(frame, 3) = relative_frame_tip.GetPos().y();
            mdeflection(frame, 4) = relative_frame_tip.GetPos().z();
            mdeflection(frame, 5) = relative_frame_2.GetPos().x();
            mdeflection(frame, 6) = relative_frame_2.GetPos().y();
            mdeflection(frame, 7) = relative_frame_2.GetPos().z();

            if (frame % 100 == 0) {
                GetLog() << "t: " << sys.GetChTime() << "\t";
                GetLog() << "Rot Angle (deg): " << mdeflection(frame, 1) << "\t";
                GetLog() << "Rel. Def.:\t" << relative_frame_tip.GetPos().x() << "\t" << relative_frame_tip.GetPos().y()
                         << "\t" << relative_frame_tip.GetPos().z() << "\n";
            }
            frame++;
        }
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
