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
const std::string out_dir = GetChronoOutputPath() + "SLIDER_CRANK";

constexpr double time_step = 0.001;
constexpr double time_length = 10;

constexpr bool RUN_ORIGIN = true;
constexpr bool RUN_MODAL = true;
constexpr bool USE_HERTING = false;
constexpr bool USE_LINEAR_INERTIAL_TERM = true;

void MakeAndRunDemo_SliderCrank(bool do_modal_reduction, ChMatrixDynamic<>& mdeflection) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double L_crank = 10.0;
    double L_rod = 20.0;
    int n_elements = 10;

    double E = 7.0e10;
    double rho = 2700;
    double miu = 0.31;
    double G = E / (2.0 * (1 + miu));

    double b = 0.4;
    double h = 0.4;
    double Area = b * h;  //=0.16
    double Iyy = 1 / 12.0 * b * h * h * h;
    double Izz = 1 / 12.0 * h * b * b * b;
    double Iyz = 0;
    double Ip = Iyy + Izz;

    double EA = E * Area;
    double EIyy = E * Iyy;
    double EIzz = E * Izz;
    double GIp = G * Ip;

    double mass_per_unit_length = rho * Area;
    double Jyy = rho * Iyy;
    double Jzz = rho * Izz;
    double Jxx = rho * Ip;
    double Jyz = rho * Iyz;

    // parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(VNULL);
    my_ground->SetBodyFixed(true);
    sys.AddBody(my_ground);

    // slider
    auto my_slider = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.2, 0.2, 10);
    my_slider->SetPos(my_ground->GetPos() + ChVector<>(L_crank + L_rod, 0, 0));
    my_slider->SetMass(1.0);
    my_slider->SetInertiaXX(ChVector<>(1, 1, 1));
    sys.AddBody(my_slider);

    auto my_link_slider = chrono_types::make_shared<ChLinkMatePrismatic>();
    my_link_slider->Initialize(my_slider, my_ground, ChFrame<>(my_ground->GetCoord()));
    sys.AddLink(my_link_slider);

    // Prepare for mesh: Beam section:
    auto section = chrono_types::make_shared<ChBeamSectionRayleighAdvancedGeneric>();
    section->SetAxialRigidity(EA);
    section->SetXtorsionRigidity(GIp);
    section->SetYbendingRigidity(EIyy);
    section->SetZbendingRigidity(EIzz);
    section->SetMassPerUnitLength(mass_per_unit_length);
    section->SetInertiaJxxPerUnitLength(Jxx);
    section->SetInertiasPerUnitLength(Jyy, Jzz, Jyz);
    section->SetArtificialJyyJzzFactor(0.0 / 500.0);
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetBeamRaleyghDampingBeta(0.0002);
    section->SetBeamRaleyghDampingAlpha(0.000);

    // for crank
    auto assembly_crank = chrono_types::make_shared<ChModalAssembly>();
    assembly_crank->use_linear_inertial_term = USE_LINEAR_INERTIAL_TERM;
    if (USE_HERTING)
        assembly_crank->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Herting;
    else
        assembly_crank->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Craig_Bampton;
    sys.Add(assembly_crank);

    auto mesh_internal_crank = chrono_types::make_shared<ChMesh>();
    assembly_crank->AddInternal(mesh_internal_crank);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()
    auto mesh_boundary_crank = chrono_types::make_shared<ChMesh>();
    assembly_crank->Add(mesh_boundary_crank);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()
    mesh_internal_crank->SetAutomaticGravity(false);
    mesh_boundary_crank->SetAutomaticGravity(false);

    auto my_node_O = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_ground->GetPos(), QUNIT));
    my_node_O->SetMass(0);
    my_node_O->GetInertia().setZero();
    mesh_boundary_crank->AddNode(my_node_O);

    auto my_node_B1 =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_node_O->GetPos() + ChVector<>(L_crank, 0, 0), QUNIT));
    my_node_B1->SetMass(0);
    my_node_B1->GetInertia().setZero();
    mesh_boundary_crank->AddNode(my_node_B1);

    auto my_link_crank = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_link_crank->Initialize(my_node_O, my_ground, ChFrame<>(my_ground->GetPos(), QUNIT));
    auto driving_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    driving_fun->SetSetpoint(0, 0);
    my_link_crank->SetAngleFunction(driving_fun);
    sys.AddLink(my_link_crank);

    ChBuilderBeamEuler builder_crank;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_crank.BuildBeam(mesh_internal_crank,  // the mesh where to put the created nodes and elements
                            section,              // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                            n_elements,           // the number of ChElementBeamEuler to create
                            my_node_O,            // the 'A' point in space (beginning of beam)
                            my_node_B1,           // the 'B' point in space (end of beam)
                            ChVector<>(0, 1, 0)   // the 'Y' up direction of the section for the beam
    );

    // for connecting rod
    auto assembly_rod = chrono_types::make_shared<ChModalAssembly>();
    assembly_rod->use_linear_inertial_term = USE_LINEAR_INERTIAL_TERM;
    if (USE_HERTING)
        assembly_rod->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Herting;
    else
        assembly_rod->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Craig_Bampton;
    sys.Add(assembly_rod);

    auto mesh_internal_rod = chrono_types::make_shared<ChMesh>();
    assembly_rod->AddInternal(mesh_internal_rod);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()
    auto mesh_boundary_rod = chrono_types::make_shared<ChMesh>();
    assembly_rod->Add(mesh_boundary_rod);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()
    mesh_internal_rod->SetAutomaticGravity(false);
    mesh_boundary_rod->SetAutomaticGravity(false);

    auto my_node_B2 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_node_B1->GetPos(), QUNIT));
    my_node_B2->SetMass(0);
    my_node_B2->GetInertia().setZero();
    mesh_boundary_rod->AddNode(my_node_B2);

    auto my_link_rodL = chrono_types::make_shared<ChLinkMateRevolute>();
    my_link_rodL->Initialize(my_node_B1, my_node_B2, ChFrame<>(my_node_B2->GetCoord()));
    sys.AddLink(my_link_rodL);

    auto my_node_S = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_slider->GetPos(), QUNIT));
    my_node_S->SetMass(0);
    my_node_S->GetInertia().setZero();
    mesh_boundary_rod->AddNode(my_node_S);

    auto my_link_rodR = chrono_types::make_shared<ChLinkMateRevolute>();
    my_link_rodR->Initialize(my_slider, my_node_S, ChFrame<>(my_slider->GetCoord()));
    sys.AddLink(my_link_rodR);

    ChBuilderBeamEuler builder_rod;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_rod.BuildBeam(mesh_internal_rod,   // the mesh where to put the created nodes and elements
                          section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                          2 * n_elements,      // the number of ChElementBeamEuler to create
                          my_node_B2,          // the 'A' point in space (beginning of beam)
                          my_node_S,           // the 'B' point in space (end of beam)
                          ChVector<>(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

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

    if (do_modal_reduction) {
        // ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        auto modes_settings = ChModalSolveUndamped(12, 1e-5, 500, 1e-10, false, eigen_solver);

        auto damping_crank = ChModalDampingReductionR(*assembly_crank);
        // assembly_crank->verbose = true;
        assembly_crank->SwitchModalReductionON(modes_settings, damping_crank);
        assembly_crank->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/assembly_crank_reduced").c_str());

        auto damping_rod = ChModalDampingReductionR(*assembly_rod);
        // assembly_rod->verbose = true;
        assembly_rod->SwitchModalReductionON(modes_settings, damping_rod);
        assembly_rod->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/assembly_rod_reduced").c_str());
    }

    // Do dynamics simulation
    {
        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            // hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
            // hht_stepper->SetRelTolerance(1e-4);
            // hht_stepper->SetAbsTolerances(1e-8);
            // hht_stepper->SetAlpha(-0.2);
            // hht_stepper->SetModifiedNewton(false);
            // hht_stepper->SetMaxiters(20);
        }

        int Nframes = (int)(time_length / time_step);
        int frame = 0;
        double t0 = 5.0;  // the time instant of rotation stop

        mdeflection.resize(Nframes, 14);
        mdeflection.setZero();
        while (frame < Nframes) {
            double t = sys.GetChTime();
            if (t < t0)
                driving_fun->SetSetpoint(3.0 * CH_C_PI * t * t / 50.0, t);
            else {
                driving_fun->SetSetpoint(3.0 * CH_C_PI * t0 * t0 / 50.0, t);
            }

            sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame_mid;  // The middle node of connecting rod
            my_node_B2->TransformParentToLocal(builder_rod.GetLastBeamNodes()[n_elements]->Frame(), relative_frame_mid);

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = my_node_O->GetRot().Q_to_Rotv().z() * CH_C_RAD_TO_DEG;
            // transverse displacement of middle point of the connecting rod
            mdeflection(frame, 2) = relative_frame_mid.GetPos().x();
            mdeflection(frame, 3) = relative_frame_mid.GetPos().y();
            mdeflection(frame, 4) = relative_frame_mid.GetPos().z();
            mdeflection(frame, 5) = relative_frame_mid.GetRot().Q_to_Rotv().x();
            mdeflection(frame, 6) = relative_frame_mid.GetRot().Q_to_Rotv().y();
            mdeflection(frame, 7) = relative_frame_mid.GetRot().Q_to_Rotv().z();
            if (do_modal_reduction) {
                ChFrameMoving<> relative_frame_F;
                my_node_B2->TransformParentToLocal(assembly_rod->GetFloatingFrameOfReference(), relative_frame_F);
                mdeflection(frame, 8) = relative_frame_F.GetPos().x();
                mdeflection(frame, 9) = relative_frame_F.GetPos().y();
                mdeflection(frame, 10) = relative_frame_F.GetPos().z();
                mdeflection(frame, 11) = relative_frame_F.GetRot().Q_to_Rotv().x();
                mdeflection(frame, 12) = relative_frame_F.GetRot().Q_to_Rotv().y();
                mdeflection(frame, 13) = relative_frame_F.GetRot().Q_to_Rotv().z();
            }

            if (frame % 100 == 0) {
                GetLog() << "t: " << sys.GetChTime() << "\t";
                GetLog() << "Rot Angle (deg): " << mdeflection(frame, 1) << "\t";
                GetLog() << "Rel. Def.:\t" << relative_frame_mid.GetPos().x() << "\t" << relative_frame_mid.GetPos().y()
                         << "\t" << relative_frame_mid.GetPos().z() << "\n";
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
            MakeAndRunDemo_SliderCrank(false, rel_deflection_corot);

            ChStreamOutAsciiFile file_defl_corot((out_dir + "/deflection_corot.dat").c_str());
            file_defl_corot.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_corot, file_defl_corot);
        }

        if (RUN_MODAL) {
            ChMatrixDynamic<> rel_deflection_modal;
            MakeAndRunDemo_SliderCrank(true, rel_deflection_modal);

            ChStreamOutAsciiFile file_defl_modal((out_dir + "/deflection_modal.dat").c_str());
            file_defl_modal.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_modal, file_defl_modal);
        }
    } else {
        GetLog() << "  ...Error creating subdirectories\n";
    }

    return 0;
}
