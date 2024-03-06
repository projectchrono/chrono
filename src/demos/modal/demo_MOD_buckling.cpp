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
const std::string out_dir = GetChronoOutputPath() + "BUCKLING";

constexpr double time_step = 0.001;
constexpr double time_length = 30;

constexpr bool RUN_ORIGIN = true;
constexpr bool RUN_MODAL = true;
constexpr bool USE_HERTING = false;
constexpr bool USE_LINEAR_INERTIAL_TERM = true;

void MakeAndRunDemo_Buckling(bool do_modal_reduction, double P0, ChMatrixDynamic<>& mdeflection) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double L_beam = 1.0;
    int n_elements = 16;

    double E = 7.3e10;
    double rho = 2.68e3;
    double miu = 0.31;
    double G = E / (2.0 * (1 + miu));

    double b = 0.005;
    double h = 0.02;
    double Area = b * h;
    double Iyy = 1 / 12.0 * b * h * h * h;  //=243.3
    double Izz = 1 / 12.0 * h * b * b * b;  //=15.21
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
    auto assembly_beam = chrono_types::make_shared<ChModalAssembly>();
    assembly_beam->use_linear_inertial_term = USE_LINEAR_INERTIAL_TERM;
    if (USE_HERTING)
        assembly_beam->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Herting;
    else
        assembly_beam->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Craig_Bampton;
    sys.Add(assembly_beam);

    auto mesh_internal_beam = chrono_types::make_shared<ChMesh>();
    assembly_beam->AddInternal(mesh_internal_beam);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()
    auto mesh_boundary_beam = chrono_types::make_shared<ChMesh>();
    assembly_beam->Add(mesh_boundary_beam);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()
    mesh_internal_beam->SetAutomaticGravity(false);
    mesh_boundary_beam->SetAutomaticGravity(false);

    auto my_node_root = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_ground->GetPos(), QUNIT));
    my_node_root->SetMass(0);
    my_node_root->GetInertia().setZero();
    mesh_boundary_beam->AddNode(my_node_root);

    auto my_rootlink = chrono_types::make_shared<ChLinkMateFix>();
    my_rootlink->Initialize(my_node_root, my_ground, ChFrame<>(my_ground->GetPos(), QUNIT));
    sys.AddLink(my_rootlink);

    auto my_node_tip =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_node_root->GetPos() + ChVector<>(L_beam, 0, 0), QUNIT));
    my_node_tip->SetMass(0);
    my_node_tip->GetInertia().setZero();
    mesh_boundary_beam->AddNode(my_node_tip);

    ChBuilderBeamEuler builder_beam;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_beam.BuildBeam(mesh_internal_beam,  // the mesh where to put the created nodes and elements
                           section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                           n_elements,          // the number of ChElementBeamEuler to create
                           my_node_root,        // the 'A' point in space (beginning of beam)
                           my_node_tip,         // the 'B' point in space (end of beam)
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

        auto damping_beam = ChModalDampingReductionR(*assembly_beam);
        // assembly_crank->verbose = true;
        assembly_beam->SwitchModalReductionON(modes_settings, damping_beam);
        assembly_beam->DumpSubassemblyMatrices(true, true, true, true,
                                               (out_dir + "/assembly_buckling_reduced").c_str());
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
        double T = 2.0;
        double P = 0;
        double Fy = 10e-9;
        GetLog() << "The applied lateral perturbation force is:\t" << Fy << " N.\n";

        mdeflection.resize(Nframes, 14);
        mdeflection.setZero();
        while (frame < Nframes) {
            double t = sys.GetChTime();
            if (t <= 0.5 * T)
                P = P0 * (1.0 - cos(CH_C_2PI * t / T)) * 0.5;
            else
                P = P0;

            my_node_tip->SetForce(ChVector<>(-P, Fy, 0));

            sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame_tip;
            my_node_root->TransformParentToLocal(my_node_tip->Frame(), relative_frame_tip);

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = P;
            // transverse displacement of the tip node
            mdeflection(frame, 2) = relative_frame_tip.GetPos().x();
            mdeflection(frame, 3) = relative_frame_tip.GetPos().y();
            mdeflection(frame, 4) = relative_frame_tip.GetPos().z();
            mdeflection(frame, 5) = relative_frame_tip.GetRot().Q_to_Rotv().x();
            mdeflection(frame, 6) = relative_frame_tip.GetRot().Q_to_Rotv().y();
            mdeflection(frame, 7) = relative_frame_tip.GetRot().Q_to_Rotv().z();
            if (do_modal_reduction) {
                ChFrameMoving<> relative_frame_F;
                my_node_root->TransformParentToLocal(assembly_beam->GetFloatingFrameOfReference(), relative_frame_F);
                mdeflection(frame, 8) = relative_frame_F.GetPos().x();
                mdeflection(frame, 9) = relative_frame_F.GetPos().y();
                mdeflection(frame, 10) = relative_frame_F.GetPos().z();
                mdeflection(frame, 11) = relative_frame_F.GetRot().Q_to_Rotv().x();
                mdeflection(frame, 12) = relative_frame_F.GetRot().Q_to_Rotv().y();
                mdeflection(frame, 13) = relative_frame_F.GetRot().Q_to_Rotv().z();
            }

            if (frame % 100 == 0) {
                GetLog() << "t: " << sys.GetChTime() << "\t";
                GetLog() << "Tip force Px: " << mdeflection(frame, 1) << "\t";
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
            double P0 = 37.6;
            GetLog() << "The applied axial compressive force is:\t" << P0 << " N.\n";
            MakeAndRunDemo_Buckling(false, P0, rel_deflection_corot);

            ChStreamOutAsciiFile file_defl_corot((out_dir + "/deflection_corot.dat").c_str());
            file_defl_corot.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_corot, file_defl_corot);
        }

        if (RUN_MODAL) {
            ChMatrixDynamic<> rel_deflection_modal;
            double P0 = 43.6;
            GetLog() << "The applied axial compressive force is:\t" << P0 << " N.\n";
            MakeAndRunDemo_Buckling(true, P0, rel_deflection_modal);

            ChStreamOutAsciiFile file_defl_modal((out_dir + "/deflection_modal.dat").c_str());
            file_defl_modal.SetNumFormat("%.12g");
            StreamOutDenseMatlabFormat(rel_deflection_modal, file_defl_modal);
        }
    } else {
        GetLog() << "  ...Error creating subdirectories\n";
    }

    return 0;
}
