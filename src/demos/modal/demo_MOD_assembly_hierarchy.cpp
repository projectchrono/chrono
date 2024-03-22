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

#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"

// #include "chrono_modal/ChModalAssembly.h"
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
const std::string out_dir = GetChronoOutputPath() + "ASSEMBLY_HIERARCHY";

double time_step = 0.001;
double time_length = 5.0;
bool use_tangent_stiffness_Kc = true;

void MakeAndRunDemo_SlewingBeam(bool add_subassembly_A, bool add_subassembly_B, ChMatrixDynamic<>& mdistance) {
    std::cout << "\n\nRUN TEST\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Prepare for mesh
    int n_elements = 10;
    double EA = 5.03e6;
    double GJxx = 6.047e5;
    double EIyy = 1.654 * 100;  // Too small value results in divergece.
    double EIzz = 566.6;
    double mass_per_unit_length = 0.2019;
    double Jxx = 2.28e-5;

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
    section->SetArtificialJyyJzzFactor(1.0 / 500);
    section->SetRayleighDampingBeta(0.002);
    section->SetRayleighDampingAlpha(0.000);

    // parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(ChVector3d(0, 0, 0));
    my_ground->SetFixed(true);
    sys.AddBody(my_ground);

    auto my_truss = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 10);
    my_truss->SetPos(ChVector3d(0.1, -1.2, 0));
    sys.AddBody(my_truss);

    auto my_link_truss = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_truss->Initialize(my_truss, my_ground, ChFrame<>(0.5 * (my_truss->GetPos() + my_ground->GetPos()), QUNIT));
    my_link_truss->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_truss->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_truss);

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    sys.AddMesh(my_mesh);

    auto my_node_Upper =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_truss->GetPos() + ChVector3d(-0.1, -1.3, 0), QUNIT));
    my_node_Upper->SetMass(0);
    my_node_Upper->GetInertia().setZero();
    my_mesh->AddNode(my_node_Upper);

    auto my_link_node_Upper = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_node_Upper->Initialize(my_node_Upper, my_truss,
                                   ChFrame<>(0.5 * (my_node_Upper->GetPos() + my_truss->GetPos()), QUNIT));
    my_link_node_Upper->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_node_Upper->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_node_Upper);

    auto my_node_Down = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(my_node_Upper->GetPos() + ChVector3d(0.5, -13.5, 0), QUNIT));
    my_node_Down->SetMass(0);
    my_node_Down->GetInertia().setZero();
    my_mesh->AddNode(my_node_Down);

    ChBuilderBeamEuler builder_par;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_par.BuildBeam(my_mesh,             // the mesh where to put the created nodes and elements
                          section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                          n_elements,          // the number of ChElementBeamEuler to create
                          my_node_Upper,       // the 'A' point in space (beginning of beam)
                          my_node_Down,        // the 'B' point in space (end of beam)
                          ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    // assembly A: the first substructure
    auto mesh_A = chrono_types::make_shared<ChMesh>();

    auto my_node_L_A = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(my_node_Down->GetPos() + ChVector3d(-0.1, -1.3, 0), QUNIT));
    my_node_L_A->SetMass(0);
    my_node_L_A->GetInertia().setZero();
    mesh_A->AddNode(my_node_L_A);

    auto my_link_L_A = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_L_A->Initialize(my_node_L_A, my_node_Down,
                            ChFrame<>(0.5 * (my_node_L_A->GetPos() + my_node_Down->GetPos()), QUNIT));
    my_link_L_A->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_L_A->SetUseTangentStiffness(use_tangent_stiffness_Kc);

    auto my_node_R_A = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(my_node_L_A->GetPos() + ChVector3d(-13.8, -12.3, 0), QUNIT));
    my_node_R_A->SetMass(0);
    my_node_R_A->GetInertia().setZero();
    mesh_A->AddNode(my_node_R_A);

    ChBuilderBeamEuler builder_A;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_A.BuildBeam(mesh_A,              // the mesh where to put the created nodes and elements
                        section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                        n_elements,          // the number of ChElementBeamEuler to create
                        my_node_L_A,         // the 'A' point in space (beginning of beam)
                        my_node_R_A,         // the 'B' point in space (end of beam)
                        ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 10);
    my_body_A->SetPos(my_node_R_A->GetPos() + ChVector3d(-2.5, -3.3, 0));

    auto my_link_R_A = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_R_A->Initialize(my_node_R_A, my_body_A,
                            ChFrame<>(0.5 * (my_node_R_A->GetPos() + my_body_A->GetPos()), QUNIT));
    my_link_R_A->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_R_A->SetUseTangentStiffness(use_tangent_stiffness_Kc);

    if (add_subassembly_A) {
        auto assembly_A = chrono_types::make_shared<ChAssembly>();
        sys.Add(assembly_A);
        assembly_A->Add(mesh_A);
        assembly_A->Add(my_body_A);
        assembly_A->Add(my_link_L_A);
        assembly_A->Add(my_link_R_A);
    } else {
        sys.Add(mesh_A);
        sys.Add(my_body_A);
        sys.Add(my_link_L_A);
        sys.Add(my_link_R_A);
    }

    // assembly B: the second substructure
    auto mesh_B = chrono_types::make_shared<ChMesh>();

    auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 1.5, 10);
    my_body_B->SetPos(my_node_Down->GetPos() + ChVector3d(0.22, -0.5, 0));

    auto my_link_L_B = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_L_B->Initialize(my_body_B, my_node_Down,
                            ChFrame<>(0.5 * (my_body_B->GetPos() + my_node_Down->GetPos()), QUNIT));
    my_link_L_B->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_L_B->SetUseTangentStiffness(use_tangent_stiffness_Kc);

    auto my_node_L_B =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_body_B->GetPos() + ChVector3d(0.2, -0.3, -0.4), QUNIT));
    my_node_L_B->SetMass(0);
    my_node_L_B->GetInertia().setZero();
    mesh_B->AddNode(my_node_L_B);

    auto my_link_R_B = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_link_R_B->Initialize(my_node_L_B, my_body_B,
                            ChFrame<>(0.5 * (my_node_L_B->GetPos() + my_body_B->GetPos()), QUNIT));
    auto driving_fun_B = chrono_types::make_shared<ChFunctionSetpoint>();
    driving_fun_B->SetSetpoint(0, 0);
    my_link_R_B->SetAngleFunction(driving_fun_B);
    my_link_R_B->SetUseTangentStiffness(use_tangent_stiffness_Kc);

    // auto my_link_R_B = chrono_types::make_shared<ChLinkMateGeneric>();
    // my_link_R_B->Initialize(my_node_L_B, my_body_B,
    //                         ChFrame<>(0.5 * (my_node_L_B->GetPos() + my_body_B->GetPos()), QUNIT));
    // my_link_R_B->SetConstrainedCoords(true, true, true, false, false, false);
    // my_link_R_B->SetUseTangentStiffness(use_tangent_stiffness_Kc);

    auto my_node_R_B = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(my_node_L_B->GetPos() + ChVector3d(0.8, -15.0, 10), QUNIT));
    my_node_R_B->SetMass(0);
    my_node_R_B->GetInertia().setZero();
    mesh_B->AddNode(my_node_R_B);

    ChBuilderBeamEuler builder_B;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_B.BuildBeam(mesh_B,              // the mesh where to put the created nodes and elements
                        section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                        n_elements,          // the number of ChElementBeamEuler to create
                        my_node_L_B,         // the 'A' point in space (beginning of beam)
                        my_node_R_B,         // the 'B' point in space (end of beam)
                        ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    if (add_subassembly_B) {
        auto assembly_B = chrono_types::make_shared<ChAssembly>();
        sys.Add(assembly_B);
        assembly_B->Add(mesh_B);
        assembly_B->Add(my_body_B);
        assembly_B->Add(my_link_L_B);
        assembly_B->Add(my_link_R_B);
    } else {
        sys.Add(mesh_B);
        sys.Add(my_body_B);
        sys.Add(my_link_L_B);
        sys.Add(my_link_R_B);
    }

    // parent part C
    auto my_body_C = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 1.5, 10);
    my_body_C->SetPos(my_node_R_B->GetPos() + ChVector3d(-0.22, 0.5, 0));
    sys.AddBody(my_body_C);

    auto my_link_body_C = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_body_C->Initialize(my_body_C, my_node_R_B,
                               ChFrame<>(0.5 * (my_body_C->GetPos() + my_node_R_B->GetPos()), QUNIT));
    my_link_body_C->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_body_C->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_body_C);

    auto mesh_C = chrono_types::make_shared<ChMesh>();
    sys.AddMesh(mesh_C);

    auto my_node_L_C = chrono_types::make_shared<ChNodeFEAxyzrot>(
        ChFrame<>(my_body_C->GetPos() + ChVector3d(-7.0, -12.0, 0.4), QUNIT));
    my_node_L_C->SetMass(0);
    my_node_L_C->GetInertia().setZero();
    mesh_C->AddNode(my_node_L_C);

    auto my_link_L_C = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_L_C->Initialize(my_node_L_C, my_body_C,
                            ChFrame<>(0.5 * (my_node_L_C->GetPos() + my_body_C->GetPos()), QUNIT));
    my_link_L_C->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_L_C->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_L_C);

    auto my_node_R_C =
        chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(my_body_A->GetPos() + ChVector3d(0.2, -0.3, 0), QUNIT));
    my_node_R_C->SetMass(0);
    my_node_R_C->GetInertia().setZero();
    mesh_C->AddNode(my_node_R_C);

    auto my_link_R_C = chrono_types::make_shared<ChLinkMateGeneric>();
    my_link_R_C->Initialize(my_node_R_C, my_body_A,
                            ChFrame<>(0.5 * (my_node_R_C->GetPos() + my_body_A->GetPos()), QUNIT));
    my_link_R_C->SetConstrainedCoords(true, true, true, false, false, false);
    my_link_R_C->SetUseTangentStiffness(use_tangent_stiffness_Kc);
    sys.AddLink(my_link_R_C);

    ChBuilderBeamEuler builder_C;
    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder_C.BuildBeam(mesh_C,              // the mesh where to put the created nodes and elements
                        section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                        n_elements,          // the number of ChElementBeamEuler to create
                        my_node_L_C,         // the 'A' point in space (beginning of beam)
                        my_node_R_C,         // the 'B' point in space (end of beam)
                        ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    bool add_force = true;
    if (add_force) {
        builder_A.GetLastBeamNodes()[(int)(n_elements / 2.0)]->SetForce(
            ChVector3d(10, -30, 0));  // to trigger some vibration at the middle point of beam A
        builder_B.GetLastBeamNodes()[(int)(n_elements / 2.0)]->SetForce(
            ChVector3d(-21.2, -12.3, 30.6));  // to trigger some vibration at the middle point of beam B
        builder_C.GetLastBeamNodes()[(int)(n_elements / 2.0)]->SetForce(
            ChVector3d(21.2, 12.3, -30.6));  // to trigger some vibration at the middle point of beam B
    }

    // gravity along -Y
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

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

    sys.WriteSystemMatrices(true, true, true, true, (out_dir + "/sys_assembly_dump").c_str());

    // sys.RemoveRedundantConstraints(true,1e-9,true);
    // sys.ShowHierarchy(std::cout);

    // Do dynamics simulation
    {
        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            // hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
            // hht_stepper->SetRelTolerance(1e-9);
            // hht_stepper->SetAbsTolerances(1e-16);
            // hht_stepper->SetAlpha(-0.2);
            //  hht_stepper->SetModifiedNewton(true);
            // hht_stepper->SetMaxiters(50);
        }

        double omega = 4.0;
        double T = 15.0;

        int Nframes = (int)(time_length / time_step);
        int frame = 0;

        mdistance.resize(Nframes, 4);
        mdistance.setZero();
        while (frame < Nframes) {
            double tao = sys.GetChTime() / T;
            double rot_angle = 0;
            if (tao < 1.0)
                rot_angle = omega * T * (tao * tao / 2.0 + (cos(CH_2PI * tao) - 1.0) / pow(CH_2PI, 2.0));
            else
                rot_angle = omega * T * (tao - 0.5);

            driving_fun_B->SetSetpoint(rot_angle, sys.GetChTime());

            sys.DoStepDynamics(time_step);

            ChFrameMoving<> relative_frame = my_body_A->TransformParentToLocal(my_node_R_B->Frame());

            mdistance(frame, 0) = sys.GetChTime();
            mdistance(frame, 1) = relative_frame.GetPos().x();
            mdistance(frame, 2) = relative_frame.GetPos().y();
            mdistance(frame, 3) = relative_frame.GetPos().z();

            if (frame % 100 == 0) {
                std::cout << "t: " << sys.GetChTime() << "\t";
                std::cout << "Distance of two tips:\t" << relative_frame.GetPos().x() << "\t"
                         << relative_frame.GetPos().y() << "\t" << relative_frame.GetPos().z() << "\n";
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

    // Build the substructures A and B in the parent system directly
    ChMatrixDynamic<> distance_ref;
    MakeAndRunDemo_SlewingBeam(false, false, distance_ref);

    // Put the substructures A and B in two seperated instances of ChAssembly()
    ChMatrixDynamic<> distance_2;
    MakeAndRunDemo_SlewingBeam(true, true, distance_2);

    // Compare the results
    assert(distance_ref.rows() == distance_2.rows());
    ChMatrixDynamic<> distance_diff;
    distance_diff.resize(distance_ref.rows(), 4);
    distance_diff.setZero();
    distance_diff = distance_2 - distance_ref;
    std::cout << "The norm of the difference of tip distances is:\t" << distance_diff.norm() << "\n";
    assert(distance_diff.norm() < 1e-5 && "The results are different, implying something is wrong in ChAssembly()");

    // Output results
    ChMatrixDynamic<> distance_cmp;
    distance_cmp.resize(distance_ref.rows(), distance_ref.cols() + distance_2.cols() + distance_diff.cols());
    distance_cmp.setZero();
    distance_cmp << distance_ref, distance_2, distance_diff;

    if (create_directory(path(out_dir))) {
        std::ofstream file_distance((out_dir + "/tip_distance_comparison.dat").c_str());
        file_distance<< std::setprecision(12) << std::scientific;
        StreamOut(distance_cmp, file_distance);
    } else {
        std::cout << "  ...Error creating subdirectories\n";
    }

    return 0;
}
