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
// Authors: Mike Taylor and Radu Serban
// =============================================================================
//
// Unit test for the formulation of the fully parameterized 4 node ANCF shell element 3443.
//
// The following items are check against textbook analytical formulas for small displacement beams
// - Tip displacement with a small axial tip load
// - Tip displacement with a small vertical tip load (cantilever beam)
// - Tip displacement with a gravity load (cantilever beam)
// - Tip angle of twist with a small torque about the beam axis
//
// The multilayer formulation is checked against the solution given in Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics
// of a large scale rigid–flexible multibody system composed of composite laminated plates." Multibody System Dynamics
// 26, no. 3 (2011): 283-305.
//
// =============================================================================
// Checks To be added later:
// =============================================================================
// - Mass Matrix (as returned from the Jacobian calculation)
// - Generalized force vector due to gravity which is a constant vector if gravity is a constant everywhere.  This is
//   the assumption with the implemented element formulation.
// - Generalized internal force vector under several conditions (aids in debugging)
// - Contribution to the Jacobian from the partial derivatives with respect to the nodal coordinates under several
//   conditions (aids in debugging)
// - Contribution to the Jacobian from the partial derivatives with respect to the time derivative of the nodal
//   coordinates under several conditions (aids in debugging)
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/fea/ChElementShellANCF_3423.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChLoadsBeam.h"

#if defined WIN32 || defined _WIN32 || defined WIN64 || defined _WIN64
    #include <windows.h>
#endif

using namespace chrono;
using namespace chrono::fea;

#define TIP_FORCE 1.0   // N
#define TIP_MOMENT 1.0  // Nm
#define Jac_Error 0.33  // Maximum allowed Jacobian percent error as decimal

// =============================================================================

static const std::string ref_dir = "../data/testing/fea/";

// =============================================================================

void print_green(std::string text) {
    std::cout << "\033[1;32m" << text << "\033[0m";
}

void print_red(std::string text) {
    std::cout << "\033[1;31m" << text << "\033[0m";
}

bool load_validation_data(const std::string& filename, ChMatrixDynamic<>& data) {
    std::ifstream file(ref_dir + filename);
    if (!file.is_open()) {
        print_red("ERROR!  Cannot open file: " + ref_dir + filename + "\n");
        return false;
    }
    for (unsigned int r = 0; r < data.rows(); r++) {
        for (unsigned int c = 0; c < data.cols(); c++) {
            file >> data(r, c);
        }
    }
    file.close();
    return true;
}

// =============================================================================

bool AxialDisplacementCheck(int msglvl) {
    // =============================================================================
    //  Check the axial displacement of a beam compared to the analytical result
    //  (some small error is expected based on assumptions and boundary conditions)
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 since this is a statics test against an analytical solution
    system->Set_G_acc(ChVector<>(0, 0, 0));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties - Dimensions and material from the Princeton Beam Experiment Addendum
    int num_elements = 20;
    double length = 20 * 0.0254;     // in->m
    double width = 0.5 * 0.0254;     // in->m
    double height = 0.125 * 0.0254;  // in->m
    // Aluminum 7075-T651 Material Properties
    double rho = 2810;  // kg/m^3
    double E = 71.7e9;  // Pa
    double nu = 0.33;

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    double dx = length / (num_elements);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    // Create the first nodes and fix them completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, -width / 2.0, 0.0), dir1);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);
    auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, width / 2.0, 0.0), dir1);
    nodeD->SetFixed(true);
    mesh->AddNode(nodeD);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, -width / 2.0, 0), dir1);
        mesh->AddNode(nodeB);
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, width / 2.0, 0), dir1);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD);
        element->SetDimensions(dx, width);
        element->AddLayer(height, 0 * CH_C_DEG_TO_RAD, material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        nodeD = nodeC;
        elementlast = element;
    }

    nodeEndPoint = nodeA;

    mesh->SetAutomaticGravity(
        false);  // Turn off the gravity at the mesh level since it is not applied in this test.  This step is not
                 // required since the acceleration due to gravity was already set to all zeros.

    // Create a custom atomic (point) load
    class MyLoaderTimeDependentTipLoad : public ChLoaderUVatomic {
      public:
        // Useful: a constructor that also sets ChLoadable
        MyLoaderTimeDependentTipLoad(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUVatomic(mloadable) {}

        // Compute F=F(u), the load at U. The load is a 6-row vector, i.e.
        // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(
            const double U,              ///< normalized position in the shell mid-plane u direction [-1...1]
            const double V,              ///< normalized position in the shell mid-plane v direction [-1...1]
            ChVectorDynamic<>& F,        ///< Load at UV
            ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
            ) override {
            assert(auxsystem);

            F.setZero();
            F(0) = TIP_FORCE;  // Apply the force along the global X axis (beam axis)
        }

      public:
        // add auxiliary data to the class, if you need to access it during ComputeF().
        ChSystem* auxsystem;
    };

    // Create the load container and to the current system
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system->Add(loadcontainer);

    // Create a custom load that uses the custom loader above.
    // The ChLoad is a 'manager' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<my_loader_class>()

    std::shared_ptr<ChLoad<MyLoaderTimeDependentTipLoad>> mload(new ChLoad<MyLoaderTimeDependentTipLoad>(elementlast));
    mload->loader.auxsystem = system;        // initialize auxiliary data of the loader, if needed
    mload->loader.SetApplication(1.0, 0.0);  // specify application point
    loadcontainer->Add(mload);               // add the load to the load container.

    // Find the static solution for the system (final axial displacement)
    system->DoStaticLinear();

    // Calculate the axial displacement of the end of the ANCF shell mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 0, point, rot);

    // For Analytical Formula, see a mechanics of materials textbook (delta = (P*L)/(A*E))
    double Displacement_Theory = (TIP_FORCE * length) / (width * height * E);
    double Displacement_Model = point.x() - length;
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Theory) / Displacement_Theory * 100;

    bool passed_displacement = abs(Percent_Error) < 2.0;
    bool passed_angles = (abs(Tip_Angles.x() * CH_C_RAD_TO_DEG) < 0.001) &&
                         (abs(Tip_Angles.y() * CH_C_RAD_TO_DEG) < 0.001) &&
                         (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.001);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Axial Pull Test - ANCF Tip Position: " << point << "m" << std::endl;
        std::cout << "Axial Pull Test - ANCF Tip Displacement: " << Displacement_Model << "m" << std::endl;
        std::cout << "Axial Pull Test - Analytical Tip Displacement: " << Displacement_Theory << "m" << std::endl;
        std::cout << "Axial Pull Test - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Axial Pull Test - Tip Displacement Check (Percent Error less than 2%) = " << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout << "Axial Pull Test - Angular misalignment Checks (all angles less than 0.001 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool CantileverTipLoadCheck(int msglvl) {
    // =============================================================================
    //  Check the vertical tip displacement of a cantilever beam with a tip load compared to the analytical result
    //  (some error is expected since this element is prone to Poisson locking)
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 since this is a statics test against an analytical solution
    system->Set_G_acc(ChVector<>(0, 0, 0));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties - Dimensions and material from the Princeton Beam Experiment Addendum
    int num_elements = 20;
    double length = 20 * 0.0254;     // in->m
    double width = 0.5 * 0.0254;     // in->m
    double height = 0.125 * 0.0254;  // in->m
    // Aluminum 7075-T651 Material Properties
    double rho = 2810;  // kg/m^3
    double E = 71.7e9;  // Pa
    double nu = 0.33;

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    double dx = length / (num_elements);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    // Create the first nodes and fix them completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, -width / 2.0, 0.0), dir1);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);
    auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, width / 2.0, 0.0), dir1);
    nodeD->SetFixed(true);
    mesh->AddNode(nodeD);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, -width / 2.0, 0), dir1);
        mesh->AddNode(nodeB);
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, width / 2.0, 0), dir1);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD);
        element->SetDimensions(dx, width);
        element->AddLayer(height, 0 * CH_C_DEG_TO_RAD, material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        nodeD = nodeC;
        elementlast = element;
    }

    nodeEndPoint = nodeA;

    mesh->SetAutomaticGravity(
        false);  // Turn off the gravity at the mesh level since it is not applied in this test.  This step is not
                 // required since the acceleration due to gravity was already set to all zeros.

    // Create a custom atomic (point) load
    class MyLoaderTimeDependentTipLoad : public ChLoaderUVatomic {
      public:
        // Useful: a constructor that also sets ChLoadable
        MyLoaderTimeDependentTipLoad(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUVatomic(mloadable) {}

        // Compute F=F(u), the load at U. The load is a 6-row vector, i.e.
        // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(
            const double U,              ///< normalized position in the shell mid-plane u direction [-1...1]
            const double V,              ///< normalized position in the shell mid-plane v direction [-1...1]
            ChVectorDynamic<>& F,        ///< Load at UV
            ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
            ) override {
            assert(auxsystem);

            F.setZero();
            F(2) = TIP_FORCE;  // Apply the force along the global Z axis (beam axis)
        }

      public:
        // add auxiliary data to the class, if you need to access it during ComputeF().
        ChSystem* auxsystem;
    };

    // Create the load container and to the current system
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system->Add(loadcontainer);

    // Create a custom load that uses the custom loader above.
    // The ChLoad is a 'manager' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<my_loader_class>()

    std::shared_ptr<ChLoad<MyLoaderTimeDependentTipLoad>> mload(new ChLoad<MyLoaderTimeDependentTipLoad>(elementlast));
    mload->loader.auxsystem = system;        // initialize auxiliary data of the loader, if needed
    mload->loader.SetApplication(1.0, 0.0);  // specify application point
    loadcontainer->Add(mload);               // add the load to the load container.

    // Find the static solution for the system (final displacement)
    system->DoStaticLinear();

    // Calculate the displacement of the end of the ANCF beam mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 0, point, rot);

    // For Analytical Formula, see a mechanics of materials textbook (delta = (P*L^3)/(3*E*I))
    double I = 1.0 / 12.0 * width * std::pow(height, 3);
    double Displacement_Theory = (TIP_FORCE * std::pow(length, 3)) / (3.0 * E * I);
    double Displacement_Model = point.z();
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Theory) / Displacement_Theory * 100.0;

    // This element is prone to Poisson locking.  If Poisson's Ratio is set to zero, then the error should be about 0.2%
    // instead of 12%
    bool passed_displacement = abs(Percent_Error) < 15;
    // check the off-axis angles which should be zeros
    bool passed_angles =
        (abs(Tip_Angles.x() * CH_C_RAD_TO_DEG) < 0.001) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.001);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Cantilever Beam (Tip Load) - ANCF Tip Position: " << point << "m" << std::endl;
        std::cout << "Cantilever Beam (Tip Load) - ANCF Tip Displacement: " << Displacement_Model << "m" << std::endl;
        std::cout << "Cantilever Beam (Tip Load) - Analytical Tip Displacement: " << Displacement_Theory << "m"
                  << std::endl;
        std::cout << "Cantilever Beam (Tip Load) - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Cantilever Beam (Tip Load) - Tip Displacement Check (Percent Error less than 15%) = "
                  << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout
            << "Cantilever Beam (Tip Load) - Off-axis Angular misalignment Checks (all angles less than 0.001 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool CantileverGravityCheck(int msglvl) {
    // =============================================================================
    //  Check the vertical tip displacement of a cantilever beam with a uniform gravity load compared to the analytical
    //  result (some small error is expected based on assumptions and boundary conditions)
    // =============================================================================

    auto system = new ChSystemSMC();
    double g = -9.80665;
    system->Set_G_acc(ChVector<>(0, 0, g));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties - Dimensions and material from the Princeton Beam Experiment Addendum
    int num_elements = 20;
    double length = 20 * 0.0254;     // in->m
    double width = 0.5 * 0.0254;     // in->m
    double height = 0.125 * 0.0254;  // in->m
    // Aluminum 7075-T651 Material Properties
    double rho = 2810;  // kg/m^3
    double E = 71.7e9;  // Pa
    double nu = 0.33;

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    double dx = length / (num_elements);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    // Create the first nodes and fix them completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, -width / 2.0, 0.0), dir1);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);
    auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, width / 2.0, 0.0), dir1);
    nodeD->SetFixed(true);
    mesh->AddNode(nodeD);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, -width / 2.0, 0), dir1);
        mesh->AddNode(nodeB);
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, width / 2.0, 0), dir1);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD);
        element->SetDimensions(dx, width);
        element->AddLayer(height, 0 * CH_C_DEG_TO_RAD, material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        nodeD = nodeC;
        elementlast = element;
    }

    nodeEndPoint = nodeA;

    // Find the static solution for the system (final displacement)
    system->DoStaticLinear();

    // Calculate the displacement of the end of the ANCF beam mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 0, point, rot);

    // For Analytical Formula, see a mechanics of materials textbook (delta = (q*L^4)/(8*E*I))
    double I = 1.0 / 12.0 * width * std::pow(height, 3);
    double Displacement_Theory = (rho * width * height * g * std::pow(length, 4)) / (8.0 * E * I);
    double Displacement_Model = point.z();
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Theory) / Displacement_Theory * 100.0;

    // This element is prone to Poisson locking.  If Poisson's Ratio is set to zero, then the error should be about 0.2%
    // instead of 12%
    bool passed_displacement = abs(Percent_Error) < 15;
    // check the off-axis angles which should be zeros
    bool passed_angles =
        (abs(Tip_Angles.x() * CH_C_RAD_TO_DEG) < 0.001) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.001);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Cantilever Beam (Gravity Load) - ANCF Tip Position: " << point << "m" << std::endl;
        std::cout << "Cantilever Beam (Gravity Load) - ANCF Tip Displacement: " << Displacement_Model << "m"
                  << std::endl;
        std::cout << "Cantilever Beam (Gravity Load) - Analytical Tip Displacement: " << Displacement_Theory << "m"
                  << std::endl;
        std::cout << "Cantilever Beam (Gravity Load) - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Cantilever Beam (Gravity Load) - Tip Displacement Check (Percent Error less than 15%) = "
                  << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout
            << "Cantilever Beam (Gravity Load) - Off-axis Angular misalignment Checks (all angles less than 0.001 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool AxialTwistCheck(int msglvl) {
    // =============================================================================
    //  Check the axial twist angle of a beam compared to the analytical result with an applied torque about the beam
    //  axis (some error is expected due to the shapes this element can assume)
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 since this is a statics test against an analytical solution
    system->Set_G_acc(ChVector<>(0, 0, 0));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties - Dimensions and material from the Princeton Beam Experiment Addendum
    int num_elements = 20;
    double length = 20 * 0.0254;     // in->m
    double width = 0.5 * 0.0254;     // in->m
    double height = 0.125 * 0.0254;  // in->m
    // Aluminum 7075-T651 Material Properties
    double rho = 2810;  // kg/m^3
    double E = 71.7e9;  // Pa
    double nu = 0.33;
    double G = E / (2 * (1 + nu));

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    double dx = length / (num_elements);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    // Create the first nodes and fix them completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, -width / 2.0, 0.0), dir1);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);
    auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, width / 2.0, 0.0), dir1);
    nodeD->SetFixed(true);
    mesh->AddNode(nodeD);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, -width / 2.0, 0), dir1);
        mesh->AddNode(nodeB);
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, width / 2.0, 0), dir1);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD);
        element->SetDimensions(dx, width);
        element->AddLayer(height, 0 * CH_C_DEG_TO_RAD, material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        nodeD = nodeC;
        elementlast = element;
    }

    nodeEndPoint = nodeA;

    mesh->SetAutomaticGravity(
        false);  // Turn off the gravity at the mesh level since it is not applied in this test.  This step is not
                 // required since the acceleration due to gravity was already set to all zeros.

    // Create a custom atomic (point) load
    class MyLoaderTimeDependentTipLoad : public ChLoaderUVatomic {
      public:
        // Useful: a constructor that also sets ChLoadable
        MyLoaderTimeDependentTipLoad(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUVatomic(mloadable) {}

        // Compute F=F(u), the load at U. The load is a 6-row vector, i.e.
        // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(
            const double U,              ///< normalized position in the shell mid-plane u direction [-1...1]
            const double V,              ///< normalized position in the shell mid-plane v direction [-1...1]
            ChVectorDynamic<>& F,        ///< Load at UV
            ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
            ) override {
            assert(auxsystem);

            F.setZero();
            F(3) = TIP_MOMENT;  // Apply the moment about the global X axis
        }

      public:
        // add auxiliary data to the class, if you need to access it during ComputeF().
        ChSystem* auxsystem;
    };

    // Create the load container and to the current system
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system->Add(loadcontainer);

    // Create a custom load that uses the custom loader above.
    // The ChLoad is a 'manager' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<my_loader_class>()

    std::shared_ptr<ChLoad<MyLoaderTimeDependentTipLoad>> mload(new ChLoad<MyLoaderTimeDependentTipLoad>(elementlast));
    mload->loader.auxsystem = system;        // initialize auxiliary data of the loader, if needed
    mload->loader.SetApplication(1.0, 0.0);  // specify application point
    loadcontainer->Add(mload);               // add the load to the load container.

    // Find the static solution for the system (final twist angle)
    system->DoStaticLinear();

    // Calculate the twist angle of the end of the ANCF beam mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 0, point, rot);
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    // For Analytical Formula, see: https://en.wikipedia.org/wiki/Torsion_constant
    double J = 0.281 * width * std::pow(height, 3);
    double Angle_Theory = TIP_MOMENT * length / (G * J);

    double Percent_Error = (Tip_Angles.x() - Angle_Theory) / Angle_Theory * 100;

    bool passed_twist = abs(Percent_Error) < 15;
    // check the off-axis angles which should be zeros
    bool passed_angles =
        (abs(Tip_Angles.y() * CH_C_RAD_TO_DEG) < 0.001) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.001);
    bool passed_tests = passed_twist && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Axial Twist - ANCF Tip Position: " << point << "m" << std::endl;
        std::cout << "Axial Twist - ANCF Twist Angles (Euler 123): " << Tip_Angles * CH_C_RAD_TO_DEG << "deg"
                  << std::endl;
        std::cout << "Axial Twist - Analytical Twist Angle: " << Angle_Theory * CH_C_RAD_TO_DEG << "deg" << std::endl;
        std::cout << "Axial Twist - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Axial Twist - Twist Angle Check (Percent Error less than 15%) = " << Percent_Error << "%";
        if (passed_twist)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout << "Axial Twist - Off-axis Angular misalignment Checks (all angles less than 0.001 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool MLCantileverCheck1A(int msglvl) {
    // =============================================================================
    //  Check the Displacement of a Composite Layup
    //  Test Problem From Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody
    //  system composed of composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.
    //  Layup 1 - All 4 Orthotropic Layers Aligned - 1st Load Case
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 to match the reference solution
    system->Set_G_acc(ChVector<>(0, 0, -9810));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties
    int num_elements_x = 32;
    int num_elements_y = 1;
    double length = 500;            // mm
    double width = 500;             // mm
    double layer_thickness = 0.25;  // mm

    double rho = 7.8e-9;                  // kg/mm^3
    ChVector<> E(177e3, 10.8e3, 10.8e3);  // MPa
    ChVector<> nu(0, 0, 0);
    ChVector<> G(7.6e3, 7.6e3, 8.504e3);  // MPa

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed plate
    double dx = length / (num_elements_x);
    double dy = width / (num_elements_y);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    // Create and add the nodes
    for (auto i = 0; i <= num_elements_x; i++) {
        for (auto j = 0; j <= num_elements_y; j++) {
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, dy * j, 0.0), dir1);
            mesh->AddNode(node);

            // Fix only the nodes along x=0
            if (i == 0) {
                node->SetFixed(true);
            }

            nodeEndPoint = node;
        }
    }

    // Create and add the elements
    for (auto i = 0; i < num_elements_x; i++) {
        for (auto j = 0; j < num_elements_y; j++) {
            int nodeA_idx = j + i * (num_elements_y + 1);
            int nodeD_idx = (j + 1) + i * (num_elements_y + 1);
            int nodeB_idx = j + (i + 1) * (num_elements_y + 1);
            int nodeC_idx = (j + 1) + (i + 1) * (num_elements_y + 1);

            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeA_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeD_idx)));
            element->SetDimensions(dx, dy);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->SetAlphaDamp(0.0);
            mesh->AddElement(element);

            elementlast = element;
        }
    }

    // Find the nonlinear static solution for the system (final displacement)
    system->DoStaticNonlinear(50);

    // Calculate the displacement of the corner of the ANCF shell mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 1, point, rot);

    // Expect Value From Liu et al. ABAQUS Model
    double Displacement_Expected = -40.3;
    double Displacement_Model = point.z();
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Expected) / Displacement_Expected * 100.0;

    bool passed_displacement = abs(Percent_Error) < 5.0;
    bool passed_angles =
        (abs(Tip_Angles.x() * CH_C_RAD_TO_DEG) < 0.01) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.01);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Multilayer Plate Layup 1A - ANCF Tip Position: " << point << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 1A - ANCF Tip Displacement: " << Displacement_Model << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 1A - Expected Tip Displacement: " << Displacement_Expected << "mm"
                  << std::endl;
        std::cout << "Multilayer Plate Layup 1A - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Multilayer Plate Layup 1A - Tip Displacement Check (Percent Error less than 5%) = "
                  << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout << "Multilayer Plate Layup 1A - Angular misalignment Checks (all angles less than 0.01 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool MLCantileverCheck1B(int msglvl) {
    // =============================================================================
    //  Check the Displacement of a Composite Layup
    //  Test Problem From Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody
    //  system composed of composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.
    //  Layup 1 - All 4 Orthotropic Layers Aligned - 2nd Load Case
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 to match the reference solution
    system->Set_G_acc(ChVector<>(0, 0, -9810));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties
    int num_elements_x = 1;
    int num_elements_y = 32;
    double length = 500;            // mm
    double width = 500;             // mm
    double layer_thickness = 0.25;  // mm

    double rho = 7.8e-9;                  // kg/mm^3
    ChVector<> E(177e3, 10.8e3, 10.8e3);  // MPa
    ChVector<> nu(0, 0, 0);
    ChVector<> G(7.6e3, 7.6e3, 8.504e3);  // MPa

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed plate
    double dx = length / (num_elements_x);
    double dy = width / (num_elements_y);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    // Create and add the nodes
    for (auto i = 0; i <= num_elements_x; i++) {
        for (auto j = 0; j <= num_elements_y; j++) {
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, dy * j, 0.0), dir1);
            mesh->AddNode(node);

            // Fix only the nodes along y=0
            if (j == 0) {
                node->SetFixed(true);
            }

            nodeEndPoint = node;
        }
    }

    // Create and add the elements
    for (auto i = 0; i < num_elements_x; i++) {
        for (auto j = 0; j < num_elements_y; j++) {
            int nodeA_idx = j + i * (num_elements_y + 1);
            int nodeD_idx = (j + 1) + i * (num_elements_y + 1);
            int nodeB_idx = j + (i + 1) * (num_elements_y + 1);
            int nodeC_idx = (j + 1) + (i + 1) * (num_elements_y + 1);

            // std::cout << "A:" << nodeA_idx << "  B:" << nodeB_idx << "  C:" << nodeC_idx << "  D:" << nodeD_idx <<
            // std::endl;

            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeA_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeD_idx)));
            element->SetDimensions(dx, dy);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->SetAlphaDamp(0.0);
            mesh->AddElement(element);

            elementlast = element;
        }
    }

    // Find the nonlinear static solution for the system (final displacement)
    system->DoStaticNonlinear(50);

    // Calculate the displacement of the corner of the ANCF shell mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 1, point, rot);

    // Expect Value From Liu et al. ABAQUS Model
    double Displacement_Expected = -357.5;
    double Displacement_Model = point.z();
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Expected) / Displacement_Expected * 100.0;

    bool passed_displacement = abs(Percent_Error) < 5.0;
    bool passed_angles =
        (abs(Tip_Angles.y() * CH_C_RAD_TO_DEG) < 0.01) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.01);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Multilayer Plate Layup 1B - ANCF Tip Position: " << point << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 1B - ANCF Tip Displacement: " << Displacement_Model << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 1B - Expected Tip Displacement: " << Displacement_Expected << "mm"
                  << std::endl;
        std::cout << "Multilayer Plate Layup 1B - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Multilayer Plate Layup 1B - Tip Displacement Check (Percent Error less than 5%) = "
                  << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout << "Multilayer Plate Layup 1B - Angular misalignment Checks (all angles less than 0.01 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool MLCantileverCheck2A(int msglvl) {
    // =============================================================================
    //  Check the Displacement of a Composite Layup
    //  Test Problem From Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody
    //  system composed of composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.
    //  Layup 2 - Top and Bottom Orthotropic Layers Aligned, Middle Aligned at 90deg - 1st Load Case
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 to match the reference solution
    system->Set_G_acc(ChVector<>(0, 0, -9810));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties
    int num_elements_x = 32;
    int num_elements_y = 1;
    double length = 500;            // mm
    double width = 500;             // mm
    double layer_thickness = 0.25;  // mm

    double rho = 7.8e-9;                  // kg/mm^3
    ChVector<> E(177e3, 10.8e3, 10.8e3);  // MPa
    ChVector<> nu(0, 0, 0);
    ChVector<> G(7.6e3, 7.6e3, 8.504e3);  // MPa

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed plate
    double dx = length / (num_elements_x);
    double dy = width / (num_elements_y);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    // Create and add the nodes
    for (auto i = 0; i <= num_elements_x; i++) {
        for (auto j = 0; j <= num_elements_y; j++) {
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, dy * j, 0.0), dir1);
            mesh->AddNode(node);

            // Fix only the nodes along x=0
            if (i == 0) {
                node->SetFixed(true);
            }

            nodeEndPoint = node;
        }
    }

    // Create and add the elements
    for (auto i = 0; i < num_elements_x; i++) {
        for (auto j = 0; j < num_elements_y; j++) {
            int nodeA_idx = j + i * (num_elements_y + 1);
            int nodeD_idx = (j + 1) + i * (num_elements_y + 1);
            int nodeB_idx = j + (i + 1) * (num_elements_y + 1);
            int nodeC_idx = (j + 1) + (i + 1) * (num_elements_y + 1);

            // std::cout << "A:" << nodeA_idx << "  B:" << nodeB_idx << "  C:" << nodeC_idx << "  D:" << nodeD_idx <<
            // std::endl;

            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeA_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeD_idx)));
            element->SetDimensions(dx, dy);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 90 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 90 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->SetAlphaDamp(0.0);
            mesh->AddElement(element);

            elementlast = element;
        }
    }

    // Find the nonlinear static solution for the system (final displacement)
    system->DoStaticNonlinear(50);

    // Calculate the displacement of the end of the ANCF beam mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 1, point, rot);

    // Expect Value From Liu et al. ABAQUS Model
    double Displacement_Expected = -45.6;
    double Displacement_Model = point.z();
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Expected) / Displacement_Expected * 100.0;

    bool passed_displacement = abs(Percent_Error) < 5.0;
    bool passed_angles =
        (abs(Tip_Angles.x() * CH_C_RAD_TO_DEG) < 0.01) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.01);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Multilayer Plate Layup 2A - ANCF Tip Position: " << point << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 2A - ANCF Tip Displacement: " << Displacement_Model << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 2A - Expected Tip Displacement: " << Displacement_Expected << "mm"
                  << std::endl;
        std::cout << "Multilayer Plate Layup 2A - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Multilayer Plate Layup 2A - Tip Displacement Check (Percent Error less than 5%) = "
                  << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout << "Multilayer Plate Layup 2A - Angular misalignment Checks (all angles less than 0.01 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool MLCantileverCheck2B(int msglvl) {
    // =============================================================================
    //  Check the Displacement of a Composite Layup
    //  Test Problem From Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody
    //  system composed of composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.
    //  Layup 2 - Top and Bottom Orthotropic Layers Aligned, Middle Aligned at 90deg - 2nd Load Case
    // =============================================================================

    auto system = new ChSystemSMC();
    // Set gravity to 0 to match the reference solution
    system->Set_G_acc(ChVector<>(0, 0, -9810));

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    system->SetSolver(solver);

    // Set up integrator
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties
    int num_elements_x = 1;
    int num_elements_y = 32;
    double length = 500;            // mm
    double width = 500;             // mm
    double layer_thickness = 0.25;  // mm

    double rho = 7.8e-9;                  // kg/mm^3
    ChVector<> E(177e3, 10.8e3, 10.8e3);  // MPa
    ChVector<> nu(0, 0, 0);
    ChVector<> G(7.6e3, 7.6e3, 8.504e3);  // MPa

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    system->Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed plate
    double dx = length / (num_elements_x);
    double dy = width / (num_elements_y);

    // Setup shell position vector gradient to initially align with the global z direction
    ChVector<> dir1(0, 0, 1);

    auto elementlast = chrono_types::make_shared<ChElementShellANCF_3423>();
    std::shared_ptr<ChNodeFEAxyzD> nodeEndPoint;

    // Create and add the nodes
    for (auto i = 0; i <= num_elements_x; i++) {
        for (auto j = 0; j <= num_elements_y; j++) {
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(dx * i, dy * j, 0.0), dir1);
            mesh->AddNode(node);

            // Fix only the nodes along y=0
            if (j == 0) {
                node->SetFixed(true);
            }

            nodeEndPoint = node;
        }
    }

    // Create and add the elements
    for (auto i = 0; i < num_elements_x; i++) {
        for (auto j = 0; j < num_elements_y; j++) {
            int nodeA_idx = j + i * (num_elements_y + 1);
            int nodeD_idx = (j + 1) + i * (num_elements_y + 1);
            int nodeB_idx = j + (i + 1) * (num_elements_y + 1);
            int nodeC_idx = (j + 1) + (i + 1) * (num_elements_y + 1);

            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeA_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeD_idx)));
            element->SetDimensions(dx, dy);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 90 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 90 * CH_C_DEG_TO_RAD, material);
            element->AddLayer(layer_thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->SetAlphaDamp(0.0);
            mesh->AddElement(element);

            elementlast = element;
        }
    }

    // Find the nonlinear static solution for the system (final displacement)
    system->DoStaticNonlinear(50);

    // Calculate the displacement of the end of the ANCF beam mesh
    ChVector<> point;
    ChQuaternion<> rot;
    elementlast->EvaluateSectionFrame(1, 1, point, rot);

    // Expect Value From Liu et al. ABAQUS Model
    double Displacement_Expected = -198.0;
    double Displacement_Model = point.z();
    ChVector<> Tip_Angles = rot.Q_to_Euler123();

    double Percent_Error = (Displacement_Model - Displacement_Expected) / Displacement_Expected * 100.0;

    bool passed_displacement = abs(Percent_Error) < 5.0;
    bool passed_angles =
        (abs(Tip_Angles.y() * CH_C_RAD_TO_DEG) < 0.01) && (abs(Tip_Angles.z() * CH_C_RAD_TO_DEG) < 0.01);
    bool passed_tests = passed_displacement && passed_angles;

    if (msglvl >= 2) {
        std::cout << "Multilayer Plate Layup 2B - ANCF Tip Position: " << point << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 2B - ANCF Tip Displacement: " << Displacement_Model << "mm" << std::endl;
        std::cout << "Multilayer Plate Layup 2B - Expected Tip Displacement: " << Displacement_Expected << "mm"
                  << std::endl;
        std::cout << "Multilayer Plate Layup 2B - ANCF Tip Angles: (" << Tip_Angles.x() * CH_C_RAD_TO_DEG << ", "
                  << Tip_Angles.y() * CH_C_RAD_TO_DEG << ", " << Tip_Angles.z() * CH_C_RAD_TO_DEG << ")deg"
                  << std::endl;
    }
    if (msglvl >= 1) {
        std::cout << "Multilayer Plate Layup 2B - Tip Displacement Check (Percent Error less than 5%) = "
                  << Percent_Error << "%";
        if (passed_displacement)
            print_green(" - Test PASSED\n");
        else
            print_red(" - Test FAILED\n");

        std::cout << "Multilayer Plate Layup 2B - Angular misalignment Checks (all angles less than 0.01 deg)";
        if (passed_angles)
            print_green(" - Test PASSED\n\n");
        else
            print_red(" - Test FAILED\n\n");
    }

    return (passed_tests);
}

bool RunElementChecks(int msglvl) {
    bool tests_passed = true;

    tests_passed = (tests_passed && AxialDisplacementCheck(msglvl));
    tests_passed = (tests_passed && CantileverTipLoadCheck(msglvl));
    tests_passed = (tests_passed && CantileverGravityCheck(msglvl));
    tests_passed = (tests_passed && AxialTwistCheck(msglvl));

    tests_passed = (tests_passed && MLCantileverCheck1A(msglvl));
    tests_passed = (tests_passed && MLCantileverCheck1B(msglvl));
    tests_passed = (tests_passed && MLCantileverCheck2A(msglvl));
    tests_passed = (tests_passed && MLCantileverCheck2B(msglvl));

    return (tests_passed);
}

int main(int argc, char* argv[]) {
#if defined WIN32 || defined _WIN32 || defined WIN64 || defined _WIN64
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hOut, &dwMode);
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);

    // References:
    // SetConsoleMode() and ENABLE_VIRTUAL_TERMINAL_PROCESSING?
    // https://stackoverflow.com/questions/38772468/setconsolemode-and-enable-virtual-terminal-processing

    // Windows console with ANSI colors handling
    // https://superuser.com/questions/413073/windows-console-with-ansi-colors-handling
#endif

    bool tests_passed = true;

    std::cout << "-------------------------------------" << std::endl;
    if (RunElementChecks(1))
        print_green("ChElementShellANCF_3423 Element Checks = PASSED\n");
    else {
        print_red("ChElementShellANCF_3423 Element Checks = FAILED\n");
        tests_passed = false;
    }

    return !tests_passed;
}
