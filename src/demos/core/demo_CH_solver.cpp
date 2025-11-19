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

#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChMatrix.h"

#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintTwoBodies.h"
#include "chrono/solver/ChKRMBlock.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_thirdparty/filesystem/path.h"

#undef EIGEN_DBG_SPARSE
#define EIGEN_DBG_SPARSE(X)

using namespace chrono;

// Test 1
// First example: how to manage the data structures of
// the solver, and how to get results after the solution.
// All systems are made of two types of items: the
//  VARIABLES (unknowns q in the scheme below) and
//  CONSTRAINTS (unknowns reactions 'l' in the scheme).
// As an example, let's solve the following mixed LCP:
//
//  | 10  0  0          .  1  0 | |q_a0|   |  1|   |  0|
//  |  0 10  0          .  2  1 | |q_a1|   |  2|   |  0|
//  |  0  0 10          . -1  0 | |q_a2|   |  0|   |  0|
//  |          20  0  0 .  1  0 | |q_b0|   |  0|   |  0|
//  |           0 20  0 . -2 -1 | |q_b1| - |  0| = |  0|
//  |           0  0 20 .  0  0 | |q_b2|   |  0|   |  0|
//  | ..........................| |....|   |...|   |...|
//  |  1  2 -1  1 -2  0 .       | |-l_1|   |  5|   |c_1|
//  |  0  1  0  0 -1  0 .       | |-l_2|   | -1|   |c_2|

void test_1(const std::string& out_dir) {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: generic system with two constraints\n" << std::endl;

    // Important: create a 'system descriptor' object that
    // contains variables and constraints:

    ChSystemDescriptor mdescriptor;

    // Now let's add variables and constraints, as sparse data:

    mdescriptor.BeginInsertion();  // ----- system description starts here

    // create C++ objects representing 'variables':

    ChVariablesGeneric mvarA(3);
    mvarA.GetMass().setIdentity();
    mvarA.GetMass() *= 10;
    mvarA.GetInvMass() = mvarA.GetMass().inverse();
    mvarA.Force()(0) = 1;
    mvarA.Force()(1) = 2;

    ChVariablesGeneric mvarB(3);
    mvarB.GetMass().setIdentity();
    mvarB.GetMass() *= 20;
    mvarB.GetInvMass() = mvarB.GetMass().inverse();

    mdescriptor.InsertVariables(&mvarA);
    mdescriptor.InsertVariables(&mvarB);

    // create C++ objects representing 'constraints' between variables:

    ChConstraintTwoGeneric mca(&mvarA, &mvarB);
    mca.SetRightHandSide(-5);
    mca.Get_Cq_a()(0) = 1;
    mca.Get_Cq_a()(1) = 2;
    mca.Get_Cq_a()(2) = -1;
    mca.Get_Cq_b()(0) = 1;
    mca.Get_Cq_b()(1) = -2;
    mca.Get_Cq_b()(2) = 0;

    ChConstraintTwoGeneric mcb(&mvarA, &mvarB);
    mcb.SetRightHandSide(1);
    mcb.Get_Cq_a()(0) = 0;
    mcb.Get_Cq_a()(1) = 1;
    mcb.Get_Cq_a()(2) = 0;
    mcb.Get_Cq_b()(0) = 0;
    mcb.Get_Cq_b()(1) = -2;
    mcb.Get_Cq_b()(2) = 0;

    mdescriptor.InsertConstraint(&mca);
    mdescriptor.InsertConstraint(&mcb);

    mdescriptor.EndInsertion();  // ----- system description ends here

    // Solve the problem with an iterative fixed-point solver, for an
    // approximate (but very fast) solution:

    // Create the solver
    ChSolverPSOR solver;
    solver.SetMaxIterations(20);
    solver.EnableWarmStart(false);
    solver.SetTolerance(0.0);
    solver.SetOmega(0.8);

    {
        // Solver information (generic)
        ChSolver* s = &solver;
        std::cout << "Solver Info" << std::endl;
        std::cout << "Type: " << static_cast<std::underlying_type<ChSolver::Type>::type>(s->GetType()) << std::endl;
        std::cout << "Iterative? " << s->IsIterative() << std::endl;
        std::cout << "Direct?    " << s->IsDirect() << std::endl;
        std::cout << "Access as iterative solver: " << s->AsIterative() << std::endl;
        std::cout << "Access as direct solver:    " << s->AsDirect() << std::endl;
    }

    // Pass the constraint and the variables to the solver to solve
    solver.Setup(mdescriptor, true);
    solver.Solve(mdescriptor);

    // Output results
    double max_res, max_LCPerr;
    mdescriptor.ComputeFeasabilityViolation(max_res, max_LCPerr);

    // If needed, dump the full system M and Cq matrices
    // on disk, in Matlab sparse format:
    ChSparseMatrix matrM(mdescriptor.CountActiveVariables(), mdescriptor.CountActiveVariables());
    ChSparseMatrix matrCq(mdescriptor.CountActiveConstraints(), mdescriptor.CountActiveVariables());

    matrM.setZeroValues();
    matrCq.setZeroValues();

    mdescriptor.PasteMassKRMMatrixInto(matrM);
    mdescriptor.PasteConstraintsJacobianMatrixInto(matrCq);

    std::cout << "Matrix M (" << matrM.rows() << "x" << matrM.cols() << ")" << std::endl;
    StreamOut(matrM, std::cout, true);
    std::cout << "Matrix Cq (" << matrCq.rows() << "x" << matrCq.cols() << ")" << std::endl;
    StreamOut(matrCq, std::cout, true);

    std::cout << "**** Using ChSolverPSOR  **********\n" << std::endl;
    std::cout << "METRICS: max residual: " << max_res << "  max LCP error: " << max_LCPerr << "\n" << std::endl;
    std::cout << "vars q_a and q_b -------------------" << std::endl;
    std::cout << mvarA.State() << std::endl;
    std::cout << mvarB.State() << "  " << std::endl;
    std::cout << "multipliers l_1 and l_2 ------------\n" << std::endl;
    std::cout << mca.GetLagrangeMultiplier() << " " << std::endl;
    std::cout << mcb.GetLagrangeMultiplier() << " \n" << std::endl;
    std::cout << "constraint residuals c_1 and c_2 ---" << std::endl;
    std::cout << mca.GetResidual() << " " << std::endl;
    std::cout << mcb.GetResidual() << " \n" << std::endl;

    // reset variables
    mvarA.State().setZero();
    mvarB.State().setZero();
}

// Test 2
// Create a bunch of monodimensional vars and simple
// constraints between them, using 'for' loops, as a benchmark that
// represents, from a physical point of view, a long inverted multipendulum.

void test_2(const std::string& out_dir) {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: 1D vertical pendulum\n" << std::endl;

    ChSystemDescriptor mdescriptor;

    mdescriptor.BeginInsertion();  // ----- system description starts here

    int n_masses = 11;

    std::vector<ChVariablesGeneric*> vars;
    std::vector<ChConstraintTwoGeneric*> constraints;

    for (int im = 0; im < n_masses; im++) {
        vars.push_back(new ChVariablesGeneric(1));
        vars[im]->GetMass()(0) = 10;
        vars[im]->GetInvMass()(0) = 1. / vars[im]->GetMass()(0);
        vars[im]->Force()(0) = -9.8 * vars[im]->GetMass()(0) * 0.01;
        // if (im==5) vars[im]->Force()(0)= 50;
        mdescriptor.InsertVariables(vars[im]);
        if (im > 0) {
            constraints.push_back(new ChConstraintTwoGeneric(vars[im], vars[im - 1]));
            constraints[im - 1]->SetRightHandSide(0);
            constraints[im - 1]->Get_Cq_a()(0) = 1;
            constraints[im - 1]->Get_Cq_b()(0) = -1;
            mdescriptor.InsertConstraint(constraints[im - 1]);
        }
    }

    // First variable of 1st domain is 'fixed' like in a hanging chain
    vars[0]->SetDisabled(true);

    mdescriptor.EndInsertion();  // ----- system description is finished

    // Create the solver...
    ChSolverMINRES solver;
    solver.SetMaxIterations(20);
    solver.SetTolerance(1e-5);
    solver.SetVerbose(true);

    {
        // Solver information (generic)
        ChSolver* s = &solver;
        std::cout << "Solver Info" << std::endl;
        std::cout << "Type: " << static_cast<std::underlying_type<ChSolver::Type>::type>(s->GetType()) << std::endl;
        std::cout << "Iterative? " << s->IsIterative() << std::endl;
        std::cout << "Direct?    " << s->IsDirect() << std::endl;
        std::cout << "Access as iterative solver: " << s->AsIterative() << std::endl;
        std::cout << "Access as direct solver:    " << s->AsDirect() << std::endl;
    }

    // .. pass the constraint and the variables to the solver to solve
    solver.Setup(mdescriptor, true);
    solver.Solve(mdescriptor);

    // Output values
    std::cout << "VARIABLES: " << std::endl;
    for (int im = 0; im < vars.size(); im++)
        std::cout << "   " << vars[im]->State()(0) << std::endl;

    std::cout << "CONSTRAINTS: " << std::endl;
    for (int ic = 0; ic < constraints.size(); ic++)
        std::cout << "   " << constraints[ic]->GetLagrangeMultiplier() << std::endl;
}

// Test 3
// Create three variables, with some mass, and also add a stiffness item
// that connects two of these variables with random stiffness.
// Also use the ChSystemDescriptor functions FromUnknownsToVector
// and FromVectorToUnknowns for doing checks.
//
//  | M+K   K       . Cq' | |q_a |   |f_a|   |  0|
//  |  K   M+K      . Cq' | |q_b |   |f_b|   |  0|
//  |            M  .     | |q_c |   |f_c| = |  0|
//  | ....................| |... |   |...|   |...|
//  |  Cq   Cq      .     | |-l_1|   |  5|   |c_1|

void test_3(const std::string& out_dir) {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: generic system with stiffness blocks\n" << std::endl;

    // Important: create a 'system descriptor' object that
    // contains variables and constraints:

    ChSystemDescriptor mdescriptor;

    // Now let's add variables, constraints and stiffness, as sparse data:

    mdescriptor.BeginInsertion();  // ----- system description

    // Create C++ objects representing 'variables', set their M blocks
    // (the masses) and set their known terms 'fb'

    ChMatrix33<> minertia(6);

    ChVariablesBodyOwnMass mvarA;
    mvarA.SetBodyMass(5);
    mvarA.SetBodyInertia(minertia);
    mvarA.Force().fillRandom(-3, 5);

    ChVariablesBodyOwnMass mvarB;
    mvarB.SetBodyMass(4);
    mvarB.SetBodyInertia(minertia);
    mvarB.Force().fillRandom(1, 3);

    ChVariablesBodyOwnMass mvarC;
    mvarC.SetBodyMass(5.5);
    mvarC.SetBodyInertia(minertia);
    mvarC.Force().fillRandom(-8, 3);

    ////ChMatrixDynamic<> foo(3, 4);
    ////foo.fillRandom(0, 10);
    ////std::cout << "foo....\n"<< foo << std::endl;
    ////return;

    mdescriptor.InsertVariables(&mvarA);
    mdescriptor.InsertVariables(&mvarB);
    mdescriptor.InsertVariables(&mvarC);

    // Create two C++ objects representing 'constraints' between variables
    // and set the jacobian to random values;
    // Also set cfm term (E diagonal = -cfm)

    ChConstraintTwoBodies mca(&mvarA, &mvarB);
    mca.SetRightHandSide(3);
    mca.Get_Cq_a().fillRandom(-1, 1);
    mca.Get_Cq_b().fillRandom(-1, 1);
    mca.SetComplianceTerm(0.2);

    ChConstraintTwoBodies mcb(&mvarA, &mvarB);
    mcb.SetRightHandSide(5);
    mcb.Get_Cq_a().fillRandom(-1, 1);
    mcb.Get_Cq_b().fillRandom(-1, 1);
    mcb.SetComplianceTerm(0.1);

    mdescriptor.InsertConstraint(&mca);
    mdescriptor.InsertConstraint(&mcb);

    // Create two C++ objects representing 'stiffness' between variables:

    ChKRMBlock mKa;
    // set the affected variables (so this K is a 12x12 matrix, relative to 4 6x6 blocks)
    std::vector<ChVariables*> mvarsa;
    mvarsa.push_back(&mvarA);
    mvarsa.push_back(&mvarB);
    mKa.SetVariables(mvarsa);

    // just fill K with random values (but symmetric, by making a product of matr*matrtransposed)
    ChMatrixDynamic<> mtempA = mKa.GetMatrix();
    mtempA.fillRandom(-0.3, 0.3);
    ChMatrixDynamic<> mtempB(mtempA);
    mKa.GetMatrix() = -mtempA * mtempB;

    mdescriptor.InsertKRMBlock(&mKa);

    ChKRMBlock mKb;
    // set the affected variables (so this K is a 12x12 matrix, relative to 4 6x6 blocks)
    std::vector<ChVariables*> mvarsb;
    mvarsb.push_back(&mvarB);
    mvarsb.push_back(&mvarC);
    mKb.SetVariables(mvarsb);

    mKb.GetMatrix() = mKa.GetMatrix();

    mdescriptor.InsertKRMBlock(&mKb);

    mdescriptor.EndInsertion();  // ----- system description ends here

    // Create the solver (MINRES) ...
    ChSolverMINRES solver;
    solver.SetMaxIterations(100);
    solver.SetTolerance(1e-12);
    solver.EnableDiagonalPreconditioner(true);
    solver.SetVerbose(true);

    {
        // Solver information (generic)
        ChSolver* s = &solver;
        std::cout << "Solver Info" << std::endl;
        std::cout << "Type: " << static_cast<std::underlying_type<ChSolver::Type>::type>(s->GetType()) << std::endl;
        std::cout << "Iterative? " << s->IsIterative() << std::endl;
        std::cout << "Direct?    " << s->IsDirect() << std::endl;
        std::cout << "Access as iterative solver: " << s->AsIterative() << std::endl;
        std::cout << "Access as direct solver:    " << s->AsDirect() << std::endl;
    }

    // .. solve the system (passing variables, constraints, stiffness
    //    blocks with the ChSystemDescriptor that we populated above)
    solver.Setup(mdescriptor, true);
    solver.Solve(mdescriptor);

    // .. optional: get the result as a single vector (it collects all q_i and l_i
    //    solved values stored in variables and constraints), just for check.
    chrono::ChVectorDynamic<double> mx;
    mdescriptor.FromUnknownsToVector(mx);  // x ={q,-l}

    /*
    // Alternatively, instead of using FromUnknownsToVector, to fetch
    // result, you could just loop over the variables (q values) and
    // over the constraints (l values), as already shown in previous examples:

    for (int im = 0; im < mdescriptor.GetVariables().size(); im++)
        std::cout << "   " << mdescriptor.GetVariables()[im]->State()(0) << std::endl;

    for (int ic = 0; ic < mdescriptor.GetConstraints().size(); ic++)
        std::cout << "   " << mdescriptor.GetConstraints()[ic]->GetLagrangeMultiplier() << std::endl;
    */
}

// Test 4
// Same as Test 1, but use a direct sparse linear solver.

void test_4(const std::string& out_dir) {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: generic system with two constraints\n" << std::endl;

    ChSystemDescriptor mdescriptor;
    mdescriptor.BeginInsertion();  // ----- system description starts here

    ChVariablesGeneric mvarA(3);
    mvarA.GetMass().setIdentity();
    mvarA.GetMass() *= 10;
    mvarA.GetInvMass() = mvarA.GetMass().inverse();
    mvarA.Force()(0) = 1;
    mvarA.Force()(1) = 2;

    ChVariablesGeneric mvarB(3);
    mvarB.GetMass().setIdentity();
    mvarB.GetMass() *= 20;
    mvarB.GetInvMass() = mvarB.GetMass().inverse();

    mdescriptor.InsertVariables(&mvarA);
    mdescriptor.InsertVariables(&mvarB);

    ChConstraintTwoGeneric mca(&mvarA, &mvarB);
    mca.SetRightHandSide(-5);
    mca.Get_Cq_a()(0) = 1;
    mca.Get_Cq_a()(1) = 2;
    mca.Get_Cq_a()(2) = -1;
    mca.Get_Cq_b()(0) = 1;
    mca.Get_Cq_b()(1) = -2;
    mca.Get_Cq_b()(2) = 0;

    ChConstraintTwoGeneric mcb(&mvarA, &mvarB);
    mcb.SetRightHandSide(1);
    mcb.Get_Cq_a()(0) = 0;
    mcb.Get_Cq_a()(1) = 1;
    mcb.Get_Cq_a()(2) = 0;
    mcb.Get_Cq_b()(0) = 0;
    mcb.Get_Cq_b()(1) = -2;
    mcb.Get_Cq_b()(2) = 0;

    mdescriptor.InsertConstraint(&mca);
    mdescriptor.InsertConstraint(&mcb);

    mdescriptor.EndInsertion();  // ----- system description ends here

    // Create the solver...
    ChSolverSparseQR solver;
    solver.UseSparsityPatternLearner(true);
    solver.LockSparsityPattern(true);
    solver.SetVerbose(false);

    {
        // Solver information (generic)
        ChSolver* s = &solver;
        std::cout << "Solver Info" << std::endl;
        std::cout << "Type: " << static_cast<std::underlying_type<ChSolver::Type>::type>(s->GetType()) << std::endl;
        std::cout << "Iterative? " << s->IsIterative() << std::endl;
        std::cout << "Direct?    " << s->IsDirect() << std::endl;
        std::cout << "Access as iterative solver: " << s->AsIterative() << std::endl;
        std::cout << "Access as direct solver:    " << s->AsDirect() << std::endl;
    }

    // .. pass the constraint and the variables to the solver to solve
    solver.Setup(mdescriptor, true);
    solver.Solve(mdescriptor);

    // Print results
    double max_res, max_LCPerr;
    mdescriptor.ComputeFeasabilityViolation(max_res, max_LCPerr);

    ChSparseMatrix matrM(mdescriptor.CountActiveVariables(), mdescriptor.CountActiveVariables());
    ChSparseMatrix matrCq(mdescriptor.CountActiveConstraints(), mdescriptor.CountActiveVariables());
    matrM.setZeroValues();
    matrCq.setZeroValues();

    mdescriptor.PasteMassKRMMatrixInto(matrM);
    mdescriptor.PasteConstraintsJacobianMatrixInto(matrCq);

    std::cout << "Matrix M (" << matrM.rows() << "x" << matrM.cols() << ")" << std::endl;
    StreamOut(matrM, std::cout, true);
    std::cout << "Matrix Cq (" << matrCq.rows() << "x" << matrCq.cols() << ")" << std::endl;
    StreamOut(matrCq, std::cout, true);

    std::cout << "**** Using ChSolverSparseQR  **********\n" << std::endl;
    std::cout << "METRICS: max residual: " << max_res << "  max LCP error: " << max_LCPerr << "\n" << std::endl;
    std::cout << "vars q_a and q_b -------------------" << std::endl;
    std::cout << mvarA.State() << std::endl;
    std::cout << mvarB.State() << "  " << std::endl;
    std::cout << "multipliers l_1 and l_2 ------------\n" << std::endl;
    std::cout << mca.GetLagrangeMultiplier() << " " << std::endl;
    std::cout << mcb.GetLagrangeMultiplier() << " \n" << std::endl;
    std::cout << "constraint residuals c_1 and c_2 ---" << std::endl;
    std::cout << mca.GetResidual() << " " << std::endl;
    std::cout << mcb.GetResidual() << " \n" << std::endl;

    // reset variables
    mvarA.State().setZero();
    mvarB.State().setZero();
}

// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\n"
              << "Chrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_SOLVER";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Test 1: an introductory problem:
    test_1(out_dir);

    // Test 2: the 'inverted pendulum' benchmark (compute reactions with Krylov solver)
    test_2(out_dir);

    // Test 3: the stiffness benchmark (add also sparse stiffness blocks over M)
    test_3(out_dir);

    // Test 4: same as 1, but using a direct sparse linear solver
    test_4(out_dir);

    return 0;
}
