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

// Include some headers used by this tutorial...

#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintTwoBodies.h"
#include "chrono/solver/ChKblockGeneric.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChSolverSOR.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChLinkedListMatrix.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;

//  The HyperOCTANT solver is aimed at solving linear problems and
// VI/LCP/CCP complementarity problems, as those arising
// from QP optimization problems.
//  The problem is described by a variational inequality VI(Z*x-d,K):
//
//  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y
//  | Cq -E | |l|  |-b|  |c|
//
// Also w.symmetric Z by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|
//                                             |Cq  E | |-l| |-b| |c|
// * case linear problem:  all Y_i = R, Ny=0, ex. all bilateral constr.
// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0 ex. unilateral constr.
// * case CCP: Y_i are friction cones, etc.
//
//  The HyperOCTANT technology is mostly targeted
// at solving multibody problems in Chrono::Engine
// so it offers optimizations for the case where the M matrix
// is diagonal or block-diagonal: each block refers to a
// ChVariables object, and each line of jacobian Cq belongs
// to a ChConstraint object.
//
// NOTE: the frictional contact problem is a special type of nonlinear
// complementarity, called Cone Complementary (CCP) and this is
// solved as well by HyperOctant, using the same framework.

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

void test_1() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: generic system with two constraints \n\n";

    // Important: create a 'system descriptor' object that
    // contains variables and constraints:

    ChSystemDescriptor mdescriptor;

    // Now let's add variables and constraints, as sparse data:

    mdescriptor.BeginInsertion();  // ----- system description starts here

    // create C++ objects representing 'variables':

    ChVariablesGeneric mvarA(3);
    mvarA.GetMass().SetIdentity();
    mvarA.GetMass() *= 10;
    ChLinearAlgebra::Invert(mvarA.GetInvMass(), &mvarA.GetMass());
    mvarA.Get_fb()(0) = 1;
    mvarA.Get_fb()(1) = 2;

    ChVariablesGeneric mvarB(3);
    mvarB.GetMass().SetIdentity();
    mvarB.GetMass() *= 20;
    ChLinearAlgebra::Invert(mvarB.GetInvMass(), &mvarB.GetMass());

    mdescriptor.InsertVariables(&mvarA);
    mdescriptor.InsertVariables(&mvarB);

    // create C++ objects representing 'constraints' between variables:

    ChConstraintTwoGeneric mca(&mvarA, &mvarB);
    mca.Set_b_i(-5);
    mca.Get_Cq_a()->ElementN(0) = 1;
    mca.Get_Cq_a()->ElementN(1) = 2;
    mca.Get_Cq_a()->ElementN(2) = -1;
    mca.Get_Cq_b()->ElementN(0) = 1;
    mca.Get_Cq_b()->ElementN(1) = -2;
    mca.Get_Cq_b()->ElementN(2) = 0;

    ChConstraintTwoGeneric mcb(&mvarA, &mvarB);
    mcb.Set_b_i(1);
    mcb.Get_Cq_a()->ElementN(0) = 0;
    mcb.Get_Cq_a()->ElementN(1) = 1;
    mcb.Get_Cq_a()->ElementN(2) = 0;
    mcb.Get_Cq_b()->ElementN(0) = 0;
    mcb.Get_Cq_b()->ElementN(1) = -2;
    mcb.Get_Cq_b()->ElementN(2) = 0;

    mdescriptor.InsertConstraint(&mca);
    mdescriptor.InsertConstraint(&mcb);

    mdescriptor.EndInsertion();  // ----- system description ends here

    // Solve the problem with an iterative fixed-point solver, for an
    // approximate (but very fast) solution:
    //
    // .. create the solver

    ChSolverSOR msolver_iter(1,      // max iterations
                             false,  // don't use warm start
                             0.0,    // termination tolerance
                             0.8);   // omega

    // .. pass the constraint and the variables to the solver
    //    to solve - that's all.
    msolver_iter.Solve(mdescriptor);

    // Ok, now present the result to the user, with some
    // statistical information:
    double max_res, max_LCPerr;
    mdescriptor.ComputeFeasabilityViolation(max_res, max_LCPerr);

    // If needed, dump the full system M and Cq matrices
    // on disk, in Matlab sparse format:
    ChLinkedListMatrix matrM;
    ChLinkedListMatrix matrCq;

    mdescriptor.ConvertToMatrixForm(&matrCq, &matrM, 0, 0, 0, 0, false, false);

    try {
        ChStreamOutAsciiFile fileM("dump_M.dat");
        ChStreamOutAsciiFile fileCq("dump_Cq.dat");
        matrM.StreamOUTsparseMatlabFormat(fileM);
        matrCq.StreamOUTsparseMatlabFormat(fileCq);
    } catch (ChException myex) {
        GetLog() << "FILE ERROR: " << myex.what();
    }

    matrM.StreamOUT(GetLog());
    matrCq.StreamOUT(GetLog());

    GetLog() << "**** Using ChSolverSOR  ********** \n\n";
    GetLog() << "METRICS: max residual: " << max_res << "  max LCP error: " << max_LCPerr << "  \n\n";
    GetLog() << "vars q_a and q_b -------------------\n";
    GetLog() << mvarA.Get_qb();
    GetLog() << mvarB.Get_qb() << "  \n";
    GetLog() << "multipliers l_1 and l_2 ------------\n\n";
    GetLog() << mca.Get_l_i() << " \n";
    GetLog() << mcb.Get_l_i() << " \n\n";
    GetLog() << "constraint residuals c_1 and c_2 ---\n";
    GetLog() << mca.Get_c_i() << "  \n";
    GetLog() << mcb.Get_c_i() << "  \n\n\n";

    // reset variables
    mvarA.Get_qb().FillElem(0.);
    mvarB.Get_qb().FillElem(0.);
}

// Test 2
// Create a bunch of monodimensional vars and simple
// constraints between them, using 'for' loops, as a benchmark that
// represents, from a physical point of view, a long inverted multipendulum.

void test_2() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: 1D vertical pendulum - ChSolverPMINRES \n\n";

    ChSystemDescriptor mdescriptor;

    mdescriptor.BeginInsertion();  // ----- system description starts here

    int n_masses = 11;

    std::vector<ChVariablesGeneric*> vars;
    std::vector<ChConstraintTwoGeneric*> constraints;

    for (int im = 0; im < n_masses; im++) {
        vars.push_back(new ChVariablesGeneric(1));
        vars[im]->GetMass()(0) = 10;
        vars[im]->GetInvMass()(0) = 1. / vars[im]->GetMass()(0);
        vars[im]->Get_fb()(0) = -9.8 * vars[im]->GetMass()(0) * 0.01;
        // if (im==5) vars[im]->Get_fb()(0)= 50;
        mdescriptor.InsertVariables(vars[im]);
        if (im > 0) {
            constraints.push_back(new ChConstraintTwoGeneric(vars[im], vars[im - 1]));
            constraints[im - 1]->Set_b_i(0);
            constraints[im - 1]->Get_Cq_a()->ElementN(0) = 1;
            constraints[im - 1]->Get_Cq_b()->ElementN(0) = -1;
            mdescriptor.InsertConstraint(constraints[im - 1]);
        }
    }

    // First variable of 1st domain is 'fixed' like in a hanging chain
    vars[0]->SetDisabled(true);

    mdescriptor.EndInsertion();  // ----- system description is finished

    try {
        chrono::ChLinkedListMatrix mdM;
        chrono::ChLinkedListMatrix mdCq;
        chrono::ChLinkedListMatrix mdE;
        chrono::ChMatrixDynamic<double> mdf;
        chrono::ChMatrixDynamic<double> mdb;
        chrono::ChMatrixDynamic<double> mdfric;
        mdescriptor.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
        chrono::ChStreamOutAsciiFile file_M("dump_M.dat");
        mdM.StreamOUTsparseMatlabFormat(file_M);
        chrono::ChStreamOutAsciiFile file_Cq("dump_Cq.dat");
        mdCq.StreamOUTsparseMatlabFormat(file_Cq);
        chrono::ChStreamOutAsciiFile file_E("dump_E.dat");
        mdE.StreamOUTsparseMatlabFormat(file_E);
        chrono::ChStreamOutAsciiFile file_f("dump_f.dat");
        mdf.StreamOUTdenseMatlabFormat(file_f);
        chrono::ChStreamOutAsciiFile file_b("dump_b.dat");
        mdb.StreamOUTdenseMatlabFormat(file_b);
        chrono::ChStreamOutAsciiFile file_fric("dump_fric.dat");
        mdfric.StreamOUTdenseMatlabFormat(file_fric);
    } catch (chrono::ChException myexc) {
        chrono::GetLog() << myexc.what();
    }

    // Create a solver of Krylov type
    ChSolverPMINRES msolver_krylov(20,        // max iterations
                                   false,     // warm start
                                   0.00001);  // tolerance

    // .. pass the constraint and the variables to the solver
    //    to solve - that's all.
    msolver_krylov.Solve(mdescriptor);

    // Output values
    GetLog() << "VARIABLES: \n";
    for (int im = 0; im < vars.size(); im++)
        GetLog() << "   " << vars[im]->Get_qb()(0) << "\n";

    GetLog() << "CONSTRAINTS: \n";
    for (int ic = 0; ic < constraints.size(); ic++)
        GetLog() << "   " << constraints[ic]->Get_l_i() << "\n";
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

void test_3() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: generic system with stiffness blocks \n\n";

    // Important: create a 'system descriptor' object that
    // contains variables and constraints:

    ChSystemDescriptor mdescriptor;

    // Now let's add variables, constraints and stiffness, as sparse data:

    mdescriptor.BeginInsertion();  // ----- system description

    // Create C++ objects representing 'variables', set their M blocks
    // (the masses) and set their known terms 'fb'

    ChMatrix33<> minertia;
    minertia.FillDiag(6);

    ChVariablesBodyOwnMass mvarA;
    mvarA.SetBodyMass(5);
    mvarA.SetBodyInertia(minertia);
    mvarA.Get_fb().FillRandom(-3, 5);

    ChVariablesBodyOwnMass mvarB;
    mvarB.SetBodyMass(4);
    mvarB.SetBodyInertia(minertia);
    mvarB.Get_fb().FillRandom(1, 3);

    ChVariablesBodyOwnMass mvarC;
    mvarC.SetBodyMass(5.5);
    mvarC.SetBodyInertia(minertia);
    mvarC.Get_fb().FillRandom(-8, 3);

    mdescriptor.InsertVariables(&mvarA);
    mdescriptor.InsertVariables(&mvarB);
    mdescriptor.InsertVariables(&mvarC);

    // Create two C++ objects representing 'constraints' between variables
    // and set the jacobian to random values;
    // Also set cfm term (E diagonal = -cfm)

    ChConstraintTwoBodies mca(&mvarA, &mvarB);
    mca.Set_b_i(3);
    mca.Get_Cq_a()->FillRandom(-1, 1);
    mca.Get_Cq_b()->FillRandom(-1, 1);
    mca.Set_cfm_i(0.2);

    ChConstraintTwoBodies mcb(&mvarA, &mvarB);
    mcb.Set_b_i(5);
    mcb.Get_Cq_a()->FillRandom(-1, 1);
    mcb.Get_Cq_b()->FillRandom(-1, 1);
    mcb.Set_cfm_i(0.1);

    mdescriptor.InsertConstraint(&mca);
    mdescriptor.InsertConstraint(&mcb);

    // Create two C++ objects representing 'stiffness' between variables:

    ChKblockGeneric mKa;
    // set the affected variables (so this K is a 12x12 matrix, relative to 4 6x6 blocks)
    std::vector<ChVariables*> mvarsa;
    mvarsa.push_back(&mvarA);
    mvarsa.push_back(&mvarB);
    mKa.SetVariables(mvarsa);

    // just fill K with random values (but symmetric, by making a product of matr*matrtransposed)
    ChMatrixDynamic<> mtempA = *mKa.Get_K();  // easy init to same size of K
    mtempA.FillRandom(-0.3, 0.3);
    ChMatrixDynamic<> mtempB;
    mtempB.CopyFromMatrixT(mtempA);
    *mKa.Get_K() = -mtempA * mtempB;

    mdescriptor.InsertKblock(&mKa);

    ChKblockGeneric mKb;
    // set the affected variables (so this K is a 12x12 matrix, relative to 4 6x6 blocks)
    std::vector<ChVariables*> mvarsb;
    mvarsb.push_back(&mvarB);
    mvarsb.push_back(&mvarC);
    mKb.SetVariables(mvarsb);

    *mKb.Get_K() = *mKa.Get_K();

    mdescriptor.InsertKblock(&mKb);

    mdescriptor.EndInsertion();  // ----- system description ends here

    // SOLVE the problem with an iterative Krylov solver.
    // In this case we use a MINRES-like solver, that features
    // very good convergence, it supports indefinite cases (ex.
    // redundant constraints) and also supports the presence
    // of ChStiffness blocks (other solvers cannot cope with this!)

    // .. create the solver

    ChSolverPMINRES msolver_mr(80,      // max iterations
                               false,   // don't use warm start
                               1e-12);  // termination tolerance

    // .. set optional parameters of solver
    msolver_mr.SetDiagonalPreconditioning(true);
    msolver_mr.SetVerbose(true);

    // .. solve the system (passing variables, constraints, stiffness
    //    blocks with the ChSystemDescriptor that we populated above)

    msolver_mr.Solve(mdescriptor);

    // .. optional: get the result as a single vector (it collects all q_i and l_i
    //    solved values stored in variables and constraints), just for check.
    chrono::ChMatrixDynamic<double> mx;
    mdescriptor.FromUnknownsToVector(mx);  // x ={q,-l}

    // CHECK. Test if, with the solved x, we really have Z*x-d=0 ...
    // to this end do the multiplication with the special function
    // SystemProduct() that is 'sparse-friendly' and does not build Z explicitly:

    chrono::ChMatrixDynamic<double> md;
    mdescriptor.BuildDiVector(md);  // d={f;-b}

    chrono::ChMatrixDynamic<double> mZx;
    mdescriptor.SystemProduct(mZx, &mx);  // Zx = Z*x

    GetLog() << "CHECK: norm of solver residual: ||Z*x-d|| -------------------\n";
    GetLog() << (mZx - md).NormInf() << "\n";

    /*
    // Alternatively, instead of using FromUnknownsToVector, to fetch
    // result, you could just loop over the variables (q values) and
    // over the constraints (l values), as already shown in previous examples:

    for (int im = 0; im < mdescriptor.GetVariablesList().size(); im++)
        GetLog() << "   " << mdescriptor.GetVariablesList()[im]->Get_qb()(0) << "\n";

    for (int ic = 0; ic < mdescriptor.GetConstraintsList().size(); ic++)
        GetLog() << "   " << mdescriptor.GetConstraintsList()[ic]->Get_l_i() << "\n";
    */
}

// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    GetLog() << " Example: the HyperOCTANT technology for solving LCP\n\n\n";

    // Test: an introductory problem:
    test_1();

    // Test: the 'inverted pendulum' benchmark (compute reactions with Krylov solver)
    test_2();

    // Test: the stiffness benchmark (add also sparse stiffness blocks over M)
    test_3();


	getchar();

    return 0;
}
