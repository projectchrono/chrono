///////////////////////////////////////////////////
//
//   Demos code about 
//
//     - LCP solvers
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
// 
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
     
// Include some headers used by this tutorial...

#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "core/ChLinearAlgebra.h"

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;



// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{

	GetLog() << " Example: the HyperOCTANT techology for solving LCP\n\n\n";

	//  The HyperOCTANT solver is aimed at solving
	// LCP linear complementarity problems arising
	// from QP optimization problems.
	//  The LCP problem must be in this (symmetric) 
    // form:
    //
    //   | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
    //   | Cq  0 | |l|  |-b|  |c|
    //
    // as arising in the solution of QP with
    // inequalities or in multibody problems.

	//  The HyperOCTANT technology is mostly targeted
	// at solving LCP for multibody problems of Chrono::Engine
	// so it offers optimizations for the case where the M matrix
	// is diagonal or block-diagonal: each block refers to a 
	// ChLcpVariables object, and each line of jacobian Cq belongs
	// to a ChLcpConstraint object. 

	// As an example, let's solve the following mixed LCP (equivalent
	//
	//  | 10  0  0          . -1  0 | |q_a0|   |  1|   |  0|
	//  |  0 10  0          . -2 -1 | |q_a1|   |  2|   |  0|
	//  |  0  0 10          .  1  0 | |q_a2|   |  0|   |  0|
	//  |          20  0  0 . -1  0 | |q_b0|   |  0|   |  0|
	//  |           0 20  0 .  2  1 | |q_b1| - |  0| = |  0|
	//  |           0  0 20 .  0  0 | |q_b2|   |  0|   |  0|
	//  | ..........................| |....|   |...|   |...|
	//  |  1  2 -1  1 -2  0 .       | | l_1|   |  5|   |c_1|
	//  |  0  1  0  0 -1  0 .       | | l_2|   | -1|   |c_2|

	ChLcpSystemDescriptor mdescriptor;

	mdescriptor.BeginInsertion();


	ChLcpVariablesGeneric mvarA(3);
	mvarA.GetMass().SetIdentity();
	mvarA.GetMass()*=10;
	ChLinearAlgebra::Invert(mvarA.GetInvMass(),&mvarA.GetMass());
	mvarA.Get_fb()(0)=1;
	mvarA.Get_fb()(1)=2;

	ChLcpVariablesGeneric mvarB(3);
	mvarB.GetMass().SetIdentity();
	mvarB.GetMass()*=20;
	ChLinearAlgebra::Invert(mvarB.GetInvMass(),&mvarB.GetMass());

	
	mdescriptor.InsertVariables(&mvarA);
	mdescriptor.InsertVariables(&mvarB);


	ChLcpConstraintTwoGeneric mca(&mvarA, &mvarB);
	mca.Set_b_i(-5);
	mca.Get_Cq_a()->ElementN(0)=1;
	mca.Get_Cq_a()->ElementN(1)=2;
	mca.Get_Cq_a()->ElementN(2)=-1;
	mca.Get_Cq_b()->ElementN(0)=1;
	mca.Get_Cq_b()->ElementN(1)=-2;
	mca.Get_Cq_b()->ElementN(2)=0;
	mca.SetMode(CONSTRAINT_UNILATERAL);

	ChLcpConstraintTwoGeneric mcb(&mvarA, &mvarB);
	mcb.Set_b_i( 1);
	mcb.Get_Cq_a()->ElementN(0)=0;
	mcb.Get_Cq_a()->ElementN(1)=1;
	mcb.Get_Cq_a()->ElementN(2)=0;
	mcb.Get_Cq_b()->ElementN(0)=0;
	mcb.Get_Cq_b()->ElementN(1)=-2;
	mcb.Get_Cq_b()->ElementN(2)=0;
	mcb.SetMode(CONSTRAINT_UNILATERAL);

	std::vector<ChLcpConstraint*> vconstr;
	
	mdescriptor.InsertConstraint(&mca);	
	mdescriptor.InsertConstraint(&mcb);


	mdescriptor.EndInsertion();

  
	// Solve the problem with an iterative solver, for an
	// approximate (but very fast) solution:
	//
	// .. create the solver 
	ChLcpIterativeSOR msolver_iter( 50,		// max iterations
									false,	// don't use warm start
									0.0,	// termination tolerance
									1.0);   // omega

	// .. pass the constraint and the variables to the solver 
	//    to solve - that's all.
	msolver_iter.Solve(mdescriptor);

	// Ok, now present the result to the user, with some
	// statistical information:
	double max_res, max_LCPerr;
	msolver_iter.ComputeFeasabilityViolation(vconstr, max_res, max_LCPerr);
	GetLog() << "**** Using ChLcpIterativeSOR  ********** \n\n"; 
	GetLog() << "METRICS: max residual: " << max_res << "  max LCP error: " << max_LCPerr << "  \n\n";
	GetLog() << "vars q_a and q_b -------------------\n";
	GetLog() << mvarA.Get_qb();
	GetLog() << mvarB.Get_qb() << "  \n";;
	GetLog() << "multipliers l_1 and l_2 ------------\n\n";
	GetLog() << mca.Get_l_i() << " \n";
	GetLog() << mcb.Get_l_i() << " \n\n";
	GetLog() << "constraint residuals c_1 and c_2 ---\n";
	GetLog() << mca.Get_c_i() << "  \n";
	GetLog() << mcb.Get_c_i() << "  \n\n\n";

ChMatrixDynamic<> mXiter(8,1);
mXiter.PasteMatrix(&mvarA.Get_qb(),0,0);
mXiter.PasteMatrix(&mvarB.Get_qb(),3,0);
mXiter(6,0) = -mca.Get_l_i();
mXiter(7,0) = -mcb.Get_l_i();

	// reset variables
	mvarA.Get_qb().FillElem(0.);
	mvarB.Get_qb().FillElem(0.);


	// ------------------------------------------------------------------
	
	
	// Now solve it again, but using the simplex solver.
	// The simplex solver is much slower, and may experience
	// exponential complexity explosion (that is, it may run almost
	// forever with endless pivotings) but provides a more 
	// precise solution.

	ChLcpSimplexSolver msolver_simpl;

	msolver_simpl.Solve(mdescriptor);
 
	msolver_simpl.ComputeFeasabilityViolation(vconstr, max_res, max_LCPerr);
	GetLog() << "**** Using ChLcpSimplexSolver ********* \n\n"; 
	GetLog() << "METRICS: max residual: " << max_res << "  max LCP error: " << max_LCPerr << "  \n\n";
	GetLog() << "vars q_a and q_b -------------------\n";
	GetLog() << mvarA.Get_qb();
	GetLog() << mvarB.Get_qb() << "  \n";;
	GetLog() << "multipliers l_1 and l_2 ------------\n\n";
	GetLog() << mca.Get_l_i() << " \n";
	GetLog() << mcb.Get_l_i() << " \n\n";
	GetLog() << "constraint residuals c_1 and c_2 ---\n";
	GetLog() << mca.Get_c_i() << "  \n";
	GetLog() << mcb.Get_c_i() << "  \n";

ChMatrixDynamic<> mXsim(8,1);
mXsim.PasteMatrix(&mvarA.Get_qb(),0,0);
mXsim.PasteMatrix(&mvarB.Get_qb(),3,0);
mXsim(6,0) = -mca.Get_l_i();
mXsim(7,0) = -mcb.Get_l_i();


GetLog() << "\n\n\n**** Using the matrix LCP solver ********* \n\n"; 
ChSparseMatrix mma(8,8);
mma.PasteMatrix(&mvarA.GetMass(),0,0);
mma.PasteMatrix(&mvarB.GetMass(),3,3);
mma.PasteMatrixFloat(mca.Get_Cq_a(), 6,0);
mma.PasteMatrixFloat(mca.Get_Cq_b(), 6,3);
mma.PasteMatrixFloat(mcb.Get_Cq_a(), 7,0);
mma.PasteMatrixFloat(mcb.Get_Cq_b(), 7,3);
mma.PasteTranspMatrixFloat(mca.Get_Cq_a(), 0,6);
mma.PasteTranspMatrixFloat(mca.Get_Cq_b(), 3,6);
mma.PasteTranspMatrixFloat(mcb.Get_Cq_a(), 0,7);
mma.PasteTranspMatrixFloat(mcb.Get_Cq_b(), 3,7);
ChMatrixDynamic<> mX(8,1);
ChMatrixDynamic<> mB(8,1);
mB.PasteMatrix(&mvarA.Get_fb(), 0,0);
mB.PasteMatrix(&mvarB.Get_fb(), 3,0);
mB(6,0)=mca.Get_b_i();
mB(7,0)=mcb.Get_b_i();

ChMatrixDynamic<> mma_full; 
mma.CopyToMatrix(&mma_full); 

GetLog() << "\n [A]=" << mma_full << "\n";
GetLog() << "\n B=" << mB << "\n";



//double mdet;
//ChMatrixDynamic<> mXmatr(8,1);
//mma.DecomposeAndSolve_LDL(&mB,&mXmatr,mdet);
//ChUnilateralData* myunilaterals = new ChUnilateralData[2];
//myunilaterals[0].status=CONSTR_UNILATERAL_OFF;
//myunilaterals[1].status=CONSTR_UNILATERAL_OFF;
//mma.SolveLCP(&mB, &mXmatr, 0,2, 100, false, myunilaterals);

//GetLog() << "\n Xmatr=" << mXmatr << "\n";
//GetLog() << " \n\nVerify A*mXmatr \n" << (mma_full * mXmatr);

GetLog() << "\n Xiter=" << mXiter << "\n";
ChMatrixDynamic<> mverify(mma_full * mXiter);
GetLog() << " \n\nVerify A*mXiter \n" << mverify;

GetLog() << "\n\n Xsim=" << mXsim << "\n";
mverify = mma_full * mXsim;
GetLog() << " \n\nVerify A*mXsim \n" << mverify;



	return 0;
}


