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
#include "lcp/ChLcpIterativeMINRES.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "core/ChLinearAlgebra.h"

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;



//  The HyperOCTANT solver is aimed at solving
// LCP/CCP complementarity problems arising
// from QP optimization problems.
//  The LCP problem must be in this (symmetric) 
// form:
//
//   | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
//   | Cq  0 | |l|  |-b|  |c|
//
// as arising in the solution of QP with
// inequalities or in multibody problems.
//
//  The HyperOCTANT technology is mostly targeted
// at solving LCP for multibody problems of Chrono::Engine
// so it offers optimizations for the case where the M matrix
// is diagonal or block-diagonal: each block refers to a 
// ChLcpVariables object, and each line of jacobian Cq belongs
// to a ChLcpConstraint object. 
//
// NOTE: the frictional contact problem is a special type of nonlinear
// complementarity, called Cone Complementarty (CCP) and this is 
// solved as well by HyperOctant, using the same framework.





// Test 1
// First example: how to manage the data structures of 
// the solver, and how to get results after the solution.
// All systems are made of two types of items: the 
//  VARIABLES (unknowns q in the scheme below) and
//  CONSTRAINTS (unknowns reactions 'l' in the scheme).
// As an example, let's solve the following mixed LCP:
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

void test_1()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: generic system with two constraints \n\n";

	// Important: create a 'system descriptor' object that 
	// contains variables and constraints:

	ChLcpSystemDescriptor mdescriptor;

	// Now let's add variables and constraints, as sparse data:

	mdescriptor.BeginInsertion();  // ----- system description is finished

		// create C++ objects representing 'variables':

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

		// create C++ objects representing 'constraints' between variables:

	ChLcpConstraintTwoGeneric mca(&mvarA, &mvarB);
	mca.Set_b_i(-5);
	mca.Get_Cq_a()->ElementN(0)=1;
	mca.Get_Cq_a()->ElementN(1)=2;
	mca.Get_Cq_a()->ElementN(2)=-1;
	mca.Get_Cq_b()->ElementN(0)=1;
	mca.Get_Cq_b()->ElementN(1)=-2;
	mca.Get_Cq_b()->ElementN(2)=0;
	//	mca.SetMode(CONSTRAINT_UNILATERAL);

	ChLcpConstraintTwoGeneric mcb(&mvarA, &mvarB);
	mcb.Set_b_i( 1);
	mcb.Get_Cq_a()->ElementN(0)=0;
	mcb.Get_Cq_a()->ElementN(1)=1;
	mcb.Get_Cq_a()->ElementN(2)=0;
	mcb.Get_Cq_b()->ElementN(0)=0;
	mcb.Get_Cq_b()->ElementN(1)=-2;
	mcb.Get_Cq_b()->ElementN(2)=0;
	//	mcb.SetMode(CONSTRAINT_UNILATERAL);

	std::vector<ChLcpConstraint*> vconstr;
	
	mdescriptor.InsertConstraint(&mca);	
	mdescriptor.InsertConstraint(&mcb);


	mdescriptor.EndInsertion();  // ----- system description ends here

  
	// Solve the problem with an iterative interior-point solver, for an
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
	mdescriptor.ComputeFeasabilityViolation(max_res, max_LCPerr);

	// If needed, dump the full system M and Cq matrices 
	// on disk, in Matlab sparse format:
	ChSparseMatrix matrM;
	ChSparseMatrix matrCq;

	mdescriptor.BuildMatrices(&matrCq, &matrM);

	try
	{
		ChStreamOutAsciiFile fileM ("dump_M.dat");
		ChStreamOutAsciiFile fileCq ("dump_Cq.dat");
		matrM.StreamOUTsparseMatlabFormat(fileM);
		matrCq.StreamOUTsparseMatlabFormat(fileCq);
	}
	catch (ChException myex)
	{
		GetLog() << "FILE ERROR: " << myex.what();
	}


	// Other checks

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

	// reset variables
	mvarA.Get_qb().FillElem(0.);
	mvarB.Get_qb().FillElem(0.);


	// Now solve it again, but using the simplex solver.
	// The simplex solver is much slower, and it cannot handle
	// the case of unilateral constraints. This is reccomended 
	// only for reference or very precise solution of systems with only 
	// bilateral constraints, in a limited number.

	ChLcpSimplexSolver msolver_simpl;

	msolver_simpl.Solve(mdescriptor);
 
	mdescriptor.ComputeFeasabilityViolation(max_res, max_LCPerr);
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

}


// Test 2
// Create a bunch of monodimensional vars and simple
// constraints between them, using 'for' loops, as a benchmark that
// represents, from a physical point of view, a long inverted multipendulum.

void test_2()
{
	
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: 1D vertical pendulum - ChLcpIterativeMINRES \n\n";

	ChLcpSystemDescriptor mdescriptor;

	mdescriptor.BeginInsertion(); 	// ----- system description starts here
	
	int n_masses = 11;

	std::vector<ChLcpVariablesGeneric*> vars;
	std::vector<ChLcpConstraintTwoGeneric*> constraints;

	for (int im = 0; im < n_masses; im++)
	{
		vars.push_back (new ChLcpVariablesGeneric(1));
		vars[im]->GetMass()(0)=10;
		vars[im]->GetInvMass()(0)=1./vars[im]->GetMass()(0);
		vars[im]->Get_fb()(0)= -9.8*vars[im]->GetMass()(0)*0.01; 
		//if (im==5) vars[im]->Get_fb()(0)= 50;
		mdescriptor.InsertVariables(vars[im]);
		if (im>0)
		{
			constraints.push_back (new ChLcpConstraintTwoGeneric(vars[im], vars[im-1]) );
			constraints[im-1]->Set_b_i(0);
			constraints[im-1]->Get_Cq_a()->ElementN(0)=  1;
			constraints[im-1]->Get_Cq_b()->ElementN(0)= -1;
			//constraints[im-1]->SetMode(CONSTRAINT_UNILATERAL); // not supported by  ChLcpSimplexSolver
			mdescriptor.InsertConstraint(constraints[im-1]);
		}
	}
	
	// First variable of 1st domain is 'fixed' like in a hanging chain	
	vars[0]->SetDisabled(true);

	mdescriptor.EndInsertion();		// ----- system description is finished


	// Create a solver of Krylov type
	ChLcpIterativeMINRES msolver_krylov(50,			// max iterations
										false,		// warm start
										0.00001);	// tolerance  

	msolver_krylov.SetOmega(0.2);
	msolver_krylov.SetMaxIterations(80);
	msolver_krylov.SetFeasTolerance(0.1);
	msolver_krylov.SetMaxFixedpointSteps(15);

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


	// Try again, for reference, using a direct solver.
	// This type of solver is much slower, and it cannot handle
	// the case of unilateral constraints. This is reccomended 
	// only for reference or very precise solution of systems with only 
	// bilateral constraints, in a limited number.

	GetLog() << "\n\nTEST: 1D vertical pendulum - ChLcpSimplexSolver \n\n";

	ChLcpSimplexSolver msolver_simpl;
	msolver_simpl.Solve(mdescriptor);

	GetLog() << "VARIABLES: \n";
	for (int im = 0; im < vars.size(); im++)
		GetLog() << "   " << vars[im]->Get_qb()(0) << "\n";

	GetLog() << "CONSTRAINTS: \n";
	for (int ic = 0; ic < constraints.size(); ic++)
		GetLog() << "   " << constraints[ic]->Get_l_i() << "\n"; 

}




// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{

	GetLog() << " Example: the HyperOCTANT techology for solving LCP\n\n\n";

	// Test: an introductory problem:
	test_1();

	// Test: the 'inverted pendulum' benchmark (compute reactions with Krylov solver)
	test_2();

	return 0;
}


