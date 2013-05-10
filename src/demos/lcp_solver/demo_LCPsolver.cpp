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
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpConstraintTwoBodies.h"
#include "lcp/ChLcpKstiffnessGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "core/ChLinearAlgebra.h"

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
//  | 10  0  0          .  1  0 | |q_a0|   |  1|   |  0|
//  |  0 10  0          .  2  1 | |q_a1|   |  2|   |  0|
//  |  0  0 10          . -1  0 | |q_a2|   |  0|   |  0|
//  |          20  0  0 .  1  0 | |q_b0|   |  0|   |  0|
//  |           0 20  0 . -2 -1 | |q_b1| - |  0| = |  0|
//  |           0  0 20 .  0  0 | |q_b2|   |  0|   |  0|
//  | ..........................| |....|   |...|   |...|
//  |  1  2 -1  1 -2  0 .       | |-l_1|   |  5|   |c_1|
//  |  0  1  0  0 -1  0 .       | |-l_2|   | -1|   |c_2|

void test_1()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: generic system with two constraints \n\n";

	// Important: create a 'system descriptor' object that 
	// contains variables and constraints:

	ChLcpSystemDescriptor mdescriptor;

	// Now let's add variables and constraints, as sparse data:

	mdescriptor.BeginInsertion();  // ----- system description starts here

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

	ChLcpConstraintTwoGeneric mcb(&mvarA, &mvarB);
	mcb.Set_b_i( 1);
	mcb.Get_Cq_a()->ElementN(0)=0;
	mcb.Get_Cq_a()->ElementN(1)=1;
	mcb.Get_Cq_a()->ElementN(2)=0;
	mcb.Get_Cq_b()->ElementN(0)=0;
	mcb.Get_Cq_b()->ElementN(1)=-2;
	mcb.Get_Cq_b()->ElementN(2)=0;

	
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


			try
			{
				chrono::ChSparseMatrix mdM;
				chrono::ChSparseMatrix mdCq;
				chrono::ChSparseMatrix mdE;
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
			} 
			catch(chrono::ChException myexc)
			{
				chrono::GetLog() << myexc.what();
			}

	// Create a solver of Krylov type
	ChLcpIterativePMINRES msolver_krylov(20,		// max iterations
										false,		// warm start
										0.00001);	// tolerance  


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



// Test 3
// Create three variables, with some mass, and also add a
// ChLcpStiffness item that connects two of these variables
// with random stiffness. 
// Also use the ChLcpSystemDescriptor functions FromUnknownsToVector
// and FromVectorToUnknowns for doing checks.
//
//  | M+K   K       . Cq' | |q_a |   |f_a|   |  0|
//  |  K   M+K      . Cq' | |q_b |   |f_b|   |  0|
//  |            M  .     | |q_c |   |f_c| = |  0|
//  | ....................| |... |   |...|   |...|
//  |  Cq   Cq      .     | |-l_1|   |  5|   |c_1|


void test_3()
{
	GetLog() << "\n-------------------------------------------------\n";
	GetLog() << "TEST: generic system with stiffness blocks \n\n";

	// Important: create a 'system descriptor' object that 
	// contains variables and constraints:

	ChLcpSystemDescriptor mdescriptor;

	// Now let's add variables and constraints, as sparse data:

	mdescriptor.BeginInsertion();  // ----- system description 

		// Create C++ objects representing 'variables', set their M blocks
		// (the masses) and set their known terms 'fb'

	ChMatrix33<> minertia;
	minertia.FillDiag(6);

	ChLcpVariablesBodyOwnMass mvarA;
	mvarA.SetBodyMass(5);
	mvarA.SetBodyInertia(&minertia);
	mvarA.Get_fb().FillRandom(-3,3);

	ChLcpVariablesBodyOwnMass mvarB;
	mvarB.SetBodyMass(4);
	mvarB.SetBodyInertia(&minertia);
	mvarB.Get_fb().FillRandom(2,5);

	ChLcpVariablesBodyOwnMass mvarC;
	mvarC.SetBodyMass(5.5);
	mvarC.SetBodyInertia(&minertia);
	mvarC.Get_fb().FillRandom(-8,3);

	mdescriptor.InsertVariables(&mvarA);
	mdescriptor.InsertVariables(&mvarB);
	mdescriptor.InsertVariables(&mvarC);

		// Create two C++ objects representing 'constraints' between variables
		// and set the jacobian to random values; 
		// Also set cfm term (E diagonal = -cfm)

	ChLcpConstraintTwoBodies mca(&mvarA, &mvarB);
	mca.Set_b_i(5);
	mca.Get_Cq_a()->FillRandom(-1,1);
	mca.Get_Cq_b()->FillRandom(-1,1);
	mca.Set_cfm_i(0.2);

	ChLcpConstraintTwoBodies mcb(&mvarA, &mvarB);
	mcb.Set_b_i(5);
	mcb.Get_Cq_a()->FillRandom(-1,1);
	mcb.Get_Cq_b()->FillRandom(-1,1);
	mcb.Set_cfm_i(0.1);

	mdescriptor.InsertConstraint(&mca);
	mdescriptor.InsertConstraint(&mcb);


		// Create two C++ objects representing 'stiffness' between variables:

	ChLcpKstiffnessGeneric mKa;
	// set the affected variables (so this K is a 12x12 matrix, relative to 4 6x6 blocks)
	std::vector<ChLcpVariables*> mvarsa;
	mvarsa.push_back(&mvarA);
	mvarsa.push_back(&mvarB);
	mKa.SetVariables(mvarsa); 

	mKa.Get_K()->FillRandom(0.3,0.3);
	
	mdescriptor.InsertKstiffness(&mKa);
	

	ChLcpKstiffnessGeneric mKb;
	// set the affected variables (so this K is a 12x12 matrix, relative to 4 6x6 blocks)
	std::vector<ChLcpVariables*> mvarsb;
	mvarsb.push_back(&mvarB);
	mvarsb.push_back(&mvarC);
	mKb.SetVariables(mvarsb); 

	mKb.Get_K()->FillRandom(-9.2,9.02);
	
	mdescriptor.InsertKstiffness(&mKb);


	mdescriptor.EndInsertion();  // ----- system description ends here

  
	// A- Check functionality of the full system product Z*x

	int nv = mdescriptor.CountActiveVariables();
	int nc = mdescriptor.CountActiveConstraints();

	chrono::ChMatrixDynamic<double> mx(nv+nc, 1);
	chrono::ChMatrixDynamic<double> prod_e(nv+nc, 1);
	chrono::ChMatrixDynamic<double> prod_f(nv+nc, 1);
	mx.FillRandom(-3,3);	// a random x vector

	mdescriptor.SystemProduct(prod_e, &mx);   //! HERE DO: e = Z*x


	// check exactness by doing the product with matrices:

	chrono::ChSparseMatrix mdM;
	chrono::ChSparseMatrix mdCq;
	chrono::ChSparseMatrix mdE;
	chrono::ChMatrixDynamic<double> mdf;
	chrono::ChMatrixDynamic<double> mdb;
	chrono::ChMatrixDynamic<double> mdfric;
	// fill the matrices given the system:
	mdescriptor.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, 0);
	// make Z from [M+K,Cq';Cq',E]
	chrono::ChSparseMatrix mdZ(nv+nc, nv+nc);
	mdZ.PasteMatrix(&mdM,0,0);
	mdZ.PasteMatrix(&mdE,nv,nv);
	mdZ.PasteMatrix(&mdCq,nv,0);
	mdZ.PasteTranspMatrix(&mdCq,0,nv);
	// convert sparse to full 
	chrono::ChMatrixDynamic<double> mdZd(nv+nc, nv+nc);
	mdZ.CopyToMatrix(&mdZd);
	prod_f.MatrMultiply(mdZd,mx);

			try
			{
				chrono::ChStreamOutAsciiFile file_M("dump_M.dat");
				mdM.StreamOUTsparseMatlabFormat(file_M);
				chrono::ChStreamOutAsciiFile file_Cq("dump_Cq.dat");
				mdCq.StreamOUTsparseMatlabFormat(file_Cq);
				chrono::ChStreamOutAsciiFile file_E("dump_E.dat");
				mdE.StreamOUTsparseMatlabFormat(file_E);
				chrono::ChStreamOutAsciiFile file_Z("dump_Z.dat");
				mdZ.StreamOUTsparseMatlabFormat(file_Z);
				chrono::ChStreamOutAsciiFile file_f("dump_f.dat");
				mdf.StreamOUTdenseMatlabFormat(file_f);
				chrono::ChStreamOutAsciiFile file_b("dump_b.dat");
				mdb.StreamOUTdenseMatlabFormat(file_b);
				chrono::ChStreamOutAsciiFile file_x("dump_x.dat");
				mx.StreamOUTdenseMatlabFormat(file_x);
				chrono::ChStreamOutAsciiFile file_e_cpp("dump_e_cpp.dat");
				prod_e.StreamOUTdenseMatlabFormat(file_e_cpp);
			} 
			catch(chrono::ChException myexc)
			{
				chrono::GetLog() << myexc.what();
			}

	GetLog() << "SystemProduct Z*x -------------------\n";
	GetLog() << prod_e << "\n";

	GetLog() << "Matrix (check) product Z*x -------------------\n";
	GetLog() << prod_f << "\n";

	GetLog() << "norm err check product Z*x -------------------\n";
	GetLog() << (prod_f - prod_e).NormInf() << "\n";

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

	// Test: the stiffness benchmark
	test_3();

	return 0;
}


