#ifndef CHLCPSYSTEMDESCRIPTOR_H
#define CHLCPSYSTEMDESCRIPTOR_H

//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptor.h
//
//    Base class for collecting objects inherited 
//   from ChLcpConstraint, ChLcpVariables, ChLcpStiffness. 
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "lcp/ChLcpVariables.h"
#include "lcp/ChLcpConstraint.h"
#include "lcp/ChLcpKstiffness.h"
#include <vector>
#include "parallel/ChThreads.h"
#include "parallel/ChThreadsSync.h"


namespace chrono
{


/// Base class for collecting objects inherited from ChLcpConstraint ,
/// ChLcpVariables and, optionally ChLcpKstiffness. These objects
/// can be used to define a sparse representation of the system.
///  The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|  
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones
/// Optionally, also objects inherited from ChLcpKstiffness can be added 
/// too, hence M becomes M+K (but not all solvers handle ChLcpKstiffness!)
/// All LCP solvers require that the description of
/// the system is passed by means of a ChLcpSystemDescriptor,
/// where all constraints, variables, masses, known terms 
///	(ex.forces) are represented as sparse data that
/// are objects inherited from ChLcpConstraint or ChLcpVariables. 
/// Within this default implementation, the ChLcpSystemDescriptor
/// simply contains vectors with pointers to the variables
/// and constraints, but more advanced implementation (ex. for
/// supporting parallel GPU solvers) could store constraints
/// and variables structures with more efficient data schemes.


class ChApi ChLcpSystemDescriptor
{

protected:
			//
			// DATA
			//
		std::vector<ChLcpConstraint*> vconstraints;
		std::vector<ChLcpVariables*>  vvariables;
		std::vector<ChLcpKstiffness*>  vstiffness;
		
		int num_threads;
		ChThreads* solver_threads;
		ChMutexSpinlock* locktable;

public:

			//
			// CONSTRUCTORS
			//
	ChLcpSystemDescriptor();

	virtual ~ChLcpSystemDescriptor();

	
		/// Access the vector of constraints
	std::vector<ChLcpConstraint*>& GetConstraintsList() {return vconstraints;};

		/// Access the vector of variables
	std::vector<ChLcpVariables*>& GetVariablesList() {return vvariables;};

		/// Access the vector of stiffness matrix blocks
	std::vector<ChLcpKstiffness*>& GetKstiffnessLis() {return vstiffness;};


		/// Begin insertion of items
	virtual void BeginInsertion()
					{
						vconstraints.clear();
						vvariables.clear();
						vstiffness.clear();
					}

		/// Insert reference to a ChLcpConstraint object
	virtual void InsertConstraint(ChLcpConstraint* mc) { vconstraints.push_back(mc); }

		/// Insert reference to a ChLcpVariables object
	virtual void InsertVariables(ChLcpVariables* mv) { vvariables.push_back(mv); }

		/// Insert reference to a ChLcpKstiffness object (a piece of K matrix)
	virtual void InsertKstiffness(ChLcpKstiffness* mk) { vstiffness.push_back(mk); }


		/// Begin insertion of items
	virtual void EndInsertion()
					{
					}


			// UTILITY FUNCTIONS

				/// Set the number of threads (some operations like ShurComplementProduct
				/// are CPU intensive, so they can be run in parallel threads).
	virtual void SetNumThreads(int nthreads);
	virtual int  GetNumThreads() {return this->num_threads;}


				/// The following function may be called after a LCP solver's 'Solve()'
				/// operation has been performed. This gives an extimate of 'how
				/// good' the solver had been in finding the proper solution.
				/// Resulting estimates are passed as references in member arguments.

	virtual void ComputeFeasabilityViolation(
					double& resulting_maxviolation,		///< gets the max constraint violation (either bi- and unilateral.)
					double& resulting_lcpfeasability	///< gets the max feasability as max |l*c| , for unilateral only
				);

				/// Count the scalar variables in the system (excluding ChLcpVariable objects
				/// that have  IsActive() as false). Note: the number of scalar variables is not necessarily
				/// the number of inserted ChLcpVariable objects.
				/// Note: this function also updates the offsets of all variables (see GetOffset() in ChLcpVariables)
	virtual int CountActiveVariables();

				/// Count the scalar constraints in the system (excluding ChLcpConstraint objects
				/// that have  IsActive() as false). 
	virtual int CountActiveConstraints();



				/// Get a vector with all the 'fb' known terms ('forces'etc.) associated to all variables,
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length if necessary.
	virtual int BuildFbVector(
								ChMatrix<>& Fvector 	///< matrix which will contain the entire vector of 'f'
							);
				/// Get a vector with all the 'bi' known terms ('constraint residuals' etc.) associated to all constraints,
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length if necessary.
	virtual int BuildBiVector(
								ChMatrix<>& Bvector  	///< matrix which will contain the entire vector of 'b'
							);

				/// Using this function, one may get a vector with all the variables 'q'
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length if necessary
				/// (but if you are sure that the vector has already the proper size, you can optimize
				/// the performance a bit by setting resize_vector as false).
				/// \return  the number of scalar variables (i.e. the rows of the column vector).
	virtual int FromVariablesToVector(
								ChMatrix<>& mvector,	///< matrix which will contain the entire vector of 'q'
								bool resize_vector=true	///< if true the vector size will be checked & resized if necessary
								);

				/// Using this function, one may go in the opposite direction of the FromVariablesToVector()
				/// function, i.e. one gives a vector with all the variables 'q' ordered into a column vector, and
				/// the variables objects are updated according to these values.
				/// NOTE!!! differently from  FromVariablesToVector(), which always works, this
				/// function will fail if mvector does not match the amount and ordering of
				/// the variable objects!!! (it is up to the user to check this!) btw: most often,
				/// this is called after FromVariablesToVector() to do a kind of 'undo', for example.
				/// \return  the number of scalar variables (i.e. the rows of the column vector).
	virtual int FromVectorToVariables(
								ChMatrix<>& mvector					///< matrix which contains the entire vector of 'q'
								);


				/// Using this function, one may get a vector with all the constraint reactions 'l_i'
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length if necessary
				/// (but uf you are sure that the vector has already the proper size, you can optimize
				/// the performance a bit by setting resize_vector as false).
				/// Optionally, you can pass an 'enabled' vector of bools, that must have the same
				/// length of the l_i reactions vector; constraints with enabled=false are not handled.
				/// \return  the number of scalar constr.multipliers (i.e. the rows of the column vector).
	virtual int FromConstraintsToVector(
								ChMatrix<>& mvector,		///< matrix which will contain the entire vector of 'l_i'
								bool resize_vector=true		///< if true the vector size will be checked & resized if necessary
								);

				/// Using this function, one may go in the opposite direction of the FromConstraintsToVector()
				/// function, i.e. one gives a vector with all the constr.reactions 'l_i' ordered into a column vector, and
				/// the constraint objects are updated according to these values.
				/// Optionally, you can pass an 'enabled' vector of bools, that must have the same
				/// length of the l_i reactions vector; constraints with enabled=false are not handled.
				/// NOTE!!! differently from  FromConstraintsToVector(), which always works, this
				/// function will fail if mvector does not match the amount and ordering of
				/// the variable objects!!! (it is up to the user to check this!) btw: most often,
				/// this is called after FromConstraintsToVector() to do a kind of 'undo', for example.
				/// \return  the number of scalar constraint multipliers (i.e. the rows of the column vector).
	virtual int FromVectorToConstraints(
								ChMatrix<>& mvector			///< matrix which contains the entire vector of 'l_i'
								);



				/// Using this function, one may get a vector with all the unknowns x={q,l} i.e. q variables & l_i constr.
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length if necessary
				/// (but if you are sure that the vector has already the proper size, you can optimize
				/// the performance a bit by setting resize_vector as false).
				/// \return  the number of scalar unknowns
	virtual int FromUnknownsToVector(
								ChMatrix<>& mvector,	///< matrix which will contain the entire vector x={q,l}
								bool resize_vector=true	///< if true the vector size will be checked & resized if necessary
								);

				/// Using this function, one may go in the opposite direction of the FromUnknownsToVector()
				/// function, i.e. one gives a vector with all the unknowns x={q,l} ordered into a column vector, and
				/// the variables q and constr.multipliers l objects are updated according to these values.
				/// NOTE!!! differently from  FromUnknownsToVector(), which always works, this
				/// function will fail if mvector does not match the amount and ordering of
				/// the variable and constraint objects!!! (it is up to the user to check this!)
	virtual int FromVectorToUnknowns(
								ChMatrix<>& mvector		///< matrix which contains the entire vector x={q,l}
								);


				/// Performs the product of N, the Shur complement of the KKT matrix, by an 
				/// l vector (if x not provided, use current lagrangian multipliers l_i), that is 
				///    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] * l
				/// where [Cq] are the jacobians, [M] is the mass matrix, [E] is the matrix
				/// of the optional cfm 'constraint force mixing' terms for compliant constraints.
				/// The N matrix is not built explicitly, to exploit sparsity, it is described by the 
				/// inserted constraints and inserted variables.
				/// Optionally, you can pass an 'enabled' vector of bools, that must have the same
				/// length of the l_i reactions vector; constraints with enabled=false are not handled.
				/// NOTE! the 'q' data in the ChVariables of the system descriptor is changed by this
				/// operation, so it may happen that you need to backup them via FromVariablesToVector()
				/// NOTE! currently this function does NOT support the cases that use also ChLcpKstiffness
				/// objects, because it would need to invert the global M+K, that is not diagonal,
				/// for doing = [N]*l = [ [Cq][(M+K)^(-1)][Cq'] - [E] ] * l
	virtual void ShurComplementProduct(	
								ChMatrix<>&	result,			///< matrix which contains the result of  N*l_i 
								ChMatrix<>* lvector,		///< optional matrix with the vector to be multiplied (if null, use current constr. multipliers l_i)
								std::vector<bool>* enabled=0 ///< optional: vector of enable flags, one per scalar constraint. true=enable, false=disable (skip)
								);


				/// Performs the product of the entire system matrix (KKT matrix), by a vector x ={q,l} 
				/// (if x not provided, use values in current lagrangian multipliers l_i 
				/// and current q variables)
				/// NOTE! the 'q' data in the ChVariables of the system descriptor is changed by this
				/// operation, so it may happen that you need to backup them via FromVariablesToVector()
	virtual void SystemProduct(	
								ChMatrix<>&	result,			///< matrix which contains the result of matrix by x 
								ChMatrix<>* x		        ///< optional matrix with the vector to be multiplied (if null, use current l_i and q)
								// std::vector<bool>* enabled=0 ///< optional: vector of enable flags, one per scalar constraint. true=enable, false=disable (skip)
								);


				/// Performs projecton of constraint multipliers onto allowed set (in case
				/// of bilateral constraints it does not affect multipliers, but for frictional 
				/// constraints, for example, it projects multipliers onto the friction cones)
				/// Note! the 'l_i' data in the ChConstraints of the system descriptor are changed
				/// by this operation (they get the value of 'multipliers' after the projection), so 
				/// it may happen that you need to backup them via FromConstraintToVector().
	virtual void ConstraintsProject(	
								ChMatrix<>&	multipliers		///< matrix which contains the entire vector of 'l_i' multipliers to be projected
								);


				/// The following function may be used to create the Jacobian and the 
				/// mass matrix of the variational problem in matrix form, by assembling all 
				/// the jacobians of all the constraints/contacts, all the mass matrices, all vectors,
				/// as they are _currently_ stored in the sparse data of all ChConstraint and ChVariables 
				/// contained in this ChLcpSystemDescriptor. 
				/// The matrices define the VI variational inequality:
				///
				///  | M -Cq'|*|q|- | f|= |0| , l \in Y (es friction cone), C \in normal cone to Y  
				///  | Cq -E | |l|  |-b|  |C|    (case no friction: LCP C>=0, l>=0, l*c=0;)        
				///                              (case only bilaterals: linear system, c=0)
				///
				/// also symmetrizable by flipping the sign of l_i terms:  | M  Cq'|*| q|- | f|= |0|  
				///                                                        | Cq -E | |-l|  |-b|  |C|
				/// Note 1: most often you'll call ConvertToMatrixForm() right after a dynamic simulation timestep,
				///         because the system matrices are properly initialized,
				/// Note 2: when using Anitescu default stepper, the 'f' vector contains forces*timestep = F*dt
				/// Note 3: when using Anitescu default stepper, q represents the 'delta speed',
				/// Note 4: when using Anitescu default stepper, b represents the dt/phi stabilization term.
				///  This can be useful for debugging, data dumping, and similar purposes (most solvers avoid 
				/// using these matrices, for performance), for example you will load these matrices in Matlab.
				/// Optionally, tangential (u,v) contact jacobians may be skipped, or only bilaterals can be considered
				/// The matrices and vectors are automatically resized if needed.
	virtual void ConvertToMatrixForm 
								 (ChSparseMatrix* Cq, ///< fill this system jacobian matrix, if not null
								  ChSparseMatrix* M,  ///< fill this system mass matrix, if not null
								  ChSparseMatrix* E,  ///< fill this system 'compliance' matrix , if not null
								  ChMatrix<>* Fvector,///< fill this vector as the known term 'f', if not null
								  ChMatrix<>* Bvector,///< fill this vector as the known term 'b', if not null
								  ChMatrix<>* Frict,  ///< fill as a vector with friction coefficients (=-1 for tangent comp.; =-2 for bilaterals), if not null
								bool only_bilaterals = false, 
								bool skip_contacts_uv = false);



				/// OBSOLETE. Kept only for backward compability. Use rather: ConvertToMatrixForm
	virtual void BuildMatrices (ChSparseMatrix* Cq,
								ChSparseMatrix* M,
								bool only_bilaterals = false, 
								bool skip_contacts_uv = false);
				/// OBSOLETE. Kept only for backward compability. Use rather: ConvertToMatrixForm, or BuildFbVector or BuildBiVector
	virtual void BuildVectors (ChSparseMatrix* f, 
								ChSparseMatrix* b,	
								bool only_bilaterals = false, 
								bool skip_contacts_uv = false);
};





} // END_OF_NAMESPACE____






#endif  
