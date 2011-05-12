#ifndef CHLCPSYSTEMDESCRIPTOR_H
#define CHLCPSYSTEMDESCRIPTOR_H

//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptor.h
//
//    Base class for collecting objects inherited 
//   from ChLcpConstraint or ChLcpVariables. 
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
#include <vector>

namespace chrono
{


/// Base class for collecting objects inherited 
/// from ChLcpConstraint or ChLcpVariables. 
/// All LCP solvers require that the description of
/// the system is passed by means of a ChLcpSystemDescriptor,
/// where all constraints, variables, masses, known terms 
///	(ex.forces) are represented as sparse data that
/// are objects inherited from ChLcpConstraint or ChLcpVariables. 
/// Within this default implementation, the ChLcpSystemDescriptor
/// simply contains two vectors with pointers to the variables
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

public:

			//
			// CONSTRUCTORS
			//
	ChLcpSystemDescriptor()
					{
						vconstraints.clear();
						vvariables.clear();
					};


	virtual ~ChLcpSystemDescriptor()
					{
						vconstraints.clear();
						vvariables.clear();
					};

	
		/// Access the vector of constraints
	std::vector<ChLcpConstraint*>& GetConstraintsList() {return vconstraints;};

		/// Access the vector of variables
	std::vector<ChLcpVariables*>& GetVariablesList() {return vvariables;};


		/// Begin insertion of items
	virtual void BeginInsertion()
					{
						vconstraints.clear();
						vvariables.clear();
					}

		/// Insert reference to a ChLcpConstraint object
	virtual void InsertConstraint(ChLcpConstraint* mc) { vconstraints.push_back(mc); }

		/// Insert reference to a ChLcpVariables object
	virtual void InsertVariables(ChLcpVariables* mv) { vvariables.push_back(mv); }

		/// Begin insertion of items
	virtual void EndInsertion()
					{
					}


			// UTILITY FUNCTIONS

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
	virtual int CountActiveVariables();

				/// Count the scalar constraints in the system (excluding ChLcpConstraint objects
				/// that have  IsActive() as false). 
	virtual int CountActiveConstraints();


				/// The following function may be used to create the full Jacobian and the full
				/// mass matrix of the complementarity problem in matrix form, by assembling all 
				/// the jacobians of all the constraints/contacts and all the mass matrices. This
				/// can be useful for debugging, data dumping, and similar purposes.
				/// Optionally, tangential (u,v) contact jacobians may be skipped, etc.
				/// The sparse matrices are automatically resized if needed.
	virtual void BuildMatrices (ChSparseMatrix* Cq, ///< fill this system jacobian matrix, if not null
								ChSparseMatrix* M,	///< fill this system mass matrix, if not null
								bool only_bilaterals = false, 
								bool skip_contacts_uv = false);
	
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
				/// The following function may be used to create the full f and b vectors
				/// of the complementarity problem, by assembling all the terms. This
				/// can be useful for debugging, data dumping, and similar purposes.
				/// Optionally, tangential (u,v) components may be skipped, etc.
	virtual void BuildVectors (ChSparseMatrix* f, ///< fill this vector, if not null
								ChSparseMatrix* b,	///< fill this vector, if not null
								bool only_bilaterals = false, 
								bool skip_contacts_uv = false);
				/// Using this function, one may get a vector with all the variables 'q'
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length if necessary
				/// (but uf you are sure that the vector has already the proper size, you can optimize
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
								//std::vector<bool>* enabled  ///< optional: vector of enable flags, one per scalar constraint. true=enable, false=disable (skip)
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
								//std::vector<bool>* enabled  ///< optional: vector of enable flags, one per scalar constraint. true=enable, false=disable (skip)
								);


				/// Performs the product of N, the Shur complement of the KKT matrix, by an 
				/// l vector (if x not provided, use current lagrangian multipliers l_i), that is 
				///    result = [N]*l = [Cq][M^(-1)][Cq']*l
				/// The N matrix is not built explicitly, to exploit sparsity.
				/// Optionally, you can pass an 'enabled' vector of bools, that must have the same
				/// length of the l_i reactions vector; constraints with enabled=false are not handled.
	virtual void ShurComplementProduct(	
								ChMatrix<>&	result,			///< matrix which contains the result of  N*l_i 
								ChMatrix<>* lvector,		///< optional matrix with the vector to be multiplied (if null, use current constr. multipliers l_i)
								std::vector<bool>* enabled  ///< optional: vector of enable flags, one per scalar constraint. true=enable, false=disable (skip)
								);

};





} // END_OF_NAMESPACE____






#endif  
