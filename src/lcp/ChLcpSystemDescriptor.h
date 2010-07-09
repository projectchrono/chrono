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


class ChLcpSystemDescriptor
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

				/// The following function may be used to create the full Jacobian and the full
				/// mass matrix of the complementarity problem in matrix form, by assembling all 
				/// the jacobians of all the constraints/contacts and all the mass matrices. This
				/// can be useful for debugging, data dumping, and similar purposes.
				/// Optionally, tangential (u,v) contact jacobians may be skipped, etc.
				/// The sparse matrix is automatically resized if needed.
	virtual void BuildMatrices (ChSparseMatrix* Cq, ///< fill this system jacobian matrix, if not null
								ChSparseMatrix* M,	///< fill this system mass matrix, if not null
								bool only_bilaterals = false, 
								bool skip_contacts_uv = false);

				/// Using this function, one may get a vector with all the variables 'q'
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length.
				/// \return  the number of scalar variables (i.e. the rows of the column vector).
	virtual int FromVariablesToVector(
								ChMatrix<>& mvector						///< matrix which will contain the entire vector of 'q'
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



};





} // END_OF_NAMESPACE____






#endif  
