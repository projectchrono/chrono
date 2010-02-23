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

};





} // END_OF_NAMESPACE____






#endif  
