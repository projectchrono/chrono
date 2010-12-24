#ifndef CHLCPCONSTRAINTTWOFRICTIONT_H
#define CHLCPCONSTRAINTTWOFRICTIONT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoFrictionT.h
//
//  Class used to represent friction constraint
// between two ChLcpVariable() items.
// Since
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoBodies.h"
#include "ChLcpVariables.h"

namespace chrono
{


/// Base class for friction constraints (see specialized children classes
/// for more details - this is mostly an interface)

class ChApi ChLcpConstraintTwoFrictionT : public ChLcpConstraintTwoBodies
{
	CH_RTTI(ChLcpConstraintTwoFrictionT, ChLcpConstraintTwoBodies)

			//
			// DATA
			//

protected:


public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoFrictionT()
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Construct and immediately set references to variables,
						/// also setting the  and the normal constraint
						/// other tangential constraint (the latter is mandatory only
						/// for the second of the two tangential constraints)
	ChLcpConstraintTwoFrictionT( ChLcpVariablesBody* mvariables_a,
								ChLcpVariablesBody* mvariables_b)
				: ChLcpConstraintTwoBodies(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Copy constructor
	ChLcpConstraintTwoFrictionT(const ChLcpConstraintTwoFrictionT& other) 
				: ChLcpConstraintTwoBodies(other)
					{
					}

	virtual ~ChLcpConstraintTwoFrictionT() {};

	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoFrictionT(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoFrictionT& operator=(const ChLcpConstraintTwoFrictionT& other)
					{
						if (&other == this)
							return *this;

						// copy parent class data
						ChLcpConstraintTwoBodies::operator=(other);

						return *this;
					}


			//
			// FUNCTIONS
			//

					/// Tells that this constraint is not linear, that is: it cannot
					/// be solved with a plain simplex solver.
	virtual bool IsLinear() {return false;}

					/// The constraint is satisfied?
	virtual double Violation(double mc_i);

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);
};




} // END_OF_NAMESPACE____




#endif  // END of ChLcpConstraintTwoFrictionT.h
