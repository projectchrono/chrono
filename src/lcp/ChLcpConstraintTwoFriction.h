#ifndef CHLCPCONSTRAINTTWOFRICTION_H
#define CHLCPCONSTRAINTTWOFRICTION_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoFriction.h
//
//  Class used to represent friction constraint
// between two ChLcpVariable() items.
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

class ChLcpConstraintTwoFriction : public ChLcpConstraintTwoBodies
{
	CH_RTTI(ChLcpConstraintTwoFriction, ChLcpConstraintTwoBodies)

			//
			// DATA
			//

protected:
				/// the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
	double friction;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoFriction()
					{
						friction=0.0;
					};

						/// Construct and immediately set references to variables,
						/// also setting the  and the normal constraint
						/// other tangential constraint (the latter is mandatory only
						/// for the second of the two tangential constraints)
	ChLcpConstraintTwoFriction( ChLcpVariablesBody* mvariables_a,
								ChLcpVariablesBody* mvariables_b)
				: ChLcpConstraintTwoBodies(mvariables_a, mvariables_b)
					{
						friction=0.0;
					};

						/// Copy constructor
	ChLcpConstraintTwoFriction(const ChLcpConstraintTwoFriction& other) 
				: ChLcpConstraintTwoBodies(other)
					{
						friction = other.friction;
					}

	virtual ~ChLcpConstraintTwoFriction() {};

	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoFriction(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoFriction& operator=(const ChLcpConstraintTwoFriction& other)
					{
						if (&other == this)
							return *this;

						// copy parent class data
						ChLcpConstraintTwoBodies::operator=(other);

						friction = other.friction;
						return *this;
					}


			//
			// FUNCTIONS
			//

					/// Tells that this constraint is not linear, that is: it cannot
					/// be solved with a plain simplex solver.
	virtual bool IsLinear() {return false;}

				/// Get the friction coefficient
	double GetFrictionCoefficient() {return friction; }
				/// Set the friction coefficient
	void SetFrictionCoefficient(double mcoeff) {friction = mcoeff;}


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




#endif  // END of ChLcpConstraintTwoFriction.h
