//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPCONSTRAINTTWOFRICTIONAPPROX_H
#define CHLCPCONSTRAINTTWOFRICTIONAPPROX_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoFrictionApprox.h
//
//  Class used to represent friction constraint
// between two ChLcpVariable() items.
// Since
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoFriction.h"
#include "ChLcpVariables.h"

namespace chrono
{


///  This class is inherited by the base ChLcpConstraintTwoBodies(),
/// which provides most functionalities.
/// This class implements features to have the tangential constraint
/// limits as functions of the normal contact force.
///  Since in 3D you have two tangential scalar reactions for
/// each single normal reaction, at each contact point you will
/// use a single ChLcpConstraintTwoBodies and a couple of
/// ChLcpConstraintTwoFrictionApprox. All three are 'linked' with
/// pointers, so that the Project() operation on the two
/// friction constraints will project the tangential force onto
/// the admissible Coloumb friction cone, moving parallel to the
/// contact plane (hence this method has NOT the non-extensibility
/// property, and may not assure convergence). 
///  Before starting the LCP solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChLcpConstraintTwoFrictionApprox : public ChLcpConstraintTwoFriction
{
	CH_RTTI(ChLcpConstraintTwoFrictionApprox, ChLcpConstraintTwoFriction)

			//
			// DATA
			//

protected:
				/// the pointer to the normal contact constraint
	ChLcpConstraintTwoBodies* constraint_N;

				/// the pointer to the other tangential component
	ChLcpConstraintTwoFrictionApprox* constraint_Tother;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoFrictionApprox()
					{
						mode = CONSTRAINT_LOCK;
						constraint_N = constraint_Tother= NULL;
					};

						/// Construct and immediately set references to variables,
						/// also setting the  and the normal constraint
						/// other tangential constraint (the latter is mandatory only
						/// for the second of the two tangential constraints)
	ChLcpConstraintTwoFrictionApprox( ChLcpVariablesBody* mvariables_a,
								ChLcpVariablesBody* mvariables_b,
								ChLcpConstraintTwoBodies* aN,
								ChLcpConstraintTwoFrictionApprox* aT = NULL)
				: ChLcpConstraintTwoFriction(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_LOCK;
						constraint_N = aN;
						constraint_Tother= aT;
					};


						/// Copy constructor
	ChLcpConstraintTwoFrictionApprox(const ChLcpConstraintTwoFrictionApprox& other) 
				: ChLcpConstraintTwoFriction(other)
					{
						constraint_N = other.constraint_N;
						constraint_Tother = other.constraint_Tother;
					}

	virtual ~ChLcpConstraintTwoFrictionApprox() {};


	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoFrictionApprox(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoFrictionApprox& operator=(const ChLcpConstraintTwoFrictionApprox& other)
					{
						if (&other == this)
							return *this;

						// copy parent class data
						ChLcpConstraintTwoFriction::operator=(other);

						constraint_N = other.constraint_N;
						constraint_Tother = other.constraint_Tother;
						return *this;
					}


			//
			// FUNCTIONS
			//


				/// Get pointer to the normal contact constraint
	ChLcpConstraintTwoBodies* GetNormalConstraint() {return constraint_N;}
				/// Set pointer to the normal contact constraint
	void SetNormalConstraint(ChLcpConstraintTwoBodies* mconstr) {constraint_N = mconstr;}

				/// Get pointer to the other tangential component
	ChLcpConstraintTwoFrictionApprox* GetOtherTangentialConstraint() {return constraint_Tother;}
				/// Set pointer to the other tangential component
	void SetOtherTangentialConstraint(ChLcpConstraintTwoFrictionApprox* mconstr) {constraint_Tother = mconstr;}



				/// For iterative solvers: project the value of a possible
				/// 'l_i' value of constraint reaction onto admissible set.
				/// Default behavior: if constraint is unilateral and l_i<0, reset l_i=0
				/// but this is not good for our 'friction' tangential constraint, so
				/// we override the base behaviour by projecting in this way:
				///    sqrt(l_ix^2+l_iy^2)< f*l_iz
				/// where l_ix and l_iy are the reactions of the two tangential
				/// constraints, and the l_iz is the reaction of the normal constraint.
	virtual void Project();



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




#endif  // END of ChLcpConstraintTwoFrictionApprox.h
