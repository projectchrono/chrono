//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPCONSTRAINTTWOROLLINGN_H
#define CHLCPCONSTRAINTTWOROLLINGN_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoRollingN.h
//
//    An 'easy' derived class for modeling rolling friction.
//   Used with for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoRollingT.h"
#include "ChLcpConstraintTwoContactN.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the ChLcpConstraintTwoBodies(),
/// It is used to represent the rolling friction between two objects
/// ONLY when also two ChLcpConstraintTwoRollingT objects and a 
/// ChLcpConstraintTwoContactN are used to represent contact. 

class ChApi ChLcpConstraintTwoRollingN : public ChLcpConstraintTwoBodies
{
	CH_RTTI(ChLcpConstraintTwoRollingN, ChLcpConstraintTwoBodies)

			//
			// DATA
			//

protected:
				/// the rolling friction coefficient
	float rollingfriction;
	float spinningfriction;

					/// the pointer to U tangential component
	ChLcpConstraintTwoRollingT* constraint_U;
					/// the pointer to V tangential component
	ChLcpConstraintTwoRollingT* constraint_V;
					/// the pointer to normal component
	ChLcpConstraintTwoContactN* constraint_N;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoRollingN() 
					{
						mode = CONSTRAINT_FRIC;
						rollingfriction = 0.0;
						spinningfriction = 0.0;
						constraint_U = constraint_V = 0;
						constraint_N = 0;
					};

						/// Construct and immediately set references to variables,
						/// also setting the U and V tangential friction constraints
	ChLcpConstraintTwoRollingN(ChLcpVariablesBody* mvariables_a, 
							   ChLcpVariablesBody* mvariables_b,
							   ChLcpConstraintTwoRollingT* aU = 0,
							   ChLcpConstraintTwoRollingT* aV = 0,
							   ChLcpConstraintTwoContactN* aN = 0
								)
			: ChLcpConstraintTwoBodies(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC;
						rollingfriction = 0.0;
						spinningfriction = 0.0;
						constraint_U = aU;
						constraint_V = aV;
						constraint_N = aN;
					};

						/// Copy constructor
	ChLcpConstraintTwoRollingN(const ChLcpConstraintTwoRollingN& other) 
			: ChLcpConstraintTwoBodies(other)
					{
						rollingfriction=other.rollingfriction;
						spinningfriction=other.spinningfriction;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
						constraint_N = other.constraint_N;
					}

	virtual ~ChLcpConstraintTwoRollingN()
					{
					};

	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoRollingN(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoRollingN& operator=(const ChLcpConstraintTwoRollingN& other)
					{
						if (&other == this)
							return *this;
						// copy parent class data
						ChLcpConstraintTwoBodies::operator=(other);
						
						rollingfriction=other.rollingfriction;
						spinningfriction=other.spinningfriction;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
						constraint_N = other.constraint_N;
						return *this;
					}



			//
			// FUNCTIONS
			//


				/// Get the rolling friction coefficient
	float GetRollingFrictionCoefficient() {return rollingfriction; }
				/// Set the rolling friction coefficient
	void SetRollingFrictionCoefficient(float mcoeff) {rollingfriction = mcoeff;}

				/// Get the spinning friction coefficient
	float GetSpinningFrictionCoefficient() {return spinningfriction; }
				/// Set the spinning friction coefficient
	void SetSpinningFrictionCoefficient(float mcoeff) {spinningfriction = mcoeff;}

				/// Get pointer to U tangential component
	ChLcpConstraintTwoRollingT* GetRollingConstraintU() {return constraint_U;}			
				/// Get pointer to V tangential component
	ChLcpConstraintTwoRollingT* GetRollingConstraintV() {return constraint_V;}
				/// Get pointer to normal contact component
	ChLcpConstraintTwoContactN* GetNormalConstraint() {return constraint_N;}

				/// Set pointer to U tangential component
	void SetRollingConstraintU(ChLcpConstraintTwoRollingT* mconstr) {constraint_U = mconstr;}
				/// Set pointer to V tangential component
	void SetRollingConstraintV(ChLcpConstraintTwoRollingT* mconstr) {constraint_V = mconstr;}
				/// Set pointer to normal contact component
	void SetNormalConstraint(ChLcpConstraintTwoContactN* mconstr) {constraint_N = mconstr;}


				/// For iterative solvers: project the value of a possible
				/// 'l_i' value of constraint reaction onto admissible set.
				/// This projection will also modify the l_i values of the two
				/// tangential friction constraints (projection onto the friction cone,
				/// as by Anitescu-Tasora theory).
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



#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  
