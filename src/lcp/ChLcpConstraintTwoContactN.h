//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPCONSTRAINTTWOCONTACTN_H
#define CHLCPCONSTRAINTTWOCONTACTN_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoContactN.h
//
//    An 'easy' derived class for representing a
//   constraint between two ChLcpVariable items.
//   Used with LCP systems including inequalities,
//   equalities, nonlinearities, etc.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoFrictionT.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the ChLcpConstraintTwoBodies(),
/// It is used to represent the normal reaction between two objects
/// ONLY when also two ChLcpConstraintTwoFrictionT objects are
/// used to represent friction. (If these two tangent constraint
/// are not used, for frictionless case, please use a simple ChConstraintTwo
/// with the CONSTRAINT_UNILATERAL mode.) 
/// Differently from an unilateral constraint, this does not enforce
/// projection on positive constraint, since it will be up to the 'companion'
/// ChLcpConstraintTwoFriction objects to call a projection on the cone, by
/// modifying all the three components (normal, u, v) at once.

class ChApi ChLcpConstraintTwoContactN : public ChLcpConstraintTwoBodies
{
	CH_RTTI(ChLcpConstraintTwoContactN, ChLcpConstraintTwoBodies)

			//
			// DATA
			//

protected:
				/// the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
	float friction;
				/// the cohesion 'c', positive, if any, for  sqrt(Tx^2+Ty^2)<f*(Nz+c)
	float cohesion;

					/// the pointer to U tangential component
	ChLcpConstraintTwoFrictionT* constraint_U;
					/// the pointer to V tangential component
	ChLcpConstraintTwoFrictionT* constraint_V;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoContactN() 
					{
						mode = CONSTRAINT_FRIC;
						friction = 0.0;
						cohesion = 0.0;
						constraint_U = constraint_V = 0;
					};

						/// Construct and immediately set references to variables,
						/// also setting the U and V tangential friction constraints
	ChLcpConstraintTwoContactN(ChLcpVariablesBody* mvariables_a, 
							   ChLcpVariablesBody* mvariables_b,
							   ChLcpConstraintTwoFrictionT* aU = 0,
							   ChLcpConstraintTwoFrictionT* aV = 0
								)
			: ChLcpConstraintTwoBodies(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC;
						friction=0.0;
						cohesion=0.0;
						constraint_U = aU;
						constraint_V = aV;
					};

						/// Copy constructor
	ChLcpConstraintTwoContactN(const ChLcpConstraintTwoContactN& other) 
			: ChLcpConstraintTwoBodies(other)
					{
						friction=other.friction;
						cohesion=other.cohesion;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
					}

	virtual ~ChLcpConstraintTwoContactN()
					{
					};

	virtual ChLcpConstraintTwoContactN* new_Duplicate () {return new ChLcpConstraintTwoContactN(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoContactN& operator=(const ChLcpConstraintTwoContactN& other)
					{
						if (&other == this)
							return *this;
						// copy parent class data
						ChLcpConstraintTwoBodies::operator=(other);
						
						friction = other.friction;
						cohesion = other.cohesion;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
						return *this;
					}



			//
			// FUNCTIONS
			//


				/// Get the friction coefficient
	float GetFrictionCoefficient() {return friction; }
				/// Set the friction coefficient
	void SetFrictionCoefficient(float mcoeff) {friction = mcoeff;}

				/// Get the cohesion
	float GetCohesion() {return cohesion; }
				/// Set the cohesion
	void SetCohesion(float mcoh) {cohesion = mcoh;}


				/// Get pointer to U tangential component
	ChLcpConstraintTwoFrictionT* GetTangentialConstraintU() {return constraint_U;}			
				/// Get pointer to V tangential component
	ChLcpConstraintTwoFrictionT* GetTangentialConstraintV() {return constraint_V;}

				/// Set pointer to U tangential component
	void SetTangentialConstraintU(ChLcpConstraintTwoFrictionT* mconstr) {constraint_U = mconstr;}
				/// Set pointer to V tangential component
	void SetTangentialConstraintV(ChLcpConstraintTwoFrictionT* mconstr) {constraint_V = mconstr;}


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
