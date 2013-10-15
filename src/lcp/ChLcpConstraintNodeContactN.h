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

#ifndef CHLCPCONSTRAINTNODECONTACTN_H
#define CHLCPCONSTRAINTNODECONTACTN_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintNodeContactN.h
//
//    An 'easy' derived class for representing a
//   constraint between two ChLcpVariable items.
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



#include "ChLcpConstraintNodeFrictionT.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


/// It is used to represent the normal reaction between a 3DOF node
/// and a 6 DOF body, only when also two ChLcpConstraintNodeFrictionT 
/// objects are used to represent friction. 

class ChApi ChLcpConstraintNodeContactN : public ChLcpConstraintTwoGeneric
{
	CH_RTTI(ChLcpConstraintNodeContactN, ChLcpConstraintTwoGeneric)

			//
			// DATA
			//

protected:
				/// the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
	float friction;

					/// the pointer to U tangential component
	ChLcpConstraintNodeFrictionT* constraint_U;
					/// the pointer to V tangential component
	ChLcpConstraintNodeFrictionT* constraint_V;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintNodeContactN() 
					{
						mode = CONSTRAINT_FRIC;
						friction = 0.0;
						constraint_U = constraint_V = 0;
					};

						/// Construct and immediately set references to variables,
						/// also setting the U and V tangential friction constraints
	ChLcpConstraintNodeContactN(ChLcpVariablesBody* mvariables_a, 
							    ChLcpVariablesNode* mvariables_b,
							   ChLcpConstraintNodeFrictionT* aU = 0,
							   ChLcpConstraintNodeFrictionT* aV = 0
								)
			: ChLcpConstraintTwoGeneric(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC;
						friction=0.0;
						constraint_U = aU;
						constraint_V = aV;
					};

						/// Copy constructor
	ChLcpConstraintNodeContactN(const ChLcpConstraintNodeContactN& other) 
			: ChLcpConstraintTwoGeneric(other)
					{
						friction=other.friction;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
					}

	virtual ~ChLcpConstraintNodeContactN()
					{
					};

	virtual ChLcpConstraintNodeContactN* new_Duplicate () {return new ChLcpConstraintNodeContactN(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintNodeContactN& operator=(const ChLcpConstraintNodeContactN& other)
					{
						if (&other == this)
							return *this;
						// copy parent class data
						ChLcpConstraintTwoGeneric::operator=(other);
						
						friction = other.friction;
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


				/// Get pointer to U tangential component
	ChLcpConstraintNodeFrictionT* GetTangentialConstraintU() {return constraint_U;}			
				/// Get pointer to V tangential component
	ChLcpConstraintNodeFrictionT* GetTangentialConstraintV() {return constraint_V;}

				/// Set pointer to U tangential component
	void SetTangentialConstraintU(ChLcpConstraintNodeFrictionT* mconstr) {constraint_U = mconstr;}
				/// Set pointer to V tangential component
	void SetTangentialConstraintV(ChLcpConstraintNodeFrictionT* mconstr) {constraint_V = mconstr;}


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
