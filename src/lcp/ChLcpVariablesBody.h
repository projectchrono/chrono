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

#ifndef CHLCPVARIABLESBODY_H
#define CHLCPVARIABLESBODY_H

//////////////////////////////////////////////////
//
//   ChLcpVariablesBody.h
//
//    Specialized class for representing a mass matrix
//   and associate variables (6 element vector, ex.speed)
//   for a 3D rigid body.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpVariables.h"
#include "core/ChMatrix33.h"

namespace chrono
{

///    Specialized class for representing a 6-DOF item for a
///   LCP system, that is a 3D rigid body, with mass matrix and
///   associate variables (a 6 element vector, ex.speed)
///    This is an abstract class, specialized for example in 
///   ChLcpVariablesBodyOwnMass and ChLcpVariablesBodySharedMass.

class ChApi ChLcpVariablesBody :  public ChLcpVariables
{
	CH_RTTI(ChLcpVariablesBody, ChLcpVariables)

private:
			//
			// DATA		//


			void* user_data;

public:

			//
			// CONSTRUCTORS
			//

	ChLcpVariablesBody() : ChLcpVariables(6)
				{
					user_data = 0;
				};

	virtual ~ChLcpVariablesBody()
				{
				};


				/// Assignment operator: copy from other object
	ChLcpVariablesBody& operator=(const ChLcpVariablesBody& other);


			//
			// FUNCTIONS
			//

				/// Get the mass associated with translation of body
	virtual double	GetBodyMass() =0;

				/// Access the 3x3 inertia matrix
	virtual ChMatrix33<>& GetBodyInertia() =0;

				/// Access the 3x3 inertia matrix inverted
	virtual ChMatrix33<>& GetBodyInvInertia() =0;



				// IMPLEMENT PARENT CLASS METHODS


				/// The number of scalar variables in the vector qb
				/// (dof=degrees of freedom)
	virtual int Get_ndof() const {return 6;}


	virtual void* GetUserData() {return this->user_data;}
	virtual void SetUserData(void* mdata) {this->user_data = mdata;}

};




} // END_OF_NAMESPACE____




#endif  // END of ChLcpVariablesBody.h
