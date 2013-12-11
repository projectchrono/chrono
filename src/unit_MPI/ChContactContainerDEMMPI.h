//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTCONTAINERDEMMPI_H
#define CHCONTACTCONTAINERDEMMPI_H

///////////////////////////////////////////////////
//
//   ChContactContainerDEMMPI.h
//
//   Class for container of many contacts, as CPU
//   typical linked list of ChContactDEM objects
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChApiMPI.h"
#include "physics/ChContactContainerDEM.h"


namespace chrono
{


///
/// Class representing a container of many contacts, 
/// implemented as a typical linked list of ChContactNode
/// objects (contacts between 3DOF nodes and 6DOF bodies)
///

class ChApiMPI ChContactContainerDEMMPI : public ChContactContainerDEM {

	CH_RTTI(ChContactContainerDEMMPI,ChContactContainerDEM);


public:
				//
	  			// CONSTRUCTORS
				//

	ChContactContainerDEMMPI ();

	virtual ~ChContactContainerDEMMPI ();


				//
	  			// FUNCTIONS
				//
	

	virtual void ConstraintsFbLoadForces(double factor);
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif

