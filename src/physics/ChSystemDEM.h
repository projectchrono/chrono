//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSYSTEMDEM_H
#define CHSYSTEMDEM_H

//////////////////////////////////////////////////
//
//   ChSystemDEM.h
//
//   Class for a physical system in which contact
//   is modeled using a Penalty Method (aka DEM)
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChSystem.h"


namespace chrono
{


class ChApi ChSystemDEM : public ChSystem
{

	CH_RTTI(ChSystemDEM,ChSystem);

public:
			/// Constructor
			/// Note, in case you will use collision detection, the values of
			/// 'max_objects' and 'scene_size' can be used to initialize the broadphase
			/// collision algorithm in an optimal way. Scene size should be approximately 
			/// the radius of the expected area where colliding objects will move.
	ChSystemDEM(unsigned int max_objects = 16000, double scene_size = 500);

			/// Destructor
	~ChSystemDEM() {}

	virtual void SetLcpSolverType(eCh_lcpSolver mval);
	virtual void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);
	virtual void ChangeContactContainer(ChContactContainerBase* newcontainer);
};


} // END_OF_NAMESPACE____


#endif
