//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHINTEGRATOR_H
#define CHINTEGRATOR_H

//////////////////////////////////////////////////
//
//   ChIntegrator.h
//
//   Math functions for :
//      - ORDINARY DIFFERENTIAL EQUATIONS
//
//     ***OBSOLETE*** to be improved/rewritten.
//                    Currently not used by ChSystem.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "core/ChMath.h"
#include "physics/ChObject.h"

namespace chrono
{



//#define INTEG_EULERO_EXP		0
#define INTEG_KUTTA_EXP		1
#define INTEG_RK_EXP		2
#define INTEG_EULEROSTD_EXP	3
#define INTEG_EULEROMOD_EXP	4
#define INTEG_HEUN_EXP		5


////////////////////////////////////////////////
// the 'integrator' object



/// Class for methods which perform numerical integration of ODEs
/// (Ordinary Differential Equations).
/// ***OBSOLETE*** DO NOT USE

class ChApi ChIntegrator : public ChObj {

private:

public:
	// ------ DATA

	int method;	// Integration scheme, see methods code above


	// ------ FUNCTIONS

	ChIntegrator();
	virtual ~ChIntegrator();
	virtual void Copy(ChIntegrator* source);




				// Generic ODE integrator, performs a single step of integration
				// Returns true if ok, false if something's wrong.

	int ODEintegrate_step  (
					int n_eq,				// number of equations (must be equal to rows of y0, m_y, m_ydtdt);
					ChMatrix<>* Y,				// the current state (also will contain resulting state after integration)
					double& t,				// the starting time (also will contain final time ater integration)
											// function which computes dy/dt=f(y,t), returning TRUE if OK, FALSE if error
					int getdy(ChMatrix<>* m_y, double m_t, ChMatrix<>* m_dydt, void* m_userdata),
					double h,				// step length.
					double& error,			// if possible, returns here the local error extimation
					void* userdata);		// generic pointer to pass aux. data to the getdy function
};



} // END_OF_NAMESPACE____

#endif
