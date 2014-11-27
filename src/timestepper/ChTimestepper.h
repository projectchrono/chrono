//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTIMESTEPPER_H
#define CHTIMESTEPPER_H

#include <stdlib.h>
#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "core/ChShared.h"
#include "core/ChVectorDynamic.h"
#include "timestepper/ChState.h"
#include "timestepper/ChIntegrable.h"


namespace chrono 
{



/// Base class for timesteppers, that is
/// a time integrator which can advance a system state.
/// It operates on systems inherited from ChIntegrable.

class ChTimestepper : public ChShared
{
protected:
	ChIntegrable* integrable;
	double T;

	//ChVectorDynamic<>* mL;
public:
					/// Constructor
	ChTimestepper(ChIntegrable& mintegrable) 
				{
					integrable = &mintegrable;
					T = 0;
					//mL    = new ChVectorDynamic<>(1);
				};
	
					/// Destructor
	virtual ~ChTimestepper()
				{
					//delete mL;
				};

					/// Performs an integration timestep
	virtual void Advance(
				const double dt				///< timestep to advance
				) =0;


					/// Access the lagrangian multipliers, if any
	//virtual ChVectorDynamic<>& L() { return *mL; }

					/// Get the integrable object 
	ChIntegrable* GetIntegrable() { return integrable;}
					
					/// Get the current time 
	virtual double GetTime() { return T;}

					/// Set the current time 
	virtual void SetTime(double mt) { T = mt;}
};



/// Base class for 1st order timesteppers, that is
/// a time integrator for whatever ChIntegrable.

class ChTimestepperIorder : public ChTimestepper
{
protected:
	ChState* mY;
	ChStateDelta* mdYdt;
public:

	/// Constructor
	ChTimestepperIorder(ChIntegrable& mintegrable) 
		: ChTimestepper(mintegrable)
	{
		mY = new ChState(&mintegrable);
		mdYdt = new ChStateDelta(&mintegrable);
	};

	/// Destructor
	virtual ~ChTimestepperIorder()
	{
		delete mY;
		delete mdYdt;
	};

	/// Access the state at current time
	virtual ChState& Y() { return *mY; }

	/// Access the derivative of state at current time
	virtual ChStateDelta& dYdt() { return *mdYdt; }
};


/// Base class for 1st order timesteppers, that is
/// a time integrator for whatever ChIntegrable.

class ChTimestepperIIorder : public ChTimestepper
{
protected:
	ChState* mX;
	ChStateDelta* mV;
	ChStateDelta* mA;
public:

	/// Constructor
	ChTimestepperIIorder(ChIntegrableIIorder& mintegrable)
		: ChTimestepper(mintegrable)
	{
		mX = new ChState(&mintegrable);
		mV = new ChStateDelta(&mintegrable);
		mA = new ChStateDelta(&mintegrable);
	};

	/// Destructor
	virtual ~ChTimestepperIIorder()
	{
		delete mX;
		delete mV;
		delete mA;
	};

	/// Access the state, position part, at current time
	virtual ChState& X() { return *mX; }

	/// Access the state, speed part, at current time
	virtual ChStateDelta& V() { return *mV; }

	/// Access the acceleration, at current time
	virtual ChStateDelta& A() { return *mA; }
};



/// Base properties for implicit solvers (double inheritance)
class ChImplicitTimestepper 
{
private:
	int maxiters;
	double tolerance;

public:
					/// Constructors 
	ChImplicitTimestepper() :
		maxiters(20),
		tolerance(1e-10)
	{}

		/// Set the max number of iterations using the Newton Raphson procedure
	void   SetMaxiters(int miters) { maxiters = miters; }
		/// Get the max number of iterations using the Newton Raphson procedure
	double GetMaxiters() { return maxiters;}
	
		/// Set the tolerance for terminating the Newton Raphson procedure
	void   SetTolerance(double mtol) { tolerance = mtol; }
		/// Get the tolerance for terminating the Newton Raphson procedure
	double GetTolerance() { return tolerance; }
};




/// Eulero explicit timestepper
/// This performs the typical  y_new = y+ dy/dt * dt 
/// integration with Eulero formula. 

class ChTimestepperEuleroExpl : public ChTimestepperIorder
{
public:
					/// Constructors (default empty)
	ChTimestepperEuleroExpl(ChIntegrable& mintegrable) 
		: ChTimestepperIorder(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			) 
	{
		GetIntegrable()->StateSetup(Y(), dYdt());

		GetIntegrable()->StateGather(Y(), T);	// state <- system

		// auxiliary vectors
		ChStateDelta		Dy(this->GetIntegrable()->GetNcoords_dy(), GetIntegrable());
		ChVectorDynamic<>   L (this->GetIntegrable()->GetNconstr());

		GetIntegrable()->StateSolve(Dy, L, Y(), T, dt, false);	// dY/dt = f(Y,T)   dY = f(Y,T)*dt

		// Euler formula!  
		//   y_new= y + dy/dt * dt    =  y_new= y + Dy

		Y()		= Y() + Dy;		//  also: GetIntegrable().StateIncrement(y_new, y, Dy);
		dYdt()	= Dy*(1./dt); 
		T		+= dt;

		GetIntegrable()->StateScatter(Y(), T);	// state -> system
	}
};


/// Eulero explicit timestepper customized for II order
/// This performs the typical 
///    x_new = x + v * dt 
///    v_new = v + a * dt 
/// integration with Eulero formula. 

class ChTimestepperEuleroExplIIorder : public ChTimestepperIIorder
{
public:
	/// Constructors (default empty)
	ChTimestepperEuleroExplIIorder(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable)  {};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// downcast
		ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

		mintegrable->StateSetup(X(), V(), A());

		mintegrable->StateGather(X(), V(), T);	// state <- system

		// auxiliary vectors
		ChStateDelta		Dv(mintegrable->GetNcoords_v(), GetIntegrable());
		ChVectorDynamic<>   L(mintegrable->GetNconstr());

		mintegrable->StateSolveA(Dv, L, X(), V(), T, dt, false);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		// Euler formula!  

		A() = Dv*(1. / dt);

		X() = X() + V()*dt;		// x_new= x + v * dt 

		V() = V() + Dv;			// v_new= v + a * dt 
		
		T += dt;

		mintegrable->StateScatter(X(), V(), T);	// state -> system
	}
};



/// Eulero semi-implicit timestepper
/// This performs the typical 
///    v_new = v + a * dt 
///    x_new = x + v_new * dt 
/// integration with Eulero semi-implicit formula. 

class ChTimestepperEuleroSemiImplicit : public ChTimestepperIIorder
{
public:
	/// Constructors (default empty)
	ChTimestepperEuleroSemiImplicit(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable)  {};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// downcast
		ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

		mintegrable->StateSetup(X(), V(), A());

		mintegrable->StateGather(X(), V(), T);	// state <- system

		// auxiliary vectors
		ChStateDelta		Dv(mintegrable->GetNcoords_v(), GetIntegrable());
		ChVectorDynamic<>   L(mintegrable->GetNconstr());

		mintegrable->StateSolveA(Dv, L, X(), V(), T, dt, false);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		// Semi-implicit Euler formula!   (note the order of update of x and v, respect to original Euler II order explicit)

		A() = Dv*(1. / dt);

		V() = V() + Dv;			// v_new= v + a * dt 

		X() = X() + V()*dt;		// x_new= x + v_new * dt 


		T += dt;

		mintegrable->StateScatter(X(), V(), T);	// state -> system
	}
};




/// Performs a step of a 4th order explicit Runge-Kutta
/// integration scheme.

class ChTimestepperRungeKuttaExpl : public ChTimestepperIorder
{
public:
					/// Constructors (default empty)
	ChTimestepperRungeKuttaExpl (ChIntegrable& mintegrable) 
		: ChTimestepperIorder(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			) 
	{
		GetIntegrable()->StateSetup(Y(), dYdt());

		GetIntegrable()->StateGather(Y(), T);	// state <- system

		// auxiliary vectors
		int n_y  = GetIntegrable()->GetNcoords_y();
		int n_dy = GetIntegrable()->GetNcoords_dy();
		int n_c  = GetIntegrable()->GetNconstr();
		ChState				y_new(n_y,  GetIntegrable());
		ChStateDelta		Dy1	 (n_dy, GetIntegrable());
		ChStateDelta		Dy2	 (n_dy, GetIntegrable());
		ChStateDelta		Dy3  (n_dy, GetIntegrable());
		ChStateDelta		Dy4	 (n_dy, GetIntegrable());
		ChVectorDynamic<>   L    (n_c);


		GetIntegrable()->StateSolve(Dy1, L, Y(), T, dt, false); //note, 'false'=no need to update with StateScatter before computation

		y_new = Y() + Dy1*0.5;	//integrable.StateIncrement(y_new, Y, Dy1*0.5);
		GetIntegrable()->StateSolve(Dy2, L, y_new, T+dt*0.5, dt);

		y_new = Y() + Dy2*0.5;	//integrable.StateIncrement(y_new, Y, Dy2*0.5);
		GetIntegrable()->StateSolve(Dy3, L, y_new, T+dt*0.5, dt);

		y_new = Y() + Dy3;		//integrable.StateIncrement(y_new, Y, Dy3);
		GetIntegrable()->StateSolve(Dy4, L, y_new, T+dt, dt);

		Y()		= Y() + (Dy1 + Dy2*2.0 + Dy3*2.0 + Dy4)*(1./6.);   //integrable.StateIncrement(y_new, Y, (Dy1 + Dy2*2.0 + Dy3*2.0 + Dy4)*(1./6.) );
		dYdt()	= Dy4*(1./dt);
		T		+= dt;

		GetIntegrable()->StateScatter(Y(), T);	// state -> system
	}
};










} // END_OF_NAMESPACE____
#endif 
