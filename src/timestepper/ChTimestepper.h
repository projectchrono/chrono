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


/// Base class for 2nd order timesteppers, that is
/// a time integrator for whatever ChIntegrableIIorder
/// (special sub lass of integrable objects that have a state 
/// made with position and velocity y={x,v}, and dy/dt={v,a} 
/// with a=acceleration)

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
		maxiters(10),
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


/// Eulero explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEuleroExpl,
/// but this performes a bit faster because it can exploit
/// the special structure of ChIntegrableIIorder)
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



/// Performs a step of a Heun explicit integrator. It is like
/// a 2nd Runge Kutta.

class ChTimestepperHeun : public ChTimestepperIorder
{
public:
	/// Constructors (default empty)
	ChTimestepperHeun(ChIntegrable& mintegrable)
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
		ChState				y_new(n_y, GetIntegrable());
		ChStateDelta		Dy1(n_dy, GetIntegrable());
		ChStateDelta		Dy2(n_dy, GetIntegrable());
		ChVectorDynamic<>   L(n_c);


		GetIntegrable()->StateSolve(Dy1, L, Y(), T, dt, false); //note, 'false'=no need to update with StateScatter before computation

		y_new = Y() + Dy1;
		GetIntegrable()->StateSolve(Dy2, L, y_new, T + dt,  dt);


		Y() = Y() + (Dy1 + Dy2)*(1. / 2.);   //integrable.StateIncrement(y_new, Y, (Dy1 + Dy2*2.0 + Dy3*2.0 + Dy4)*(1./6.) );
		dYdt() = Dy2*(1. / dt);
		T += dt;

		GetIntegrable()->StateScatter(Y(), T);	// state -> system
	}
};




/// Performs a step of a Leapfrog explicit integrator. 
/// It is a symplectic method, with 2nd order accuracy,
/// at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting
/// the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEuleroSemiImplicit, it gives
/// the same accuracy with a bit of faster performance.

class ChTimestepperLeapfrog : public ChTimestepperIIorder
{
public:
	/// Constructors (default empty)
	ChTimestepperLeapfrog(ChIntegrableIIorder& mintegrable)
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
		ChStateDelta		Aold(A());

		// advance X (uses last A)
		X() = X() + V()*dt + Aold*(0.5*dt*dt);

		// computes new A  (NOTE!!true for imposing a state-> system scatter update,because X changed..)
		mintegrable->StateSolveA(Dv, L, X(), V(), T, dt, true);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt
		
		A() = Dv*(1. / dt);

		// advance V

		V() = V() + (Aold + A())* (0.5*dt);	

		T += dt;

		mintegrable->StateScatter(X(), V(), T);	// state -> system
	}
};


/// Performs a step of Euler implicit for II order systems

class ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChImplicitTimestepper
{
public:
	/// Constructors (default empty)
	ChTimestepperEulerImplicit(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable) ,
		  ChImplicitTimestepper() 
	{};

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
		ChVectorDynamic<>   Dl(mintegrable->GetNconstr());
		ChVectorDynamic<>   L (mintegrable->GetNconstr());

		ChState			Xnew(mintegrable->GetNcoords_x(), mintegrable);
		ChStateDelta	Vnew(mintegrable->GetNcoords_v(), mintegrable);

		// use Euler explicit to extrapolate a prediction 

		mintegrable->StateSolveA(Dv, L, X(), V(), T, dt, false);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		A() = Dv*(1. / dt);

		Xnew = X() + V()*dt;		// x_new= x + v * dt 
		Vnew = V() + Dv;			// v_new= v + a * dt 
		
		// use Newton Raphson iteration to solve implicit Euler for v_new
		//
		// [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f + dt*Cq*l ]
		// [ Cq                           0   ] [ -dt*Dl ] = [ C/dt  ]

		ChVectorDynamic<> R(mintegrable->GetNcoords_v());
		ChVectorDynamic<> Qc(mintegrable->GetNconstr());


		for (int i = 0; i < this->GetMaxiters(); ++i)
		{
			mintegrable->StateScatter(Xnew, Vnew, T+dt);	// state -> system
			R.Reset();
			Qc.Reset();
			mintegrable->LoadResidual_F  (R, dt);
			mintegrable->LoadResidual_Mv (R, (V()-Vnew), 1.0);
			mintegrable->LoadResidual_CqL(R, L, dt);
			mintegrable->LoadConstraint_C(Qc, 1.0/dt);
			
			if (R.NormInf() < this->GetTolerance())
				break;

			mintegrable->StateSolveCorrection(
				Dv, 
				Dl,
				R,
				1.0,  // factor for  M
				-dt,   // factor for  dF/dv
				-dt*dt,// factor for  dF/dx
				Xnew, Vnew, T+dt,
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

			Dl *= -(1.0/dt);
			L += Dl;

			Vnew += Dv;

			Xnew = X() + Vnew *dt;
		}

		X() = Xnew;
		V() = Vnew;
		T += dt;

		mintegrable->StateScatter(X(), V(), T);	// state -> system
	}
};





/// Performs a step of trapezoidal implicit for II order systems

class ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChImplicitTimestepper
{
public:
	/// Constructors (default empty)
	ChTimestepperTrapezoidal(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitTimestepper()
	{};

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
		ChVectorDynamic<>   Dl(mintegrable->GetNconstr());
		ChVectorDynamic<>   L(mintegrable->GetNconstr());

		ChState			Xnew(mintegrable->GetNcoords_x(), mintegrable);
		ChStateDelta	Vnew(mintegrable->GetNcoords_v(), mintegrable);

		// use Euler explicit to extrapolate a prediction 

		mintegrable->StateSolveA(Dv, L, X(), V(), T, dt, false);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		A() = Dv*(1. / dt);

		Xnew = X() + V()*dt;		// x_new= x + v * dt 
		Vnew = V() + Dv;			// v_new= v + a * dt 

		// use Newton Raphson iteration to solve implicit Euler for v_new
		//
		// [ M - dt/2*dF/dv - dt^2/4*dF/dx    Cq' ] [ Dv       ] = [ M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old + Cq*l_new)]
		// [ Cq                               0   ] [ -dt/2*Dl ] = [ C/dt                                                          ]

		ChVectorDynamic<> R   (mintegrable->GetNcoords_v());
		ChVectorDynamic<> Rold(mintegrable->GetNcoords_v());
		ChVectorDynamic<> Qc  (mintegrable->GetNconstr());
		
		mintegrable->LoadResidual_F  (Rold, dt*0.5);    // dt/2*f_old
		mintegrable->LoadResidual_Mv (Rold, V(), 1.0);  // M*v_old
		mintegrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old

		for (int i = 0; i < this->GetMaxiters(); ++i)
		{
			mintegrable->StateScatter(Xnew, Vnew, T + dt);	// state -> system
			R = Rold;
			Qc.Reset();
			mintegrable->LoadResidual_F  (R, dt*0.5);    // + dt/2*f_new
			mintegrable->LoadResidual_Mv (R, Vnew, -1.0); // - M*v_new
			mintegrable->LoadResidual_CqL(R, L, dt*0.5); // + dt/2*Cq*l_new
			mintegrable->LoadConstraint_C(Qc, 1.0 / dt); // C/dt

			if (R.NormInf() < this->GetTolerance())
				break;

			mintegrable->StateSolveCorrection(
				Dv,
				Dl,
				R,
				1.0,        // factor for  M
				-dt*0.5,    // factor for  dF/dv
				-dt*dt*0.25,// factor for  dF/dx
				Xnew, Vnew, T + dt,
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

			Dl *= -(2.0 / dt);
			L += Dl;

			Vnew += Dv;

			Xnew = X() + ((Vnew + V())*(dt*0.5));  // Xnew = Xold + h/2(Vnew+Vold)
		}

		X() = Xnew;
		V() = Vnew;
		T += dt;

		mintegrable->StateScatter(X(), V(), T);	// state -> system
	}
};



/// Performs a step of HHT (generalized alpha) implicit for II order systems

class ChTimestepperHHT : public ChTimestepperIIorder, public ChImplicitTimestepper
{
private:
	double alpha;
	double gamma;
	double beta;
public:
	/// Constructors (default empty)
	ChTimestepperHHT(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitTimestepper()
	{
		SetAlpha(0.0);
	};


		
	/// Set the numerical damping parameter. 
	/// It must be in the [-1/3, 0] interval. 
	/// The closer to -1/3, the more damping.
	/// The closer to 0, the less damping (for 0, it is the trapezoidal method).
	void SetAlpha(double malpha)
	{
		alpha = malpha;
		if (alpha < -1.0 / 3.0)
			alpha = -1.0 / 3.0;
		if (alpha > 0)
			alpha = 0;
		gamma = (1.0 - 2.0* alpha) / 2.0;
		beta = pow ((1.0 - alpha), 2) / 4.0;
	}

	double GetAlpha() { return alpha; }

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
		ChStateDelta		Da(mintegrable->GetNcoords_a(), GetIntegrable());
		ChVectorDynamic<>   Dl(mintegrable->GetNconstr());
		ChVectorDynamic<>   L(mintegrable->GetNconstr());

		ChState			Xnew(mintegrable->GetNcoords_x(), mintegrable);
		ChStateDelta	Vnew(mintegrable->GetNcoords_v(), mintegrable);
		ChStateDelta	Anew(mintegrable->GetNcoords_a(), mintegrable);

		// use Euler explicit to extrapolate a prediction 
		Anew = A();

		mintegrable->StateSolveA(Dv, L, X(), V(), T, dt, false);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		A() = Dv*(1. / dt);
		mintegrable->LoadResidual_F(Rold, -(alpha / (1.0 + alpha)));     // -alpha/(1.0+alpha) * f_old
		mintegrable->LoadResidual_CqL(Rold, L, -(alpha / (1.0 + alpha)));   // -alpha/(1.0+alpha) * Cq'*l_old

		Xnew = X() + V()*dt;		// x_new= x + v * dt 
		Vnew = V() + Dv;			// v_new= v + a * dt 
		//Anew = A();

		// use Newton Raphson iteration to solve implicit Euler for a_new
		//
		// [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Da       ] = [-1/(1+alpha)*M*(a_new) + (f_new +Cq*l_new) + (alpha/(1+alpha))(f_old +Cq*l_old)]
		// [ Cq                                      0   ] [ Dl       ] = [-1/(beta*dt^2)*C                                                                          ]

		ChVectorDynamic<> R(mintegrable->GetNcoords_v());
		ChVectorDynamic<> Rold(mintegrable->GetNcoords_v());
		ChVectorDynamic<> Qc(mintegrable->GetNconstr());


		for (int i = 0; i < this->GetMaxiters(); ++i)
		{
			mintegrable->StateScatter(Xnew, Vnew, T + dt);	// state -> system
			R = Rold;
			Qc.Reset();
			mintegrable->LoadResidual_F  (R, 1.0);        // + f_new
			mintegrable->LoadResidual_Mv (R, Anew, -(1.0/(1.0+alpha)) ); // -1/(1+alpha)*M*a_new
			mintegrable->LoadResidual_CqL(R, L, 1.0);     // + Cq*l_new
			mintegrable->LoadConstraint_C(Qc, -(1.0/(beta*dt*dt)) );  // -1/(beta*dt^2)*C
			GetLog() << "  R=" << R(0) << "    gamma " << gamma<< "  beta " << beta << "\n";
			if (R.NormInf() < this->GetTolerance())
				break;

			mintegrable->StateSolveCorrection(
				Da,
				Dl,
				R,
				1.0,        // factor for  M
				-dt*gamma,  // factor for  dF/dv
				-dt*dt*beta,// factor for  dF/dx
				Xnew, Vnew, T + dt,
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

			L    += Dl;
			Anew += Da;
			
			Xnew = X() + V()*dt + A()*(dt*dt*(0.5 - beta)) + Anew*(dt*dt*beta);

			Vnew = V() + A()*(dt*(1.0 - gamma)) + Anew*(dt*gamma);
		}

		X() = Xnew;
		V() = Vnew;
		A() = Anew;
		T += dt;

		mintegrable->StateScatter(X(), V(), T);	// state -> system
	}
};






} // END_OF_NAMESPACE____
#endif 
