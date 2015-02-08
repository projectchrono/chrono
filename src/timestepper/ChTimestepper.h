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

	ChVectorDynamic<> L;

public:
					/// Constructor
	ChTimestepper(ChIntegrable& mintegrable) 
				{
					integrable = &mintegrable;
					T = 0;
					L.Reset(0);
				};
	
					/// Destructor
	virtual ~ChTimestepper()
				{
				};

					/// Performs an integration timestep
	virtual void Advance(
				const double dt				///< timestep to advance
				) =0;


					/// Access the lagrangian multipliers, if any
	virtual ChVectorDynamic<>& get_L() { return L; }

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
	ChState Y;
	ChStateDelta dYdt;

public:

	/// Constructor
	ChTimestepperIorder(ChIntegrable& mintegrable) 
		: ChTimestepper(mintegrable)
	{
		Y.Reset    (1, &mintegrable);
		dYdt.Reset (1, &mintegrable);
	};

	/// Destructor
	virtual ~ChTimestepperIorder()
	{
	};

	/// Access the state at current time
	virtual ChState& get_Y() { return Y; }

	/// Access the derivative of state at current time
	virtual ChStateDelta& get_dYdt() { return dYdt; }
};


/// Base class for 2nd order timesteppers, that is
/// a time integrator for whatever ChIntegrableIIorder
/// (special sub lass of integrable objects that have a state 
/// made with position and velocity y={x,v}, and dy/dt={v,a} 
/// with a=acceleration)

class ChTimestepperIIorder : public ChTimestepper
{
protected:
	ChState X;
	ChStateDelta V;
	ChStateDelta A;

public:

	/// Constructor
	ChTimestepperIIorder(ChIntegrableIIorder& mintegrable)
		: ChTimestepper(mintegrable)
	{
		X.Reset(1, &mintegrable);
		V.Reset(1, &mintegrable);
		A.Reset(1, &mintegrable);
	};

	/// Destructor
	virtual ~ChTimestepperIIorder()
	{
	};

	/// Access the state, position part, at current time
	virtual ChState& get_X() { return X; }

	/// Access the state, speed part, at current time
	virtual ChStateDelta& get_V() { return V; }

	/// Access the acceleration, at current time
	virtual ChStateDelta& get_A() { return A; }
};



/// Base properties for implicit solvers (double inheritance)
class ChImplicitTimestepper 
{
};


/// Base properties for implicit solvers that compute the solution by iterative
/// process up to a desired tolerance
class ChImplicitIterativeTimestepper  : public ChImplicitTimestepper
{
private:
	int maxiters;
	double tolerance;

public:
					/// Constructors 
	ChImplicitIterativeTimestepper() :
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




/// Euler explicit timestepper
/// This performs the typical  y_new = y+ dy/dt * dt 
/// integration with Euler formula. 

class ChTimestepperEulerExpl : public ChTimestepperIorder
{
public:
					/// Constructors (default empty)
	ChTimestepperEulerExpl(ChIntegrable& mintegrable) 
		: ChTimestepperIorder(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			) 
	{
		// setup main vectors
		GetIntegrable()->StateSetup(Y, dYdt);

		// setup auxiliary vectors
		L.Reset(this->GetIntegrable()->GetNconstr());


		GetIntegrable()->StateGather(Y, T);	// state <- system

		
		GetIntegrable()->StateSolve(dYdt, L, Y, T, dt, false);	// dY/dt = f(Y,T)

		// Euler formula!  
		//   y_new= y + dy/dt * dt    

		Y		= Y + dYdt * dt;		//  also: GetIntegrable().StateIncrement(y_new, y, Dy);

		T		+= dt;

		GetIntegrable()->StateScatter(Y, T);	// state -> system
	}
};


/// Euler explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEulerExpl,
/// but this performes a bit faster because it can exploit
/// the special structure of ChIntegrableIIorder)
/// This performs the typical 
///    x_new = x + v * dt 
///    v_new = v + a * dt 
/// integration with Euler formula. 

class ChTimestepperEulerExplIIorder : public ChTimestepperIIorder
{
protected:
	ChStateDelta		Dv;

public:
	/// Constructors (default empty)
	ChTimestepperEulerExplIIorder(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable)  {};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// downcast
		ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		// setup auxiliary vectors
		Dv.Reset	(mintegrable->GetNcoords_v(), GetIntegrable());
		L.Reset		(mintegrable->GetNconstr());


		mintegrable->StateGather(X, V, T);	// state <- system

		mintegrable->StateSolveA( A, L, X, V, T, dt, false);	// Dv/dt = f(x,v,T)

		// Euler formula!  

		X = X + V*dt;		// x_new= x + v * dt 

		V = V + A*dt;		// v_new= v + a * dt 
		
		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};



/// Euler semi-implicit timestepper
/// This performs the typical 
///    v_new = v + a * dt 
///    x_new = x + v_new * dt 
/// integration with Euler semi-implicit formula. 

class ChTimestepperEulerSemiImplicit : public ChTimestepperIIorder
{
public:
	/// Constructors (default empty)
	ChTimestepperEulerSemiImplicit(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable)  {};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// downcast
		ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		// setup auxiliary vectors
		L.Reset(mintegrable->GetNconstr());


		mintegrable->StateGather(X, V, T);	// state <- system

		mintegrable->StateSolveA( A, L, X, V, T, dt, false);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		// Semi-implicit Euler formula!   (note the order of update of x and v, respect to original Euler II order explicit)

		V = V + A*dt;		// v_new= v + a * dt 

		X = X + V*dt;		// x_new= x + v_new * dt 

		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};




/// Performs a step of a 4th order explicit Runge-Kutta
/// integration scheme.

class ChTimestepperRungeKuttaExpl : public ChTimestepperIorder
{
protected:
	ChState				y_new;
	ChStateDelta		Dydt1;
	ChStateDelta		Dydt2;
	ChStateDelta		Dydt3;
	ChStateDelta		Dydt4;

public:
					/// Constructors (default empty)
	ChTimestepperRungeKuttaExpl (ChIntegrable& mintegrable) 
		: ChTimestepperIorder(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			) 
	{
		// setup main vectors
		GetIntegrable()->StateSetup(Y, dYdt);

		// setup auxiliary vectors
		int n_y  = GetIntegrable()->GetNcoords_y();
		int n_dy = GetIntegrable()->GetNcoords_dy();
		int n_c  = GetIntegrable()->GetNconstr();
		y_new.Reset(n_y, GetIntegrable());
		Dydt1.Reset(n_dy, GetIntegrable());
		Dydt2.Reset(n_dy, GetIntegrable());
		Dydt3.Reset(n_dy, GetIntegrable());
		Dydt4.Reset(n_dy, GetIntegrable());
		L.Reset(n_c);


		GetIntegrable()->StateGather(Y, T);	// state <- system

		GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt, false); //note, 'false'=no need to update with StateScatter before computation

		y_new = Y + Dydt1*0.5*dt;	//integrable.StateIncrement(y_new, Y, Dydt1*0.5*dt);
		GetIntegrable()->StateSolve(Dydt2, L, y_new, T+dt*0.5, dt);

		y_new = Y + Dydt2*0.5*dt;	//integrable.StateIncrement(y_new, Y, Dydt2*0.5*dt);
		GetIntegrable()->StateSolve(Dydt3, L, y_new, T+dt*0.5, dt);

		y_new = Y + Dydt3*dt;		//integrable.StateIncrement(y_new, Y, Dydt3*dt);
		GetIntegrable()->StateSolve(Dydt4, L, y_new, T+dt, dt);

		Y		= Y + (Dydt1 + Dydt2*2.0 + Dydt3*2.0 + Dydt4)*(1./6.)*dt;   //integrable.StateIncrement(...);
		dYdt	= Dydt4; // to check
		T		+= dt;

		GetIntegrable()->StateScatter(Y, T);	// state -> system
	}
};



/// Performs a step of a Heun explicit integrator. It is like
/// a 2nd Runge Kutta.

class ChTimestepperHeun : public ChTimestepperIorder
{
protected:
	ChState				y_new;
	ChStateDelta		Dydt1;
	ChStateDelta		Dydt2;

public:
	/// Constructors (default empty)
	ChTimestepperHeun(ChIntegrable& mintegrable)
		: ChTimestepperIorder(mintegrable)  {};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// setup main vectors
		GetIntegrable()->StateSetup(Y, dYdt);

		// setup auxiliary vectors
		int n_y = GetIntegrable()->GetNcoords_y();
		int n_dy = GetIntegrable()->GetNcoords_dy();
		int n_c = GetIntegrable()->GetNconstr();
		y_new.Reset(n_y, GetIntegrable());
		Dydt1.Reset(n_dy, GetIntegrable());
		Dydt2.Reset(n_dy, GetIntegrable());
		L.Reset(n_c);


		GetIntegrable()->StateGather(Y, T);	// state <- system

		GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt, false); //note, 'false'=no need to update with StateScatter before computation

		y_new = Y + Dydt1*dt;
		GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt,  dt);


		Y = Y + (Dydt1 + Dydt2)*(dt / 2.);   
		dYdt = Dydt2;
		T += dt;

		GetIntegrable()->StateScatter(Y, T);	// state -> system
	}
};




/// Performs a step of a Leapfrog explicit integrator. 
/// It is a symplectic method, with 2nd order accuracy,
/// at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting
/// the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives
/// the same accuracy with a bit of faster performance.

class ChTimestepperLeapfrog : public ChTimestepperIIorder
{
protected:
	ChStateDelta		Aold;

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

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		mintegrable->StateGather(X, V, T);	// state <- system

		// setup auxiliary vectors
		L.Reset(mintegrable->GetNconstr());
		Aold = A;

		// advance X (uses last A)
		X = X + V*dt + Aold*(0.5*dt*dt);

		// computes new A  (NOTE!!true for imposing a state-> system scatter update,because X changed..)
		mintegrable->StateSolveA( A, L, X, V, T, dt, true);	// Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

		// advance V

		V = V + (Aold + A)* (0.5*dt);	

		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};


/// Performs a step of Euler implicit for II order systems

class ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
{
protected:
	ChStateDelta		Dv;
	ChVectorDynamic<>   Dl;
	ChState				Xnew;
	ChStateDelta		Vnew;
	ChVectorDynamic<>   R;
	ChVectorDynamic<>   Qc;

public:
	/// Constructors (default empty)
	ChTimestepperEulerImplicit(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable) ,
		  ChImplicitIterativeTimestepper() 
	{};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// downcast
		ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		// setup auxiliary vectors
		Dv.Reset  (mintegrable->GetNcoords_v(), GetIntegrable());
		Dl.Reset  (mintegrable->GetNconstr());
		Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
		Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
		R.Reset   (mintegrable->GetNcoords_v());
		Qc.Reset  (mintegrable->GetNconstr());
		L.Reset   (mintegrable->GetNconstr());


		mintegrable->StateGather(X, V, T);	// state <- system	

		// Extrapolate a prediction as warm start

		Xnew = X + V*dt;		 
		Vnew = V;  //+ A()*dt;		 
		
		// use Newton Raphson iteration to solve implicit Euler for v_new
		//
		// [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f + dt*Cq'*l ]
		// [ Cq                           0   ] [ -dt*Dl ] = [ C/dt  ]

		for (int i = 0; i < this->GetMaxiters(); ++i)
		{
			mintegrable->StateScatter(Xnew, Vnew, T+dt);	// state -> system
			R.Reset();
			Qc.Reset();
			mintegrable->LoadResidual_F  (R, dt);
			mintegrable->LoadResidual_Mv (R, (V-Vnew), 1.0);
			mintegrable->LoadResidual_CqL(R, L, dt);
			mintegrable->LoadConstraint_C(Qc, 1.0/dt);
GetLog()<< "Euler iteration=" << i << "  |R|=" << R.NormInf() << "  |Qc|=" << Qc.NormInf() << "\n";						
			if ((R.NormInf()  < this->GetTolerance()) &&
				(Qc.NormInf() < this->GetTolerance()))
				break;

			mintegrable->StateSolveCorrection(
				Dv, 
				Dl,
				R,
				Qc,
				1.0,  // factor for  M
				-dt,   // factor for  dF/dv
				-dt*dt,// factor for  dF/dx
				Xnew, Vnew, T+dt,
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

			Dl *= (1.0/dt); // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl
			L += Dl;

			Vnew += Dv;

			Xnew = X + Vnew *dt;
		}

		X = Xnew;
		V = Vnew;
		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};


/// Performs a step of Euler implicit for II order systems
/// using the Anitescu/Stewart/Trinkle single-iteration method,
/// that is a bit like an implicit Euler where one performs only
/// the first NR corrector iteration.
/// If the solver in StateSolveCorrection is a CCP complementarity
/// solver, this is the typical Anitescu stabilized timestepper for DVIs.

class ChTimestepperEulerImplicitLinearized : public ChTimestepperIIorder, public ChImplicitTimestepper
{
protected:
	ChStateDelta		Dv;
	ChVectorDynamic<>   Dl;
	ChVectorDynamic<>   R;
	ChVectorDynamic<>   Qc;

public:
	/// Constructors (default empty)
	ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder& mintegrable)
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

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		// setup auxiliary vectors
		Dv.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
		Dl.Reset(mintegrable->GetNconstr());
		R.Reset(mintegrable->GetNcoords_v());
		Qc.Reset(mintegrable->GetNconstr());
		L.Reset(mintegrable->GetNconstr());


		mintegrable->StateGather(X, V, T);	// state <- system

		// solve only 1st NR step, using v_new = 0, so  Dv = v_new , therefore 
		//
		// [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f]
		// [ Cq                           0   ] [ -dt*Dl ] = [ C/dt + Ct ]
		//
		// becomes the Anitescu/Trinkle timestepper:
		// 
		// [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
		// [ Cq                           0   ] [ -dt*l  ] = [ C/dt + Ct ]

		mintegrable->LoadResidual_F(R, dt);
		mintegrable->LoadResidual_Mv(R, V, 1.0);
		mintegrable->LoadConstraint_C (Qc, 1.0 / dt);
		mintegrable->LoadConstraint_Ct(Qc, 1.0);

		mintegrable->StateSolveCorrection(
				V,
				L,
				R,
				Qc,
				1.0,  // factor for  M
				-dt,   // factor for  dF/dv
				-dt*dt,// factor for  dF/dx
				X, V, T + dt, // not needed 
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

		L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

		X += V *dt;

		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};





/// Performs a step of trapezoidal implicit for II order systems

class ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
{
protected:
	ChStateDelta		Dv;
	ChVectorDynamic<>   Dl;
	ChState				Xnew;
	ChStateDelta		Vnew;
	ChVectorDynamic<>	R;
	ChVectorDynamic<>	Rold;
	ChVectorDynamic<>	Qc;

public:
	/// Constructors (default empty)
	ChTimestepperTrapezoidal(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitIterativeTimestepper()
	{};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		)
	{
		// downcast
		ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		// setup auxiliary vectors
		Dv.Reset  (mintegrable->GetNcoords_v(), GetIntegrable());
		Dl.Reset  (mintegrable->GetNconstr());
		Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
		Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
		L.Reset   (mintegrable->GetNconstr());
		R.Reset   (mintegrable->GetNcoords_v());
		Rold.Reset(mintegrable->GetNcoords_v());
		Qc.Reset  (mintegrable->GetNconstr());


		mintegrable->StateGather(X, V, T);	// state <- system
GetLog()<< "\n\n Trapezoidal \n";
//GetLog()<< "trapezoidal T=" << T<< " , X=" << X <<"\n";
//GetLog()<< "trapezoidal T=" << T<< " , V=" << V <<"\n";
		// extrapolate a prediction as a warm start

		Xnew = X + V*dt;		
		Vnew = V;  // +A()*dt;		

		// use Newton Raphson iteration to solve implicit Euler for v_new
		//
		// [ M - dt/2*dF/dv - dt^2/4*dF/dx    Cq' ] [ Dv       ] = [ M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old + Cq*l_new)]
		// [ Cq                               0   ] [ -dt/2*Dl ] = [ C/dt                                                          ]

		mintegrable->LoadResidual_F(Rold, dt*0.5);    // dt/2*f_old
		mintegrable->LoadResidual_Mv(Rold, V, 1.0);  // M*v_old
		mintegrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old
	
		for (int i = 0; i < this->GetMaxiters(); ++i)
		{
			mintegrable->StateScatter(Xnew, Vnew, T + dt);	// state -> system
			R = Rold;
			Qc.Reset();
			mintegrable->LoadResidual_F(R, dt*0.5);    // + dt/2*f_new
			mintegrable->LoadResidual_Mv(R, Vnew, -1.0); // - M*v_new
			mintegrable->LoadResidual_CqL(R, L, dt*0.5); // + dt/2*Cq*l_new
			mintegrable->LoadConstraint_C(Qc, 1.0 / dt); // C/dt
	//GetLog()<< "trapezoidal iter="<<i<<" R =" << R <<"\n";
	//GetLog()<< "trapezoidal iter="<<i<<" Qc =" << Qc <<"\n";
	GetLog()<< "trapezoidal iteration=" << i << "  |R|=" << R.NormTwo() << "  |Qc|=" << Qc.NormTwo() << "\n";
			if ((R.NormInf()  < this->GetTolerance()) &&
				(Qc.NormInf() < this->GetTolerance()))
				break;

			mintegrable->StateSolveCorrection(
				Dv,
				Dl,
				R,
				Qc,
				1.0,        // factor for  M
				-dt*0.5,    // factor for  dF/dv
				-dt*dt*0.25,// factor for  dF/dx
				Xnew, Vnew, T + dt,
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

			Dl *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl
			L += Dl;

			Vnew += Dv;

			Xnew = X + ((Vnew + V)*(dt*0.5));  // Xnew = Xold + h/2(Vnew+Vold)
		}

		X = Xnew;
		V = Vnew;
		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};



/// Performs a step of HHT (generalized alpha) implicit for II order systems
/// See Negrut et al. 2007.

class ChTimestepperHHT : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
{
private:
	double alpha;
	double gamma;
	double beta;
	ChStateDelta		Da;
	ChVectorDynamic<>   Dl;
	ChState				Xnew;
	ChStateDelta		Vnew;
	ChStateDelta		Anew;
	ChVectorDynamic<>	R;
	ChVectorDynamic<>	Rold;
	ChVectorDynamic<>	Qc;

public:
	/// Constructors (default empty)
	ChTimestepperHHT(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitIterativeTimestepper()
	{
		SetAlpha(0.0);
	};


		
	/// Set the numerical damping parameter. 
	/// It must be in the [-1/3, 0] interval. 
	/// The closer to -1/3, the more damping.
	/// The closer to 0, the less damping (for 0, it is the trapezoidal method).
	/// It automatically sets gamma and beta.
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

		// setup main vectors
		mintegrable->StateSetup(X, V, A);

		// setup auxiliary vectors
		Da.Reset(mintegrable->GetNcoords_a(), GetIntegrable());
		Dl.Reset(mintegrable->GetNconstr());
		Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
		Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
		Anew.Reset(mintegrable->GetNcoords_a(), mintegrable);
		R.Reset(mintegrable->GetNcoords_v());
		Rold.Reset(mintegrable->GetNcoords_v());
		Qc.Reset(mintegrable->GetNconstr());
		L.Reset(mintegrable->GetNconstr());


		mintegrable->StateGather(X, V, T);	// state <- system

		// extrapolate a prediction as a warm start

		Vnew = V; //+ Anew*dt;	
		Xnew = X + Vnew*dt;		 
		 
		mintegrable->LoadResidual_F(Rold, -(alpha / (1.0 + alpha)));     // -alpha/(1.0+alpha) * f_old
		mintegrable->LoadResidual_CqL(Rold, L, -(alpha / (1.0 + alpha)));   // -alpha/(1.0+alpha) * Cq'*l_old


		// use Newton Raphson iteration to solve implicit Euler for a_new
		// Note: l and l_new and Dl with opposite sign if compared to Negrut et al. 2007.

		//
		// [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Da       ] = [-1/(1+alpha)*M*(a_new) + (f_new +Cq*l_new) - (alpha/(1+alpha))(f_old +Cq*l_old)]
		// [ Cq                                      0   ] [ Dl       ] = [ 1/(beta*dt^2)*C                                                                          ]

		for (int i = 0; i < this->GetMaxiters(); ++i)
		{
			mintegrable->StateScatter(Xnew, Vnew, T + dt);	// state -> system
			R = Rold;
			Qc.Reset();
			mintegrable->LoadResidual_F  (R,  1.0);         //  f_new
			mintegrable->LoadResidual_CqL(R, L,  1.0);      //  Cq'*l_new
			mintegrable->LoadResidual_Mv (R, Anew, -(1.0 / (1.0 + alpha)) ); // -1/(1+alpha)*M*a_new
			mintegrable->LoadConstraint_C(Qc,  (1.0/(beta*dt*dt)) );  //  1/(beta*dt^2)*C

			if ((R.NormInf()  < this->GetTolerance()) &&
				(Qc.NormInf() < this->GetTolerance()))
				break;

			mintegrable->StateSolveCorrection(
				Da,
				Dl,
				R,
				Qc,
				(1.0 / (1.0 + alpha)),        // factor for  M (was 1 in Negrut paper ?!)
				-dt*gamma,  // factor for  dF/dv
				-dt*dt*beta,// factor for  dF/dx
				Xnew, Vnew, T + dt,
				false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
				);

			L    -= Dl;  // Note it is not += Dl because we assume StateSolveCorrection flips sign of Dl
			Anew += Da;

			Xnew = X + V*dt + A*(dt*dt*(0.5 - beta)) + Anew*(dt*dt*beta);

			Vnew = V + A*(dt*(1.0 - gamma)) + Anew*(dt*gamma);
		}

		X = Xnew;
		V = Vnew;
		A = Anew;
		T += dt;

		mintegrable->StateScatter(X, V, T);	// state -> system
	}
};






} // END_OF_NAMESPACE____
#endif 
