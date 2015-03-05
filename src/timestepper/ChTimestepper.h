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

class ChApi ChTimestepper : public ChShared
{
protected:
	ChIntegrable* integrable;
	double T;

	ChVectorDynamic<> L;

	bool verbose;

	bool	Qc_do_clamp;
	double	Qc_clamping;
public:
					/// Constructor
	ChTimestepper(ChIntegrable& mintegrable) 
				{
					integrable = &mintegrable;
					T = 0;
					L.Reset(0);
					verbose = false;
					Qc_do_clamp = false;
					Qc_clamping =1e30;
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

					/// Turn on/off logging of messages
	void SetVerbose(bool mverbose) { verbose = mverbose; }

					/// Turn on/off clamping on the Qcterm 
	void SetQcDoClamp(bool mdc) { Qc_do_clamp = mdc; }

					/// Turn on/off clamping on the Qcterm 
	void SetQcClamping(double mcl) { Qc_clamping = mcl; }
};



/// Base class for 1st order timesteppers, that is
/// a time integrator for whatever ChIntegrable.

class ChApi ChTimestepperIorder : public ChTimestepper
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

class ChApi ChTimestepperIIorder : public ChTimestepper
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
class ChApi ChImplicitTimestepper 
{
};


/// Base properties for implicit solvers that compute the solution by iterative
/// process up to a desired tolerance
class ChApi ChImplicitIterativeTimestepper  : public ChImplicitTimestepper
{
private:
	int maxiters;
	double tolerance;

public:
					/// Constructors 
	ChImplicitIterativeTimestepper() :
		maxiters(6),
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

class  ChApi ChTimestepperEulerExpl : public ChTimestepperIorder
{
public:
					/// Constructors (default empty)
	ChTimestepperEulerExpl(ChIntegrable& mintegrable) 
		: ChTimestepperIorder(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			);
};


/// Euler explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEulerExpl,
/// but this performes a bit faster because it can exploit
/// the special structure of ChIntegrableIIorder)
/// This performs the typical 
///    x_new = x + v * dt 
///    v_new = v + a * dt 
/// integration with Euler formula. 

class  ChApi ChTimestepperEulerExplIIorder : public ChTimestepperIIorder
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
		);
};



/// Euler semi-implicit timestepper
/// This performs the typical 
///    v_new = v + a * dt 
///    x_new = x + v_new * dt 
/// integration with Euler semi-implicit formula. 

class ChApi ChTimestepperEulerSemiImplicit : public ChTimestepperIIorder
{
public:
	/// Constructors (default empty)
	ChTimestepperEulerSemiImplicit(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable)  {};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		);
};




/// Performs a step of a 4th order explicit Runge-Kutta
/// integration scheme.

class ChApi ChTimestepperRungeKuttaExpl : public ChTimestepperIorder
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
			);
};



/// Performs a step of a Heun explicit integrator. It is like
/// a 2nd Runge Kutta.

class ChApi ChTimestepperHeun : public ChTimestepperIorder
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
		);
};




/// Performs a step of a Leapfrog explicit integrator. 
/// It is a symplectic method, with 2nd order accuracy,
/// at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting
/// the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives
/// the same accuracy with a bit of faster performance.

class ChApi ChTimestepperLeapfrog : public ChTimestepperIIorder
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
		);
};


/// Performs a step of Euler implicit for II order systems

class ChApi ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
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
		);
};


/// Performs a step of Euler implicit for II order systems
/// using the Anitescu/Stewart/Trinkle single-iteration method,
/// that is a bit like an implicit Euler where one performs only
/// the first NR corrector iteration.
/// If the solver in StateSolveCorrection is a CCP complementarity
/// solver, this is the typical Anitescu stabilized timestepper for DVIs.

class ChApi ChTimestepperEulerImplicitLinearized : public ChTimestepperIIorder, public ChImplicitTimestepper
{
protected:
	ChStateDelta		Vold;
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
		);
};





/// Performs a step of trapezoidal implicit for II order systems.
/// NOTE this is a modified version of the trapezoidal for DAE: the
/// original derivation would lead to a scheme that produces oscillatory 
/// reactions in constraints, so this is a modified version that is first
/// order in constraint reactions. Use damped HHT or damped Newmark for
/// more advanced options.

class ChApi ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
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
		);
};


/// Performs a step of trapezoidal implicit linearized for II order systems

class ChApi ChTimestepperTrapezoidalLinearized : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
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
	ChTimestepperTrapezoidalLinearized(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitIterativeTimestepper()
	{};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		);
};


/// Performs a step of trapezoidal implicit linearized for II order systems
///*** SIMPLIFIED VERSION -DOES NOT WORK - PREFER ChTimestepperTrapezoidalLinearized

class ChApi ChTimestepperTrapezoidalLinearized2 : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
{
protected:
	ChStateDelta		Dv;
	ChState				Xnew;
	ChStateDelta		Vnew;
	ChVectorDynamic<>	R;
	ChVectorDynamic<>	Qc;

public:
	/// Constructors (default empty)
	ChTimestepperTrapezoidalLinearized2(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitIterativeTimestepper()
	{};

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		);
};


/// Performs a step of HHT (generalized alpha) implicit for II order systems
/// See Negrut et al. 2007.

class ChApi ChTimestepperHHT : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
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
		SetAlpha(-0.2); // default: some dissipation
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
		);
};




/// Performs a step of Newmark constrained implicit for II order DAE systems
/// See Negrut et al. 2007.

class ChApi ChTimestepperNewmark : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper
{
private:
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
	ChTimestepperNewmark(ChIntegrableIIorder& mintegrable)
		: ChTimestepperIIorder(mintegrable),
		ChImplicitIterativeTimestepper()
	{
		SetGammaBeta(0.6,0.3); // default values with some damping, and that works also with DAE constraints
	};


		
	/// Set the numerical damping parameter gamma and the beta parameter. 
	/// Gamma: in the [1/2, 1] interval. 
	/// For gamma = 1/2, no numerical damping
	/// For gamma > 1/2, more damping
	/// Beta: in the [0, 1] interval. 
	/// For beta = 1/4, gamma = 1/2 -> constant acceleration method
	/// For beta = 1/6, gamma = 1/2 -> linear acceleration method
	/// Method is second order accurate only for gamma = 1/2
	void SetGammaBeta(double mgamma, double mbeta)
	{
		gamma = mgamma;
		if (gamma <  0.5)
			gamma = 0.5;
		if (gamma > 1)
			gamma = 1;
		beta = mbeta;
		if (beta <  0)
			beta = 0;
		if (beta > 1)
			beta = 1;
	}

	double GetGamma() { return gamma; }

	double GetBeta() { return beta; }

	/// Performs an integration timestep
	virtual void Advance(
		const double dt				///< timestep to advance
		);
};







} // END_OF_NAMESPACE____
#endif 
