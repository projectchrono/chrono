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


namespace chrono 
{

// forward reference
class ChIntegrable;


/// Class for state of time-integrable objects. 
/// This is a vector (one-column matrix), hence inherited from ChMatrixDynamic.
/// It is used in ChIntegrable, ChTimestepper, etc.

class ChState : public ChMatrixDynamic<double>
{
	public:
					/// Constructors
	explicit ChState (ChIntegrable& mint) 
		: ChMatrixDynamic<double>(1,1) { integrable = &mint; };

	explicit ChState (const int nrows, ChIntegrable& mint) 
		: ChMatrixDynamic<double>(nrows,1) { integrable = &mint; };

	explicit ChState (const ChMatrixDynamic<double> matr, ChIntegrable& mint) 
		: ChMatrixDynamic<double>(matr) { integrable = &mint; };

	explicit ChState (const int nrows, const int ncolumns, ChIntegrable& mint ) 
		: ChMatrixDynamic<double>(nrows,ncolumns) { integrable = &mint; };

					/// Copy constructor
	ChState(const ChState& msource)
		: ChMatrixDynamic<double>(msource) { integrable = msource.integrable; };


		/// Multiplies this matrix by a factor, in place
	template <class Real>
	ChState& operator*=(const Real factor) { MatrScale(factor); return *this; }
	
		/// Increments this matrix by another matrix, in place
	template <class RealB>
	ChState& operator+=(const ChMatrix<RealB>& matbis) { MatrInc(matbis); return *this; }
	
		/// Decrements this matrix by another matrix, in place
	template <class RealB>
	ChState& operator-=(const ChMatrix<RealB>& matbis) { MatrDec(matbis); return *this; }

			/// Negates sign of the matrix. 
		/// Performance warning: a new object is created.
	ChState operator-() const
		{
			ChState result(*this);
			result.MatrNeg();
			return result;
		}

		/// Sums this matrix and another matrix.
		/// Performance warning: a new object is created.
	template <class RealB>
	ChState operator+(const ChMatrix<RealB>& matbis) const
		{
			ChState result(this->rows, *this->integrable);
			result.MatrAdd(*this, matbis);
			return result;
		}

		/// Subtracts this matrix and another matrix.
		/// Performance warning: a new object is created.
	template <class RealB>
	ChState operator-(const ChMatrix<RealB>& matbis) const
		{
			ChState result(this->rows, *this->integrable);
			result.MatrSub(*this, matbis);
			return result;
		}

		/// Multiplies this matrix by a scalar value.
		/// Performance warning: a new object is created.
	template <class Real>
	ChState operator*(const Real factor) const
		{
			ChState result(*this);
			result.MatrScale(factor);
			return result;
		}


	ChIntegrable* GetIntegrable() const {return integrable;}

private:
	ChIntegrable* integrable;

};


/// Class for incremental form of state of time-integrable objects. 
/// Note that for many cases, this would be superfluous, because one could
/// do y_new = y_old + dydt*td, where dydt is a ChState just like y and y_new, but there 
/// are cases where such simple "+" operations between vectors is not practical, for instance 
/// when integrating rotations in 3D space, where it is better to work with quaterions in y
/// and y_new, but with spinors/angular velocities/etc. in dydt; so dim(y) is not dim(dydt);
/// hence the need of this specific class for increments in states.
/// This is a vector (one-column matrix), hence inherited from ChMatrixDynamic.
/// It is used in ChIntegrable, ChTimestepper, etc.

class ChStateDelta : public ChMatrixDynamic<double>
{
	public:
					/// Constructors
	explicit ChStateDelta (ChIntegrable& mint) 
		: ChMatrixDynamic<double>(1,1) { integrable = &mint; };

	explicit ChStateDelta (const int nrows, ChIntegrable& mint) 
		: ChMatrixDynamic<double>(nrows,1) { integrable = &mint; };

	explicit ChStateDelta (const ChMatrixDynamic<double> matr, ChIntegrable& mint) 
		: ChMatrixDynamic<double>(matr) { integrable = &mint; };

	explicit ChStateDelta (const int nrows, const int ncolumns, ChIntegrable& mint) 
		: ChMatrixDynamic<double>(nrows,ncolumns) { integrable = &mint; };

					/// Copy constructor
	ChStateDelta(const ChStateDelta& msource)
		: ChMatrixDynamic<double>(msource) { integrable = msource.integrable; };

			/// Multiplies this matrix by a factor, in place
	template <class Real>
	ChStateDelta operator*=(const Real factor) { MatrScale(factor); return *this; }
	
		/// Increments this matrix by another matrix, in place
	template <class RealB>
	ChStateDelta operator+=(const ChMatrix<RealB>& matbis) { MatrInc(matbis); return *this; }
	
		/// Decrements this matrix by another matrix, in place
	template <class RealB>
	ChStateDelta operator-=(const ChMatrix<RealB>& matbis) { MatrDec(matbis); return *this; }

			/// Negates sign of the matrix. 
		/// Performance warning: a new object is created.
	ChStateDelta operator-() const
		{
			ChStateDelta result(*this);
			result.MatrNeg();
			return result;
		}

		/// Sums this matrix and another matrix.
		/// Performance warning: a new object is created.
	template <class RealB>
	ChStateDelta operator+(const ChMatrix<RealB>& matbis) const
		{
			ChStateDelta result(this->rows, *this->integrable);
			result.MatrAdd(*this, matbis);
			return result;
		}

		/// Subtracts this matrix and another matrix.
		/// Performance warning: a new object is created.
	template <class RealB>
	ChStateDelta operator-(const ChMatrix<RealB>& matbis) const
		{
			ChStateDelta result(this->rows, *this->integrable);
			result.MatrSub(*this, matbis);
			return result;
		}

		/// Multiplies this matrix by a scalar value.
		/// Performance warning: a new object is created.
	template <class Real>
	ChStateDelta operator*(const Real factor) const
		{
			ChStateDelta result(*this);
			result.MatrScale(factor);
			return result;
		}

	ChIntegrable* GetIntegrable() const {return integrable;}

private:
	ChIntegrable* integrable;

};




///////////////////// INTEGRABLE SYSTEMS ////////////////////////////////



	/// Interface class for all objects that support time integration.
	/// You can inherit your class from this class, by implementing those
	/// four functions. By doing this, you can use time integrators from
	/// the ChTimestepper hierarchy to integrate in time.

class ChIntegrable
{
	public:

			/// Tells the number of coordinates in the state Y. 
			/// Children classes MUST implement this! 
	virtual int GetNcoords_y() = 0;

			/// Tells the number of coordinates in the state increment. 
			/// This is a base implementation that works in many cases where dim(Y) = dim(dy), 
			/// but it can be overridden in the case that y contains quaternions for rotations
			/// rather than simple y+dy
	virtual int GetNcoords_dy() { return GetNcoords_y();};

			/// This sets up the system state.
			/// Children classes must properly resize Y to the number of coordinates.
			/// Children classes must properly resize Dy to the number of coordinates of 
			/// state increment (usually, the same of number of coordinates).
	virtual void StateSetup(ChState& y, ChStateDelta& dy) 
			{
				y.Resize(GetNcoords_y(),1);
				dy.Resize(GetNcoords_dy(),1);
			};

			/// From system to state Y
			/// Optionally, they will copy system private state, if any, to Y.
	virtual void StateGather(ChState& y, double& T) {};

			/// From state Y to system.
			/// This is important because it is called by time integrators all times
			/// they modify the Y state. In some cases, the ChIntegrable object might 
			/// contain dependent data structures that might need an update at each change of Y,
			/// if so, this function must be overridden. 
	virtual void StateScatter(const ChState& y, const double T) {};

			/// dy/dt = f(y,t)
			/// Given current state y , computes the state derivative dy/dt and
			/// lagrangian multipliers F (if any). Note that rather than computing
			/// dy/dt, here it must compute Dy, same for F that should be rather Fdt = impulses,
			/// so this fits better in measure differential inclusion generalization (later,
			/// one can multiply Fdt and Dy  by (1/dt) and still get dt/dt and F, if needed.
			/// NOTE! children classes must take care of calling StateScatter(y,T) before 
			/// computing Dy, only if force_state_scatter = true (otherwise it is assumed state is already in sync)
			/// NOTE! children classes must take care of resizing Dy if needed.
	virtual void StateSolve(ChStateDelta& Dy,	///< result: computed Dy
							const ChState& y,	///< current state y
							const double T,		///< current time T
							const double dt,	///< timestep (if needed)
							bool force_state_scatter = true ///< if false, y and T are not scattered to the system, assuming that someone has done StateScatter just before
							) = 0;

			/// Perform y_new = y + Dy
			/// This is a base implementation that works in many cases, but it can be overridden 
			/// in the case that y contains quaternions for rotations, rot. exponential is needed 
			/// rather than simple y+Dy
			/// NOTE: the system is not updated automatically after the state increment, so one might
			/// need to call StateScatter() if needed. 
	virtual void StateIncrement(
							ChState& y_new,			///< resulting y_new = y + Dy
							const ChState& y,		///< initial state y
							const ChStateDelta& Dy	///< state increment Dy
							) 
			{
				y_new.Resize(y.GetRows(), y.GetColumns());
				for (int i = 0; i< y.GetRows(); ++i)
				{
					y_new(i) = y(i) + Dy(i);
				}
			}



};


/// This is a custom operator "+" that takes care of incremental update
/// of a state y by an increment Dy, if one types "y_new = y+Dy", by calling 
/// the specialized StateIncrement() in the ChIntegrable (if any, otherwise 
/// it will be a simple vector sum).

ChState operator+ (const ChState& y, const ChStateDelta& Dy) 
		{
			ChState result(y.GetRows(), *y.GetIntegrable());
			y.GetIntegrable()->StateIncrement(result, y, Dy);
			return result;
		}

/// This is a custom operator "+" that takes care of incremental update
/// of a state y by an increment Dy, if one types "y_new = Dy+y", by calling 
/// the specialized StateIncrement() in the ChIntegrable (if any, otherwise 
/// it will be a simple vector sum).

ChState operator+ (const ChStateDelta& Dy, const ChState& y) 
		{
			ChState result(y.GetRows(), *y.GetIntegrable());
			y.GetIntegrable()->StateIncrement(result, y, Dy);
			return result;
		}



///////////////////// TIMESTEPPERS ////////////////////////////////



/// Base class for timesteppers, that is
/// a time integrator which can advance a system state.
/// It operates on systems inherited from ChIntegrable.

class ChTimestepper : public ChShared
{
protected:
	ChIntegrable* integrable;
	double T;

	ChState* mY;
	ChStateDelta* mdYdt;
public:
					/// Constructor
	ChTimestepper(ChIntegrable& mintegrable) 
				{
					integrable = &mintegrable;
					T = 0;
					mY    = new ChState(mintegrable);
					mdYdt = new ChStateDelta(mintegrable);
				};
	
					/// Destructor
	virtual ~ChTimestepper()
				{
					delete mY;
					delete mdYdt;
				};

					/// Performs an integration timestep
	virtual void Advance(
				const double dt				///< timestep to advance
				) =0;

					/// Access the state at current time
	virtual ChState& Y() {return *mY;}

					/// Access the derivative of state at current time
	virtual ChStateDelta& dYdt() {return *mdYdt;}

					/// Get the integrable object 
	ChIntegrable& GetIntegrable() { return *integrable;}
					
					/// Get the current time 
	virtual double GetTime() { return T;}

					/// Set the current time 
	virtual void SetTime(double mt) { T = mt;}
};


/// Base class for explicit solvers
class ChTimestepperExplicit : public ChTimestepper
{
public:
					/// Constructors
	ChTimestepperExplicit(ChIntegrable& mintegrable) 
				: ChTimestepper(mintegrable)  {};
};


/// Base class for implicit solvers
class ChTimestepperImplicit : public ChTimestepper
{
public:
					/// Constructors 
	ChTimestepperImplicit(ChIntegrable& mintegrable) 
				: ChTimestepper(mintegrable)  {};
};



/// Eulero explicit timestepper
/// This performs the typical  y_new = y+ dy/dt * dt 
/// integration with Eulero formula. 

class ChTimestepperEuleroExpl : public ChTimestepperExplicit
{
public:
					/// Constructors (default empty)
	ChTimestepperEuleroExpl(ChIntegrable& mintegrable) 
				: ChTimestepperExplicit(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			) 
	{
		GetIntegrable().StateSetup(Y(), dYdt());
		GetIntegrable().StateGather(Y(), T);	// state <- system

		ChStateDelta Dy(GetIntegrable());

		GetIntegrable().StateSolve(Dy, Y(), T, dt, false);	// dY/dt = f(Y,T)   dY = f(Y,T)*dt

		// Euler formula!  
		//   y_new= y + dy/dt * dt    =  y_new= y + Dy

		Y()		= Y() + Dy;		//  also: GetIntegrable().StateIncrement(y_new, y, Dy);
		dYdt()	= Dy*(1./dt); 
		T		+= dt;

		GetIntegrable().StateScatter(Y(), T);	// state -> system
	}
};


/// Performs a step of a 4th order explicit Runge-Kutta
/// integration scheme.

class ChTimestepperRungeKuttaExpl : public ChTimestepperExplicit
{
public:
					/// Constructors (default empty)
	ChTimestepperRungeKuttaExpl (ChIntegrable& mintegrable) 
				: ChTimestepperExplicit(mintegrable)  {};

					/// Performs an integration timestep
	virtual void Advance(
			const double dt				///< timestep to advance
			) 
	{
		GetIntegrable().StateSetup(Y(), dYdt());
		GetIntegrable().StateGather(Y(), T);	// state <- system

		ChState y_new(GetIntegrable());

		ChStateDelta Dy1(GetIntegrable());
		GetIntegrable().StateSolve(Dy1, Y(), T, dt, false); //note, 'false'=no need to update with StateScatter before computation

		y_new = Y() + Dy1*0.5;	//integrable.StateIncrement(y_new, Y, Dy1*0.5);
		ChStateDelta Dy2(GetIntegrable());
		GetIntegrable().StateSolve(Dy2, y_new, T+dt*0.5, dt);

		y_new = Y() + Dy2*0.5;	//integrable.StateIncrement(y_new, Y, Dy2*0.5);
		ChStateDelta Dy3(GetIntegrable());
		GetIntegrable().StateSolve(Dy3, y_new, T+dt*0.5, dt);

		y_new = Y() + Dy3;		//integrable.StateIncrement(y_new, Y, Dy3);
		ChStateDelta Dy4(GetIntegrable());
		GetIntegrable().StateSolve(Dy4, y_new, T+dt, dt);

		Y()		= Y() + (Dy1 + Dy2*2.0 + Dy3*2.0 + Dy4)*(1./6.);   //integrable.StateIncrement(y_new, Y, (Dy1 + Dy2*2.0 + Dy3*2.0 + Dy4)*(1./6.) );
		dYdt()	= Dy4*(1./dt);
		T		+= dt;

		GetIntegrable().StateScatter(Y(), T);	// state -> system
	}
};



} // END_OF_NAMESPACE____
#endif 
