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

///////////////////////////////////////////////////
//
//   Demo on low-level functionality for time 
//   integration of differential equations.
//
///////////////////////////////////////////////////
  
 

#include <math.h>
 
       
#include "core/ChLog.h"
#include "timestepper/ChTimestepper.h"

using namespace chrono;
           
		        
int main(int argc, char* argv[])
{

	GetLog() << "CHRONO demo about low-level time integration of differential equations: \n\n";   

	if (true)
	{
		//
		// EXAMPLE 1:
		//
 
		GetLog() << " Example 1: integrate dx/dt=e^t \n";


						// Define a class inherited from ChIntegrable,
						// it will represent the differential equations
						// by implementing the StateSolve() function, and few other interfaces:
		class MyIntegrable : public ChIntegrable
		{
		private: 
		public:
				MyIntegrable() {}

							/// the number of coordinates in the state:
				virtual int GetNcoords_y() {return 1;}

							/// compute  dy/dt=f(y,t) 
				virtual void StateSolve(ChStateDelta& Dy,	///< result: computed Dy
										ChVectorDynamic<>& L,	///< result: computed lagrangian multipliers, if any
										const ChState& y,	///< current state y
										const double T,		///< current time T
										const double dt,	///< timestep (if needed)
										bool force_state_scatter = true ///< if false, y and T are not scattered to the system, assuming that someone has done StateScatter just before
										) 
				{
					if(force_state_scatter) 
						StateScatter(y,T);  // state -> system   (not needed here, btw.)

					Dy(0) = exp(T)*dt;  // dx/dt=e^t   then:  dx=e^t * dt
				}
		};


							// Create and object from your custom integrable class:
		MyIntegrable mintegrable;

							// Create a time-integrator:
		ChTimestepperEuleroExpl mystepper(mintegrable);
		//ChTimestepperRungeKuttaExpl mystepper(mintegrable);

							// Execute the time integration
		while (mystepper.GetTime() <4)
		{
			mystepper.Advance(0.1);

			double exact_solution = exp(mystepper.GetTime())-1;
			GetLog() << " T = " << mystepper.GetTime() 
					 << "  x="  << mystepper.Y()(0) 
					 << "  x_exact="<< exact_solution << "\n";
		}

	}


	if (false)
	{
		//
		// EXAMPLE 2:
		//
 
		GetLog() << " Example 2: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) \n";


						// Define a class inherited from ChIntegrable,
						// it will represent the differential equations
						// by implementing the StateSolve() function, and few other interfaces:
		class MyIntegrable : public ChIntegrable
		{
		private: 
				double M;
				double K;
				double R;
				double T;
				double x;
				double v;
		public:
				MyIntegrable()
				{
					T = 0;
					M = 10;
					K = 30;
					R = 1;
					x = 0;
					v = 0;
				}

								/// the number of coordinates in the state:
				virtual int GetNcoords_y() {return 2;}

								/// system -> state
				virtual void StateGather(ChState& y, double& mT) 
										{
											y(0)=x;
											y(1)=v;
											mT = T;
										};

								/// state -> system  
				virtual void StateScatter(const ChState& y, const double mT) 
										{
											x=y(0);
											v=y(1);
											T=mT;
										};

								/// compute  dy/dt=f(y,t) 
				virtual void StateSolve(ChStateDelta& Dy,		///< result: computed Dy
										ChVectorDynamic<>& L,	///< result: computed lagrangian multipliers, if any
										const ChState& y,	///< current state y
										const double T,		///< current time T
										const double dt,	///< timestep (if needed)
										bool force_state_scatter = true ///< if false, y and T are not scattered to the system, assuming that someone has done StateScatter just before
										) 
				{
					if(force_state_scatter) 
						StateScatter(y,T);

					double F = sin(T*4)*2;

					Dy(0) = dt*v;
					Dy(1) = dt*(1./M) * (F - K*x - R*v);
				}
		};


							// Create and object from your custom integrable class:
		MyIntegrable mintegrable;

							// Create a time-integrator:
		ChTimestepperEuleroExpl mystepper(mintegrable);

							// Execute the time integration
		while (mystepper.GetTime() <1)
		{
			mystepper.Advance(0.01);

			GetLog() << " T = " << mystepper.GetTime() << "  x=" << mystepper.Y()(0) << "  v=" << mystepper.Y()(1) << "\n";
		}

	}


	GetLog() << "\n  CHRONO execution terminated.";
	

	system("pause");
	return 0;
}
 

