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

							// File to dump results
		ChStreamOutAsciiFile log_file1("log_timestepper_1.dat");


							// Create and object from your custom integrable class:
		MyIntegrable mintegrable;

							// Create a time-integrator, class: Eulero explicit
		ChTimestepperEuleroExpl mystepper(mintegrable);


							// Execute the time integration
		while (mystepper.GetTime() <4)
		{
			mystepper.Advance(0.1);

			double exact_solution = exp(mystepper.GetTime())-1;
			GetLog() << " T = " << mystepper.GetTime() 
					 << "  x="  << mystepper.Y()(0) 
					 << "  x_exact="<< exact_solution << "\n";
			log_file1 << mystepper.GetTime() << ", " << mystepper.Y()(0) << ", " << exact_solution << "\n";
		}

	}


	if (true)
	{
		//
		// EXAMPLE 2:
		//
 
		GetLog() << "\n\n Example 2: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) with a 1st order integrator. \n";


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

		// File to dump results
		ChStreamOutAsciiFile log_file2("log_timestepper_2.dat");


		// Try integrator Eulero explicit

							// Create and object from your custom integrable class:
		MyIntegrable mintegrable;
							// Create a time-integrator:
		ChTimestepperEuleroExpl mystepper(mintegrable);


		// Try integrator Runge Kutta 4st  explicit

							// Create and object from your custom integrable class:
		MyIntegrable mintegrable_rk;
							// Create a time-integrator, class: Runge Kutta 4 explicit
		ChTimestepperRungeKuttaExpl mystepper_rk(mintegrable_rk);




							// Execute the time integration
		while (mystepper.GetTime() <1)
		{
			mystepper.Advance(0.01);
			mystepper_rk.Advance(0.01);

			GetLog() << " T = " << mystepper.GetTime() << "  x=" << mystepper.Y()(0) << "  v=" << mystepper.Y()(1) << "\n";
			log_file2 << mystepper.GetTime() << ", " << mystepper.Y()(0)    << ", " << mystepper.Y()(1) 
				                             << ", " << mystepper_rk.Y()(0) << ", " << mystepper_rk.Y()(1) << "\n";
		}

	}




	if (true)
	{
		//
		// EXAMPLE 3:
		//

		GetLog() << "\n\n Example 3: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) with a 2nd order integrator. \n";


		// Define a class inherited from ChIntegrableIIorder,
		// it will represent the differential equations
		// by implementing the StateSolveA() function, and few other interfaces.
		// Compared to previous example 2, this is inherited from ChIntegrableIIorder,
		// so one can use more advanced integrators such as ChTimestepperEuleroSemiImplicit,
		// that can exploit the II order nature of the problem. Anyway, also all I order
		// integrators can still be used.
		class MyIntegrable : public ChIntegrableIIorder
		{
		private:
			double M;
			double K;
			double R;
			double T;
			double mx;
			double mv;
		public:
			MyIntegrable()
			{
				T = 0;
				M = 10;
				K = 30;
				R = 1;
				mx = 0;
				mv = 0;
			}

			/// the number of coordinates in the state, x position part:
			virtual int GetNcoords_x() { return 1; }

			/// system -> state
			virtual void StateGather(ChState& x, ChStateDelta& v, double& mT)
			{
				x(0) = mx;
				v(0) = mv;
				mT = T;
			};

			/// state -> system  
			virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double mT)
			{
				mx = x(0);
				mv = v(0);
				T = mT;
			};

			/// compute  dy/dt=f(y,t) 
			virtual void StateSolveA(ChStateDelta& Dv,		///< result: computed Dv
				ChVectorDynamic<>& L,	///< result: computed lagrangian multipliers, if any
				const ChState& x,		///< current state, x
				const ChStateDelta& v,	///< current state, v
				const double T,		///< current time T
				const double dt,	///< timestep (if needed)
				bool force_state_scatter = true ///< if false, y and T are not scattered to the system, assuming that someone has done StateScatter just before
				)
			{
				if (force_state_scatter)
					StateScatter(x, v, T);
				double F = sin(T * 4) * 2;
				Dv(0) = dt*(1. / M) * (F - K*mx - R*mv);
			}
		};


		// Create a file to dump results
		ChStreamOutAsciiFile log_file3("log_timestepper_3.dat");


		// Create and object from your custom integrable class:
		MyIntegrable mintegrable1;
		MyIntegrable mintegrable2;
		MyIntegrable mintegrable3;

		// Create few time-integrators to be compared:
		ChTimestepperEuleroExpl		     mystepper1(mintegrable1);
		ChTimestepperEuleroExplIIorder   mystepper2(mintegrable2);
		ChTimestepperEuleroSemiImplicit  mystepper3(mintegrable3);

		
		// Execute the time integration
		while (mystepper1.GetTime() <1)
		{
			mystepper1.Advance(0.01);
			mystepper2.Advance(0.01);
			mystepper3.Advance(0.01);

			GetLog() << "T = " << mystepper1.GetTime() << "  x=" << mystepper1.Y()(0) << "  v=" << mystepper1.Y()(1) << "\n";
			log_file3 << mystepper1.GetTime()	<< ", " << mystepper1.Y()(0) << ", " << mystepper1.Y()(1)
												<< ", " << mystepper2.X()(0) << ", " << mystepper2.V()(0)
												<< ", " << mystepper3.X()(0) << ", " << mystepper3.V()(0)
												<< "\n";
		}

	}





	GetLog() << "\n  CHRONO execution terminated.";
	

	system("pause");
	return 0;
}
 

