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
#include "core/ChLinearAlgebra.h"
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
				virtual void StateSolve(ChStateDelta& dydt,	///< result: computed dy/dt
										ChVectorDynamic<>& L,	///< result: computed lagrangian multipliers, if any
										const ChState& y,	///< current state y
										const double T,		///< current time T
										const double dt,	///< timestep (if needed)
										bool force_state_scatter = true ///< if false, y and T are not scattered to the system, assuming that someone has done StateScatter just before
										) 
				{
					if(force_state_scatter) 
						StateScatter(y,T);  // state -> system   (not needed here, btw.)

					dydt(0) = exp(T);  // dx/dt=e^t  
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
				virtual void StateSolve(ChStateDelta& dydt,	///< result: computed dy/dt
										ChVectorDynamic<>& L,	///< result: computed lagrangian multipliers, if any
										const ChState& y,	///< current state y
										const double T,		///< current time T
										const double dt,	///< timestep (if needed)
										bool force_state_scatter = true ///< if false, y and T are not scattered to the system, assuming that someone has done StateScatter just before
										) 
				{
					if(force_state_scatter) 
						StateScatter(y,T);

					double F = cos(T*20)*2;

					dydt(0) = v;	// speed
					dydt(1) = (1./M) * (F - K*x - R*v); // acceleration
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
			virtual void StateSolveA(ChStateDelta& dvdt,		///< result: computed accel. a = dv/dt
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

				double F = cos(T * 20) * 2;
				dvdt(0) = (1. / M) * (F - K*mx - R*mv);
			}
		};


		// Create a file to dump results
		ChStreamOutAsciiFile log_file3("log_timestepper_3.dat");


		// Create and object from your custom integrable class:
		MyIntegrable mintegrable1;
		MyIntegrable mintegrable2;
		MyIntegrable mintegrable3;

		// Create few time-integrators to be compared:
		ChTimestepperRungeKuttaExpl		 mystepper1(mintegrable1);
		ChTimestepperEuleroExplIIorder   mystepper2(mintegrable2);
		ChTimestepperEuleroSemiImplicit  mystepper3(mintegrable3);

		
		// Execute the time integration
		while (mystepper1.GetTime() <4)
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




	if (true)
	{
		//
		// EXAMPLE 4:
		//

		GetLog() << "\n\n Example 4: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) with an implicit integrator \n";


		// Define a class inherited from ChIntegrableIIorder,
		// it will represent the differential equations
		// by implementing the interfaces to implicit solvers.
		//  We assume   M*a = F(x,v,t) 

		class MyIntegrable : public ChIntegrableIIorder
		{
		private:
			double M;
			double K;
			double R;
			double mT;
			double mx;
			double mv;
		public:
			MyIntegrable()
			{
				mT = 0;
				M = 1;
				K = 30;
				R = 0;
				mx = 0;
				mv = 0.6;
			}

			/// the number of coordinates in the state, x position part:
			virtual int GetNcoords_x() { return 1; }

			/// system -> state
			virtual void StateGather(ChState& x, ChStateDelta& v, double& T)
			{
				x(0) = mx;
				v(0) = mv;
				T = mT;
			};

			/// state -> system  
			virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T)
			{
				mx = x(0);
				mv = v(0);
				mT = T;
			};

			/// compute  dy/dt=f(y,t) 
			virtual void StateSolveA(ChStateDelta& dvdt,		///< result: computed accel. a=dv/dt
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
				double F = sin(mT * 20) * 0.02;
				dvdt(0) = (1. / M) * (F - K*mx - R*mv);
			}

			/// Compute the correction with linear system
			///  Dv = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R 
			virtual void StateSolveCorrection(
				ChStateDelta& Dv,	  ///< result: computed Dv 
				ChVectorDynamic<>& L, ///< result: computed lagrangian multipliers, if any
				const ChVectorDynamic<>& R, ///< the R residual
				const ChVectorDynamic<>& Qc,///< the Qc residual
				const double c_a,	  ///< the factor in c_a*M
				const double c_v,	  ///< the factor in c_v*dF/dv
				const double c_x,	  ///< the factor in c_x*dF/dv
				const ChState& x,	  ///< current state, x part
				const ChStateDelta& v,///< current state, v part
				const double T,		  ///< current time T
				bool force_state_scatter = true ///< if false, x,v and T are not scattered to the system, assuming that someone has done StateScatter just before 
				)
			{
				if (force_state_scatter)
					this->StateScatter(x, v, T);

				Dv(0) = R(0) *  1.0 / (c_a*this->M + c_v *(-this->R) + c_x * (-this->K));
			}

			///    R += c*F
			void LoadResidual_F(
				ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*F 
				const double c				 ///< a scaling factor
				)
			{
				R(0) += c * (sin(mT * 20) * 0.02 - this->K*mx - this->R*mv);
			};
  
			///    R += c*M*w 
			void LoadResidual_Mv(
				ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*M*v 
				const ChVectorDynamic<>& w,  ///< the w vector 
				const double c				 ///< a scaling factor
				)
			{
				R(0) += c * this->M * w(0);
			};

			/// nothing to do here- no constraints
			virtual void LoadResidual_CqL(
				ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
				const ChVectorDynamic<>& L,  ///< the L vector 
				const double c				 ///< a scaling factor
				)
			{};

			/// nothing to do here- no constraints
			virtual void LoadConstraint_C(
				ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
				const double c				 ///< a scaling factor
				)
			{};

			/// nothing to do here- no constraints
			virtual void LoadConstraint_Ct(
				ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*Ct 
				const double c				 ///< a scaling factor
				)
			{};

		};


		// Create a file to dump results
		ChStreamOutAsciiFile log_file4("log_timestepper_4.dat");


		// Create and object from your custom integrable class:
		MyIntegrable mintegrable1;
		MyIntegrable mintegrable2;
		MyIntegrable mintegrable3;
		MyIntegrable mintegrable4;
		MyIntegrable mintegrable5;

		// Create few time-integrators to be compared:
		ChTimestepperEulerImplicit		 mystepper1(mintegrable1);
		ChTimestepperTrapezoidal		 mystepper2(mintegrable2);
		ChTimestepperEuleroExplIIorder   mystepper3(mintegrable3);
		ChTimestepperHHT				 mystepper4(mintegrable4);
		ChTimestepperHHT				 mystepper5(mintegrable5);
		mystepper4.SetAlpha(0);		// HHT with no dissipation -> trapezoidal
		mystepper5.SetAlpha(-0.33);  // HHT with max dissipation 

		// Execute the time integration
		while (mystepper1.GetTime() <4)
		{
			mystepper1.Advance(0.05);
			mystepper2.Advance(0.05);
			mystepper3.Advance(0.05);
			mystepper4.Advance(0.05);
			mystepper5.Advance(0.05);

			GetLog() << "T = " << mystepper1.GetTime() << "  x=" << mystepper1.X()(0) << "  v=" << mystepper1.V()(0) << "\n";
			log_file4 << mystepper1.GetTime()	<< ", " << mystepper1.X()(0) << ", " << mystepper1.V()(0)
												<< ", " << mystepper2.X()(0) << ", " << mystepper2.V()(0)
												<< ", " << mystepper3.X()(0) << ", " << mystepper3.V()(0)
												<< ", " << mystepper4.X()(0) << ", " << mystepper4.V()(0)
												<< ", " << mystepper5.X()(0) << ", " << mystepper5.V()(0)
												<< "\n";
		}

	}



	if (true)
	{
		//
		// EXAMPLE 5:
		//

		GetLog() << "\n\n Example 5: integrate pendulum DAE \n";


		// Define a class inherited from ChIntegrableIIorder,
		// it will represent the differential equations
		// by implementing the interfaces to implicit solvers.
		//  We assume   M*a = F(x,v,t) 
		//            C(x,t)=0;

		class MyIntegrable : public ChIntegrableIIorder
		{
		private:
			double M;
			double K;
			double R;
			double mT;
			double mpx;
			double mpy;
			double mvx;
			double mvy;
			double mlength;
		public:
			MyIntegrable()
			{
				M = 1;
				K = 2;
				R = 0;
				mT = 0;
				mpx =  0;
				mpy = -1;
				mvx = 0.2;
				mvy = 0;	
				mlength = 1;
			}

			/// the number of coordinates in the state, x position part:
			virtual int GetNcoords_x() { return 2; }

			/// Tells the number of lagrangian multipliers (constraints)
			virtual int GetNconstr() { return 1; }

			/// system -> state
			virtual void StateGather(ChState& x, ChStateDelta& v, double& T)
			{
				x(0) = mpx;
				x(1) = mpy;
				v(0) = mvx;
				v(1) = mvy;
				T = mT;
			};

			/// state -> system  
			virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T)
			{
				mpx = x(0);
				mpy = x(1);
				mvx = v(0);
				mvy = v(1);
				mT = T;
			};

			/// compute  dy/dt=f(y,t) 
			virtual void StateSolveA(ChStateDelta& dvdt,		///< result: computed accel. a=dv/dt
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

				ChVector<> dirpend(-mpx, -mpy, 0);
				dirpend.Normalize();
				ChVectorDynamic<> b(3);
				b(0) = (sin(mT * 20) * 0.0002 -K*mpx - R*mvx);
				b(1) = 0;
				b(2) = 0;
				ChMatrixDynamic<> A(3, 3);
				A(0, 0) = M;
				A(1, 1) = M;
				A(0, 2) = dirpend.x;
				A(1, 2) = dirpend.y;
				A(2, 0) = dirpend.x;
				A(2, 1) = dirpend.y;
				ChVectorDynamic<> w(3);
				ChLinearAlgebra::Solve_LinSys(A, &b, &w);
		GetLog() << "SolveA:  w=" << w << "\n";
		GetLog() << "     for x=" << mpx << " y=" << mpy << " vx=" << mvx << " vy=" << mvy << " dir=" << dirpend << "\n";
				dvdt(0) = w(0);
				dvdt(1) = w(1);
				L(0)    = w(2);
			}

			/// Compute the correction with linear system
			///  Dv = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R 
			virtual void StateSolveCorrection(
				ChStateDelta& Dv,	  ///< result: computed Dv 
				ChVectorDynamic<>& L, ///< result: computed lagrangian multipliers, if any
				const ChVectorDynamic<>& R, ///< the R residual
				const ChVectorDynamic<>& Qc,///< the Qc residual
				const double c_a,	  ///< the factor in c_a*M
				const double c_v,	  ///< the factor in c_v*dF/dv
				const double c_x,	  ///< the factor in c_x*dF/dv
				const ChState& x,	  ///< current state, x part
				const ChStateDelta& v,///< current state, v part
				const double T,		  ///< current time T
				bool force_state_scatter = true ///< if false, x,v and T are not scattered to the system, assuming that someone has done StateScatter just before 
				)
			{
				if (force_state_scatter)
					this->StateScatter(x, v, T);

				ChVector<> dirpend(-mpx, -mpy, 0);
				dirpend.Normalize();
				ChVectorDynamic<> b(3);
				b(0) = R(0);
				b(1) = R(1);
				b(2) = Qc(0);
				ChMatrixDynamic<> A(3, 3);
				A(0, 0) = c_a*this->M + c_v *(-this->R) + c_x * (-this->K);
				A(1, 1) = c_a*this->M;
				A(0, 2) = dirpend.x;
				A(1, 2) = dirpend.y;
				A(2, 0) = dirpend.x;
				A(2, 1) = dirpend.y;
				ChVectorDynamic<> w(3);
				ChLinearAlgebra::Solve_LinSys(A, &b, &w);
				Dv(0) = w(0);
				Dv(1) = w(1);
				L(0)  = w(2);
			}

			///    R += c*F
			void LoadResidual_F(
				ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*F 
				const double c				 ///< a scaling factor
				)
			{
				R(0) += c * (sin(mT * 20) * 0.0002 - this->K*mpx - this->R*mvx);
				R(1) += c * 0;
			};

			///    R += c*M*w 
			void LoadResidual_Mv(
				ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*M*v 
				const ChVectorDynamic<>& w,  ///< the w vector 
				const double c				 ///< a scaling factor
				)
			{
				R(0) += c * this->M * w(0);
				R(1) += c * this->M * w(1);
			};

			///   R += Cq'*l
			virtual void LoadResidual_CqL(
				ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
				const ChVectorDynamic<>& L,  ///< the L vector 
				const double c				 ///< a scaling factor
				)
			{
				ChVector<> dirpend(-mpx, -mpy, 0);
				dirpend.Normalize();
				R(0) += c * dirpend.x * L(0);
				R(1) += c * dirpend.y * L(0);
			};

			///  Qc += c * C
			virtual void LoadConstraint_C(
				ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
				const double c				 ///< a scaling factor
				)
			{
				ChVector<> distpend(-mpx, -mpy, 0);
				Qc(0) += -c * (-distpend.Length() + mlength);
			};

			/// nothing to do here- no rheonomic part
			virtual void LoadConstraint_Ct(
				ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*Ct 
				const double c				 ///< a scaling factor
				)
			{};

		};


		// Create a file to dump results
		ChStreamOutAsciiFile log_file5("log_timestepper_5.dat");


		// Create and object from your custom integrable class:
		MyIntegrable mintegrable1;
		MyIntegrable mintegrable2;
		MyIntegrable mintegrable3;
		MyIntegrable mintegrable4;
		MyIntegrable mintegrable5;

		// Create few time-integrators to be compared:
		ChTimestepperEuleroExplIIorder   mystepper1(mintegrable3);
		ChTimestepperEulerImplicit		 mystepper2(mintegrable1);
		ChTimestepperTrapezoidal		 mystepper3(mintegrable2);
		ChTimestepperHHT				 mystepper4(mintegrable4);
		ChTimestepperHHT				 mystepper5(mintegrable5);
		mystepper4.SetAlpha(0);		// HHT with no dissipation -> trapezoidal
		mystepper5.SetAlpha(-0.33);  // HHT with max dissipation 

		// Execute the time integration
		while (mystepper1.GetTime() <4)
		{
			mystepper1.Advance(0.05);
			mystepper2.Advance(0.05);
			mystepper3.Advance(0.05);
			mystepper4.Advance(0.05);
			mystepper5.Advance(0.05);

			GetLog() << "T = " << mystepper1.GetTime() << "  x=" << mystepper1.X()(0) << "  y=" << mystepper1.X()(1) << "\n";
			log_file5 << mystepper1.GetTime() 
				<< ", " << mystepper1.X()(0) << ", " << mystepper1.X()(1) << ", " << mystepper1.V()(0) << ", " << mystepper1.V()(1)
				<< ", " << mystepper2.X()(0) << ", " << mystepper2.X()(1) << ", " << mystepper2.V()(0) << ", " << mystepper2.V()(1)
				<< ", " << mystepper3.X()(0) << ", " << mystepper3.X()(1) << ", " << mystepper3.V()(0) << ", " << mystepper3.V()(1)
				<< ", " << mystepper4.X()(0) << ", " << mystepper4.X()(1) << ", " << mystepper4.V()(0) << ", " << mystepper4.V()(1)
				<< ", " << mystepper5.X()(0) << ", " << mystepper5.X()(1) << ", " << mystepper5.V()(0) << ", " << mystepper5.V()(1)
				<< "\n";
		}

	}





	GetLog() << "\n  CHRONO execution terminated.";
	

	//system("pause");
	return 0;
}
 

