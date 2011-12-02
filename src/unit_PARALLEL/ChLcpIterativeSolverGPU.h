#ifndef CHLCPITERATIVESOLVERGPUSIMPLE_H
#define CHLCPITERATIVESOLVERGPUSIMPLE_H
///////////////////////////////////////////////////
//
//   ChLcpIterativeSolverGPU.h
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "lcp/ChLcpIterativeSolver.h"
#include "ChContactContainerGPUsimple.h"
#include "ChLcpSystemDescriptorGPU.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "ChApiGPU.h"
#include "ChLcpIterativeGPU.h"
#include "ChForceSolverGPU.h"
namespace chrono {
	///    An iterative LCP solver based on projective
	///   fixed point method, with overrelaxation.
	///   This solver runs on GPU boards from NVIDIA, version G80
	///   or more recent. The GPU programming relies on CUDA library.

	class ChApiGPU ChLcpIterativeSolverGPUsimple: public ChLcpIterativeSolver {
		public:
			//
			// CONSTRUCTORS
			//

			ChLcpIterativeSolverGPUsimple(ChContactContainerGPUsimple* container = 0); ///< this solver can be used _only_ together with a ChContactContainerGPUsimple

			virtual ~ChLcpIterativeSolverGPUsimple();

			//
			// FUNCTIONS
			//

			/// Performs the solution of the LCP.
			/// \return  the maximum constraint violation after termination.

			virtual double Solve(ChLcpSystemDescriptor& sysd, ///< system description with constraints and variables
					bool add_Mq_to_f = false); ///< if true, takes the initial 'q' and adds [M]*q to 'f' vector

			/// This solver can be used _only_ with a ChContactContainerGPUsimple !!
			/// Set it when you create.
			void SetGPUcontactContainer(ChContactContainerGPUsimple* mcont) {
				this->gpu_contact_container = mcont;
			}

			/// Set the integration dt step. Remember to update this value before calling Solve() , because
			/// the postprocessing will take care of time step integration, which depends on the dt.
			/// ***OBSOLETE**** do not use this
			void SetDt(double dt) {
				if (dt > 0) mDt = dt;
			}
			double GetDt() {
				return mDt;
			}

			/// Set the maximum recovery speed for stabilization. Remember to update this value
			/// before calling Solve() , because the preprocessing will take care of building contact
			/// residuals with stabilization factors, which depends on this recovery speed value.
			void SetMaxRecoverySpeed(double mrs) {
				if (mrs > 0) mMaxRecoverySpeed = mrs;
			}
			double GetMaxRecoverySpeed() {
				return mMaxRecoverySpeed;
			}

			/// Set the multiplier for the contact residuals. Remember to update this value
			/// before calling Solve() , because the preprocessing will take care of building contact
			/// residuals with stabilization factors, which depends on this recovery speed value.
			void SetC_factor(double mf) {
				if (mf > 0) mCFactor = mf;
			}
			double GetC_factor() {
				return mCFactor;
			}

			void SetSystemDescriptor(ChLcpSystemDescriptorGPU* mdescriptor);
			float Total_KineticEnergy() {
				return gpu_solver->Total_KineticEnergy();
			}

			ChGPUDataManager * data_container;
			ChLcpIterativeGPU * gpu_solver;
			int number_of_bodies;

			double mDt; //Timestep
			double mTolerance; //Tolerance for Solver
			double mMaxRecoverySpeed;
			double mCFactor;
			double mOmegaBilateral;
			double mOmegaContact;
			unsigned int mMaxIterations;

		protected:
			//
			// DATA
			//

			// for the preprocesing of contacts
			ChContactContainerGPUsimple* gpu_contact_container; // will be asked which was C factor,max.speed.clamping etc. so to perform contact preprocessing in solver (Dan' jacobian/residual gpu computation)

		private:
	};
} // END_OF_NAMESPACE____

#endif
