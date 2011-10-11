#ifndef CHLCPITERATIVESOLVERGPUSIMPLE_H
#define CHLCPITERATIVESOLVERGPUSIMPLE_H
///////////////////////////////////////////////////
//
//   ChLcpIterativeSolverGPUsimple.h
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
#include "ChLcpIterativeSolverGPU.h"
namespace chrono{
	///    An iterative LCP solver based on projective
	///   fixed point method, with overrelaxation.
	///   This solver runs on GPU boards from NVIDIA, version G80 
	///   or more recent. The GPU programming relies on CUDA library.

	class ChApiGPU ChLcpIterativeSolverGPUsimple : public ChLcpIterativeSolver
	{
	public:
		//
		// CONSTRUCTORS
		//

		ChLcpIterativeSolverGPUsimple(
			ChContactContainerGPUsimple* container = 0, ///< this solver can be used _only_ together with a ChContactContainerGPUsimple 
			ChLcpSystemDescriptorGPU* descriptor=0,

			int maxIteration=50,						///< max.number of iterations
			double dt=.01,
			double omega_contact=.2,
			double omega_bilateral=.2,
			bool DEM=false);							///< use DEM solver for contacts


		virtual ~ChLcpIterativeSolverGPUsimple();

		//
		// FUNCTIONS
		//

		/// Performs the solution of the LCP.
		/// \return  the maximum constraint violation after termination.

		virtual double Solve(
			ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables
			bool add_Mq_to_f = false);					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector


		/// This solver can be used _only_ with a ChContactContainerGPUsimple !!
		/// Set it when you create.
		void SetGPUcontactContainer(ChContactContainerGPUsimple* mcont) {this->gpu_contact_container = mcont;}

		/// Set the integration dt step. Remember to update this value before calling Solve() , because 
		/// the postprocessing will take care of time step integration, which depends on the dt.
		/// ***OBSOLETE**** do not use this
		void SetDt(double dt) {if (dt>0) mDt = dt; }
		double GetDt() {return mDt;} 

		/// Set the maximum recovery speed for stabilization. Remember to update this value 
		/// before calling Solve() , because the preprocessing will take care of building contact 
		/// residuals with stabilization factors, which depends on this recovery speed value.
		void SetMaxRecoverySpeed(double mrs) {if (mrs>0) mMaxRecoverySpeed = mrs; }
		double GetMaxRecoverySpeed() {return mMaxRecoverySpeed;}

		/// Set the multiplier for the contact residuals. Remember to update this value 
		/// before calling Solve() , because the preprocessing will take care of building contact 
		/// residuals with stabilization factors, which depends on this recovery speed value.
		void SetC_factor(double mf) {if (mf>0) mCFactor = mf; }
		double GetC_factor() {return mCFactor;}

		void SetSystemDescriptor(ChLcpSystemDescriptorGPU* mdescriptor);

		ChGPUDataManager		*		data_container;
int number_of_bodies;
	protected:
		//
		// DATA
		//

		// for the preprocesing of contacts
		double mDt;					//Timestep
		double mTolerance;			//Tolerance for Solver
		double mMaxRecoverySpeed;
		double mCFactor;
		double mOmega;
		unsigned int mMaxIterations;

		ChContactContainerGPUsimple*  	gpu_contact_container; // will be asked which was C factor,max.speed.clamping etc. so to perform contact preprocessing in solver (Dan' jacobian/residual gpu computation)
		ChLcpSystemDescriptorGPU*	  	mSystemDescriptor;
		ChLcpIterativeSolverGPU *		gpu_solver;

	private:
	};

} // END_OF_NAMESPACE____


#endif 
