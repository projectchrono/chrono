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
			ChContactContainerGPUsimple* mcont = 0, ///< this solver can be used _only_ together with a ChContactContainerGPUsimple 
			int mmax_iters=50,      ///< max.number of iterations
			bool mwarm_start=false,	///< uses warm start?
			double mtolerance=0.0,  ///< tolerance for termination criterion
			double momega=0.2,      ///< overrelaxation criterion
			int max_num_GPU_contacts=60000,  ///< maximum allowed number of contacts
			int max_num_GPU_bodies=10000,	 ///< maximum allowed number of bodies
			int max_num_GPU_bilaterals=1024  ///< maximum allowed number of bilateral constraints
			)  ;


		virtual ~ChLcpIterativeSolverGPUsimple();

		//
		// FUNCTIONS
		//

		/// Performs the solution of the LCP.
		/// \return  the maximum constraint violation after termination.

		virtual double Solve(
			ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
			bool add_Mq_to_f = false					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
			);

		/// This solver can be used _only_ with a ChContactContainerGPUsimple !!
		/// Set it when you create.
		void SetGPUcontactContainer(ChContactContainerGPUsimple* mcont) {this->gpu_contact_container = mcont;}

		/// Set the integration dt step. Remember to update this value before calling Solve() , because 
		/// the postprocessing will take care of time step integration, which depends on the dt.
		/// ***OBSOLETE**** do not use this
		void SetDt(double mdt) {if (mdt>0) this->dt = mdt; }
		double GetDt() {return dt;} 

		/// Set the maximum recovery speed for stabilization. Remember to update this value 
		/// before calling Solve() , because the preprocessing will take care of building contact 
		/// residuals with stabilization factors, which depends on this recovery speed value.
		void SetMaxRecoverySpeed(double mrs) {if (mrs>0) this->max_recoveryspeed = mrs; }
		double GetMaxRecoverySpeed() {return max_recoveryspeed;}

		/// Set the multiplier for the contact residuals. Remember to update this value 
		/// before calling Solve() , because the preprocessing will take care of building contact 
		/// residuals with stabilization factors, which depends on this recovery speed value.
		void SetC_factor(double mf) {if (mf>0) this->C_factor = mf; }
		double GetC_factor() {return C_factor;}



		/// After you performed Solve(), you can call the following function to execute a
		/// GPU kernel for doing the first order Eulero integration of body positions as p=p+v*dt 
		/// The integration kernel reuses the same data structures just used for the Solve(), so it
		/// can be much faster then doing the integration on the serial processor.
		/// ***OBSOLETE**** do not use this
		virtual void IntegrateTimeStep(double mdt);

		/// Turn on this functionality if you want that the integration step is
		/// performed by a CUDA kernel, right after the solution of the LCP.
		virtual void Set_do_integration_step(bool md) {this->do_integration_step = md;}
		virtual bool Get_do_integration_step() {return this->do_integration_step;}

		void SetSystemDescriptor(ChLcpSystemDescriptorGPU* mdescriptor);

	protected:
		//
		// DATA
		//

		// for the preprocesing of contacts
		double dt;
		double max_recoveryspeed;
		double C_factor;

		unsigned int max_GPU_bodies;
		unsigned int max_GPU_contacts;
		unsigned int max_GPU_bilaterals;

		unsigned int n_bodies_GPU;
		unsigned int n_contacts_GPU;
		unsigned int n_bilaterals_GPU;

		bool do_integration_step;
		unsigned int step_counter; // this to have normalization on quaternion only each N steps

		thrust::host_vector<float4>   h_bodies;
		thrust::host_vector<float4>   h_bilaterals;

		ChContactContainerGPUsimple* gpu_contact_container; // will be asked which was C factor,max.speed.clamping etc. so to perform contact preprocessing in solver (Dan' jacobian/residual gpu computation)
		ChLcpSystemDescriptorGPU* mSystemDescriptor;


	private:
		/// We assume each thread processes an item, on the GPU.
		///  Given an array of items on the GPU, each with Hsize*Vsize layout, 
		/// a basic approach would lead to allocate (used_items*Hsize*Vsize) bytes
		/// on the GPU, that's the strict necessary, but this may be wrong because
		/// later you may want to process that buffer with kernels that go in parallel
		/// in grids of threadblocks, where threadblocks is some value like 128 or such, 
		/// but since the number of threadblocks must be integer, for some buffer sizes
		/// either it may happen that the last elements are not processed or the buffer 
		/// is processed beyond its boundary. 
		///  Therefore, this function suggests you a buffer size that is an integer multiple 
		/// of threadblock size, at the cost of leaving some elements unused at the end of the buffer.
		/// It will be up to you to take care of such padding of unused elements.
		///  Also, this function tells you which is the smallest integer number of threadblocks 
		/// that you must use.
		/// \return : the suggested buffer size in bytes

		unsigned int ComputeGpuMemSize(unsigned int used_items,	///< number of items in buffer
			unsigned int Hsize,			///< horizontal size of each item in bytes
			unsigned int Vsize,			///< vertical size ('rows') of each item (if item is a structure, let=1)
			unsigned int nkernels,		///< number of types of kernels (ie. with different threadblock sizes)
			unsigned int* BlocksHSizes, ///< pass here a buffer of the threadblock sizes, for each kernel type
			unsigned int* GridHsizes,	///< here you will get the buffer of widths of the grids for each kernel type
			unsigned int* TotThreads,	///< here you will get the buffer of total computed threads for each kernel type
			unsigned int& pitch			///< here you will get the pitch for accessing n-th el. of m-th row (ex. my_y = buffer[n+ m*pitch].y)
			);


	};

	
void ChRunSolverTimestep(
		bool do_integration_step,
		CH_REALNUMBER max_recovery_speed,
		CH_REALNUMBER mCfactor,
		CH_REALNUMBER mstepSize,
		uint bodies_data_pitch,
		uint contacts_data_pitch,
		uint bilaterals_data_pitch,
		// uint reduction_data_pitch,
		float mLcpOmega,
		uint max_iterations,
		uint n_bilaterals_GPU,
		uint n_contacts_GPU,
		uint n_bodies_GPU,
		float4* d_buffer_contacts,
		thrust::host_vector<float4> &d_bodies,
		thrust::host_vector<float4> &d_bilaterals);

} // END_OF_NAMESPACE____


#endif 