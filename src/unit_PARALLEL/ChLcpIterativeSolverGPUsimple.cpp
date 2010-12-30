///////////////////////////////////////////////////
//
//   ChLcpIterativeSolverGPUsimple.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cutil.h>
#include <cuda_runtime_api.h>
//#include "physics/ChLinkFastContact.h"
#include "ChLcpIterativeSolverGPUsimple.h"
#include "ChLcpConstraintTwoGPUcontN.h"
#include "ChCuda.h"
#include "ChLcpIterativeSolverGPUsimpleCU.h"
#include "physics/ChBody.h"


// Forward declarations

extern "C" void ChRunKernelLCPaddForces( dim3 dim_grid, 
										 dim3 dim_threads, 
										 CH_REALNUMBER4* d_buffer_bodies,
										 unsigned int bodies_data_pitch,
										 CH_REALNUMBER mforceFactor);

extern "C" void ChRunKernelLCPiteration( dim3 dim_grid, 
										 dim3 dim_threads, 
										 //unsigned int shmem_size, 
										 CH_REALNUMBER4* d_buffer_contacts, 
										 CH_REALNUMBER4* d_buffer_bodies,
										 CH_REALNUMBER4* d_buffer_reduction,
										 unsigned int contacts_data_pitch,
										 unsigned int bodies_data_pitch,
										 unsigned int reduction_data_pitch,
										 CH_REALNUMBER mLcpOmega);

extern "C" void ChRunKernelLCPiterationBilateral( dim3 dim_grid, 
										 dim3 dim_threads, 
										 CH_REALNUMBER4* d_buffer_bilaterals, 
										 CH_REALNUMBER4* d_buffer_bodies,
										 CH_REALNUMBER4* d_buffer_reduction,
										 unsigned int bilaterals_data_pitch,
										 unsigned int bodies_data_pitch,
										 unsigned int reduction_data_pitch,
										 CH_REALNUMBER mLcpOmega); 

extern "C" void ChRunKernelContactsPreprocess(dim3 dim_grid, 
										 dim3 dim_threads,
										 CH_REALNUMBER4*  d_buffer_contacts, 
										 CH_REALNUMBER4*  d_buffer_bodies, 
										 unsigned int contacts_data_pitch,
										 unsigned int bodies_data_pitch,
										 CH_REALNUMBER mCfactor,
										 CH_REALNUMBER max_recovery_speed);

extern "C" void ChRunKernelIntegrateTimeStep(dim3 dim_grid, 
										 dim3 dim_threads, 
										 CH_REALNUMBER4*  d_buffer_bodies,
										 CH_REALNUMBER4*  d_buffer_reduction,
										 unsigned int bodies_data_pitch,
										 unsigned int reduction_data_pitch,
										 CH_REALNUMBER mstepSize,
										 bool normalize_quaternion);

extern "C" void ChRunKernelLCPreduction(dim3 dim_reduction_grid, 
										dim3 dim_reduction_threads, 
										CH_REALNUMBER4* d_buffer_reduction, 
										CH_REALNUMBER4* d_buffer_bodies,
									    unsigned int reduction_data_pitch,
										unsigned int bodies_data_pitch,
										int max_repetitions);
	
extern "C" void  ChRunKernelLCPspeedupdate(dim3 dim_integration_grid, 
										dim3 dim_integration_threads, 
										CH_REALNUMBER4* d_buffer_bodies,
										CH_REALNUMBER4* d_buffer_reduction,
										unsigned int bodies_data_pitch,
										unsigned int reduction_data_pitch);

namespace chrono
{






ChLcpIterativeSolverGPUsimple::ChLcpIterativeSolverGPUsimple( 
				ChContactContainerGPUsimple* mcont,
				int mmax_iters,     ///< max.number of iterations
				bool mwarm_start,	///< uses warm start?
				double mtolerance,  ///< tolerance for termination criterion
				double momega,      ///< overrelaxation criterion
				int max_num_GPU_contacts,
				int max_num_GPU_bodies,
				int max_num_GPU_bilaterals
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance, momega)
{
	#ifndef CH_CUDAGPUEMULATION
		//CUT_DEVICE_INIT(); // explicited as below
		do {                                             
			int deviceCount;                                                         
			CUDA_SAFE_CALL_NO_SYNC(cudaGetDeviceCount(&deviceCount));                
			if (deviceCount == 0) {  
				GetLog() << "There is no GPU device for CUDA.\n"; 
				exit(EXIT_FAILURE);                                                  
			}                                                                        
			int dev; 
			GetLog() << "ChLcpIterativeSolverGPUsimple device count = " << deviceCount << "\n";
			for (dev = 0; dev < deviceCount; ++dev) {                                
				cudaDeviceProp deviceProp;                                           
				CUDA_SAFE_CALL_NO_SYNC(cudaGetDeviceProperties(&deviceProp, dev)); 
				GetLog() << " Installed CUDA device n." << dev << " n.mprocessors=" << deviceProp.multiProcessorCount << " - " <<  deviceProp.name  << "\n";
				if (deviceProp.major == 9999)
					GetLog() << "Warning:  CUDA hardware not available!\n\n";

				if (deviceProp.major >= 1)                                           
					break;                                                           
			}                                                                        
			if (dev == deviceCount) {                                                
				GetLog() << "There's no device supporting CUDA v.1.0 or higher \n";
				exit(EXIT_FAILURE);                                                  
			}                                                                       
			else 
			{
				int already_dev;
				if (! (cudaSuccess == cudaGetDevice(&already_dev)))
				{
					GetLog() << "ChLcpIterativeSolverGPUsimple will set GPU device.\n";
					CUDA_SAFE_CALL(cudaSetDevice(dev));
				}
				else
					GetLog() << "ChLcpIterativeSolverGPUsimple tried to set GPU device, but already =" << already_dev << "\n";
			}
		} while (0);

	#endif

	this->dt= 0;
	this->max_recoveryspeed = 0;
	this->C_factor= 0;
	this->n_bodies_GPU=0;
	this->n_contacts_GPU=0; 
	this->n_bilaterals_GPU=0; 
	this->do_integration_step=false;
	this->step_counter = 0;


	max_GPU_contacts  = max_num_GPU_contacts;
	max_GPU_bodies    = max_num_GPU_bodies;
	max_GPU_bilaterals= max_num_GPU_bilaterals;


	// Allocate memory for contact buffer on host
	unsigned int contacts_mem_size = max_GPU_contacts * CH_CONTACT_VSIZE * CH_CONTACT_HSIZE;
	CUDA_SAFE_CALL( cudaMallocHost( &h_buffer_contacts, contacts_mem_size )); // pinned memory - faster than malloc()!

	// Allocate memory for contact buffer on device
	CUDA_SAFE_CALL( cudaMalloc( (void**) &d_buffer_contacts, contacts_mem_size));


	// Allocate memory for reduction buffer on device
	unsigned int reduction_mem_size = 2 * (max_GPU_contacts + max_GPU_bilaterals) *  CH_REDUCTION_VSIZE * CH_REDUCTION_HSIZE;
	CUDA_SAFE_CALL( cudaMalloc( (void**) &d_buffer_reduction, reduction_mem_size));


	// Allocate memory for bilaterals buffer on host
	unsigned int bilaterals_mem_size = max_GPU_bilaterals * CH_BILATERAL_VSIZE * CH_BILATERAL_HSIZE;
	CUDA_SAFE_CALL( cudaMallocHost( &h_buffer_bilaterals, bilaterals_mem_size )); // pinned memory - faster than malloc()!

	// Allocate memory for bilaterals buffer on device
	CUDA_SAFE_CALL( cudaMalloc( (void**) &d_buffer_bilaterals, bilaterals_mem_size));


	// Allocate memory for body buffer on host
	unsigned int bodies_mem_size = max_GPU_bodies * CH_BODY_VSIZE * CH_BODY_HSIZE;
	CUDA_SAFE_CALL( cudaMallocHost( &h_buffer_bodies, bodies_mem_size )); // pinned memory - faster than malloc()!

	// Allocate memory for body buffer on device
	CUDA_SAFE_CALL( cudaMalloc( (void**) &d_buffer_bodies, bodies_mem_size)); 

	gpu_contact_container = mcont;
};
				

ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple() 
{
	// cleanup memory

	CUDA_SAFE_CALL(cudaFreeHost(h_buffer_contacts));
	CUDA_SAFE_CALL(cudaFreeHost(h_buffer_bodies));
	CUDA_SAFE_CALL(cudaFreeHost(h_buffer_bilaterals));
    CUDA_SAFE_CALL(cudaFree(d_buffer_contacts));
	CUDA_SAFE_CALL(cudaFree(d_buffer_bodies));
	CUDA_SAFE_CALL(cudaFree(d_buffer_bilaterals));
	CUDA_SAFE_CALL(cudaFree(d_buffer_reduction));

	//CUT_EXIT(argc, argv); //***TO DO***???
};

		




unsigned int ChLcpIterativeSolverGPUsimple::ComputeGpuMemSize(unsigned int used_items,	/// number of items in buffer
								unsigned int Hsize,			/// horizontal size of each item in bytes
								unsigned int Vsize,			/// vertical size of each item (if item is a structure, let=1)
								unsigned int nkernels,		/// number of types of kernels (ie. with different threadblock sizes)
								unsigned int* BlocksHSizes, /// pass here a buffer of the threadblock sizes, for each kernel type
								unsigned int* GridHsizes,	/// here you will get the buffer of widths of the grids for each kernel type
								unsigned int* TotThreads,	/// here you will get the buffer of total computed threads for each kernel type
								unsigned int& pitch			/// here you will get the pitch for accessing m-th row (ex. my_y = buffer[n+ 2*pitch].y)
								)
{
	unsigned int gpu_el_size = Hsize*Vsize;
	unsigned int gpu_mem_size = 0;
	pitch = 0;

	for (int i=0; i<nkernels; i++)
	{
		GridHsizes[i] = (unsigned int)ceil(  ((double)used_items) / ((double)(BlocksHSizes[i]))  );
		TotThreads[i] = GridHsizes[i]*BlocksHSizes[i];
		if (pitch < TotThreads[i])
		{
			pitch = TotThreads[i];
			gpu_mem_size = gpu_el_size * TotThreads[i];
		}
	}

	return gpu_mem_size;
}





float __host_int_as_float(int a)
{ 
 union {int a; float b;} u; 
 u.a = a; 
 return u.b; 
}

 
double ChLcpIterativeSolverGPUsimple::Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f							///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				)
{ 
	assert(gpu_contact_container); // must know where is the ChContactContainerGPUsimple

	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	CH_REALNUMBER4* h_contacts   = (CH_REALNUMBER4*)this->h_buffer_contacts;
	CH_REALNUMBER4* h_bodies     = (CH_REALNUMBER4*)this->h_buffer_bodies;
	CH_REALNUMBER4* h_bilaterals = (CH_REALNUMBER4*)this->h_buffer_bilaterals;

	unsigned int reduction_steps = 0;


	// -1-  Count active variables and initialize GPU body buffer
	//

	unsigned int bodies_data_pitch = max_GPU_bodies; //=mvariables.size(); // upper limit on n. of bodies

	n_bodies_GPU=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvariables[iv]->SetOffset(n_bodies_GPU);	// also store offset, useful for GPU bookkeeping
			
			assert(dynamic_cast<ChLcpVariablesBody*>(mvariables[iv]));

			ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[iv];
			ChBody* mbody = (ChBody*)mbodyvars->GetUserData();

			unsigned int memLocation = n_bodies_GPU;
			if(mvariables[iv]->IsActive() && (add_Mq_to_f))
			{
				h_bodies[memLocation].x = (CH_REALNUMBER) mvariables[iv]->Get_qb().GetElementN(0);
				h_bodies[memLocation].y = (CH_REALNUMBER) mvariables[iv]->Get_qb().GetElementN(1);
				h_bodies[memLocation].z = (CH_REALNUMBER) mvariables[iv]->Get_qb().GetElementN(2);
				h_bodies[memLocation].w = 0;
				memLocation += bodies_data_pitch;
				h_bodies[memLocation].x = (CH_REALNUMBER) mvariables[iv]->Get_qb().GetElementN(3);
				h_bodies[memLocation].y = (CH_REALNUMBER) mvariables[iv]->Get_qb().GetElementN(4);
				h_bodies[memLocation].z = (CH_REALNUMBER) mvariables[iv]->Get_qb().GetElementN(5);
				h_bodies[memLocation].w = 0;
				memLocation += bodies_data_pitch;
			}
			else
			{
				h_bodies[memLocation].x = (CH_REALNUMBER) 0;
				h_bodies[memLocation].y = (CH_REALNUMBER) 0;
				h_bodies[memLocation].z = (CH_REALNUMBER) 0;
				h_bodies[memLocation].w = 0;
				memLocation += bodies_data_pitch;
				h_bodies[memLocation].x = (CH_REALNUMBER) 0;
				h_bodies[memLocation].y = (CH_REALNUMBER) 0;
				h_bodies[memLocation].z = (CH_REALNUMBER) 0;
				h_bodies[memLocation].w = 0;
				memLocation += bodies_data_pitch;
			}
			h_bodies[memLocation].x = (CH_REALNUMBER) mbody->GetPos().x;
			h_bodies[memLocation].y = (CH_REALNUMBER) mbody->GetPos().y;
			h_bodies[memLocation].z = (CH_REALNUMBER) mbody->GetPos().z;
			h_bodies[memLocation].w = 0;
			memLocation += bodies_data_pitch;
			h_bodies[memLocation].x = (CH_REALNUMBER) mbody->GetRot().e0;
			h_bodies[memLocation].y = (CH_REALNUMBER) mbody->GetRot().e1;
			h_bodies[memLocation].z = (CH_REALNUMBER) mbody->GetRot().e2;
			h_bodies[memLocation].w = (CH_REALNUMBER) mbody->GetRot().e3;
			memLocation += bodies_data_pitch;
			if(mvariables[iv]->IsActive())
			{
				h_bodies[memLocation].x = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->GetBodyInvInertia())(0,0);
				h_bodies[memLocation].y = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->GetBodyInvInertia())(1,1);
				h_bodies[memLocation].z = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->GetBodyInvInertia())(2,2);
				h_bodies[memLocation].w = (CH_REALNUMBER) (1.0/((ChLcpVariablesBody*)mvariables[iv])->GetBodyMass());
				memLocation += bodies_data_pitch;
			
				h_bodies[memLocation].x = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->Get_fb().ElementN(0)); // mbody->Get_Xforce().x;
				h_bodies[memLocation].y = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->Get_fb().ElementN(1)); // mbody->Get_Xforce().y;
				h_bodies[memLocation].z = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->Get_fb().ElementN(2)); //mbody->Get_Xforce().z;
				h_bodies[memLocation].w = 0.; 
				memLocation += bodies_data_pitch;
				
				h_bodies[memLocation].x = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->Get_fb().ElementN(3)); //mbody->Get_Xtorque().x;
				h_bodies[memLocation].y = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->Get_fb().ElementN(4)); //mbody->Get_Xtorque().y;
				h_bodies[memLocation].z = (CH_REALNUMBER) (((ChLcpVariablesBody*)mvariables[iv])->Get_fb().ElementN(5)); //mbody->Get_Xtorque().z;
				h_bodies[memLocation].w = 0.;
			}
			else
			{
				h_bodies[memLocation].x = (CH_REALNUMBER) 0;
				h_bodies[memLocation].y = (CH_REALNUMBER) 0;
				h_bodies[memLocation].z = (CH_REALNUMBER) 0;
				h_bodies[memLocation].w = (CH_REALNUMBER) 0;
				memLocation += bodies_data_pitch;

				h_bodies[memLocation].x = (CH_REALNUMBER) 0;
				h_bodies[memLocation].y = (CH_REALNUMBER) 0;
				h_bodies[memLocation].z = (CH_REALNUMBER) 0;
				h_bodies[memLocation].w = 0.; 
				memLocation += bodies_data_pitch;
				
				h_bodies[memLocation].x = (CH_REALNUMBER) 0;
				h_bodies[memLocation].y = (CH_REALNUMBER) 0;
				h_bodies[memLocation].z = (CH_REALNUMBER) 0;
				h_bodies[memLocation].w = 0.;
			}
			
			
			n_bodies_GPU ++;

			if (n_bodies_GPU >= max_GPU_bodies)
				GetLog() << "ERROR Overflow of GPU bodies!!! \n";
			assert(n_bodies_GPU < max_GPU_bodies);
		}

	}
	
	int* counter_repetitions = new int[n_bodies_GPU];
	int* counter_rep_address = new int[n_bodies_GPU];
	for (unsigned int te = 0; te < n_bodies_GPU; te++)
	{
		counter_repetitions[te]=0;
		counter_rep_address[te]=0;
	}



	// -2-  Count active constraints and initialize GPU contact buffer.
	//      Only constraints generated by contacts will be dealt by the GPU.

	unsigned int contacts_data_pitch    = max_GPU_contacts;
	unsigned int bilaterals_data_pitch  = max_GPU_bilaterals;
	unsigned int reduction_data_pitch   = 2 * (max_GPU_contacts + max_GPU_bilaterals);


	n_contacts_GPU   =0;
	n_bilaterals_GPU =0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		{
			assert(!dynamic_cast<ChLcpConstraintTwoContactN*>(mconstraints[ic]));

			if (ChLcpConstraintTwoGPUcontN* mcont = dynamic_cast<ChLcpConstraintTwoGPUcontN*>(mconstraints[ic]))
			{
				// Update auxiliary data in all constraints before starting,
				//  // mconstraints[ic]->Update_auxiliary(); NOT NEEDED because will be computed by contact preprocesing kernel

				unsigned int memLocation = n_contacts_GPU;
				  //normal vector, in world coords
				h_contacts[memLocation].x = (CH_REALNUMBER) mcont->GetNormal().x;
				h_contacts[memLocation].y = (CH_REALNUMBER) mcont->GetNormal().y;
				h_contacts[memLocation].z = (CH_REALNUMBER) mcont->GetNormal().z;
				h_contacts[memLocation].w = 0;   //not used...
				memLocation += (3*contacts_data_pitch);

				  //contact point on body 1, in world coords
				h_contacts[memLocation].x = (CH_REALNUMBER) mcont->GetP1().x;  //s_1,w . x
				h_contacts[memLocation].y = (CH_REALNUMBER) mcont->GetP1().y;  //s_1,w . y 
				h_contacts[memLocation].z = (CH_REALNUMBER) mcont->GetP1().z;  //s_1,w . z
				if (mcont->GetVariables_a()->IsActive())
				{
					unsigned int B1 = mcont->GetVariables_a()->GetOffset();
					h_contacts[memLocation].w = (CH_REALNUMBER) B1; //pointer to body B1 info in body buffer
					counter_repetitions[B1] +=1; // increment reference count to body
				}
				else 
					h_contacts[memLocation].w = (CH_REALNUMBER) -1; // set as inactive body 1
				memLocation += (3*contacts_data_pitch);

				  //contact point on body 2, in world coords
				h_contacts[memLocation].x = (CH_REALNUMBER) mcont->GetP2().x;  //s_2,w . x
				h_contacts[memLocation].y = (CH_REALNUMBER) mcont->GetP2().y;  //s_2,w . y
				h_contacts[memLocation].z = (CH_REALNUMBER) mcont->GetP2().z;  //s_2,w . z
				if (mcont->GetVariables_b()->IsActive())
				{
					unsigned int B2 = mcont->GetVariables_b()->GetOffset();
					h_contacts[memLocation].w = (CH_REALNUMBER) B2; //pointer to body B2 info in body buffer
					counter_repetitions[B2] +=1; // increment reference count to body
				}
				else 
					h_contacts[memLocation].w = (CH_REALNUMBER) -1; // set as inactive body 2
				memLocation += (3*contacts_data_pitch);

				  //mu & gamma information for warm starting
				if (this->warm_start)
				{
					h_contacts[memLocation].x = (CH_REALNUMBER) mcont->Get_l_i();  //gamma x
					h_contacts[memLocation].y = (CH_REALNUMBER) mcont->GetTangentialConstraintU()->Get_l_i();  //gamma y
					h_contacts[memLocation].z = (CH_REALNUMBER) mcont->GetTangentialConstraintV()->Get_l_i();  //gamma z
				}
				else
				{
					h_contacts[memLocation].x = (CH_REALNUMBER) 0; //gamma x
					h_contacts[memLocation].y = (CH_REALNUMBER) 0; //gamma y
					h_contacts[memLocation].z = (CH_REALNUMBER) 0; //gamma z
				}
				h_contacts[memLocation].w = (CH_REALNUMBER) mcont->GetFrictionCoefficient();  //mu

				n_contacts_GPU++;

				if (n_contacts_GPU >= max_GPU_contacts)
					GetLog() << "ERROR Overflow of GPU contacts!!! \n";

				assert(n_contacts_GPU < max_GPU_contacts);
			}

			else
			{
				if (ChLcpConstraintTwoBodies* mbilateral = dynamic_cast<ChLcpConstraintTwoBodies*>(mconstraints[ic]))
				{
					// Update auxiliary data in all constraints before starting,
					// that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
					mconstraints[ic]->Update_auxiliary();  //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed

					unsigned int memLocation = n_bilaterals_GPU;
					  //J1x
					h_bilaterals[memLocation].x = (CH_REALNUMBER) mbilateral->Get_Cq_a()->GetElementN(0);
					h_bilaterals[memLocation].y = (CH_REALNUMBER) mbilateral->Get_Cq_a()->GetElementN(1);
					h_bilaterals[memLocation].z = (CH_REALNUMBER) mbilateral->Get_Cq_a()->GetElementN(2);
					if (mbilateral->GetVariables_a()->IsActive())
					{
						unsigned int B1 = mbilateral->GetVariables_a()->GetOffset();
						h_bilaterals[memLocation].w = (CH_REALNUMBER) B1; //pointer to body B1 info in body buffer
						counter_repetitions[B1] +=1; // increment reference count to body
					}
					else
						h_bilaterals[memLocation].w = (CH_REALNUMBER) -1;
					memLocation += bilaterals_data_pitch;

					  //J2x
					h_bilaterals[memLocation].x = (CH_REALNUMBER) mbilateral->Get_Cq_b()->GetElementN(0);
					h_bilaterals[memLocation].y = (CH_REALNUMBER) mbilateral->Get_Cq_b()->GetElementN(1);
					h_bilaterals[memLocation].z = (CH_REALNUMBER) mbilateral->Get_Cq_b()->GetElementN(2);
					if (mbilateral->GetVariables_b()->IsActive())
					{
						unsigned int B2 = mbilateral->GetVariables_b()->GetOffset();
						h_bilaterals[memLocation].w = (CH_REALNUMBER) B2; //pointer to body B2 info in body buffer
						counter_repetitions[B2] +=1; // increment reference count to body
					}
					else 
						h_bilaterals[memLocation].w = (CH_REALNUMBER) -1; // set as inactive body 2
					memLocation += bilaterals_data_pitch;

					  //J1w
					h_bilaterals[memLocation].x = (CH_REALNUMBER) mbilateral->Get_Cq_a()->GetElementN(3);
					h_bilaterals[memLocation].y = (CH_REALNUMBER) mbilateral->Get_Cq_a()->GetElementN(4);
					h_bilaterals[memLocation].z = (CH_REALNUMBER) mbilateral->Get_Cq_a()->GetElementN(5);
					h_bilaterals[memLocation].w = 0; 
					memLocation += bilaterals_data_pitch;
					  //J2w
					h_bilaterals[memLocation].x = (CH_REALNUMBER) mbilateral->Get_Cq_b()->GetElementN(3);
					h_bilaterals[memLocation].y = (CH_REALNUMBER) mbilateral->Get_Cq_b()->GetElementN(4);
					h_bilaterals[memLocation].z = (CH_REALNUMBER) mbilateral->Get_Cq_b()->GetElementN(5);
					h_bilaterals[memLocation].w = 0;
					memLocation += bilaterals_data_pitch;
					  // eta, b, gamma, 0
					h_bilaterals[memLocation].x = (CH_REALNUMBER) (1.0/mbilateral->Get_g_i());  // eta = 1/g
					h_bilaterals[memLocation].y = (CH_REALNUMBER) mbilateral->Get_b_i();  // b_i is residual b 

					if (this->warm_start)
						h_bilaterals[memLocation].z = (CH_REALNUMBER) mbilateral->Get_l_i();  // l_i is gamma multiplier
					else
						h_bilaterals[memLocation].z = (CH_REALNUMBER) 0;

					if (mbilateral->IsUnilateral())
						h_bilaterals[memLocation].w = (CH_REALNUMBER) 1.;
					else
						h_bilaterals[memLocation].w = (CH_REALNUMBER) 0;
		
					n_bilaterals_GPU++;

					if (n_bilaterals_GPU >= max_GPU_bilaterals)
						GetLog() << "ERROR Overflow of GPU bilateral constraints!!! \n";

					assert(n_bilaterals_GPU < max_GPU_bilaterals);
				}
			}

		} // end if active
	} // end loop


	// Fill the vector with the base offsets of body accumulators in the reduction buffer,
	// and understand how many repetitions of reduction kernel are necessary
	unsigned int tot = 0;
	unsigned int max_repetitions = 0;
	for (unsigned int ri = 0; ri < n_bodies_GPU; ri++)
	{
		if (counter_repetitions[ri])
			counter_rep_address[ri] = tot;
		else
			counter_rep_address[ri] = -1;
		tot += counter_repetitions[ri];

		if (counter_repetitions[ri] > max_repetitions)
			max_repetitions = counter_repetitions[ri];
	}


	// Each contact data must have R1, R2 indexes pointing to unique slots in reduction buffer,
	// and n1, n2 reflecting its counter in the repeated slots, as 0,1,2,3,0,1,2,0,1,2,3,4,5,0,1
	unsigned int my_contact_GPU = 0;
	unsigned int my_bilateral_GPU = 0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		{
			if (ChLcpConstraintTwoGPUcontN* mcont = dynamic_cast<ChLcpConstraintTwoGPUcontN*>(mconstraints[ic]))
			{
				unsigned int memLocation = my_contact_GPU + 10*contacts_data_pitch;
				if (mcont->GetVariables_a()->IsActive())
				{
					unsigned int B1 = mcont->GetVariables_a()->GetOffset();
					h_contacts[memLocation].x = (CH_REALNUMBER) counter_rep_address[B1] + counter_repetitions[B1]-1;
					h_contacts[memLocation].z = (CH_REALNUMBER) (counter_repetitions[B1]-1);
					counter_repetitions[B1] --;
				}
				if (mcont->GetVariables_b()->IsActive())
				{
					unsigned int B2 = mcont->GetVariables_b()->GetOffset();
					h_contacts[memLocation].y = (CH_REALNUMBER) counter_rep_address[B2] + counter_repetitions[B2]-1;
					h_contacts[memLocation].w = (CH_REALNUMBER) (counter_repetitions[B2]-1);
					counter_repetitions[B2] --;
				}
				my_contact_GPU++;
			}
			else
			{
				if (ChLcpConstraintTwoBodies* mbilateral = dynamic_cast<ChLcpConstraintTwoBodies*>(mconstraints[ic]))
				{
					unsigned int memLocation = my_bilateral_GPU + 5*bilaterals_data_pitch;
					if (mbilateral->GetVariables_a()->IsActive())
					{
						unsigned int B1 = mbilateral->GetVariables_a()->GetOffset();
						h_bilaterals[memLocation].x = (CH_REALNUMBER) counter_rep_address[B1] + counter_repetitions[B1]-1;
						h_bilaterals[memLocation].z = (CH_REALNUMBER) (counter_repetitions[B1]-1);
						counter_repetitions[B1] --;
					}
					if (mbilateral->GetVariables_b()->IsActive())
					{
						unsigned int B2 = mbilateral->GetVariables_b()->GetOffset();	
						h_bilaterals[memLocation].y = (CH_REALNUMBER) counter_rep_address[B2] + counter_repetitions[B2]-1;	
						h_bilaterals[memLocation].w = (CH_REALNUMBER) (counter_repetitions[B2]-1);
						counter_repetitions[B2] --;
					}
					my_bilateral_GPU++;
				}
			}

		} // end if active
	} // end loop


	// Each body data must also have the R index to fetch accumulated speeds from
	// the reduction buffer..
	unsigned int my_body_GPU=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			h_bodies[my_body_GPU].w = (CH_REALNUMBER) counter_rep_address[mvariables[iv]->GetOffset()];
			my_body_GPU ++;
		}
	}




	dim3  dim_addforces_threads( CH_LCPADDFORCES_THREADS_PER_BLOCK,     1, 1);
    dim3  dim_addforces_grid   ( (unsigned int)ceil( ((double)n_bodies_GPU)/((double)dim_addforces_threads.x) ) ,      1, 1);
	unsigned int tot_threads_addforces = dim_addforces_threads.x * dim_addforces_grid.x;

	dim3  dim_preproc_threads( CH_PREPROCESSING_THREADS_PER_BLOCK,   1, 1);
    dim3  dim_preproc_grid   ( (unsigned int)ceil( ((double)n_contacts_GPU)/((double)dim_preproc_threads.x) ), 1, 1);
	unsigned int tot_threads_preproc = dim_preproc_threads.x * dim_preproc_grid.x;

	dim3  dim_lcp_threads( CH_LCPITERATION_THREADS_PER_BLOCK,     1, 1);
	dim3  dim_lcp_grid   ( (unsigned int)ceil( ((double)n_contacts_GPU)/((double)dim_lcp_threads.x) ) ,   1, 1);
	unsigned int tot_threads_lcp = dim_lcp_threads.x * dim_lcp_grid.x;

	dim3  dim_lcpbilaterals_threads( CH_LCPITERATIONBILATERALS_THREADS_PER_BLOCK,     1, 1);
	dim3  dim_lcpbilaterals_grid   ( (unsigned int)ceil( ((double)n_bilaterals_GPU)/((double)dim_lcpbilaterals_threads.x) ),   1, 1);
	unsigned int tot_threads_lcpbilaterals = dim_lcpbilaterals_threads.x * dim_lcpbilaterals_grid.x;

	unsigned int n_reduction_GPU = 2 * (n_contacts_GPU + n_bilaterals_GPU);
	dim3  dim_reduction_threads( CH_REDUCTION_THREADS_PER_BLOCK,   1, 1);
    dim3  dim_reduction_grid   ( (unsigned int)ceil( ((double)n_reduction_GPU)/((double)dim_reduction_threads.x) ), 1, 1);
	unsigned int tot_threads_reduction = dim_reduction_threads.x * dim_reduction_grid.x;

	dim3  dim_updatespeeds_threads( CH_SPEEDUPDATE_THREADS_PER_BLOCK,     1, 1);
	dim3  dim_updatespeeds_grid   ( (unsigned int)ceil( ((double)n_bodies_GPU)/((double)dim_updatespeeds_threads.x) ) ,      1, 1);
	unsigned int tot_threads_updatespeeds = dim_updatespeeds_threads.x * dim_updatespeeds_grid.x;


	 // Last constraints for padding thread block: avoid 'garbage integers' for body pointers (so, at
	 // least set them  as inactive bodies)
	
	int tot_threads_contacts = ChMax((int)tot_threads_preproc,(int)tot_threads_lcp);
	for (int ipadding = n_contacts_GPU; ipadding < tot_threads_contacts; ipadding++)
	{
		h_contacts[ipadding+(3*contacts_data_pitch)].w = (CH_REALNUMBER) -1; 
		h_contacts[ipadding+(6*contacts_data_pitch)].w = (CH_REALNUMBER) -1;
		h_contacts[ipadding+(10*contacts_data_pitch)].x = (CH_REALNUMBER) 0;
		h_contacts[ipadding+(10*contacts_data_pitch)].y = (CH_REALNUMBER) 0;
		h_contacts[ipadding+(10*contacts_data_pitch)].z = (CH_REALNUMBER) 0;
		h_contacts[ipadding+(10*contacts_data_pitch)].w = (CH_REALNUMBER) 0;
	}

	for (int ipadding = n_bilaterals_GPU; ipadding < tot_threads_lcpbilaterals; ipadding++)
	{
		h_bilaterals[ipadding                       ].w = (CH_REALNUMBER) -1;
		h_bilaterals[ipadding+ bilaterals_data_pitch].w = (CH_REALNUMBER) -1;
		h_bilaterals[ipadding+(5*bilaterals_data_pitch)].x = (CH_REALNUMBER) 0;
		h_bilaterals[ipadding+(5*bilaterals_data_pitch)].y = (CH_REALNUMBER) 0;
		h_bilaterals[ipadding+(5*bilaterals_data_pitch)].z = (CH_REALNUMBER) 0;
		h_bilaterals[ipadding+(5*bilaterals_data_pitch)].w = (CH_REALNUMBER) 0;
	}

	// Copy bodies from host to device
	CUDA_SAFE_CALL( cudaMemcpy(  d_buffer_bodies,
								 h_buffer_bodies, 
								 max_GPU_bodies * CH_BODY_VSIZE * CH_BODY_HSIZE,
								 cudaMemcpyHostToDevice) );

	 // Copy contacts from host to device
	CUDA_SAFE_CALL( cudaMemcpy(  d_buffer_contacts,
								 h_buffer_contacts, 
								 max_GPU_contacts * CH_CONTACT_VSIZE * CH_CONTACT_HSIZE,
								 cudaMemcpyHostToDevice) );

     // Copy bilaterals from host to device
	CUDA_SAFE_CALL( cudaMemcpy(  d_buffer_bilaterals,
								 h_buffer_bilaterals, 
								 max_GPU_bilaterals * CH_BILATERAL_VSIZE * CH_BILATERAL_HSIZE,
								 cudaMemcpyHostToDevice) );

	// Reset to zero the reduction buffer
	CUDA_SAFE_CALL( cudaMemset(  d_buffer_reduction, 
								 0, 
								 2*(max_GPU_contacts + max_GPU_bilaterals)* CH_REDUCTION_VSIZE * CH_REDUCTION_HSIZE) );



	// -3-  EXECUTE JACOBIAN PREPROCESSING KERNEL ===============
	//      Computes the jacobians from contact points and normals.
	//      Computes the eta value.
	//      This kernel acts on the stream of contacts.

	double cfactor = gpu_contact_container->Get_load_C_factor();
	double maxrecspeed = gpu_contact_container->Get_load_max_recovery_speed();
	if (gpu_contact_container->Get_load_do_clamp() == false) 
		maxrecspeed = 10e25;

	ChRunKernelContactsPreprocess(  dim_preproc_grid, 
									dim_preproc_threads, 
									(CH_REALNUMBER4*)d_buffer_contacts, 
									(CH_REALNUMBER4*)d_buffer_bodies,
									contacts_data_pitch,
									bodies_data_pitch,
									(CH_REALNUMBER)cfactor,
									(CH_REALNUMBER)maxrecspeed);


	// -4-  EXECUTE KERNEL FOR ADDING FORCES ======================
	//      Set [inv_M]*k term into the speeds 
	//      Do old_speeds+= h*[inv_M]*f  (so skip the computation of k on host, where other CCP solvers did +=[inv_M]*k)
	//      This kernel acts on the stream of bodies.

	ChRunKernelLCPaddForces  (		dim_addforces_grid, 
									dim_addforces_threads, 
									(CH_REALNUMBER4*)d_buffer_bodies,
									bodies_data_pitch,
									1.0 ); // was.. (CH_REALNUMBER)this->F_factor);  but now forces come from 'fb', already scaled by 1/dt
 

	// -5-  EXECUTE KERNEL FOR LOOPING ALL THE LCP ITERATIONS =====
	//      This kernel is executed multiple times up to the
	//      maximum number of iterations.
	//      This kernel is applied to the stream of contacts.
	
	for (int iter = 0; iter < max_iterations; iter++)
	{
		ChRunKernelLCPiteration(	dim_lcp_grid, 
									dim_lcp_threads, 
									(CH_REALNUMBER4*)d_buffer_contacts, 
									(CH_REALNUMBER4*)d_buffer_bodies,
									(CH_REALNUMBER4*)d_buffer_reduction,
									contacts_data_pitch,
									bodies_data_pitch,
									reduction_data_pitch,
									(CH_REALNUMBER)this->omega);
		if(n_bilaterals_GPU)
			ChRunKernelLCPiterationBilateral( dim_lcpbilaterals_grid, 
									dim_lcpbilaterals_threads, 
									(CH_REALNUMBER4*)d_buffer_bilaterals, 
									(CH_REALNUMBER4*)d_buffer_bodies,
									(CH_REALNUMBER4*)d_buffer_reduction,
									bilaterals_data_pitch,
									bodies_data_pitch,
									reduction_data_pitch,
									(CH_REALNUMBER)this->omega);


		ChRunKernelLCPreduction(	dim_reduction_grid, 
									dim_reduction_threads, 
									(CH_REALNUMBER4*)d_buffer_reduction, 
									(CH_REALNUMBER4*)d_buffer_bodies,
									reduction_data_pitch,
									bodies_data_pitch,
									max_repetitions);

			// from reduction buffer to body buffer : v_new = v_old + accumulated_delta_speeds
		ChRunKernelLCPspeedupdate(	dim_updatespeeds_grid, 
									dim_updatespeeds_threads, 
									(CH_REALNUMBER4*)d_buffer_bodies,
									(CH_REALNUMBER4*)d_buffer_reduction,
									bodies_data_pitch,
									reduction_data_pitch);

	}



	// -6-  Get back the values of the multipliers
	//      and copy them into the constraints (only the last row, the one with the multipliers!)

	CUDA_SAFE_CALL( cudaMemcpy(		 h_buffer_contacts,
									 d_buffer_contacts, 
									 max_GPU_contacts * CH_CONTACT_VSIZE * CH_CONTACT_HSIZE,
									 cudaMemcpyDeviceToHost) );

	CUDA_SAFE_CALL( cudaMemcpy(		 h_buffer_bilaterals,
									 d_buffer_bilaterals, 
									 max_GPU_bilaterals * CH_BILATERAL_VSIZE * CH_BILATERAL_HSIZE,
									 cudaMemcpyDeviceToHost) );



	unsigned int fetchLocation = 9*contacts_data_pitch;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		{
			if (mconstraints[ic]->IsGPUcompatible())
			{
				ChLcpConstraintTwoGPUcontN* mcont;
				if (mcont = dynamic_cast<ChLcpConstraintTwoGPUcontN*>(mconstraints[ic]))
				{
					mcont->Set_l_i((double) h_contacts[fetchLocation].x );  //gamma x
					mcont->GetTangentialConstraintU()->Set_l_i((double) h_contacts[fetchLocation].y); // gamma y
					mcont->GetTangentialConstraintV()->Set_l_i((double) h_contacts[fetchLocation].z); // gamma z
					fetchLocation++;
				}
			}
		}
	}

 
	// -7-  Perform also a 1st order Eulero integration (advance the positions
	//      for one timestep as p=p+v*dt) if requested (so avoid doing this after the Solve() on the
	//      cpu, which is slower than gpu.)

	if (this->do_integration_step)
	{
		dim3  dim_integration_threads( CH_LCPINTEGRATE_THREADS_PER_BLOCK,     1, 1);
		dim3  dim_integration_grid   ( (unsigned int)ceil( ((double)n_bodies_GPU)/((double)dim_integration_threads.x) ) ,      1, 1);
		unsigned int tot_threads_integration = dim_integration_threads.x * dim_integration_grid.x;

		bool normalize=false;
		this->step_counter++; 
		if (step_counter>10)
		{
			normalize = true;
			this->step_counter = 0;
		}
			// fetch from reduction buffer and integrate
		ChRunKernelIntegrateTimeStep(	dim_integration_grid, 
										dim_integration_threads, 
										(CH_REALNUMBER4*)d_buffer_bodies,
										(CH_REALNUMBER4*)d_buffer_reduction,
										bodies_data_pitch,
										reduction_data_pitch,
										(CH_REALNUMBER)this->dt,
										normalize);
	}




	// -8-  Get back the values of body buffer 
	// 

	CUDA_SAFE_CALL( cudaMemcpy(		 h_buffer_bodies,
									 d_buffer_bodies, 
									 max_GPU_bodies * CH_BODY_VSIZE * CH_BODY_HSIZE,
									 cudaMemcpyDeviceToHost) );

	// -9-  Get back the values of the q vectors (the speeds, only the first two rows of the body buffer)
	//      and copy them into the ChLcpVariable objects 

	unsigned int fetchbLocation = 0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{		
			mvariables[iv]->Get_qb().SetElementN(0, (double)h_bodies[fetchbLocation].x );
			mvariables[iv]->Get_qb().SetElementN(1, (double)h_bodies[fetchbLocation].y );
			mvariables[iv]->Get_qb().SetElementN(2, (double)h_bodies[fetchbLocation].z );

		    mvariables[iv]->Get_qb().SetElementN(3, (double)h_bodies[fetchbLocation + bodies_data_pitch].x );
			mvariables[iv]->Get_qb().SetElementN(4, (double)h_bodies[fetchbLocation + bodies_data_pitch].y );
			mvariables[iv]->Get_qb().SetElementN(5, (double)h_bodies[fetchbLocation + bodies_data_pitch].z );

			fetchbLocation ++;
		}
	}
	
	// -10-  Update also body positions if integration has been performed

	if (this->do_integration_step)
	{
		unsigned int fetchbLocation = 0;
		for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		{
			if (mvariables[iv]->IsActive()) 
			{	
				ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[iv];
				ChBody* mbody = (ChBody*)mbodyvars->GetUserData();
				
				CH_REALNUMBER4 hv = h_bodies[fetchbLocation];
				ChVector<> newv( hv.x, hv.y, hv.z );

				CH_REALNUMBER4 hw = h_bodies[fetchbLocation+ 1*bodies_data_pitch];
				ChVector<> newangvel( hw.x, hw.y, hw.z );

				CH_REALNUMBER4 hpos = h_bodies[fetchbLocation+ 2*bodies_data_pitch];
				ChVector<> newpos( hpos.x, hpos.y, hpos.z );

				CH_REALNUMBER4 hrot = h_bodies[fetchbLocation+ 3*bodies_data_pitch];
				ChQuaternion<> newrot( hrot.x, hrot.y, hrot.z, hrot.w );
				/* //DEBUG
				ChQuaternion<> mdeltarot;
				ChQuaternion<> moldrot = mbody->GetRot();
				ChVector<> newwel_abs = (*mbody->GetA()) * newangvel;
				double mangle = newwel_abs.Length() * this->dt;
				newwel_abs.Normalize();
				mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
				ChQuaternion<> mnnnewrot = mdeltarot % moldrot;

				GetLog() << "Var q=" << newrot << " len=" << newrot.Length();
				GetLog() << "  should be q=" << mnnnewrot;
				GetLog() << "  \n\n";
				*/

				mbody->SetCoord(ChCoordsys<>(newpos, newrot));
				mbody->SetPos_dt(newv);
				mbody->SetWvel_loc(newangvel);
				mbody->ClampSpeed();
				mbody->ComputeGyro();

				/* ***TO DO***
				if (step)
				{
					mbody->SetPos_dtdt( (mbody->GetCoord_dt().pos - old_coord_dt.pos)  / step);
					mbody->SetRot_dtdt( (mbody->GetCoord_dt().rot - old_coord_dt.rot)  / step);
				}
				*/

				fetchbLocation ++;
			} 
		}

	}

	delete [] counter_repetitions; counter_repetitions = 0;
	delete [] counter_rep_address; counter_rep_address = 0;

	return 0;
}




void ChLcpIterativeSolverGPUsimple::IntegrateTimeStep(double mdt)
{
	assert (false); 
	//***DEPRECATED***
}





} // END_OF_NAMESPACE____


 