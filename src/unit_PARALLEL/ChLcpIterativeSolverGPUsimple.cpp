

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
#include "physics/ChBody.h"
#include "physics/ChSystem.h"

// Forward declarations



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
		int max_num_GPU_bilaterals)  
		: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance, momega)
	{
		this->dt= 0;
		this->max_recoveryspeed = 0;
		this->C_factor= 0;
		this->n_bodies_GPU=0;
		this->n_contacts_GPU=0; 
		this->n_bilaterals_GPU=0; 
		this->do_integration_step=true;
		this->step_counter = 0;
		this->max_iterations=mmax_iters;
		gpu_contact_container = mcont;
	};


	ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple() 
	{
		// cleanup memory

		//CUDA_SAFE_CALL(cudaFreeHost(h_buffer_contacts));
		//CUDA_SAFE_CALL(cudaFreeHost(h_buffer_bodies));
		//CUDA_SAFE_CALL(cudaFreeHost(h_buffer_bilaterals));
		//		CUDA_SAFE_CALL(cudaFree(d_buffer_contacts));
		//CUDA_SAFE_CALL(cudaFree(d_buffer_bodies));
		//CUDA_SAFE_CALL(cudaFree(d_buffer_bilaterals));
		//CUDA_SAFE_CALL(cudaFree(d_buffer_reduction));

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

	void ChLcpIterativeSolverGPUsimple::SetSystemDescriptor(ChLcpSystemDescriptorGPU* mdescriptor){

		mSystemDescriptor=mdescriptor;

		max_GPU_contacts  = mSystemDescriptor->maxContacts;
		max_GPU_bodies    = mSystemDescriptor->maxBodies;
		max_GPU_bilaterals= mSystemDescriptor->maxBilaterals;

		h_bilaterals.resize(max_GPU_bilaterals * CH_BILATERAL_VSIZE);
		//h_bodies.resize(max_GPU_bodies * CH_BODY_VSIZE);
	}

	double ChLcpIterativeSolverGPUsimple::Solve(
		ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
		bool add_Mq_to_f					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
		)
	{ 
		assert(gpu_contact_container); // must know where is the ChContactContainerGPUsimple

		std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
		std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

		unsigned int reduction_steps = 0;


		// -1-  Count active variables and initialize GPU body buffer
		//
		unsigned int bodies_data_pitch = mvariables.size();// upper limit on n. of bodies
		h_bodies.resize(mvariables.size()*CH_BODY_VSIZE);
		n_bodies_GPU=mvariables.size();
		for (unsigned int iv = 0; iv< bodies_data_pitch; iv++)
		{
			mvariables[iv]->SetOffset(iv);	// also store offset, useful for GPU bookkeeping
			assert(dynamic_cast<ChLcpVariablesBody*>(mvariables[iv]));
			ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[iv];
			ChBody* mbody = (ChBody*)mbodyvars->GetUserData();
			int state=0;
			if(mbody->IsActive()){state=0;}else{state=2;}
			if(add_Mq_to_f){
				h_bodies[iv+bodies_data_pitch*0] = F4(mbodyvars->Get_qb().GetElementN(0),mbodyvars->Get_qb().GetElementN(1),mbodyvars->Get_qb().GetElementN(2),state);
				h_bodies[iv+bodies_data_pitch*1] = F4(mbodyvars->Get_qb().GetElementN(3),mbodyvars->Get_qb().GetElementN(4),mbodyvars->Get_qb().GetElementN(5),0);
			}else{
				h_bodies[iv+bodies_data_pitch*0]=F4(0,0,0,state);
				h_bodies[iv+bodies_data_pitch*1]=F4(0);
			}
			h_bodies[iv+bodies_data_pitch*2] = F4(mbody->GetPos().x,mbody->GetPos().y,mbody->GetPos().z,0);
			h_bodies[iv+bodies_data_pitch*3] = F4(mbody->GetRot().e0,mbody->GetRot().e1,mbody->GetRot().e2,mbody->GetRot().e3);
			h_bodies[iv+bodies_data_pitch*4] = F4(mbodyvars->GetBodyInvInertia()(0,0),mbodyvars->GetBodyInvInertia()(1,1),mbodyvars->GetBodyInvInertia()(2,2),(1.0)/(mbodyvars->GetBodyMass()));//forces
			h_bodies[iv+bodies_data_pitch*5] = F4(mbodyvars->Get_fb().ElementN(0),mbodyvars->Get_fb().ElementN(1),mbodyvars->Get_fb().ElementN(2),0);//torques
			h_bodies[iv+bodies_data_pitch*6] = F4(mbodyvars->Get_fb().ElementN(3),mbodyvars->Get_fb().ElementN(4),mbodyvars->Get_fb().ElementN(5),0);
			//if (n_bodies_GPU >= max_GPU_bodies)
			//	GetLog() << "ERROR Overflow of GPU bodies!!! \n";
			//assert(n_bodies_GPU < max_GPU_bodies);

		}
		// -2-  Count active constraints and initialize GPU contact buffer.
		//      Only constraints generated by contacts will be dealt by the GPU.

		unsigned int contacts_data_pitch    = max_GPU_contacts;
		unsigned int bilaterals_data_pitch  = max_GPU_bilaterals;

		n_contacts_GPU=mSystemDescriptor->nContactsGPU;

		n_bilaterals_GPU =0;
		for (unsigned int ic = 0; ic< mconstraints.size(); ic++){
			if (mconstraints[ic]->IsActive()){	
				if (ChLcpConstraintTwoBodies* mbilateral = dynamic_cast<ChLcpConstraintTwoBodies*>(mconstraints[ic])){
					// Update auxiliary data in all constraints before starting,
					// that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
					mconstraints[ic]->Update_auxiliary();  //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed

					//J1x
					h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*0] = F4(mbilateral->Get_Cq_a()->GetElementN(0),mbilateral->Get_Cq_a()->GetElementN(1),mbilateral->Get_Cq_a()->GetElementN(2),0);
					if (mbilateral->GetVariables_a()->IsActive()){
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*0].w = (CH_REALNUMBER) mbilateral->GetVariables_a()->GetOffset(); //pointer to body B1 info in body buffer
					}else{
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*0].w = (CH_REALNUMBER) -1;
					}
					//J2x
					h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*1] = F4(mbilateral->Get_Cq_b()->GetElementN(0),mbilateral->Get_Cq_b()->GetElementN(1),mbilateral->Get_Cq_b()->GetElementN(2),0);
					if (mbilateral->GetVariables_b()->IsActive()){
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*1].w = (CH_REALNUMBER) mbilateral->GetVariables_b()->GetOffset(); //pointer to body B2 info in body buffer
					}else{ 
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*1].w = (CH_REALNUMBER) -1; // set as inactive body 2
					}
					//J1w
					h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*2] = F4(mbilateral->Get_Cq_a()->GetElementN(3),mbilateral->Get_Cq_a()->GetElementN(4),mbilateral->Get_Cq_a()->GetElementN(5),0);
					//J2w
					h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*3] = F4(mbilateral->Get_Cq_b()->GetElementN(3),mbilateral->Get_Cq_b()->GetElementN(4),mbilateral->Get_Cq_b()->GetElementN(5),0);
					// eta, b, gamma, 0
					h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*4].x = (CH_REALNUMBER) (1.0/mbilateral->Get_g_i());		// eta = 1/g
					h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*4].y = (CH_REALNUMBER) mbilateral->Get_b_i();			// b_i is residual b 

					if (this->warm_start){
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*4].z = (CH_REALNUMBER) mbilateral->Get_l_i();// l_i is gamma multiplier
					}else{
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*4].z = (CH_REALNUMBER) 0;
					}

					if (mbilateral->IsUnilateral()){
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*4].w = (CH_REALNUMBER) 1.;
					}else{
						h_bilaterals[n_bilaterals_GPU+bilaterals_data_pitch*4].w = (CH_REALNUMBER) 0;
					}
					n_bilaterals_GPU++;
				}


			} // end if active
		} // end loop
		dim3  dim_addforces_threads( CH_LCPADDFORCES_TPB,     1, 1);
		dim3  dim_addforces_grid   ( (unsigned int)ceil( ((double)n_bodies_GPU)/((double)dim_addforces_threads.x) ) ,      1, 1);
		unsigned int tot_threads_addforces = dim_addforces_threads.x * dim_addforces_grid.x;

		dim3  dim_preproc_threads( CH_PREPROCESSING_TPB,   1, 1);
		dim3  dim_preproc_grid   ( (unsigned int)ceil( ((double)n_contacts_GPU)/((double)dim_preproc_threads.x) ), 1, 1);
		unsigned int tot_threads_preproc = dim_preproc_threads.x * dim_preproc_grid.x;

		dim3  dim_lcp_threads( CH_LCPITERATION_TPB,     1, 1);
		dim3  dim_lcp_grid   ( (unsigned int)ceil( ((double)n_contacts_GPU)/((double)dim_lcp_threads.x) ) ,   1, 1);
		unsigned int tot_threads_lcp = dim_lcp_threads.x * dim_lcp_grid.x;

		dim3  dim_lcpbilaterals_threads( CH_LCPITERATIONBILATERALS_TPB,     1, 1);
		dim3  dim_lcpbilaterals_grid   ( (unsigned int)ceil( ((double)n_bilaterals_GPU)/((double)dim_lcpbilaterals_threads.x) ),   1, 1);
		unsigned int tot_threads_lcpbilaterals = dim_lcpbilaterals_threads.x * dim_lcpbilaterals_grid.x;

		unsigned int n_reduction_GPU = 2 * (n_contacts_GPU + n_bilaterals_GPU);
		dim3  dim_reduction_threads( CH_REDUCTION_TPB,   1, 1);
		dim3  dim_reduction_grid   ( (unsigned int)ceil( ((double)n_reduction_GPU)/((double)dim_reduction_threads.x) ), 1, 1);
		unsigned int tot_threads_reduction = dim_reduction_threads.x * dim_reduction_grid.x;

		dim3  dim_updatespeeds_threads( CH_SPEEDUPDATE_TPB,     1, 1);
		dim3  dim_updatespeeds_grid   ( (unsigned int)ceil( ((double)n_bodies_GPU)/((double)dim_updatespeeds_threads.x) ) ,      1, 1);
		unsigned int tot_threads_updatespeeds = dim_updatespeeds_threads.x * dim_updatespeeds_grid.x;

		// -3-  EXECUTE KERNELS ===============

		double maxrecspeed = gpu_contact_container->Get_load_max_recovery_speed();
		if (gpu_contact_container->Get_load_do_clamp() == false) {maxrecspeed = 10e25;}
		ChRunSolverTimestep(
			this->do_integration_step,
			maxrecspeed, 
			gpu_contact_container->Get_load_C_factor(), 
			(CH_REALNUMBER)this->dt,
			bodies_data_pitch, 
			contacts_data_pitch, 
			bilaterals_data_pitch,
			(CH_REALNUMBER)this->omega*2, 
			1000, 
			n_bilaterals_GPU,
			mSystemDescriptor->nContactsGPU , 
			n_bodies_GPU,
			(mSystemDescriptor->vContactsGPU),
			(h_bodies),
			(h_bilaterals));


		if (this->do_integration_step)
		{
			for (unsigned int iv = 0; iv< mvariables.size(); iv++){	
				mvariables[iv]->Get_qb().SetElementN(0, (double)h_bodies[iv].x );
				mvariables[iv]->Get_qb().SetElementN(1, (double)h_bodies[iv].y );
				mvariables[iv]->Get_qb().SetElementN(2, (double)h_bodies[iv].z );
				mvariables[iv]->Get_qb().SetElementN(3, (double)h_bodies[iv + bodies_data_pitch].x );
				mvariables[iv]->Get_qb().SetElementN(4, (double)h_bodies[iv + bodies_data_pitch].y );
				mvariables[iv]->Get_qb().SetElementN(5, (double)h_bodies[iv + bodies_data_pitch].z );

				if (this->do_integration_step){
					ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[iv];
					ChBody* mbody = (ChBody*)mbodyvars->GetUserData();
					if(mbody->IsActive()){
						CH_REALNUMBER4 hv   = h_bodies[iv+ 0*bodies_data_pitch];
						CH_REALNUMBER4 hw   = h_bodies[iv+ 1*bodies_data_pitch];
						CH_REALNUMBER4 hpos = h_bodies[iv+ 2*bodies_data_pitch];
						CH_REALNUMBER4 hrot = h_bodies[iv+ 3*bodies_data_pitch];
						ChVector<> newv( hv.x, hv.y, hv.z );
						ChVector<> newangvel( hw.x, hw.y, hw.z );
						ChVector<> newpos( hpos.x, hpos.y, hpos.z );
						ChQuaternion<> newrot( hrot.x, hrot.y, hrot.z, hrot.w );
						mbody->SetCoord(ChCoordsys<>(newpos, newrot));
						mbody->SetPos_dt(newv);
						mbody->SetWvel_loc(newangvel);
						mbody->ClampSpeed();
						mbody->ComputeGyro();
					}
				}
			}
		}
		return 0;
	}

	void ChLcpIterativeSolverGPUsimple::IntegrateTimeStep(double mdt)
	{
		assert (false); 
		//***DEPRECATED***
	}





} // END_OF_NAMESPACE____

