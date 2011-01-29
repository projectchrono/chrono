

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
		ChContactContainerGPUsimple* container,
		int maxIteration,		///< max.number of iterations
		double tolerance,		///< tolerance for termination criterion
		double omega			///< overrelaxation criterion
		){
		mDt= 0;
		mDoIntegrationStep=true;
		mStepCounter = 0;
		mMaxIterations=maxIteration;
		mTolerance=tolerance;
		gpu_contact_container = container;
		mOmega=omega;

	};


	ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple() {
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

		for (unsigned int i=0; i<nkernels; i++)
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






	void ChLcpIterativeSolverGPUsimple::SetSystemDescriptor(ChLcpSystemDescriptorGPU* mdescriptor){
		mSystemDescriptor=mdescriptor;
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
		h_bodies.resize(bodies_data_pitch*CH_BODY_VSIZE);
		mSystemDescriptor->nBodiesGPU=bodies_data_pitch;

		for (unsigned int iv = 0; iv< bodies_data_pitch; iv++)
		{
			//assert(dynamic_cast<ChLcpVariablesBody*>(mvariables[iv]));
			ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[iv];
			ChBody* mbody = (ChBody*)mbodyvars->GetUserData();
						int state=0;
			float3 vel=F3(mbodyvars->Get_qb().GetElementN(0),mbodyvars->Get_qb().GetElementN(1),mbodyvars->Get_qb().GetElementN(2));
		
			if(mbody->IsActive()){state=0;}else{state=2;}
			
			if(mbody->IsActive()){
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
			
		}

		// -2-  Count active constraints and initialize GPU contact buffer.
		//      Only constraints generated by contacts will be dealt by the GPU.

		unsigned int contacts_data_pitch    = mSystemDescriptor->maxContacts;
		unsigned int bilaterals_data_pitch  = mSystemDescriptor->maxBilaterals;

		mSystemDescriptor->nContactsGPU=mSystemDescriptor->nContactsGPU;
		h_bilaterals.resize(mSystemDescriptor->maxBilaterals * CH_BILATERAL_VSIZE);
		unsigned int n_bilaterals_GPU=0;
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
		mSystemDescriptor->nBilateralsGPU =n_bilaterals_GPU;
		// -3-  EXECUTE KERNELS ===============

		double maxrecspeed = gpu_contact_container->Get_load_max_recovery_speed();
		if (gpu_contact_container->Get_load_do_clamp() == false) {maxrecspeed = 10e25;}
		ChRunSolverTimestep(
			maxrecspeed, 
			gpu_contact_container->Get_load_C_factor(), 
			mDt,
			mTolerance,
			bodies_data_pitch, 
			contacts_data_pitch, 
			bilaterals_data_pitch,
			mOmega, 
			mMaxIterations, 
			n_bilaterals_GPU,
			mSystemDescriptor->nContactsGPU , 
			mSystemDescriptor->nBodiesGPU,
			mSystemDescriptor->vContactsGPU,
			h_bodies,
			h_bilaterals);


		if (mDoIntegrationStep)
		{
			for (unsigned int iv = 0; iv< mvariables.size(); iv++){	
				mvariables[iv]->Get_qb().SetElementN(0, (double)h_bodies[iv].x );
				mvariables[iv]->Get_qb().SetElementN(1, (double)h_bodies[iv].y );
				mvariables[iv]->Get_qb().SetElementN(2, (double)h_bodies[iv].z );
				mvariables[iv]->Get_qb().SetElementN(3, (double)h_bodies[iv + bodies_data_pitch].x );
				mvariables[iv]->Get_qb().SetElementN(4, (double)h_bodies[iv + bodies_data_pitch].y );
				mvariables[iv]->Get_qb().SetElementN(5, (double)h_bodies[iv + bodies_data_pitch].z );


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
		return 0;
	}

	void ChLcpIterativeSolverGPUsimple::IntegrateTimeStep(double mdt)
	{
		assert (false); 
		//***DEPRECATED***
	}





} // END_OF_NAMESPACE____

