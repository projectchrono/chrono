

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
#include "ChLcpIterativeSolverGPUsimple.h"
#include "ChLcpConstraintTwoGPUcontN.h"
#include "ChBodyGPU.h"
#include "ChCuda.h"
#include "physics/ChBody.h"
#include "physics/ChSystem.h"

// Forward declarations
namespace chrono
{
	ChLcpIterativeSolverGPUsimple::ChLcpIterativeSolverGPUsimple( 
		ChContactContainerGPUsimple* container,
		ChLcpSystemDescriptorGPU* descriptor,
		int maxIteration,		///< max.number of iterations
		double dt,
		double tolerance,		///< tolerance for termination criterion
		double omega, bool cpu){			///< overrelaxation criterion

			mDt= dt;
			mDoIntegrationStep=true;
			mStepCounter = 0;
			mMaxIterations=maxIteration;
			mTolerance=tolerance;
			gpu_contact_container = container;
			mSystemDescriptor=descriptor;
			mOmega=omega;
			use_cpu=cpu;
			gpu_solver=mSystemDescriptor->gpu_solver;
			gpu_solver->c_factor				=gpu_contact_container->Get_load_C_factor();
			gpu_solver->tolerance				=mTolerance;
			double maxrecspeed = gpu_contact_container->Get_load_max_recovery_speed();
					if (gpu_contact_container->Get_load_do_clamp() == false) {maxrecspeed = 10e25;}
			gpu_solver->negated_recovery_speed		=-maxrecspeed;
			gpu_solver->step_size				=mDt;
			gpu_solver->lcp_omega				=mOmega;
			gpu_solver->maximum_iterations			=mMaxIterations;
			gpu_solver->use_cpu				=use_cpu;
			
	};
	ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple() {/*CUT_EXIT(argc, argv);*/};

	double ChLcpIterativeSolverGPUsimple::Solve(
		ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
		bool add_Mq_to_f)					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
	{
		std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
		std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

		// -1-  Count active variables and initialize GPU body buffer
		uint number_of_bodies = mvariables.size();
		mSystemDescriptor->number_of_bodies=number_of_bodies;

		gpu_solver->host_body_data.resize(number_of_bodies*CH_BODY_VSIZE);
		
		for (uint i = 0; i< number_of_bodies; i++){
			ChLcpVariablesBody* mbodyvar = (ChLcpVariablesBody*) mvariables[i];
			ChBodyGPU* mbody = (ChBodyGPU*)mbodyvar->GetUserData();
			mbody->id=i;
			float inv_mass=(1.0)/(mbodyvar->GetBodyMass());
			gpu_solver->host_body_data[i+number_of_bodies*0] = F4(mbodyvar->Get_qb().GetElementN(0),mbodyvar->Get_qb().GetElementN(1),mbodyvar->Get_qb().GetElementN(2),1)*mbody->IsActive();
			gpu_solver->host_body_data[i+number_of_bodies*1] = F4(mbodyvar->Get_qb().GetElementN(3),mbodyvar->Get_qb().GetElementN(4),mbodyvar->Get_qb().GetElementN(5),mbody->GetKfriction());
			gpu_solver->host_body_data[i+number_of_bodies*2] = F4(mbody->GetPos().x,mbody->GetPos().y,mbody->GetPos().z,0);
			gpu_solver->host_body_data[i+number_of_bodies*3] = F4(mbody->GetRot().e0,mbody->GetRot().e1,mbody->GetRot().e2,mbody->GetRot().e3);
			gpu_solver->host_body_data[i+number_of_bodies*4] = F4(mbodyvar->GetBodyInvInertia().GetElement(0,0),mbodyvar->GetBodyInvInertia().GetElement(1,1),mbodyvar->GetBodyInvInertia().GetElement(2,2),inv_mass);
			gpu_solver->host_body_data[i+number_of_bodies*5] = F4(mbodyvar->Get_fb().ElementN(0),mbodyvar->Get_fb().ElementN(1),mbodyvar->Get_fb().ElementN(2),0);//forces
			gpu_solver->host_body_data[i+number_of_bodies*6] = F4(mbodyvar->Get_fb().ElementN(3),mbodyvar->Get_fb().ElementN(4),mbodyvar->Get_fb().ElementN(5),0);//torques
		}

		// -2-  Count active constraints and initialize GPU bilateral buffer.

		unsigned int number_of_bilaterals=0;
		for (uint i = 0; i< mconstraints.size(); i++){if (mconstraints[i]->IsActive()){number_of_bilaterals++;}}

		gpu_solver->host_bilateral_data.resize(number_of_bilaterals * CH_BILATERAL_VSIZE);
		uint counter=0;
		for (uint ic = 0; ic< mconstraints.size(); ic++){
			if (!mconstraints[ic]->IsActive()){	continue;}
			ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[ic]);

			ChBodyGPU* temp=(ChBodyGPU*)(((ChLcpVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData());
			
			int idA=((ChBodyGPU*)((ChLcpVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->id;
			int idB=((ChBodyGPU*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->id;
			
			// Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
			mconstraints[ic]->Update_auxiliary();  //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
			float4 A,B,C,D;
			A=F4(mbilateral->Get_Cq_a()->GetElementN(0),mbilateral->Get_Cq_a()->GetElementN(1),mbilateral->Get_Cq_a()->GetElementN(2),0);//J1x
			B=F4(mbilateral->Get_Cq_b()->GetElementN(0),mbilateral->Get_Cq_b()->GetElementN(1),mbilateral->Get_Cq_b()->GetElementN(2),0);//J2x
			C=F4(mbilateral->Get_Cq_a()->GetElementN(3),mbilateral->Get_Cq_a()->GetElementN(4),mbilateral->Get_Cq_a()->GetElementN(5),0);//J1w
			D=F4(mbilateral->Get_Cq_b()->GetElementN(3),mbilateral->Get_Cq_b()->GetElementN(4),mbilateral->Get_Cq_b()->GetElementN(5),0);//J2w
			A.w=idA;//pointer to body B1 info in body buffer
			B.w=idB;//pointer to body B2 info in body buffer	

			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*0]   = A;		
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*1]   = B;
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*2]   = C;
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*3]   = D;
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*4].x = (1.0/mbilateral->Get_g_i());	// eta = 1/g
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*4].y = mbilateral->Get_b_i();		// b_i is residual b 
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*4].z = 0;							//gammma, no warm starting
			gpu_solver->host_bilateral_data[counter+number_of_bilaterals*4].w = (mbilateral->IsUnilateral()) ?	1 :  0;				
			counter++;
		}// end loop
		mSystemDescriptor->number_of_bilaterals =number_of_bilaterals;
		// -3-  EXECUTE KERNELS ===============


		gpu_solver->number_of_bilaterals		=number_of_bilaterals;
		gpu_solver->number_of_bodies			=number_of_bodies;
		gpu_solver->number_of_contacts			=mSystemDescriptor->gpu_collision->number_of_contacts;
		gpu_solver->RunTimeStep();

		gpu_contact_container->SetNcontacts(mSystemDescriptor->gpu_collision->number_of_contacts);

		if (mDoIntegrationStep){
			for (unsigned int iv = 0; iv< mvariables.size(); iv++){
				mvariables[iv]->Get_qb().SetElementN(0, (double)gpu_solver->host_body_data[iv].x );
				mvariables[iv]->Get_qb().SetElementN(1, (double)gpu_solver->host_body_data[iv].y );
				mvariables[iv]->Get_qb().SetElementN(2, (double)gpu_solver->host_body_data[iv].z );
				mvariables[iv]->Get_qb().SetElementN(3, (double)gpu_solver->host_body_data[iv + number_of_bodies].x );
				mvariables[iv]->Get_qb().SetElementN(4, (double)gpu_solver->host_body_data[iv + number_of_bodies].y );
				mvariables[iv]->Get_qb().SetElementN(5, (double)gpu_solver->host_body_data[iv + number_of_bodies].z );

				ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[iv];
				ChBody* mbody = (ChBody*)mbodyvars->GetUserData();
				if(mbody->IsActive()){

					CH_REALNUMBER4 hp	= gpu_solver->host_body_data[iv+ 2*number_of_bodies];
					CH_REALNUMBER4 hr	= gpu_solver->host_body_data[iv+ 3*number_of_bodies];

					ChVector<> newpos	 ( hp.x, hp.y, hp.z );
					ChQuaternion<> newrot( hr.x, hr.y, hr.z, hr.w );
					mbody->SetCoord(ChCoordsys<>(newpos, newrot));

					mbody->VariablesQbIncrementPosition(mDt);
					mbody->VariablesQbSetSpeed(mDt);
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
