#include "ChSystemMultiGPU.h"

namespace chrono {

	ChSystemMultiGPU::ChSystemMultiGPU(unsigned int max_objects) :ChSystem(0, 0) {
		gpu_data_manager = new ChGPUDataManager();
		counter = 0;
		max_obj = max_objects;
		InitGPU();

	}

	void ChSystemMultiGPU::InitGPU(){

		cudaGetDeviceCount(&nGPUs);
		cudaDeviceProp prop[64];



		for (int i=0; i< nGPUs; i++) {
			cudaGetDeviceProperties(&prop[i],i);
			if (prop[i].unifiedAddressing) {
				gpus.push_back(i);
			}
		}

		nGPUs = gpus.size();

		for (int i=0; i<nGPUs;i++){
			cudaSetDevice(gpus[i]);
			for (int j=0; j<nGPUs;j++) {
				if (i != j) {
					cudaDeviceEnablePeerAccess(gpus[j],0);
				}
			}
		}
	}
	int ChSystemMultiGPU::DoStepDynamics (double m_step)
	{
		this->step=m_step;
		return Integrate_Y();
	}

	int ChSystemMultiGPU::Integrate_Y()
	{
		return Integrate_Y_impulse_Anitescu();
		return true;
	}

	int ChSystemMultiGPU::Integrate_Y_impulse_Anitescu() {
		ChTimer<double> mtimer_lcp, mtimer_step, mtimer_cd;
		mtimer_step.start();
		this->stepcount++;
		Setup();
		Update();
		LCPprepare_inject(*this->LCP_descriptor);// make vectors of variables and constraints, used by the following LCP solver
		
		//Generate AABB's - Host
		//Perform BB CD	  - Host
		//Thrust partition- 



		//detmine GPU number of each object
		//run scan on array
		//determine number of objects on each gpu
		//allocate memory
		//copy memory to device
		
		
		
		
		
		gpu_data_manager->HostToDevice();
		gpu_data_manager->HostToDevice_CD();


		mtimer_cd.start();
		((ChCollisionSystemGPU*) (collision_system))->Run();
		this->ncontacts = gpu_data_manager->number_of_contacts;
		mtimer_cd.stop();

		mtimer_lcp.start();
		(LCP_solver_speed)->Solve(*this->LCP_descriptor, true);
		((ChContactContainerGPUsimple*) this->contact_container)->SetNcontacts(gpu_data_manager->number_of_contacts);
		mtimer_lcp.stop();




		// Device to host
		gpu_data_manager->DeviceToHost();
#pragma omp parallel for
		for (int i = 0; i < bodylist.size(); i++) {

			float3 new_pos = gpu_data_manager->host_pos_data[i];
			float4 new_rot = gpu_data_manager->host_rot_data[i];
			float3 new_vel = gpu_data_manager->host_vel_data[i];
			float3 new_acc = gpu_data_manager->host_acc_data[i];
			float3 new_omg = gpu_data_manager->host_omg_data[i];
			float3 new_fap = gpu_data_manager->host_fap_data[i];

			ChBodyGPU* mbody = (ChBodyGPU*) bodylist[i];
			if (mbody->IsActive()) {
				mbody->SetPos(CHVECCAST(new_pos));
				mbody->SetRot(CHQUATCAST(new_rot));
				mbody->SetPos_dt(CHVECCAST(new_vel));
				mbody->SetPos_dtdt(CHVECCAST(new_acc));
				mbody->SetWvel_loc(CHVECCAST(new_omg));
				mbody->SetAppliedForce(CHVECCAST(new_fap));
			}
		}

		LCPresult_Li_into_speed_cache();

		// updates the reactions of the constraint
		LCPresult_Li_into_reactions(1.0 / this->GetStep()); // R = l/dt  , approximately
		timer_lcp = mtimer_lcp();
		timer_collision_broad = mtimer_cd();
		ChTime += GetStep();
		mtimer_step.stop();
		timer_step = mtimer_step();// Time elapsed for step..
		return 1;
	}

	double ChSystemMultiGPU::ComputeCollisions() {
		double mretC = 0.0;
		return mretC;
	}

	void ChSystemMultiGPU::RemoveBody(ChSharedPtr<ChBodyGPU> mbody) {
		assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()) != bodylist.end());

		// remove from collision system
		if (mbody->GetCollide()) mbody->RemoveCollisionModelsFromSystem();

		// warning! linear time search, to erase pointer from container.
		bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()));

		// nullify backward link to system
		mbody->SetSystem(0);
		// this may delete the body, if none else's still referencing it..
		mbody->RemoveRef();
	}

	void ChSystemMultiGPU::Update() {
		ChTimer<double> mtimer;
		mtimer.start(); // Timer for profiling
#pragma omp parallel for
		for (int i = 0; i < bodylist.size(); i++) // Updates recursively all other aux.vars
		{
			bodylist[i]->Update(ChTime);
			bodylist[i]->VariablesFbReset();
			bodylist[i]->VariablesFbLoadForces(GetStep());
			bodylist[i]->VariablesQbLoadSpeed();

			ChLcpVariablesBodyOwnMass* mbodyvar = &(bodylist[i]->Variables());
			gpu_data_manager->host_vel_data[i] = (F3(bodylist[i]->GetPos_dt().x, bodylist[i]->GetPos_dt().y, bodylist[i]->GetPos_dt().z));
			gpu_data_manager->host_omg_data[i] = (F3(bodylist[i]->GetWvel_loc().x, bodylist[i]->GetWvel_loc().y, bodylist[i]->GetWvel_loc().z));
			gpu_data_manager->host_pos_data[i] = (F3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
			gpu_data_manager->host_rot_data[i] = (F4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
			gpu_data_manager->host_inr_data[i] = (F3(mbodyvar->GetBodyInvInertia().GetElement(0, 0), mbodyvar->GetBodyInvInertia().GetElement(1, 1), mbodyvar->GetBodyInvInertia().GetElement(2, 2)));
			gpu_data_manager->host_frc_data[i] = (F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
			gpu_data_manager->host_trq_data[i] = (F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
			gpu_data_manager->host_aux_data[i] = (F3(bodylist[i]->IsActive(), bodylist[i]->GetKfriction(), 1.0f / mbodyvar->GetBodyMass()));
			gpu_data_manager->host_lim_data[i] = (F3(bodylist[i]->GetLimitSpeed(), bodylist[i]->GetMaxSpeed(), bodylist[i]->GetMaxWvel()));
		}
		mtimer.stop();
		timer_update += mtimer();
	}

	void ChSystemMultiGPU::ChangeLcpSolverSpeed(ChLcpSolver* newsolver) {
		assert(newsolver);
		if (this->LCP_solver_speed) delete (this->LCP_solver_speed);
		this->LCP_solver_speed = newsolver;

		((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->data_container = gpu_data_manager;
	}

	void ChSystemMultiGPU::ChangeCollisionSystem(ChCollisionSystem* newcollsystem) {
		assert(this->GetNbodies() == 0);
		assert(newcollsystem);
		if (this->collision_system) delete (this->collision_system);
		this->collision_system = newcollsystem;

		((ChCollisionSystemGPU*) (collision_system))->mGPU->data_container = gpu_data_manager;
	}
}
