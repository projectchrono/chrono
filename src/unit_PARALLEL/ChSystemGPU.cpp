#include "ChSystemGPU.h"
#include <omp.h>

namespace chrono {

	ChSystemGPU::ChSystemGPU(unsigned int max_objects, double scene_size) :
		ChSystem(max_objects, scene_size) {
		gpu_data_manager = new ChGPUDataManager();
		copydata = true;
		counter = 0;
		max_obj = max_objects;
		gpu_data_manager->host_vel_data.reserve(max_objects);
		gpu_data_manager->host_omg_data.reserve(max_objects);
		gpu_data_manager->host_pos_data.reserve(max_objects);
		gpu_data_manager->host_rot_data.reserve(max_objects);
		gpu_data_manager->host_inr_data.reserve(max_objects);
		gpu_data_manager->host_frc_data.reserve(max_objects);
		gpu_data_manager->host_trq_data.reserve(max_objects);
		gpu_data_manager->host_aux_data.reserve(max_objects);
		gpu_data_manager->host_lim_data.reserve(max_objects);
		bodylist.reserve(max_objects);
	}

	int ChSystemGPU::Integrate_Y_impulse_Anitescu() {
		ChTimer<double> mtimer_lcp, mtimer_step, mtimer_cd;
		mtimer_step.start();

		this->stepcount++;

		((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->force_solver->gravity = F3(Get_G_acc().x, Get_G_acc().y, Get_G_acc().z);

		Setup(); 													// Counts dofs, statistics, etc.
		Update(); // Update everything - and put to sleep bodies that need it
		//if (copydata) {
			gpu_data_manager->HostToDevice(); // Copy data from host to device
		//	copydata = false;
		//}
		{
			mtimer_cd.start(); // Collision timer
			((ChCollisionSystemGPU*) (collision_system))->Run(); // Run CD
			this->ncontacts = gpu_data_manager->number_of_contacts; // Get number of contacts
			mtimer_cd.stop(); // Collision timer
		}
		{
			mtimer_lcp.start(); // LCP Timer
			(LCP_solver_speed)->Solve(*this->LCP_descriptor, true); // Solve the LCP problem.

			((ChContactContainerGPUsimple*) this->contact_container)->SetNcontacts(gpu_data_manager->number_of_contacts); // Set number of contacts
			mtimer_lcp.stop(); // LCP timer
		}
		gpu_data_manager->DeviceToHost(); // Device to host
		LCPresult_Li_into_reactions(1.0 / this->GetStep()); // updates the reactions of the constraint  R = l/dt  , approximately
		timer_lcp = mtimer_lcp();
		timer_collision_broad = mtimer_cd();
		ChTime += GetStep();
		mtimer_step.stop();
		timer_step = mtimer_step();// Time elapsed for step..
		return 1;
	}

	double ChSystemGPU::ComputeCollisions() {
		double mretC = 0.0;
		return mretC;
	}

	void ChSystemGPU::AddBody(ChSharedPtr<ChBody> newbody) {

		newbody->AddRef();
		newbody->SetSystem(this);
		bodylist.push_back((newbody).get_ptr());

		ChBodyGPU* gpubody = ((ChBodyGPU*) newbody.get_ptr());
		gpubody->id = counter;
		gpubody->data_manager = gpu_data_manager;
		if (newbody->GetCollide()) newbody->AddCollisionModelsToSystem();

		ChLcpVariablesBodyOwnMass* mbodyvar = &(newbody->Variables());

		float inv_mass = (1.0) / (mbodyvar->GetBodyMass());
		newbody->GetRot().Normalize();
		gpu_data_manager->host_vel_data.push_back(F3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1), mbodyvar->Get_qb().GetElementN(2)));
		gpu_data_manager->host_omg_data.push_back(F3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4), mbodyvar->Get_qb().GetElementN(5)));
		gpu_data_manager->host_pos_data.push_back(F3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
		gpu_data_manager->host_rot_data.push_back(F4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
		gpu_data_manager->host_inr_data.push_back(F3(mbodyvar->GetBodyInvInertia().GetElement(0, 0), mbodyvar->GetBodyInvInertia().GetElement(1, 1), mbodyvar->GetBodyInvInertia().GetElement(2, 2)));
		gpu_data_manager->host_frc_data.push_back(F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
		gpu_data_manager->host_trq_data.push_back(F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
		gpu_data_manager->host_aux_data.push_back(F3(newbody->IsActive(), newbody->GetKfriction(), inv_mass));
		gpu_data_manager->host_lim_data.push_back(F3(newbody->GetLimitSpeed(), newbody->GetMaxSpeed(), newbody->GetMaxWvel()));

		copydata = true;
		counter++;
		gpu_data_manager->number_of_objects = counter;
		if (counter % 1000 == 0) {
			cout << "Added: " << counter << " objects" << endl;
		}
	}

	void ChSystemGPU::RemoveBody(ChSharedPtr<ChBody> mbody) {
		assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr() )!=bodylist.end());

		// remove from collision system
		if (mbody->GetCollide()) mbody->RemoveCollisionModelsFromSystem();

		// warning! linear time search, to erase pointer from container.
		bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()));

		// nullify backward link to system
		mbody->SetSystem(0);
		// this may delete the body, if none else's still referencing it..
		mbody->RemoveRef();
	}

}

void ChSystemGPU::Update() {
	ChTimer<double> mtimer;
	mtimer.start(); // Timer for profiling

	//	for (int i = 0; i < bodylist.size(); i++) // Updates recursively all other aux.vars
	//	{
	//
	//		//bodylist[i]->ClampSpeed(); //moved to gpu
	//		//bodylist[i]->ComputeGyro();//moved to gpu
	//		//bodylist[i]->UpdateMarkers(ChTime);
	//		//bodylist[i]->UpdateForces(ChTime);
	//		//bodylist[i]->VariablesFbReset();
	//
	//		//bodylist[i]->VariablesFbLoadForces(GetStep());
	//		//bodylist[i]->VariablesQbLoadSpeed();
	//
	//		//ChLcpVariablesBodyOwnMass* mbodyvar = &(bodylist[i]->Variables());
	//		ChBodyGPU* mbody = (ChBodyGPU*) bodylist[i];
	//
	//		mbody->UpdateForces(ChTime);
	//		float3 load_force = F3(mbody->GetXForce().x, mbody->GetXForce().y, mbody->GetXForce().z);
	//		float3 load_torque = F3(mbody->GetXTorque().x, mbody->GetXTorque().y, mbody->GetXTorque().z);
	//		gpu_data_manager->host_frc_data[i] = load_force; //forces
	//		gpu_data_manager->host_trq_data[i] = load_torque; //torques
	//	}

	this->LCP_descriptor->BeginInsertion();
	std::list<ChLink*>::iterator it;
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->Update(ChTime);
		(*it)->ConstraintsBiReset();
		(*it)->ConstraintsBiLoad_C(1 / GetStep(), max_penetration_recovery_speed, true);
		(*it)->ConstraintsBiLoad_Ct(1);
		(*it)->ConstraintsFbLoadForces(GetStep());
		(*it)->ConstraintsLoadJacobians();
		(*it)->InjectConstraints(*(this->LCP_descriptor));
	}
	this->LCP_descriptor->EndInsertion();
	this->contact_container->Update(); // Update all contacts, if any

	mtimer.stop();
	timer_update += mtimer();
}

void ChSystemGPU::LCPprepare_load(bool load_jacobians, bool load_v, double F_factor, double Ct_factor, double C_factor, double recovery_clamp, bool do_clamp) {

	//	if (C_factor) contact_container->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
	//	if (F_factor) contact_container->ConstraintsFbLoadForces(F_factor); // f*dt
	//	if (load_jacobians) contact_container->ConstraintsLoadJacobians();

}
void ChSystemGPU::LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor) {
	//	mdescriptor.BeginInsertion(); // This resets the vectors of constr. and var. pointers.
	//	this->contact_container->InjectConstraints(mdescriptor);
	//	mdescriptor.EndInsertion();
}

void ChSystemGPU::LCPprepare_reset() {
	//	this->contact_container->ConstraintsBiReset();
}
void ChSystemGPU::ChangeLcpSolverSpeed(ChLcpSolver* newsolver) {
	assert (newsolver);
	if (this->LCP_solver_speed) delete (this->LCP_solver_speed);
	this->LCP_solver_speed = newsolver;

	((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->data_container = gpu_data_manager;
}

void ChSystemGPU::ChangeCollisionSystem(ChCollisionSystem* newcollsystem) {
	assert (this->GetNbodies()==0);
	assert (newcollsystem);
	if (this->collision_system) delete (this->collision_system);
	this->collision_system = newcollsystem;

	((ChCollisionSystemGPU*) (collision_system))->mGPU->data_container = gpu_data_manager;
}
