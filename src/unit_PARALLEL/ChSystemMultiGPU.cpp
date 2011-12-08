#include "ChSystemMultiGPU.h"
#include <omp.h>

namespace chrono {
ChSystemMultiGPU::ChSystemMultiGPU(unsigned int max_objects) :
	ChSystem(1000, 10000, false) {
	num_gpu = 1;
	slices = U3(2, 1, 1);
	num_subdiv = slices.x * slices.y * slices.z;
	gpu_subdomains.resize(num_subdiv);

	gpu_data_manager = new ChGPUDataManager(num_subdiv);
	counter = 0;
	max_obj = max_objects;

	LCP_descriptor = new ChLcpSystemDescriptorGPU();
	contact_container = new ChContactContainerGPUsimple();
	collision_system = new ChCollisionSystemGPU();
	LCP_solver_speed = new ChLcpIterativeSolverGPUsimple((ChContactContainerGPUsimple*) contact_container);
	((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->data_container = gpu_data_manager;
	((ChCollisionSystemGPU*) (collision_system))->data_container = gpu_data_manager;

}

int ChSystemMultiGPU::Integrate_Y_impulse_Anitescu() {

	mtimer_step.start();
	this->stepcount++;
	Setup();
	Update();

	SplitData();
	ComputeCollisions();
	SolveSystem();

	// Device to host
	gpu_data_manager->DeviceToHost();
#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		ChBodyGPU* mbody = (ChBodyGPU*) bodylist[i];
		if (mbody->IsActive()) {
			mbody->SetPos(CHVECCAST(gpu_data_manager->host_pos_data[i]));
			mbody->SetRot(CHQUATCAST(gpu_data_manager->host_rot_data[i]));
			mbody->SetPos_dt(CHVECCAST(gpu_data_manager->host_vel_data[i]));
			mbody->SetPos_dtdt(CHVECCAST(gpu_data_manager->host_acc_data[i]));
			mbody->SetWvel_loc(CHVECCAST(gpu_data_manager->host_omg_data[i]));
			mbody->SetAppliedForce(CHVECCAST(gpu_data_manager->host_fap_data[i]));
		}
	}

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
	mtimer_cd.start();
	float3 bin_size_vec;
	float max_dimension;
	float collision_envelope = 0;
	ChCCollisionGPU::ComputeAABB_HOST(gpu_data_manager);
	ChCCollisionGPU::ComputeBounds_HOST(gpu_data_manager);
	ChCCollisionGPU::UpdateAABB_HOST(bin_size_vec, max_dimension, collision_envelope, gpu_data_manager);

	//determine the size of each subdomain
	float3 global_min = F3(0, 0, 0);
	float3 global_max = gpu_data_manager->max_bounding_point + gpu_data_manager->min_bounding_point;
	subdomain_dim = F3(global_max.x / float(slices.x), global_max.y / float(slices.y), global_max.y / float(slices.y));

	for (int i = 0; i < slices.x; i++) {
		for (int j = 0; j < slices.y; j++) {
			for (int k = 0; k < slices.z; k++) {
				gpu_subdomains[i * slices.x * slices.y + j * slices.y + k].min_bounding_point = F3(subdomain_dim.x * (i + 0), subdomain_dim.y * (j
						+ 0), subdomain_dim.z * (k + 0));
				gpu_subdomains[i * slices.x * slices.y + j * slices.y + k].max_bounding_point = F3(subdomain_dim.x * (i + 1), subdomain_dim.y * (j
						+ 1), subdomain_dim.z * (k + 1));
			}
		}
	}

#pragma omp parallel for
	for (int i = 0; i < gpu_subdomains.size(); i++) {
		gpu_subdomains[i].Collide(gpu_data_manager);
		gpu_subdomains[i].Copy();
		gpu_subdomains[i].Run(((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed)));
	}

	this->ncontacts = gpu_data_manager->number_of_contacts;
	mtimer_cd.stop();
	return 0;
}

double ChSystemMultiGPU::SolveSystem() {
	mtimer_lcp.start();
	(LCP_solver_speed)->Solve(*this->LCP_descriptor, true);
	((ChContactContainerGPUsimple*) this->contact_container)->SetNcontacts(gpu_data_manager->number_of_contacts);
	mtimer_lcp.stop();
	return 0;
}
double ChSystemMultiGPU::SplitData() {
	gpu_data_manager->HostToDevice();
	return 0;
}
void ChSystemMultiGPU::AddBody(ChSharedPtr<ChBodyGPU> newbody) {
	newbody->AddRef();
	newbody->SetSystem(this);
	bodylist.push_back((newbody).get_ptr());

	ChBodyGPU* gpubody = ((ChBodyGPU*) newbody.get_ptr());
	gpubody->id = counter;
	if (newbody->GetCollide()) newbody->AddCollisionModelsToSystem();

	ChLcpVariablesBodyOwnMass* mbodyvar = &(newbody->Variables());

	float inv_mass = (1.0) / (mbodyvar->GetBodyMass());
	newbody->GetRot().Normalize();
	gpu_data_manager->host_vel_data.push_back(F3(
													mbodyvar->Get_qb().GetElementN(0),
													mbodyvar->Get_qb().GetElementN(1),
													mbodyvar->Get_qb().GetElementN(2)));
	gpu_data_manager->host_omg_data.push_back(F3(
													mbodyvar->Get_qb().GetElementN(3),
													mbodyvar->Get_qb().GetElementN(4),
													mbodyvar->Get_qb().GetElementN(5)));
	gpu_data_manager->host_pos_data.push_back(F3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
	gpu_data_manager->host_rot_data.push_back(F4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
	gpu_data_manager->host_inr_data.push_back(F3(
													mbodyvar->GetBodyInvInertia().GetElement(0, 0),
													mbodyvar->GetBodyInvInertia().GetElement(1, 1),
													mbodyvar->GetBodyInvInertia().GetElement(2, 2)));
	gpu_data_manager->host_frc_data.push_back(F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
	gpu_data_manager->host_trq_data.push_back(F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
	gpu_data_manager->host_aux_data.push_back(F3(newbody->IsActive(), newbody->GetKfriction(), inv_mass));
	gpu_data_manager->host_lim_data.push_back(F3(newbody->GetLimitSpeed(), newbody->GetMaxSpeed(), newbody->GetMaxWvel()));

	counter++;
	gpu_data_manager->number_of_objects = counter;
	if (counter % 1000 == 0) {
		cout << "Added: " << counter << " objects" << endl;
	}
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
		bodylist[i]->UpdateTime(ChTime);
		bodylist[i]->UpdateMarkers(ChTime);
		bodylist[i]->UpdateForces(ChTime);
		bodylist[i]->VariablesFbReset();
		bodylist[i]->VariablesFbLoadForces(GetStep());
		bodylist[i]->VariablesQbLoadSpeed();

		ChLcpVariablesBodyOwnMass* mbodyvar = &(bodylist[i]->Variables());
		gpu_data_manager->host_vel_data[i] = (F3(bodylist[i]->GetPos_dt().x, bodylist[i]->GetPos_dt().y, bodylist[i]->GetPos_dt().z));
		gpu_data_manager->host_omg_data[i] = (F3(bodylist[i]->GetWvel_loc().x, bodylist[i]->GetWvel_loc().y, bodylist[i]->GetWvel_loc().z));
		gpu_data_manager->host_pos_data[i] = (F3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
		gpu_data_manager->host_rot_data[i] = (F4(
													bodylist[i]->GetRot().e0,
													bodylist[i]->GetRot().e1,
													bodylist[i]->GetRot().e2,
													bodylist[i]->GetRot().e3));
		gpu_data_manager->host_inr_data[i] = (F3(
													mbodyvar->GetBodyInvInertia().GetElement(0, 0),
													mbodyvar->GetBodyInvInertia().GetElement(1, 1),
													mbodyvar->GetBodyInvInertia().GetElement(2, 2)));
		gpu_data_manager->host_frc_data[i] = (F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
		gpu_data_manager->host_trq_data[i] = (F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
		gpu_data_manager->host_aux_data[i] = (F3(bodylist[i]->IsActive(), bodylist[i]->GetKfriction(), 1.0f / mbodyvar->GetBodyMass()));
		gpu_data_manager->host_lim_data[i] = (F3(bodylist[i]->GetLimitSpeed(), bodylist[i]->GetMaxSpeed(), bodylist[i]->GetMaxWvel()));

	}
	std::list<ChLink*>::iterator it;
	unsigned int number_of_bilaterals = 0;
	uint counter = 0;
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->Update(ChTime);
		if ((*it)->IsActive()) {
			number_of_bilaterals++;
		}
	}
	gpu_data_manager->number_of_bilaterals = number_of_bilaterals;
	gpu_data_manager->host_bilateral_data.resize(number_of_bilaterals * CH_BILATERAL_VSIZE);
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->ConstraintsBiReset();
	}
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->ConstraintsBiLoad_C(1 / GetStep(), max_penetration_recovery_speed, true);
		(*it)->ConstraintsBiLoad_Ct(1);
		(*it)->ConstraintsFbLoadForces(GetStep());
		(*it)->ConstraintsLoadJacobians();
		if ((*it)->IsActive()) {
			ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*) ((*it));
			// Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
			mbilateral->Update_auxiliary();//***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
			int idA = ((ChBodyGPU*) ((ChLcpVariablesBody*) (mbilateral->GetVariables_a()))->GetUserData())->id;
			int idB = ((ChBodyGPU*) ((ChLcpVariablesBody*) (mbilateral->GetVariables_b()))->GetUserData())->id;

			float4 A, B, C, D;
			A = F4(mbilateral->Get_Cq_a()->GetElementN(0), mbilateral->Get_Cq_a()->GetElementN(1), mbilateral->Get_Cq_a()->GetElementN(2), idA);//J1x
			B = F4(mbilateral->Get_Cq_b()->GetElementN(0), mbilateral->Get_Cq_b()->GetElementN(1), mbilateral->Get_Cq_b()->GetElementN(2), idB);//J2x
			C = F4(mbilateral->Get_Cq_a()->GetElementN(3), mbilateral->Get_Cq_a()->GetElementN(4), mbilateral->Get_Cq_a()->GetElementN(5), 0);//J1w
			D = F4(mbilateral->Get_Cq_b()->GetElementN(3), mbilateral->Get_Cq_b()->GetElementN(4), mbilateral->Get_Cq_b()->GetElementN(5), 0);//J2w
			bool isUni = (mbilateral->IsUnilateral()) ? 1 : 0;
			gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 0] = A;
			gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 1] = B;
			gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 2] = C;
			gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 3] = D;
			gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 4] = F4(
																							(1.0 / mbilateral->Get_g_i()),
																							mbilateral->Get_b_i(),
																							0,
																							isUni);
			counter++;

		}
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

	((ChCollisionSystemGPU*) (collision_system))->data_container = gpu_data_manager;
}

}
