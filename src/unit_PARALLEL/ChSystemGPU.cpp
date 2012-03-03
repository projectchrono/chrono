#include "ChSystemGPU.h"
#include <omp.h>

namespace chrono {

ChSystemGPU::ChSystemGPU(unsigned int max_objects) :
		ChSystem(1000, 10000, false) {
	counter = 0;
	max_obj = max_objects;
	gpu_data_manager = new ChGPUDataManager();
	LCP_descriptor = new ChLcpSystemDescriptorGPU();
	contact_container = new ChContactContainerGPUsimple();
	collision_system = new ChCollisionSystemGPU();
	LCP_solver_speed = new ChLcpSolverGPU();
	((ChCollisionSystemGPU*) (collision_system))->data_container = gpu_data_manager;
	mtuning = 0;
	bins_per_axis = F3(10, 10, 10);
	search = 1;
}

int ChSystemGPU::Integrate_Y_impulse_Anitescu() {
	use_cpu = 0;

	mtimer_step.start();
	this->stepcount++;
	Setup();
	Update();

	gpu_data_manager->HostToDevice();

	ComputeCollisions();
	SolveSystem();

	gpu_data_manager->DeviceToHost();
	std::list<ChLink*>::iterator it;
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->ConstraintsFetch_react(1.0 / GetStep());
	}
#pragma omp parallel
	{
#pragma omp parallel for
		for (int i = 0; i < bodylist.size(); i++) {
			ChBodyGPU* mbody = (ChBodyGPU*) bodylist[i];
			//if (mbody->IsActive()) {
			mbody->SetPos(CHVECCAST(gpu_data_manager->host_pos_data[i]));
			mbody->SetRot(CHQUATCAST(gpu_data_manager->host_rot_data[i]));
			mbody->SetPos_dt(CHVECCAST(gpu_data_manager->host_vel_data[i]));
			mbody->SetPos_dtdt(CHVECCAST(gpu_data_manager->host_acc_data[i]));
			mbody->SetWvel_loc(CHVECCAST(gpu_data_manager->host_omg_data[i]));
			mbody->SetAppliedForce(CHVECCAST(gpu_data_manager->host_fap_data[i]));
			//}
		}
	}
	// updates the reactions of the constraint
	//LCPresult_Li_into_reactions(1.0 / this->GetStep()); // R = l/dt  , approximately
	timer_lcp = mtimer_lcp();
	timer_collision_broad = mtimer_cd();
	ChTime += GetStep();
	mtimer_step.stop();
	timer_step = mtimer_step(); // Time elapsed for step..
	return 1;
}

float tuneCD(float3 & bin_size_vec, float & max_dimension, float & collision_envelope, gpu_container & gpu_data, float3 bins_per_axis) {
	ChTimer<double> mtimer_tuning;
	float accumulated_time = 0;
	for (int j = 0; j < 5; j++) {
		mtimer_tuning.start();
		ChCCollisionGPU::UpdateAABB(bin_size_vec, max_dimension, collision_envelope, gpu_data, bins_per_axis);
		ChCCollisionGPU::Broadphase(bin_size_vec, gpu_data);
		mtimer_tuning.stop();
		accumulated_time += mtimer_tuning();
	}
	return accumulated_time / 5.0; //time of current

}

double ChSystemGPU::ComputeCollisions() {
	mtimer_cd.start();
	float3 bin_size_vec;
	float max_dimension;
	float collision_envelope = 0;

	float old_time = 10000, new_time = 0, time1 = 0, time2 = 0, time3 = 0;
	int tests = 0;
	float3 tune_dir = F3(1, 1, 1);
	if (gpu_data_manager->gpu_data.number_of_models > 0) {
		ChCCollisionGPU::ComputeAABB(gpu_data_manager->gpu_data);
		ChCCollisionGPU::ComputeBounds(gpu_data_manager->gpu_data);

		if (mtuning % 50 == 0) {

			{
				if (search > 3) {
					search = 1;
				}
				if (search == 1) {
					tune_dir = F3(5, 0, 0);
				}
				if (search == 2) {
					tune_dir = F3(0, 5, 0);
				}
				if (search == 3) {
					tune_dir = F3(0, 0, 5);
				}

				//bins_per_axis = F3(i, i, i);
				tests++;
				time1 = tuneCD(bin_size_vec, max_dimension, collision_envelope, gpu_data_manager->gpu_data,
						F3(bins_per_axis.x - tune_dir.x, bins_per_axis.y - tune_dir.y, bins_per_axis.z - tune_dir.z));
				time2 = tuneCD(bin_size_vec, max_dimension, collision_envelope, gpu_data_manager->gpu_data, bins_per_axis);
				time3 = tuneCD(bin_size_vec, max_dimension, collision_envelope, gpu_data_manager->gpu_data,
						F3(bins_per_axis.x + tune_dir.x, bins_per_axis.y + tune_dir.y, bins_per_axis.z + tune_dir.z));

				if (time1 < time2) {
					bins_per_axis = F3(bins_per_axis.x - tune_dir.x, bins_per_axis.y - tune_dir.y, bins_per_axis.z - tune_dir.z);
				} else if (time3 < time2) {
					bins_per_axis = F3(bins_per_axis.x + tune_dir.x, bins_per_axis.y + tune_dir.y, bins_per_axis.z + tune_dir.z);
				} else {
				}
				search++;
			}

			cout << "TUNING " << search-1 <<" "<< bins_per_axis.x << " " << bins_per_axis.y << " " << bins_per_axis.z << endl;

		} else {
			ChCCollisionGPU::UpdateAABB(bin_size_vec, max_dimension, collision_envelope, gpu_data_manager->gpu_data, bins_per_axis);
			ChCCollisionGPU::Broadphase(bin_size_vec, gpu_data_manager->gpu_data);
		}
		mtuning++;

		ChCCollisionGPU::Narrowphase(gpu_data_manager->gpu_data);
	}
	this->ncontacts = gpu_data_manager->number_of_contacts;
	mtimer_cd.stop();
	return 0;
}

double ChSystemGPU::SolveSystem() {
	mtimer_lcp.start();
	((ChLcpSolverGPU*) (LCP_solver_speed))->SetCompliance(0, 0, 0);
	((ChLcpSolverGPU*) (LCP_solver_speed))->SetContactFactor(.6);
	((ChLcpSolverGPU*) (LCP_solver_speed))->RunTimeStep(GetStep(), gpu_data_manager->gpu_data);

	//gpu_data_manager->host_acc_data=gpu_data_manager->host_vel_data;
	//((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->SolveSys_HOST(gpu_data_manager);

	((ChContactContainerGPUsimple*) this->contact_container)->SetNcontacts(gpu_data_manager->number_of_contacts);
	mtimer_lcp.stop();
	return 0;
}
void ChSystemGPU::AddBody(ChSharedPtr<ChBodyGPU> newbody) {
	newbody->AddRef();
	newbody->SetSystem(this);
	bodylist.push_back((newbody).get_ptr());

	ChBodyGPU* gpubody = ((ChBodyGPU*) newbody.get_ptr());
	gpubody->id = counter;
	if (newbody->GetCollide()) {
		newbody->AddCollisionModelsToSystem();
	}

	//ChLcpVariablesBodyOwnMass* mbodyvar = &(newbody->Variables());

	//float inv_mass = (1.0) / (mbodyvar->GetBodyMass());
	//newbody->GetRot().Normalize();
	//ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();
	//gpu_data_manager->host_vel_data.push_back(F3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1),
	//        mbodyvar->Get_qb().GetElementN(2)));
	//gpu_data_manager->host_omg_data.push_back(F3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4),
	//        mbodyvar->Get_qb().GetElementN(5)));
	//gpu_data_manager->host_pos_data.push_back(F3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
	//gpu_data_manager->host_rot_data.push_back(F4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
	//gpu_data_manager->host_inr_data.push_back(F3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
	//gpu_data_manager->host_frc_data.push_back(F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
	//gpu_data_manager->host_trq_data.push_back(F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
	//gpu_data_manager->host_aux_data.push_back(F3(newbody->IsActive(), newbody->GetKfriction(), inv_mass));
	//gpu_data_manager->host_lim_data.push_back(F3(newbody->GetLimitSpeed(), (1.0 / GetStep()) * .1, newbody->GetMaxWvel()));

	gpu_data_manager->host_vel_data.push_back(F3(0));
	gpu_data_manager->host_omg_data.push_back(F3(0));
	gpu_data_manager->host_pos_data.push_back(F3(0));
	gpu_data_manager->host_rot_data.push_back(F4(1, 0, 0, 0));
	gpu_data_manager->host_inr_data.push_back(F3(0));
	gpu_data_manager->host_frc_data.push_back(F3(0)); //forces
	gpu_data_manager->host_trq_data.push_back(F3(0)); //torques
	gpu_data_manager->host_aux_data.push_back(F3(0));
	gpu_data_manager->host_lim_data.push_back(F3(0));

	counter++;
	gpu_data_manager->number_of_objects = counter;
}

void ChSystemGPU::RemoveBody(ChSharedPtr<ChBodyGPU> mbody) {
	assert( std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()) != bodylist.end());

	// remove from collision system
	if (mbody->GetCollide())
		mbody->RemoveCollisionModelsFromSystem();

	// warning! linear time search, to erase pointer from container.
	bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()));

	// nullify backward link to system
	mbody->SetSystem(0);
	// this may delete the body, if none else's still referencing it..
	mbody->RemoveRef();
}

void ChSystemGPU::Update() {
	ChTimer<double> mtimer;
	mtimer.start(); // Timer for profiling

#pragma omp parallel
	{

//#pragma omp parallel for
		for (int i = 0; i < bodylist.size(); i++) // Updates recursively all other aux.vars
				{
#pragma omp task
			{
				bodylist[i]->UpdateTime(ChTime);
				bodylist[i]->UpdateMarkers(ChTime);
				bodylist[i]->UpdateForces(ChTime);
				bodylist[i]->VariablesFbReset();
				bodylist[i]->VariablesFbLoadForces(GetStep());
				bodylist[i]->VariablesQbLoadSpeed();

				ChLcpVariablesBody* mbodyvar = &(bodylist[i]->Variables());
				gpu_data_manager->host_frc_data[i] = (F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
				gpu_data_manager->host_trq_data[i] = (F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
			}
#pragma omp task
			{
				ChLcpVariablesBody* mbodyvar = &(bodylist[i]->Variables());
				ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();
				gpu_data_manager->host_inr_data[i] = (F3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
				gpu_data_manager->host_vel_data[i] = (F3(bodylist[i]->GetPos_dt().x, bodylist[i]->GetPos_dt().y, bodylist[i]->GetPos_dt().z));
				gpu_data_manager->host_omg_data[i] = (F3(bodylist[i]->GetWvel_loc().x, bodylist[i]->GetWvel_loc().y, bodylist[i]->GetWvel_loc().z));
				gpu_data_manager->host_pos_data[i] = (F3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
				gpu_data_manager->host_rot_data[i] = (F4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
				gpu_data_manager->host_aux_data[i] = (F3(bodylist[i]->IsActive(), bodylist[i]->GetKfriction(), 1.0f / mbodyvar->GetBodyMass()));
				gpu_data_manager->host_lim_data[i] = (F3(bodylist[i]->GetLimitSpeed(), 100, 100));
			}
		}
	}

	std::list<ChLink*>::iterator it;
	unsigned int number_of_bilaterals = 0;
	uint counter = 0;
	this->LCP_descriptor->BeginInsertion();
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->Update(ChTime);
		(*it)->ConstraintsBiReset();
		(*it)->ConstraintsBiLoad_C(1.0 / GetStep(), max_penetration_recovery_speed, true);
		(*it)->ConstraintsBiLoad_Ct(1);
		(*it)->ConstraintsFbLoadForces(GetStep());
		(*it)->ConstraintsLoadJacobians();
		(*it)->InjectConstraints(*this->LCP_descriptor);
	}
	this->LCP_descriptor->EndInsertion();

	std::vector<ChLcpConstraint*>& mconstraints = (*this->LCP_descriptor).GetConstraintsList();
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == true) {
			number_of_bilaterals++;
		}
	}
	gpu_data_manager->number_of_bilaterals = number_of_bilaterals;
	gpu_data_manager->host_bilateral_data.resize(number_of_bilaterals * CH_BILATERAL_VSIZE);

	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == false) {
			continue;
		}
		ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*) (mconstraints[ic]);

		int idA = ((ChBodyGPU*) ((ChLcpVariablesBody*) (mbilateral->GetVariables_a()))->GetUserData())->id;
		int idB = ((ChBodyGPU*) ((ChLcpVariablesBody*) (mbilateral->GetVariables_b()))->GetUserData())->id;

		// Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
		mconstraints[ic]->Update_auxiliary(); //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
		float4 A, B, C, D;
		A = F4(mbilateral->Get_Cq_a()->GetElementN(0), mbilateral->Get_Cq_a()->GetElementN(1), mbilateral->Get_Cq_a()->GetElementN(2), 0); //J1x
		B = F4(mbilateral->Get_Cq_b()->GetElementN(0), mbilateral->Get_Cq_b()->GetElementN(1), mbilateral->Get_Cq_b()->GetElementN(2), 0); //J2x
		C = F4(mbilateral->Get_Cq_a()->GetElementN(3), mbilateral->Get_Cq_a()->GetElementN(4), mbilateral->Get_Cq_a()->GetElementN(5), 0); //J1w
		D = F4(mbilateral->Get_Cq_b()->GetElementN(3), mbilateral->Get_Cq_b()->GetElementN(4), mbilateral->Get_Cq_b()->GetElementN(5), 0); //J2w
		A.w = idA; //pointer to body B1 info in body buffer
		B.w = idB; //pointer to body B2 info in body buffer

		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 0] = A;
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 1] = B;
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 2] = C;
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 3] = D;
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 4].x = (1.0 / mbilateral->Get_g_i()); // eta = 1/g
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 4].y = mbilateral->Get_b_i(); // b_i is residual b
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 4].z = 0; //gammma, no warm starting
		gpu_data_manager->host_bilateral_data[counter + number_of_bilaterals * 4].w = (mbilateral->IsUnilateral()) ? 1 : 0;
		counter++;
	}

	mtimer.stop();
	timer_update += mtimer();
}

void ChSystemGPU::ChangeLcpSolverSpeed(ChLcpSolver* newsolver) {
	assert(newsolver);
	if (this->LCP_solver_speed)
		delete (this->LCP_solver_speed);
	this->LCP_solver_speed = newsolver;

	//((ChLcpSolverGPU*) (LCP_solver_speed))->data_container = gpu_data_manager;
}

void ChSystemGPU::ChangeCollisionSystem(ChCollisionSystem* newcollsystem) {
	assert(this->GetNbodies() == 0);
	assert(newcollsystem);
	if (this->collision_system)
		delete (this->collision_system);
	this->collision_system = newcollsystem;

	((ChCollisionSystemGPU*) (collision_system))->data_container = gpu_data_manager;
}

}
