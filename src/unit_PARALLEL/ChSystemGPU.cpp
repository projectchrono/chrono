#include "ChSystemGPU.h"
#include <omp.h>

namespace chrono {

ChSystemGPU::ChSystemGPU(unsigned int max_objects) :
		ChSystem(1000, 10000, false) {
	counter = 0;
	cuda_device = -1;
	gpu_data_manager = new ChGPUDataManager();
	LCP_descriptor = new ChLcpSystemDescriptorGPU();
	contact_container = new ChContactContainerGPUsimple();
	collision_system = new ChCollisionSystemGPU();
	LCP_solver_speed = new ChLcpSolverGPU();
	((ChCollisionSystemGPU*) (collision_system))->data_container = gpu_data_manager;

	gpu_data_manager->gpu_data.bins_per_axis = F3(50, 50, 50);

}
int ChSystemGPU::Setup() {
	this->stepcount++;
	nbodies = 0;
	nbodies_sleep = 0;
	nbodies_fixed = 0;
	ncoords = 0;
	ncoords_w = 0;
	ndoc = 0;
	ndoc_w = 0;
	ndoc_w_C = 0;
	ndoc_w_D = 0;
	nlinks = 0;
	nphysicsitems = 0;

	for (int i = 0; i < bodylist.size(); i++) { // Updates recursively all other aux.vars
		if (!bodylist[i]->GetBodyFixed()) {
			if (!bodylist[i]->GetSleeping()) {
				nbodies++; // Count bodies and indicize them.
			} else {
				nbodies_sleep++;
			}
		} else {
			nbodies_fixed++;
		}
	}
	ncoords_w += nbodies * 6;
	ncoords += nbodies * 7; // with quaternion coords
	ndoc += nbodies; // There is a quaternion constr. for each active body.

	ndoc = ndoc_w + nbodies; // sets number of constraints including quaternion constraints.
	nsysvars = ncoords + ndoc; // sets number of total variables (=coordinates + lagrangian multipliers)
	nsysvars_w = ncoords_w + ndoc_w; // sets number of total variables (with 6 dof per body)

	ndof = ncoords - ndoc; // sets number of left degrees of freedom (approximate - does not consider constr. redundancy, etc)
	std::list<ChLink*>::iterator it;
	for (it = linklist.begin(); it != linklist.end(); it++) {
		nlinks++;
		ndoc_w += (*it)->GetDOC();
		ndoc_w_C += (*it)->GetDOC_c();
		ndoc_w_D += (*it)->GetDOC_d();
	}
	ndoc_w_D += contact_container->GetDOC_d();
	return 0;
}

int ChSystemGPU::Integrate_Y_impulse_Anitescu() {

	mtimer_step.start();
	mtimer_updt.start();
	Setup();
	Update();
	gpu_data_manager->HostToDevice();
	gpu_data_manager->HostToDeviceForces();
	mtimer_updt.stop();
	timer_update = mtimer_updt();
//------------------------------------------------------------------------------------------------------------------------

	mtimer_cd.start();

	if (gpu_data_manager->number_of_models > 0) {
		mtimer_cd_broad.start();
		ChCCollisionGPU::ComputeAABB(gpu_data_manager->gpu_data);
		gpu_data_manager->host_aabb_data = gpu_data_manager->gpu_data.device_aabb_data;
		gpu_data_manager->gpu_data.bin_size_vec = (fabs(gpu_data_manager->gpu_data.max_bounding_point + fabs(gpu_data_manager->gpu_data.min_bounding_point)));
		ChCCollisionGPU::Broadphase(gpu_data_manager->gpu_data, false);
		mtimer_cd_broad.stop();
		mtimer_cd_narrow.start();
		ChCCollisionGPU::Narrowphase(gpu_data_manager->gpu_data);
		mtimer_cd_narrow.stop();
	}
	mtimer_cd.stop();

//------------------------------------------------------------------------------------------------------------------------

	mtimer_lcp.start();

	((ChLcpSolverGPU*) (LCP_solver_speed))->SetCompliance(0, 0, 0);
	((ChLcpSolverGPU*) (LCP_solver_speed))->RunTimeStep(GetStep(), gpu_data_manager->gpu_data);
	mtimer_lcp.stop();
//------------------------------------------------------------------------------------------------------------------------
	mtimer_updt.start();
	gpu_data_manager->DeviceToHost();

	float3 * vel_pointer = gpu_data_manager->host_vel_data.data();
	float3 * omg_pointer = gpu_data_manager->host_omg_data.data();
	float3 * pos_pointer = gpu_data_manager->host_pos_data.data();
	float4 * rot_pointer = gpu_data_manager->host_rot_data.data();
	float3 * acc_pointer = gpu_data_manager->host_acc_data.data();
	float3 * fap_pointer = gpu_data_manager->host_fap_data.data();

//#pragma omp parallel
	{
//#pragma omp for
		for (int i = 0; i < bodylist.size(); i++) {
			ChBodyGPU* mbody = (ChBodyGPU*) bodylist[i];
			if (mbody->IsActive()) {
					mbody->SetPos(CHVECCAST(pos_pointer[i]));
					mbody->SetRot(CHQUATCAST(rot_pointer[i]));
					mbody->SetPos_dt(CHVECCAST(vel_pointer[i]));
					mbody->SetPos_dtdt(CHVECCAST(acc_pointer[i]));
					mbody->SetWvel_loc(CHVECCAST(omg_pointer[i]));
					mbody->SetAppliedForce(CHVECCAST(fap_pointer[i]));
			}
		}
	}
	uint counter = 0;
	std::vector<ChLcpConstraint*>& mconstraints = (*this->LCP_descriptor).GetConstraintsList();
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == false) {
			continue;
		}
		ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*) (mconstraints[ic]);
		float gamma = gpu_data_manager->host_bilateral_data[counter + gpu_data_manager->number_of_bilaterals * 4].z;
		mconstraints[ic]->Set_l_i(gamma);
		counter++;
	}

// updates the reactions of the constraint
	LCPresult_Li_into_reactions(1.0 / this->GetStep()); // R = l/dt  , approximately

	std::list<ChLink*>::iterator it;
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->ConstraintsLiFetchSuggestedSpeedSolution();
	}
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->ConstraintsFetch_react(1.0 / this->GetStep());
	}
	mtimer_updt.stop();
	timer_update += mtimer_updt();
	ChTime += GetStep();
	mtimer_step.stop();
	timer_collision = mtimer_cd();
	timer_collision_broad = mtimer_cd_broad();
	timer_collision_narrow = mtimer_cd_narrow();
	timer_lcp = mtimer_lcp();
	timer_step = mtimer_step(); // Time elapsed for step..
	return 1;
}

double ChSystemGPU::ComputeCollisions() {
	return 0;
}

double ChSystemGPU::SolveSystem() {
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

	ChLcpVariablesBodyOwnMass* mbodyvar = &(newbody->Variables());

	float inv_mass = (1.0) / (mbodyvar->GetBodyMass());
	newbody->GetRot().Normalize();
	ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();
	gpu_data_manager->host_vel_data.push_back(F3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1), mbodyvar->Get_qb().GetElementN(2)));
	gpu_data_manager->host_acc_data.push_back(F3(0, 0, 0));
	gpu_data_manager->host_omg_data.push_back(F3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4), mbodyvar->Get_qb().GetElementN(5)));
	gpu_data_manager->host_pos_data.push_back(F3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
	gpu_data_manager->host_rot_data.push_back(F4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
	gpu_data_manager->host_inr_data.push_back(F3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
	gpu_data_manager->host_frc_data.push_back(F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
	gpu_data_manager->host_trq_data.push_back(F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
	gpu_data_manager->host_aux_data.push_back(F3(newbody->IsActive(), newbody->GetKfriction(), inv_mass));
	gpu_data_manager->host_lim_data.push_back(F3(newbody->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
	newbody->gpu_data_manager = gpu_data_manager;
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

void ChSystemGPU::RemoveBody(int body) {
	//assert( std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()) != bodylist.end());
	ChBodyGPU * mbody = ((ChBodyGPU *) (bodylist[body]));
// remove from collision system
	if (mbody->GetCollide())
		mbody->RemoveCollisionModelsFromSystem();

// warning! linear time search, to erase pointer from container.
	//bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()));

// nullify backward link to system
	//mbody->SetSystem(0);
// this may delete the body, if none else's still referencing it..
	//mbody->RemoveRef();
}
void ChSystemGPU::Update() {
	float3 * vel_pointer = gpu_data_manager->host_vel_data.data();
	float3 * omg_pointer = gpu_data_manager->host_omg_data.data();
	float3 * pos_pointer = gpu_data_manager->host_pos_data.data();
	float4 * rot_pointer = gpu_data_manager->host_rot_data.data();
	float3 * inr_pointer = gpu_data_manager->host_inr_data.data();
	float3 * frc_pointer = gpu_data_manager->host_frc_data.data();
	float3 * trq_pointer = gpu_data_manager->host_trq_data.data();
	float3 * aux_pointer = gpu_data_manager->host_aux_data.data();
	float3 * lim_pointer = gpu_data_manager->host_lim_data.data();

	unsigned int number_of_bilaterals = 0;
	uint cntr = 0;
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

		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 0] = A;
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 1] = B;
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 2] = C;
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 3] = D;
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 4].x = (1.0 / mbilateral->Get_g_i()); // eta = 1/g
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 4].y = mbilateral->Get_b_i(); // b_i is residual b
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 4].z = 0; //gammma, no warm starting
		gpu_data_manager->host_bilateral_data[cntr + number_of_bilaterals * 4].w = (mbilateral->IsUnilateral()) ? 1 : 0;
		cntr++;
	}

//#pragma omp parallel
	{
//#pragma omp for
		for (int i = 0; i < bodylist.size(); i++) { // Updates recursively all other aux.vars

				bodylist[i]->UpdateTime(ChTime);
				bodylist[i]->UpdateMarkers(ChTime);
				bodylist[i]->UpdateForces(ChTime);
				bodylist[i]->VariablesFbReset();
				bodylist[i]->VariablesFbLoadForces(GetStep());
				bodylist[i]->VariablesQbLoadSpeed();

				ChLcpVariablesBody* mbodyvar = &(bodylist[i]->Variables());
				ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();

				vel_pointer[i] = (F3(bodylist[i]->GetPos_dt().x, bodylist[i]->GetPos_dt().y, bodylist[i]->GetPos_dt().z));
				omg_pointer[i] = (F3(bodylist[i]->GetWvel_loc().x, bodylist[i]->GetWvel_loc().y, bodylist[i]->GetWvel_loc().z));
				pos_pointer[i] = (F3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
				rot_pointer[i] = (F4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
				inr_pointer[i] = (F3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
				frc_pointer[i] = (F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
				trq_pointer[i] = (F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
				aux_pointer[i] = (F3(bodylist[i]->IsActive(), bodylist[i]->GetKfriction(), 1.0f / mbodyvar->GetBodyMass()));
				lim_pointer[i] = (F3(bodylist[i]->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
			}
	}

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
