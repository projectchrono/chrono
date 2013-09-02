#include "ChSystemGPU.h"
#include "physics/ChBody.h"
#include <omp.h>

using namespace chrono;

ChSystemGPU::ChSystemGPU(unsigned int max_objects) :
		ChSystem(1000, 10000, false) {
	counter = 0;
	gpu_data_manager = new ChGPUDataManager();
	LCP_descriptor = new ChLcpSystemDescriptorGPU();
	contact_container = new ChContactContainerGPU();
	collision_system = new ChCollisionSystemGPU();
	LCP_solver_speed = new ChLcpSolverGPU();
	((ChCollisionSystemGPU *) (collision_system))->data_container = gpu_data_manager;
	((ChLcpSystemDescriptorGPU *) (LCP_descriptor))->data_container = gpu_data_manager;
	((ChLcpSolverGPU *) (LCP_solver_speed))->data_container = gpu_data_manager;
	((ChContactContainerGPU*) contact_container)->data_container = gpu_data_manager;


}
int ChSystemGPU::Integrate_Y_impulse_Anitescu() {
	mtimer_step.start();
	//=============================================================================================
	mtimer_updt.start();
	Setup();
	Update();
	//gpu_data_manager->Copy(HOST_TO_DEVICE);
	mtimer_updt.stop();
	timer_update = mtimer_updt();
	//=============================================================================================
//	vector<bool> body_active(gpu_data_manager->number_of_rigid, false);
//	((ChCollisionSystemGPU*) collision_system)->GetOverlappingAABB(body_active, R3(-2, -10, -2), R3(2, 10, 2));
//	for (int i = 0; i < bodylist.size(); i++) {
//		if (bodylist[i]->IsActive() == true) {
//			gpu_data_manager->host_data.active_data[i] = body_active[i];
//		}
//	}
	//=============================================================================================
	mtimer_cd.start();
	collision_system->Run();
	collision_system->ReportContacts(this->contact_container);
	mtimer_cd.stop();
	//=============================================================================================
	mtimer_lcp.start();
	((ChLcpSolverGPU *) (LCP_solver_speed))->RunTimeStep(GetStep());
	mtimer_lcp.stop();
	//=============================================================================================
	mtimer_updt.start();
	//gpu_data_manager->Copy(DEVICE_TO_HOST);
	//std::vector<ChLcpVariables*> vvariables = LCP_descriptor->GetVariablesList();

	uint counter = 0;
	std::vector<ChLcpConstraint *> &mconstraints = (*this->LCP_descriptor).GetConstraintsList();
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == false) {
			continue;
		}
		ChLcpConstraintTwoBodies *mbilateral = (ChLcpConstraintTwoBodies *) (mconstraints[ic]);
		mconstraints[ic]->Set_l_i(gpu_data_manager->host_data.gamma_bilateral[counter]);
		counter++;
	}
// updates the reactions of the constraint
	LCPresult_Li_into_reactions(1.0 / this->GetStep());     // R = l/dt  , approximately

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		if (gpu_data_manager->host_data.active_data[i] == true) {
			real3 vel = gpu_data_manager->host_data.vel_data[i];
			real3 omg = gpu_data_manager->host_data.omg_data[i];
			bodylist[i]->Variables().Get_qb().SetElement(0, 0, vel.x);
			bodylist[i]->Variables().Get_qb().SetElement(1, 0, vel.y);
			bodylist[i]->Variables().Get_qb().SetElement(2, 0, vel.z);
			bodylist[i]->Variables().Get_qb().SetElement(3, 0, omg.x);
			bodylist[i]->Variables().Get_qb().SetElement(4, 0, omg.y);
			bodylist[i]->Variables().Get_qb().SetElement(5, 0, omg.z);

			bodylist[i]->VariablesQbIncrementPosition(this->GetStep());
			bodylist[i]->VariablesQbSetSpeed(this->GetStep());
			bodylist[i]->UpdateTime(ChTime);
			//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
			//bodylist[i]->ClampSpeed();     // Apply limits (if in speed clamping mode) to speeds.
			//bodylist[i]->ComputeGyro();     // Set the gyroscopic momentum.
			//bodylist[i]->UpdateForces(ChTime);
		}

	}
	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->UpdateMarkers(ChTime);
	}

	mtimer_updt.stop();
	timer_update += mtimer_updt();
	//=============================================================================================
	ChTime += GetStep();
	mtimer_step.stop();
	timer_collision = mtimer_cd();
	if (ChCollisionSystemGPU* coll_sys = dynamic_cast<ChCollisionSystemGPU*>(collision_system)) {
		timer_collision_broad = coll_sys->mtimer_cd_broad();
		timer_collision_narrow = coll_sys->mtimer_cd_narrow();
	} else {
		timer_collision_broad = 0;
		timer_collision_narrow = 0;

	}
	timer_lcp = mtimer_lcp();
	timer_step = mtimer_step();     // Time elapsed for step..
	return 1;
}

double ChSystemGPU::ComputeCollisions() {
	return 0;
}

double ChSystemGPU::SolveSystem() {
	return 0;
}
void ChSystemGPU::AddBody(ChSharedPtr<ChBody> newbody) {

	newbody->AddRef();
	newbody->SetSystem(this);
	bodylist.push_back((newbody).get_ptr());
	ChBody *gpubody = ((ChBody *) newbody.get_ptr());
	gpubody->SetId(counter);

	if (newbody->GetCollide()) {
		newbody->AddCollisionModelsToSystem();
	}

	ChLcpVariablesBodyOwnMass *mbodyvar = &(newbody->Variables());
	real inv_mass = (1.0) / (mbodyvar->GetBodyMass());
	newbody->GetRot().Normalize();
	ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();
	gpu_data_manager->host_data.vel_data.push_back(R3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1), mbodyvar->Get_qb().GetElementN(2)));
	gpu_data_manager->host_data.acc_data.push_back(R3(0, 0, 0));
	gpu_data_manager->host_data.omg_data.push_back(R3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4), mbodyvar->Get_qb().GetElementN(5)));
	gpu_data_manager->host_data.pos_data.push_back(R3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
	gpu_data_manager->host_data.rot_data.push_back(R4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
	gpu_data_manager->host_data.inr_data.push_back(R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
	gpu_data_manager->host_data.frc_data.push_back(R3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2)));     //forces
	gpu_data_manager->host_data.trq_data.push_back(R3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5)));     //torques
	gpu_data_manager->host_data.active_data.push_back(newbody->IsActive());
	gpu_data_manager->host_data.mass_data.push_back(inv_mass);
	gpu_data_manager->host_data.fric_data.push_back(newbody->GetKfriction());
	gpu_data_manager->host_data.cohesion_data.push_back(newbody->GetMaterialSurface()->GetCohesion());
	gpu_data_manager->host_data.compliance_data.push_back(newbody->GetMaterialSurface()->GetCompliance());
	gpu_data_manager->host_data.lim_data.push_back(R3(newbody->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
	//gpu_data_manager->host_data.pressure_data.push_back(0);
	//newbody->gpu_data_manager = gpu_data_manager;
	counter++;
	gpu_data_manager->number_of_rigid = counter;
}

void ChSystemGPU::RemoveBody(ChSharedPtr<ChBody> mbody) {
	assert(std::find<std::vector<ChBody *>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()) != bodylist.end());

// remove from collision system
	if (mbody->GetCollide())
		mbody->RemoveCollisionModelsFromSystem();

// warning! linear time search, to erase pointer from container.
	bodylist.erase(std::find<std::vector<ChBody *>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()));
// nullify backward link to system
	mbody->SetSystem(0);
// this may delete the body, if none else's still referencing it..
	mbody->RemoveRef();
}

void ChSystemGPU::RemoveBody(int body) {
	//assert( std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr()) != bodylist.end());
	ChBody *mbody = ((ChBody *) (bodylist[body]));

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
	this->LCP_descriptor->BeginInsertion();
	UpdateBodies();
	UpdateBilaterals();
	LCP_descriptor->EndInsertion();
}
void ChSystemGPU::UpdateBodies() {
	real3 *vel_pointer = gpu_data_manager->host_data.vel_data.data();
	real3 *omg_pointer = gpu_data_manager->host_data.omg_data.data();
	real3 *pos_pointer = gpu_data_manager->host_data.pos_data.data();
	real4 *rot_pointer = gpu_data_manager->host_data.rot_data.data();
	real3 *inr_pointer = gpu_data_manager->host_data.inr_data.data();
	real3 *frc_pointer = gpu_data_manager->host_data.frc_data.data();
	real3 *trq_pointer = gpu_data_manager->host_data.trq_data.data();
	bool *active_pointer = gpu_data_manager->host_data.active_data.data();
	real *mass_pointer = gpu_data_manager->host_data.mass_data.data();
	real *fric_pointer = gpu_data_manager->host_data.fric_data.data();
	real *cohesion_pointer = gpu_data_manager->host_data.cohesion_data.data();
	real *compliance_pointer = gpu_data_manager->host_data.compliance_data.data();
	real3 *lim_pointer = gpu_data_manager->host_data.lim_data.data();

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->UpdateTime(ChTime);
		//bodylist[i]->TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
		bodylist[i]->ClampSpeed();     // Apply limits (if in speed clamping mode) to speeds.
		bodylist[i]->ComputeGyro();     // Set the gyroscopic momentum.
		bodylist[i]->UpdateForces(ChTime);
		bodylist[i]->VariablesFbReset();
		bodylist[i]->VariablesFbLoadForces(GetStep());
		bodylist[i]->VariablesQbLoadSpeed();
	}

	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->UpdateMarkers(ChTime);
		//bodylist[i]->InjectVariables(*this->LCP_descriptor);
	}

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		ChMatrix33<> inertia = bodylist[i]->Variables().GetBodyInvInertia();
		vel_pointer[i] = (R3(bodylist[i]->Variables().Get_qb().ElementN(0), bodylist[i]->Variables().Get_qb().ElementN(1), bodylist[i]->Variables().Get_qb().ElementN(2)));
		omg_pointer[i] = (R3(bodylist[i]->Variables().Get_qb().ElementN(3), bodylist[i]->Variables().Get_qb().ElementN(4), bodylist[i]->Variables().Get_qb().ElementN(5)));
		pos_pointer[i] = (R3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
		rot_pointer[i] = (R4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
		inr_pointer[i] = (R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
		frc_pointer[i] = (R3(bodylist[i]->Variables().Get_fb().ElementN(0), bodylist[i]->Variables().Get_fb().ElementN(1), bodylist[i]->Variables().Get_fb().ElementN(2)));     //forces
		trq_pointer[i] = (R3(bodylist[i]->Variables().Get_fb().ElementN(3), bodylist[i]->Variables().Get_fb().ElementN(4), bodylist[i]->Variables().Get_fb().ElementN(5)));     //torques
		active_pointer[i] = bodylist[i]->IsActive();
		mass_pointer[i] = 1.0f / bodylist[i]->Variables().GetBodyMass();
		fric_pointer[i] = bodylist[i]->GetKfriction();
		cohesion_pointer[i] = ((bodylist[i]))->GetMaterialSurface()->GetCohesion();
		compliance_pointer[i] = ((bodylist[i]))->GetMaterialSurface()->GetCompliance();
		lim_pointer[i] = (R3(bodylist[i]->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
		bodylist[i]->GetCollisionModel()->SyncPosition();
	}
}

void ChSystemGPU::UpdateBilaterals() {
	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->Update(ChTime);
		(*it)->ConstraintsBiReset();
		(*it)->ConstraintsBiLoad_C(1.0 / GetStep(), max_penetration_recovery_speed, true);
		(*it)->ConstraintsBiLoad_Ct(1);
		(*it)->ConstraintsFbLoadForces(GetStep());
		(*it)->ConstraintsLoadJacobians();
		(*it)->InjectConstraints(*this->LCP_descriptor);
	}
	unsigned int number_of_bilaterals = 0, cntr = 0;
	std::vector<ChLcpConstraint *> &mconstraints = (*this->LCP_descriptor).GetConstraintsList();
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == true) {
			number_of_bilaterals++;
		}
	}
	gpu_data_manager->number_of_bilaterals = number_of_bilaterals;

	gpu_data_manager->host_data.JXYZA_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.JXYZB_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.JUVWA_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.JUVWB_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.residual_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.correction_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.bids_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_data.gamma_bilateral.resize(number_of_bilaterals);
//#pragma omp parallel for
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == false) {
			continue;
		}

		ChLcpConstraintTwoBodies *mbilateral = (ChLcpConstraintTwoBodies *) (mconstraints[ic]);
		int idA = ((ChBody *) ((ChLcpVariablesBody *) (mbilateral->GetVariables_a()))->GetUserData())->GetId();
		int idB = ((ChBody *) ((ChLcpVariablesBody *) (mbilateral->GetVariables_b()))->GetUserData())->GetId();
		// Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
		mconstraints[ic]->Update_auxiliary();     //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
		real3 A, B, C, D;
		A = R3(mbilateral->Get_Cq_a()->GetElementN(0), mbilateral->Get_Cq_a()->GetElementN(1), mbilateral->Get_Cq_a()->GetElementN(2));     //J1x
		B = R3(mbilateral->Get_Cq_b()->GetElementN(0), mbilateral->Get_Cq_b()->GetElementN(1), mbilateral->Get_Cq_b()->GetElementN(2));     //J2x
		C = R3(mbilateral->Get_Cq_a()->GetElementN(3), mbilateral->Get_Cq_a()->GetElementN(4), mbilateral->Get_Cq_a()->GetElementN(5));     //J1w
		D = R3(mbilateral->Get_Cq_b()->GetElementN(3), mbilateral->Get_Cq_b()->GetElementN(4), mbilateral->Get_Cq_b()->GetElementN(5));     //J2w

		gpu_data_manager->host_data.JXYZA_bilateral[cntr] = A;
		gpu_data_manager->host_data.JXYZB_bilateral[cntr] = B;
		gpu_data_manager->host_data.JUVWA_bilateral[cntr] = C;
		gpu_data_manager->host_data.JUVWB_bilateral[cntr] = D;
		gpu_data_manager->host_data.residual_bilateral[cntr] = mbilateral->Get_b_i();     // b_i is residual b
		gpu_data_manager->host_data.correction_bilateral[cntr] = 1.0 / mbilateral->Get_g_i();     // eta = 1/g
		gpu_data_manager->host_data.bids_bilateral[cntr] = I2(idA, idB);
		gpu_data_manager->host_data.gamma_bilateral[cntr] = mbilateral->Get_l_i();
		cntr++;
	}
}

void ChSystemGPU::ChangeLcpSolverSpeed(ChLcpSolver *newsolver) {
	assert(newsolver);

	if (this->LCP_solver_speed)
		delete (this->LCP_solver_speed);

	this->LCP_solver_speed = newsolver;
}

void ChSystemGPU::ChangeCollisionSystem(ChCollisionSystem *newcollsystem) {
	assert(this->GetNbodies() == 0);
	assert(newcollsystem);

	if (this->collision_system)
		delete (this->collision_system);

	this->collision_system = newcollsystem;

	if (ChCollisionSystemGPU* coll_sys = dynamic_cast<ChCollisionSystemGPU*>(newcollsystem)) {
		((ChCollisionSystemGPU *) (collision_system))->data_container = gpu_data_manager;
	} else if (ChCollisionSystemBulletGPU* coll_sys = dynamic_cast<ChCollisionSystemBulletGPU*>(newcollsystem)) {
		((ChCollisionSystemBulletGPU *) (collision_system))->data_container = gpu_data_manager;
	}
}

void ChSystemGPU::ChangeLcpSystemDescriptor(ChLcpSystemDescriptor* newdescriptor) {
	assert(newdescriptor);
	if (this->LCP_descriptor)
		delete (this->LCP_descriptor);
	this->LCP_descriptor = newdescriptor;

	((ChLcpSystemDescriptorGPU *) (this->LCP_descriptor))->data_container = gpu_data_manager;
}
