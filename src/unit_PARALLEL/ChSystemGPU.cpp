#include "ChSystemGPU.h"
#include "ChBodyGPU.h"
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
}
//int ChSystemGPU::Setup() {
//	this->stepcount++;
//	nbodies = 0;
//	nbodies_sleep = 0;
//	nbodies_fixed = 0;
//	ncoords = 0;
//	ncoords_w = 0;
//	ndoc = 0;
//	ndoc_w = 0;
//	ndoc_w_C = 0;
//	ndoc_w_D = 0;
//	nlinks = 0;
//	nphysicsitems = 0;
//
//	for (int i = 0; i < bodylist.size(); i++) { // Updates recursively all other aux.vars
//		if (!bodylist[i]->GetBodyFixed()) {
//			if (!bodylist[i]->GetSleeping()) {
//				nbodies++; // Count bodies and indicize them.
//			} else {
//				nbodies_sleep++;
//			}
//		} else {
//			nbodies_fixed++;
//		}
//	}
//
//	ncoords_w += nbodies * 6;
//	ncoords += nbodies * 7; // with quaternion coords
//	ndoc += nbodies; // There is a quaternion constr. for each active body.
//	ndoc = ndoc_w + nbodies; // sets number of constraints including quaternion constraints.
//	nsysvars = ncoords + ndoc; // sets number of total variables (=coordinates + lagrangian multipliers)
//	nsysvars_w = ncoords_w + ndoc_w; // sets number of total variables (with 6 dof per body)
//	ndof = ncoords - ndoc; // sets number of left degrees of freedom (approximate - does not consider constr. redundancy, etc)
//	std::list<ChLink *>::iterator it;
//
//	for (it = linklist.begin(); it != linklist.end(); it++) {
//		nlinks++;
//		ndoc_w += (*it)->GetDOC();
//		ndoc_w_C += (*it)->GetDOC_c();
//		ndoc_w_D += (*it)->GetDOC_d();
//	}
//
//	ndoc_w_D += contact_container->GetDOC_d();
//	return 0;
//}

int ChSystemGPU::Integrate_Y_impulse_Anitescu() {
	mtimer_step.start();
	mtimer_updt.start();
	Setup();
	Update();
	gpu_data_manager->HostToDeviceCD();
	gpu_data_manager->HostToDevice();
	gpu_data_manager->HostToDeviceForces();
	mtimer_updt.stop();
	timer_update = mtimer_updt();
//------------------------------------------------------------------------------------------------------------------------
	mtimer_cd.start();
	if (gpu_data_manager->number_of_models > 0) {
		collision_system->Run();
		collision_system->ReportContacts(this->contact_container);
	}
	mtimer_cd.stop();
//------------------------------------------------------------------------------------------------------------------------
	mtimer_lcp.start();
	((ChLcpSolverGPU *) (LCP_solver_speed))->RunTimeStep(GetStep(), gpu_data_manager);
	mtimer_lcp.stop();
//------------------------------------------------------------------------------------------------------------------------
	mtimer_updt.start();
	gpu_data_manager->DeviceToHost();
	std::vector<ChLcpVariables*> vvariables = LCP_descriptor->GetVariablesList();

	uint counter = 0;
	std::vector<ChLcpConstraint *> &mconstraints = (*this->LCP_descriptor).GetConstraintsList();
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == false) {
			continue;
		}
		ChLcpConstraintTwoBodies *mbilateral = (ChLcpConstraintTwoBodies *) (mconstraints[ic]);
		mconstraints[ic]->Set_l_i(gpu_data_manager->host_gamma_bilateral[counter]);
		counter++;
	}
// updates the reactions of the constraint
	LCPresult_Li_into_reactions(1.0 / this->GetStep()); // R = l/dt  , approximately

#pragma omp parallel for
	for (int i = 0; i < vvariables.size(); i++) {

		real3 vel = gpu_data_manager->host_vel_data[i];
		real3 omg = gpu_data_manager->host_omg_data[i];
		vvariables[i]->Get_qb().SetElement(0, 0, vel.x);
		vvariables[i]->Get_qb().SetElement(1, 0, vel.y);
		vvariables[i]->Get_qb().SetElement(2, 0, vel.z);
		vvariables[i]->Get_qb().SetElement(3, 0, omg.x);
		vvariables[i]->Get_qb().SetElement(4, 0, omg.y);
		vvariables[i]->Get_qb().SetElement(5, 0, omg.z);
		ChBodyGPU *mbody = (ChBodyGPU *) bodylist[i];

		mbody->VariablesQbIncrementPosition(this->GetStep());
		mbody->VariablesQbSetSpeed(this->GetStep());
		mbody->UpdateTime(ChTime);
		//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
		mbody->ClampSpeed(); // Apply limits (if in speed clamping mode) to speeds.
		mbody->ComputeGyro(); // Set the gyroscopic momentum.
		mbody->UpdateForces(ChTime);

	}
	for (int i = 0; i < vvariables.size(); i++) {
		ChBodyGPU *mbody = (ChBodyGPU *) bodylist[i];
		mbody->UpdateMarkers(ChTime);
	}

//	real3 *vel_pointer = gpu_data_manager->host_vel_data.data();
//	real3 *omg_pointer = gpu_data_manager->host_omg_data.data();
//	real3 *pos_pointer = gpu_data_manager->host_pos_data.data();
//	real4 *rot_pointer = gpu_data_manager->host_rot_data.data();
//	real3 *acc_pointer = gpu_data_manager->host_acc_data.data();
//	real3 *gyr_pointer = gpu_data_manager->host_gyr_data.data();
//	real3 *fap_pointer = gpu_data_manager->host_fap_data.data();
//
//#pragma omp parallel for
//		for (int i = 0; i < bodylist.size(); i++) {
//			ChBodyGPU *mbody = (ChBodyGPU *) bodylist[i];
//
//			if (mbody->IsActive()) {
//				mbody->SetPos(CHVECCAST(pos_pointer[i]));
//				mbody->SetRot(CHQUATCAST(rot_pointer[i]));
//				mbody->SetPos_dt(CHVECCAST(vel_pointer[i]));
//				mbody->SetPos_dtdt(CHVECCAST(acc_pointer[i]));
//				mbody->SetWvel_loc(CHVECCAST(omg_pointer[i]));
//				mbody->SetAppliedForce(CHVECCAST(fap_pointer[i]));
//				mbody->SetGyro(CHVECCAST(gyr_pointer[i]));
//
//			}
//		}

	mtimer_updt.stop();
	timer_update += mtimer_updt();
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
	ChBodyGPU *gpubody = ((ChBodyGPU *) newbody.get_ptr());
	gpubody->id = counter;

	if (newbody->GetCollide()) {
		newbody->AddCollisionModelsToSystem();
	}

	ChLcpVariablesBodyOwnMass *mbodyvar = &(newbody->Variables());
	real inv_mass = (1.0) / (mbodyvar->GetBodyMass());
	newbody->GetRot().Normalize();
	ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();
	gpu_data_manager->host_vel_data.push_back(R3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1), mbodyvar->Get_qb().GetElementN(2)));
	gpu_data_manager->host_acc_data.push_back(R3(0, 0, 0));
	gpu_data_manager->host_omg_data.push_back(R3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4), mbodyvar->Get_qb().GetElementN(5)));
	gpu_data_manager->host_pos_data.push_back(R3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
	gpu_data_manager->host_rot_data.push_back(R4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
	gpu_data_manager->host_inr_data.push_back(R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
	gpu_data_manager->host_frc_data.push_back(R3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
	gpu_data_manager->host_trq_data.push_back(R3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
	gpu_data_manager->host_active_data.push_back(newbody->IsActive());
	gpu_data_manager->host_mass_data.push_back(inv_mass);
	gpu_data_manager->host_fric_data.push_back(newbody->GetKfriction());
	gpu_data_manager->host_cohesion_data.push_back(newbody->GetCohesion());
	gpu_data_manager->host_lim_data.push_back(R3(newbody->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
	//newbody->gpu_data_manager = gpu_data_manager;
	counter++;
	gpu_data_manager->number_of_objects = counter;
}

void ChSystemGPU::RemoveBody(ChSharedPtr<ChBodyGPU> mbody) {
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
	ChBodyGPU *mbody = ((ChBodyGPU *) (bodylist[body]));

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

	real3 *vel_pointer = gpu_data_manager->host_vel_data.data();
	real3 *omg_pointer = gpu_data_manager->host_omg_data.data();
	real3 *pos_pointer = gpu_data_manager->host_pos_data.data();
	real4 *rot_pointer = gpu_data_manager->host_rot_data.data();
	real3 *inr_pointer = gpu_data_manager->host_inr_data.data();
	real3 *frc_pointer = gpu_data_manager->host_frc_data.data();
	real3 *trq_pointer = gpu_data_manager->host_trq_data.data();
	bool *active_pointer = gpu_data_manager->host_active_data.data();
	real *mass_pointer = gpu_data_manager->host_mass_data.data();
	real *fric_pointer = gpu_data_manager->host_fric_data.data();
	real *cohesion_pointer = gpu_data_manager->host_cohesion_data.data();
	real3 *lim_pointer = gpu_data_manager->host_lim_data.data();

	//========================================================================

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->UpdateTime(ChTime);
		//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
		bodylist[i]->ClampSpeed(); // Apply limits (if in speed clamping mode) to speeds.
		bodylist[i]->ComputeGyro(); // Set the gyroscopic momentum.
		bodylist[i]->UpdateForces(ChTime);
		bodylist[i]->VariablesFbReset();
		bodylist[i]->VariablesFbLoadForces(GetStep());
		bodylist[i]->VariablesQbLoadSpeed();
	}
	this->LCP_descriptor->BeginInsertion();

	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->UpdateMarkers(ChTime);
		bodylist[i]->InjectVariables(*this->LCP_descriptor);
	}

	for (it = linklist.begin(); it != linklist.end(); it++) {
		(*it)->Update(ChTime);
		(*it)->ConstraintsBiReset();
		(*it)->ConstraintsBiLoad_C(1.0 / GetStep(), max_penetration_recovery_speed, true);
		(*it)->ConstraintsBiLoad_Ct(1);
		(*it)->ConstraintsFbLoadForces(GetStep());
		(*it)->ConstraintsLoadJacobians();
		(*it)->InjectConstraints(*this->LCP_descriptor);
	}
	LCP_descriptor->EndInsertion();
	//========================================================================
//	LCPprepare_reset();
	//	for (it = linklist.begin(); it != linklist.end(); it++) {
//
//	}
//
//	for (int i = 0; i < bodylist.size(); i++) {
//
//	}
	//========================================================================
//	LCPprepare_load(true, // Cq,
//			true, // v_old (needed for adding [M]*v_old to the known vector)
//			GetStep(), // f*dt
//			GetStep() * GetStep(), // dt^2*K  (nb only non-Schur based solvers support K matrix blocks)
//			GetStep(), // dt*R   (nb only non-Schur based solvers support R matrix blocks)
//			1.0, // Ct   (needed, for rheonomic motors)
//			1.0 / GetStep(), // C/dt
//			max_penetration_recovery_speed, // vlim, max penetrations recovery speed (positive for exiting)
//			true); // do above max. clamping on -C/dt

	//	for (it = linklist.begin(); it != linklist.end(); it++) {
//
//	}
//	for (int i = 0; i < bodylist.size(); i++) {
//	}
//	LCPprepare_Li_from_speed_cache();
	//========================================================================
//	this->LCP_descriptor->BeginInsertion();
//	for (it = linklist.begin(); it != linklist.end(); it++) {
//
//	}
//	for (int i = 0; i < bodylist.size(); i++) {
//
//	}
//
//	LCP_descriptor->EndInsertion();
//	LCPprepare_inject(*this->LCP_descriptor);
	//========================================================================

	//========================================================================

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		ChLcpVariablesBody *mbodyvar = &(bodylist[i]->Variables());
		ChMatrix33<> inertia = mbodyvar->GetBodyInvInertia();
		vel_pointer[i] = (R3(mbodyvar->Get_qb().ElementN(0), mbodyvar->Get_qb().ElementN(1), mbodyvar->Get_qb().ElementN(2)));
		omg_pointer[i] = (R3(mbodyvar->Get_qb().ElementN(3), mbodyvar->Get_qb().ElementN(4), mbodyvar->Get_qb().ElementN(5)));
		pos_pointer[i] = (R3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
		rot_pointer[i] = (R4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
		inr_pointer[i] = (R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
		frc_pointer[i] = (R3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
		trq_pointer[i] = (R3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
		active_pointer[i] = bodylist[i]->IsActive();
		mass_pointer[i] = 1.0f / mbodyvar->GetBodyMass();
		fric_pointer[i] = bodylist[i]->GetKfriction();
		cohesion_pointer[i] = ((ChBodyGPU*) (bodylist[i]))->GetCohesion();
		lim_pointer[i] = (R3(bodylist[i]->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
		bodylist[i]->GetCollisionModel()->SyncPosition();
	}

	unsigned int number_of_bilaterals = 0, cntr = 0;
	std::vector<ChLcpConstraint *> &mconstraints = (*this->LCP_descriptor).GetConstraintsList();
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == true) {
			number_of_bilaterals++;
		}
	}
	gpu_data_manager->number_of_bilaterals = number_of_bilaterals;

	gpu_data_manager->host_JXYZA_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_JXYZB_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_JUVWA_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_JUVWB_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_residual_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_correction_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_bids_bilateral.resize(number_of_bilaterals);
	gpu_data_manager->host_gamma_bilateral.resize(number_of_bilaterals);
#pragma omp parallel for
	for (uint ic = 0; ic < mconstraints.size(); ic++) {
		if (mconstraints[ic]->IsActive() == false) {
			continue;
		}

		ChLcpConstraintTwoBodies *mbilateral = (ChLcpConstraintTwoBodies *) (mconstraints[ic]);
		int idA = ((ChBodyGPU *) ((ChLcpVariablesBody *) (mbilateral->GetVariables_a()))->GetUserData())->id;
		int idB = ((ChBodyGPU *) ((ChLcpVariablesBody *) (mbilateral->GetVariables_b()))->GetUserData())->id;
		// Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
		mconstraints[ic]->Update_auxiliary(); //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
		real3 A, B, C, D;
		A = R3(mbilateral->Get_Cq_a()->GetElementN(0), mbilateral->Get_Cq_a()->GetElementN(1), mbilateral->Get_Cq_a()->GetElementN(2)); //J1x
		B = R3(mbilateral->Get_Cq_b()->GetElementN(0), mbilateral->Get_Cq_b()->GetElementN(1), mbilateral->Get_Cq_b()->GetElementN(2)); //J2x
		C = R3(mbilateral->Get_Cq_a()->GetElementN(3), mbilateral->Get_Cq_a()->GetElementN(4), mbilateral->Get_Cq_a()->GetElementN(5)); //J1w
		D = R3(mbilateral->Get_Cq_b()->GetElementN(3), mbilateral->Get_Cq_b()->GetElementN(4), mbilateral->Get_Cq_b()->GetElementN(5)); //J2w

//		std::cout<<"A: "<<A.x<<" "<<A.y<<" "<<A.z<<std::endl;
//		std::cout<<"B: "<<B.x<<" "<<B.y<<" "<<B.z<<std::endl;
//		std::cout<<"C: "<<C.x<<" "<<C.y<<" "<<C.z<<std::endl;
//		std::cout<<"D: "<<D.x<<" "<<D.y<<" "<<D.z<<std::endl;

//		real3 vA = gpu_data_manager->host_vel_data[idA];
//		std::cout << vA.x << " " << vA.y << " " << vA.z << " ";
//		vA = gpu_data_manager->host_omg_data[idA];
//		std::cout << vA.x << " " << vA.y << " " << vA.z << " | ";
//		vA = gpu_data_manager->host_vel_data[idB];
//		std::cout << vA.x << " " << vA.y << " " << vA.z << " ";
//		vA = gpu_data_manager->host_omg_data[idB];
//		std::cout << vA.x << " " << vA.y << " " << vA.z << "\n";

		//cout<<idA<<" "<<idB<<" "<<mbilateral->Get_b_i()<<" "<<1.0 / mbilateral->Get_g_i()<<endl;

		gpu_data_manager->host_JXYZA_bilateral[cntr] = A;
		gpu_data_manager->host_JXYZB_bilateral[cntr] = B;
		gpu_data_manager->host_JUVWA_bilateral[cntr] = C;
		gpu_data_manager->host_JUVWB_bilateral[cntr] = D;
		gpu_data_manager->host_residual_bilateral[cntr] = mbilateral->Get_b_i(); // b_i is residual b
		gpu_data_manager->host_correction_bilateral[cntr] = 1.0 / mbilateral->Get_g_i(); // eta = 1/g
		gpu_data_manager->host_bids_bilateral[cntr] = I2(idA, idB);
		gpu_data_manager->host_gamma_bilateral[cntr] = mbilateral->Get_l_i();
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
