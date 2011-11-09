#include "ChSystemGPU.h"
#include <omp.h>


#define Bpointer		    (*ibody)
#define HIER_BODY_INIT      std::vector<ChBody*>::iterator ibody = bodylist.begin();
#define HIER_BODY_NOSTOP    (ibody != bodylist.end())
#define HIER_BODY_NEXT	    ibody++;

#define Lpointer		    (*iterlink)
#define HIER_LINK_INIT      std::list<ChLink*>::iterator iterlink = linklist.begin();
#define HIER_LINK_NOSTOP    (iterlink != linklist.end())
#define HIER_LINK_NEXT	    iterlink++;

#define PHpointer		    (*iterotherphysics)
#define HIER_OTHERPHYSICS_INIT    std::list<ChPhysicsItem*>::iterator iterotherphysics = otherphysicslist.begin();
#define HIER_OTHERPHYSICS_NOSTOP  (iterotherphysics != otherphysicslist.end())
#define HIER_OTHERPHYSICS_NEXT	  iterotherphysics++;

#define Ppointer		    (*iterprobe)
#define HIER_PROBE_INIT      std::vector<ChProbe*>::iterator iterprobe = probelist.begin();
#define HIER_PROBE_NOSTOP    (iterprobe != probelist.end())
#define HIER_PROBE_NEXT	    iterprobe++;

#define Cpointer		    (*itercontrol)
#define HIER_CONTROLS_INIT      std::vector<ChControls*>::iterator itercontrol = controlslist.begin();
#define HIER_CONTROLS_NOSTOP    (itercontrol != controlslist.end())
#define HIER_CONTROLS_NEXT	    itercontrol++;


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

		//((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->force_solver->gravity = F3(Get_G_acc().x, Get_G_acc().y, Get_G_acc().z);
		Setup();
		Update();

		// reset known-term vectors
		//LCPprepare_reset();
		//LCPprepare_load(true, true, GetStep(), 1.0, 1.0 / GetStep(), max_penetration_recovery_speed, true);

		// if warm start is used, can exploit cached multipliers from last step...
		//LCPprepare_Li_from_speed_cache();

		// make vectors of variables and constraints, used by the following LCP solver
		LCPprepare_inject(*this->LCP_descriptor);

		for (int i = 0; i < bodylist.size(); i++) {
			ChLcpVariablesBodyOwnMass* mbodyvar = &(bodylist[i]->Variables());
			float inv_mass = (1.0) / (mbodyvar->GetBodyMass());
			bodylist[i]->GetRot().Normalize();
			gpu_data_manager->host_vel_data[i] = (F3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1), mbodyvar->Get_qb().GetElementN(2)));
			gpu_data_manager->host_omg_data[i] = (F3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4), mbodyvar->Get_qb().GetElementN(5)));
			gpu_data_manager->host_pos_data[i] = (F3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
			gpu_data_manager->host_rot_data[i] = (F4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
			gpu_data_manager->host_inr_data[i] = (F3(mbodyvar->GetBodyInvInertia().GetElement(0, 0), mbodyvar->GetBodyInvInertia().GetElement(1, 1), mbodyvar->GetBodyInvInertia().GetElement(2, 2)));
			gpu_data_manager->host_frc_data[i] = (F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
			gpu_data_manager->host_trq_data[i] = (F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
			gpu_data_manager->host_aux_data[i] = (F3(bodylist[i]->IsActive(), bodylist[i]->GetKfriction(), inv_mass));
			gpu_data_manager->host_lim_data[i] = (F3(bodylist[i]->GetLimitSpeed(), bodylist[i]->GetMaxSpeed(), bodylist[i]->GetMaxWvel()));
		}
		gpu_data_manager->HostToDevice();

		mtimer_cd.start();
		((ChCollisionSystemGPU*) (collision_system))->Run();
		this->ncontacts = gpu_data_manager->number_of_contacts;
		mtimer_cd.stop();

		mtimer_lcp.start();
		(LCP_solver_speed)->Solve(*this->LCP_descriptor, true);
		((ChContactContainerGPUsimple*) this->contact_container)->SetNcontacts(gpu_data_manager->number_of_contacts);
		mtimer_lcp.stop();
		// Device to host

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

	double ChSystemGPU::ComputeCollisions() {
		double mretC = 0.0;
		return mretC;
	}

//	void ChSystemGPU::AddBody(ChSharedPtr<ChBodyGPU> newbody)

	void ChSystemGPU::RemoveBody(ChSharedPtr<ChBodyGPU> mbody) {
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

	void ChSystemGPU::Update() {
		ChTimer<double> mtimer;
		mtimer.start(); // Timer for profiling

		for (int i = 0; i < bodylist.size(); i++) // Updates recursively all other aux.vars
		{
			bodylist[i]->Update(ChTime);
			bodylist[i]->VariablesFbReset();
			bodylist[i]->VariablesFbLoadForces(GetStep());
			bodylist[i]->VariablesQbLoadSpeed();
		}
		std::list<ChLink*>::iterator it;

		for (it = linklist.begin(); it != linklist.end(); it++) {
			(*it)->Update(ChTime);
		}

		for (it = linklist.begin(); it != linklist.end(); it++) {
			(*it)->ConstraintsBiReset();
		}
		for (it = linklist.begin(); it != linklist.end(); it++) {
			(*it)->ConstraintsBiLoad_C(1 / GetStep(), max_penetration_recovery_speed, true);
			(*it)->ConstraintsBiLoad_Ct(1);
			(*it)->ConstraintsFbLoadForces(GetStep());
			(*it)->ConstraintsLoadJacobians();
		}

		mtimer.stop();
		timer_update += mtimer();
	}

	//void ChSystemGPU::LCPprepare_load(bool load_jacobians, bool load_v, double F_factor, double Ct_factor, double C_factor, double recovery_clamp, bool do_clamp) {

	//	if (C_factor) contact_container->ConstraintsBiLoad_C(C_factor, recovery_clamp, do_clamp);
	//	if (F_factor) contact_container->ConstraintsFbLoadForces(F_factor); // f*dt
	//	if (load_jacobians) contact_container->ConstraintsLoadJacobians();

	//}
	//void ChSystemGPU::LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor) {
	//	mdescriptor.BeginInsertion(); // This resets the vectors of constr. and var. pointers.
	//	this->contact_container->InjectConstraints(mdescriptor);
	//	mdescriptor.EndInsertion();
	//}

	//void ChSystemGPU::LCPprepare_reset() {
	//	this->contact_container->ConstraintsBiReset();
	//}
	void ChSystemGPU::ChangeLcpSolverSpeed(ChLcpSolver* newsolver) {
		assert(newsolver);
		if (this->LCP_solver_speed) delete (this->LCP_solver_speed);
		this->LCP_solver_speed = newsolver;

		((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->data_container = gpu_data_manager;
	}

	void ChSystemGPU::ChangeCollisionSystem(ChCollisionSystem* newcollsystem) {
		assert(this->GetNbodies() == 0);
		assert(newcollsystem);
		if (this->collision_system) delete (this->collision_system);
		this->collision_system = newcollsystem;

		((ChCollisionSystemGPU*) (collision_system))->mGPU->data_container = gpu_data_manager;
	}
}
