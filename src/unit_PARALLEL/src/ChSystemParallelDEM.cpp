#include "ChSystemParallel.h"
#include <omp.h>


using namespace chrono;


ChSystemParallelDEM::ChSystemParallelDEM(unsigned int                       max_objects,
                                         ChContactDEM::NormalForceModel     normal_model,
                                         ChContactDEM::TangentialForceModel tangential_model)
:	ChSystemParallel(max_objects),
	normal_force_model(normal_model),
	tangential_force_model(tangential_model)
{
	LCP_descriptor = new ChLcpSystemDescriptorParallelDEM();
	LCP_solver_speed = new ChLcpSolverParallelDEM();
	((ChLcpSystemDescriptorParallelDEM*) (LCP_descriptor))->data_container = gpu_data_manager;
	((ChLcpSolverParallel*) LCP_solver_speed)->data_container = gpu_data_manager;

	((ChCollisionSystemParallel *) collision_system)->SetCollisionEnvelope(0);
}


void ChSystemParallelDEM::LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody)
{
	assert(typeid(*newbody.get_ptr()) == typeid(ChBodyDEM));

	ChSharedPtr<ChMaterialSurfaceDEM>& mat = ((ChBodyDEM*) newbody.get_ptr())->GetMaterialSurfaceDEM();

	gpu_data_manager->host_data.elastic_moduli.push_back(R2(mat->GetYoungModulus(), mat->GetPoissonRatio()));
	gpu_data_manager->host_data.mu.push_back(mat->GetSfriction());
	gpu_data_manager->host_data.cohesion_data.push_back(mat->GetCohesion());

	switch (normal_force_model) {
	case ChContactDEM::HuntCrossley:
		gpu_data_manager->host_data.alpha.push_back(mat->GetDissipationFactor());
		break;
	}

	//gpu_data_manager->host_data.cr.push_back(mat->GetRestitution());
}


void ChSystemParallelDEM::UpdateBodies() {
	real3 *vel_pointer = gpu_data_manager->host_data.vel_data.data();
	real3 *omg_pointer = gpu_data_manager->host_data.omg_data.data();
	real3 *pos_pointer = gpu_data_manager->host_data.pos_data.data();
	real4 *rot_pointer = gpu_data_manager->host_data.rot_data.data();
	real3 *inr_pointer = gpu_data_manager->host_data.inr_data.data();
	real3 *frc_pointer = gpu_data_manager->host_data.frc_data.data();
	real3 *trq_pointer = gpu_data_manager->host_data.trq_data.data();
	bool *active_pointer = gpu_data_manager->host_data.active_data.data();
	real *mass_pointer = gpu_data_manager->host_data.mass_data.data();
	real3 *lim_pointer = gpu_data_manager->host_data.lim_data.data();

	real2* elastic_moduli = gpu_data_manager->host_data.elastic_moduli.data();
	real*  mu             = gpu_data_manager->host_data.mu.data();
	real*  alpha          = gpu_data_manager->host_data.alpha.data();
	real*  cr             = gpu_data_manager->host_data.cr.data();
	real*  cohesion       = gpu_data_manager->host_data.cohesion_data.data();

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->UpdateTime(ChTime);
		//bodylist[i]->TrySleeping();     // See if the body can fall asleep; if so, put it to sleeping
		//bodylist[i]->ClampSpeed();      // Apply limits (if in speed clamping mode) to speeds.
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
		ChMatrix<>&     qb = bodylist[i]->Variables().Get_qb();
		ChMatrix<>&     fb = bodylist[i]->Variables().Get_fb();
		ChVector<>&     pos = bodylist[i]->GetPos();
		ChQuaternion<>& rot = bodylist[i]->GetRot();
		ChMatrix33<>&   inertia = bodylist[i]->VariablesBody().GetBodyInvInertia();

		vel_pointer[i] = (R3(qb.ElementN(0), qb.ElementN(1), qb.ElementN(2)));
		omg_pointer[i] = (R3(qb.ElementN(3), qb.ElementN(4), qb.ElementN(5)));
		pos_pointer[i] = (R3(pos.x, pos.y, pos.z));
		rot_pointer[i] = (R4(rot.e0, rot.e1, rot.e2, rot.e3));
		inr_pointer[i] = (R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
		frc_pointer[i] = (R3(fb.ElementN(0), fb.ElementN(1), fb.ElementN(2)));     //forces
		trq_pointer[i] = (R3(fb.ElementN(3), fb.ElementN(4), fb.ElementN(5)));     //torques
		active_pointer[i] = bodylist[i]->IsActive();
		mass_pointer[i] = 1.0f / bodylist[i]->VariablesBody().GetBodyMass();
		lim_pointer[i] = (R3(bodylist[i]->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));

		ChSharedPtr<ChMaterialSurfaceDEM>& mat = ((ChBodyDEM*) bodylist[i])->GetMaterialSurfaceDEM();

		elastic_moduli[i] = R2(mat->GetYoungModulus(), mat->GetPoissonRatio());
		mu[i]             = mat->GetSfriction();
		cohesion[i]       = mat->GetCohesion();
		//cr[i]             = mat->GetRestitution();

		switch (normal_force_model) {
		case ChContactDEM::HuntCrossley:
			alpha[i] = mat->GetDissipationFactor();
			break;
		}

		bodylist[i]->GetCollisionModel()->SyncPosition();
	}
}


void ChSystemParallelDEM::ChangeCollisionSystem(ChCollisionSystem* newcollsystem)
{
	ChSystemParallel::ChangeCollisionSystem(newcollsystem);
	((ChCollisionSystemParallel *) collision_system)->SetCollisionEnvelope(0);
}