#include "ChSystemParallel.h"
#include <omp.h>


using namespace chrono;


ChSystemParallelDVI::ChSystemParallelDVI(unsigned int max_objects)
:	ChSystemParallel(max_objects)
{
	LCP_descriptor = new ChLcpSystemDescriptorParallelDVI();
	LCP_solver_speed = new ChLcpSolverParallelDVI();
	((ChLcpSystemDescriptorParallelDVI*) LCP_descriptor)->data_container = gpu_data_manager;
	((ChLcpSolverParallel*) LCP_solver_speed)->data_container = gpu_data_manager;
}


void ChSystemParallelDVI::LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody)
{
	assert(typeid(*newbody.get_ptr()) == typeid(ChBody));

	ChSharedPtr<ChMaterialSurface>& mat = newbody->GetMaterialSurface();

	gpu_data_manager->host_data.fric_data.push_back(
		R3(mat->GetKfriction(), mat->GetRollingFriction(), mat->GetSpinningFriction()));
	gpu_data_manager->host_data.cohesion_data.push_back(mat->GetCohesion());
	gpu_data_manager->host_data.compliance_data.push_back(
		R4(mat->GetCompliance(), mat->GetComplianceT(), mat->GetComplianceRolling(), mat->GetComplianceSpinning()));
}


void ChSystemParallelDVI::UpdateBodies() {
	real3 *vel_pointer = gpu_data_manager->host_data.vel_data.data();
	real3 *omg_pointer = gpu_data_manager->host_data.omg_data.data();
	real3 *pos_pointer = gpu_data_manager->host_data.pos_data.data();
	real4 *rot_pointer = gpu_data_manager->host_data.rot_data.data();
	real3 *inr_pointer = gpu_data_manager->host_data.inr_data.data();
	real3 *frc_pointer = gpu_data_manager->host_data.frc_data.data();
	real3 *trq_pointer = gpu_data_manager->host_data.trq_data.data();
	bool *active_pointer = gpu_data_manager->host_data.active_data.data();
	real *mass_pointer = gpu_data_manager->host_data.mass_data.data();
	real3 *fric_pointer = gpu_data_manager->host_data.fric_data.data();
	real *cohesion_pointer = gpu_data_manager->host_data.cohesion_data.data();
	real4 *compliance_pointer = gpu_data_manager->host_data.compliance_data.data();
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
		ChMatrix33<> inertia = bodylist[i]->VariablesBody().GetBodyInvInertia();
		vel_pointer[i] = (R3(bodylist[i]->Variables().Get_qb().ElementN(0), bodylist[i]->Variables().Get_qb().ElementN(1), bodylist[i]->Variables().Get_qb().ElementN(2)));
		omg_pointer[i] = (R3(bodylist[i]->Variables().Get_qb().ElementN(3), bodylist[i]->Variables().Get_qb().ElementN(4), bodylist[i]->Variables().Get_qb().ElementN(5)));
		pos_pointer[i] = (R3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
		rot_pointer[i] = (R4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
		inr_pointer[i] = (R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
		frc_pointer[i] = (R3(bodylist[i]->Variables().Get_fb().ElementN(0), bodylist[i]->Variables().Get_fb().ElementN(1), bodylist[i]->Variables().Get_fb().ElementN(2)));     //forces
		trq_pointer[i] = (R3(bodylist[i]->Variables().Get_fb().ElementN(3), bodylist[i]->Variables().Get_fb().ElementN(4), bodylist[i]->Variables().Get_fb().ElementN(5)));     //torques
		active_pointer[i] = bodylist[i]->IsActive();
		mass_pointer[i] = 1.0f / bodylist[i]->VariablesBody().GetBodyMass();
		fric_pointer[i] = R3(bodylist[i]->GetKfriction(), ((bodylist[i]))->GetMaterialSurface()->GetRollingFriction(), ((bodylist[i]))->GetMaterialSurface()->GetSpinningFriction());
		cohesion_pointer[i] = bodylist[i]->GetMaterialSurface()->GetCohesion();
		compliance_pointer[i] = R4(bodylist[i]->GetMaterialSurface()->GetCompliance(), bodylist[i]->GetMaterialSurface()->GetComplianceT(), bodylist[i]->GetMaterialSurface()->GetComplianceRolling(),
				bodylist[i]->GetMaterialSurface()->GetComplianceSpinning());
		lim_pointer[i] = (R3(bodylist[i]->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));
		bodylist[i]->GetCollisionModel()->SyncPosition();
	}
}

