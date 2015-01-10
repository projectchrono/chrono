#include "chrono_parallel/physics/ChSystemParallel.h"
#include <omp.h>

using namespace chrono;
using namespace chrono::collision;

ChSystemParallelDEM::ChSystemParallelDEM(unsigned int                       max_objects,
                                         ChContactDEM::NormalForceModel     normal_model,
                                         ChContactDEM::TangentialForceModel tangential_model)
: ChSystemParallel(max_objects),
  normal_force_model(normal_model),
  tangential_force_model(tangential_model)
{
  LCP_descriptor = new ChLcpSystemDescriptorParallelDEM();
  LCP_solver_speed = new ChLcpSolverParallelDEM();
  ((ChLcpSystemDescriptorParallelDEM*) LCP_descriptor)->data_container = data_manager;
  ((ChLcpSolverParallel*) LCP_solver_speed)->data_container = data_manager;

  ((ChCollisionSystemParallel *) collision_system)->SetCollisionEnvelope(0);

  //Set this so that the CD can check what type of system it is (needed for narrowphase)
  data_manager->settings.system_type = SYSTEM_DEM;
  data_manager->system_timer.AddTimer("ChLcpSolverParallelDEM_ProcessContact");
}


void ChSystemParallelDEM::LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody)
{
  assert(typeid(*newbody.get_ptr()) == typeid(ChBodyDEM));

  ChSharedPtr<ChMaterialSurfaceDEM>& mat = ((ChBodyDEM*) newbody.get_ptr())->GetMaterialSurfaceDEM();

  data_manager->host_data.elastic_moduli.push_back(R2(mat->GetYoungModulus(), mat->GetPoissonRatio()));
  data_manager->host_data.mu.push_back(mat->GetSfriction());
  data_manager->host_data.cohesion_data.push_back(mat->GetCohesion());

  switch (normal_force_model) {
  case ChContactDEM::HuntCrossley:
    data_manager->host_data.alpha.push_back(mat->GetDissipationFactor());
    break;
  }

  //gpu_data_manager->host_data.cr.push_back(mat->GetRestitution());
}


void ChSystemParallelDEM::UpdateBodies()
{
  custom_vector<bool>& active_pointer = data_manager->host_data.active_data;
  custom_vector<bool>& collide_pointer = data_manager->host_data.collide_data;
  custom_vector<real>& inv_mass_pointer = data_manager->host_data.inv_mass_data;
  custom_vector<real>& cohesion_pointer = data_manager->host_data.cohesion_data;
  custom_vector<real3>& fric_pointer = data_manager->host_data.fric_data;
  custom_vector<real3>& pos_pointer = data_manager->host_data.pos_data;
  custom_vector<real4>& rot_pointer = data_manager->host_data.rot_data;
  custom_vector<M33>& inr_pointer = data_manager->host_data.inr_data;
  custom_vector<real4>& compliance_pointer = data_manager->host_data.compliance_data;

  real2* elastic_moduli = data_manager->host_data.elastic_moduli.data();
  real*  mu             = data_manager->host_data.mu.data();
  real*  alpha          = data_manager->host_data.alpha.data();
  real*  cr             = data_manager->host_data.cr.data();
  real*  cohesion       = data_manager->host_data.cohesion_data.data();

#pragma omp parallel for
  for (int i = 0; i < bodylist.size(); i++) {
    bodylist[i]->UpdateTime(ChTime);
    bodylist[i]->ClampSpeed();
    bodylist[i]->ComputeGyro();
    bodylist[i]->UpdateForces(ChTime);
    bodylist[i]->VariablesFbLoadForces(GetStep());
    bodylist[i]->VariablesQbLoadSpeed();
    bodylist[i]->UpdateMarkers(ChTime);

    ChMatrix<>&     qb = bodylist[i]->Variables().Get_qb();
    ChMatrix<>&     fb = bodylist[i]->Variables().Get_fb();
    ChVector<>&     pos = bodylist[i]->GetPos();
    ChQuaternion<>& rot = bodylist[i]->GetRot();
    ChMatrix33<>&   inertia = bodylist[i]->VariablesBody().GetBodyInvInertia();

    data_manager->host_data.v[i * 6 + 0] = bodylist[i]->Variables().Get_qb().GetElementN(0);
    data_manager->host_data.v[i * 6 + 1] = bodylist[i]->Variables().Get_qb().GetElementN(1);
    data_manager->host_data.v[i * 6 + 2] = bodylist[i]->Variables().Get_qb().GetElementN(2);
    data_manager->host_data.v[i * 6 + 3] = bodylist[i]->Variables().Get_qb().GetElementN(3);
    data_manager->host_data.v[i * 6 + 4] = bodylist[i]->Variables().Get_qb().GetElementN(4);
    data_manager->host_data.v[i * 6 + 5] = bodylist[i]->Variables().Get_qb().GetElementN(5);

    data_manager->host_data.hf[i * 6 + 0] = bodylist[i]->Variables().Get_fb().ElementN(0);
    data_manager->host_data.hf[i * 6 + 1] = bodylist[i]->Variables().Get_fb().ElementN(1);
    data_manager->host_data.hf[i * 6 + 2] = bodylist[i]->Variables().Get_fb().ElementN(2);
    data_manager->host_data.hf[i * 6 + 3] = bodylist[i]->Variables().Get_fb().ElementN(3);
    data_manager->host_data.hf[i * 6 + 4] = bodylist[i]->Variables().Get_fb().ElementN(4);
    data_manager->host_data.hf[i * 6 + 5] = bodylist[i]->Variables().Get_fb().ElementN(5);

    pos_pointer[i] = (R3(pos.x, pos.y, pos.z));
    rot_pointer[i] = (R4(rot.e0, rot.e1, rot.e2, rot.e3));
    inr_pointer[i] = (M33(R3(inertia.GetElement(0, 0), inertia.GetElement(1, 0), inertia.GetElement(2, 0)),
                          R3(inertia.GetElement(0, 1), inertia.GetElement(1, 1), inertia.GetElement(2, 1)),
                          R3(inertia.GetElement(0, 2), inertia.GetElement(1, 2), inertia.GetElement(2, 2))));

    active_pointer[i] = bodylist[i]->IsActive();
    inv_mass_pointer[i] = 1.0f / bodylist[i]->VariablesBody().GetBodyMass();

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

  if (ChCollisionSystemParallel* coll_sys = dynamic_cast<ChCollisionSystemParallel*>(collision_system))
    coll_sys->SetCollisionEnvelope(0);
}

void ChSystemParallelDEM::PrintStepStats()
{
  double timer_solver_setup = data_manager->system_timer.GetTime("ChLcpSolverParallel_Setup");
  double timer_solver_rhs   = data_manager->system_timer.GetTime("ChLcpSolverParallel_RHS");
  double timer_solver_stab  = data_manager->system_timer.GetTime("ChLcpSolverParallel_Stab");

  std::cout << std::endl;
  std::cout << "System Information" << std::endl;
  std::cout << "------------------" << std::endl;
  std::cout << "  Number of bodies     " << GetNumBodies() << std::endl;
  std::cout << "  Number of contacts   " << GetNcontacts() << std::endl;
  std::cout << "  Number of bilaterals " << GetNumBilaterals() << std::endl;
  std::cout << std::endl;
  std::cout << "Timing Information" << std::endl;
  std::cout << "------------------" << std::endl;
  std::cout << "Simulation time        " << GetTimerStep() << std::endl;
  std::cout << "  Collision detection    " << GetTimerCollision() << std::endl;
  std::cout << "    broad phase            " << GetTimerCollisionBroad() << std::endl;
  std::cout << "    narrow phase           " << GetTimerCollisionNarrow() << std::endl;
  std::cout << "  Update                 " << GetTimerUpdate() << std::endl;
  std::cout << "  Solver                 " << GetTimerLcp() << std::endl;
  std::cout << "    contact force calc     " << GetTimerProcessContact() << std::endl;
  std::cout << "    setup                  " << timer_solver_setup << std::endl;
  std::cout << "      RHS                    " << timer_solver_rhs << std::endl;
  std::cout << "    stabilization          " << timer_solver_stab << std::endl;
  std::cout << std::endl;
}
