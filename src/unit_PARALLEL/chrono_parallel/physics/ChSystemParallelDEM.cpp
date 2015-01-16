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
  LCP_solver_speed = new ChLcpSolverParallelDEM(data_manager);

  ((ChCollisionSystemParallel *) collision_system)->SetCollisionEnvelope(0);

  //Set this so that the CD can check what type of system it is (needed for narrowphase)
  data_manager->settings.system_type = SYSTEM_DEM;

  data_manager->system_timer.AddTimer("ChLcpSolverParallelDEM_ProcessContact");
}


void ChSystemParallelDEM::AddMaterialSurfaceData(ChSharedPtr<ChBody> newbody)
{
  assert(typeid(*newbody.get_ptr()) == typeid(ChBodyDEM));

  // Reserve space for material properties for the specified body. Not that the
  // actual data is set in UpdateMaterialProperties().
  data_manager->host_data.elastic_moduli.push_back(R2(0, 0));
  data_manager->host_data.mu.push_back(0);
  data_manager->host_data.cohesion_data.push_back(0);
  //data_manager->host_data.cr.push_back(0);

  switch (normal_force_model) {
  case ChContactDEM::HuntCrossley:
    data_manager->host_data.alpha.push_back(0);
    break;
  }
}


void ChSystemParallelDEM::UpdateMaterialSurfaceData(int index, ChBody* body)
{
  custom_vector<real2>& elastic_moduli = data_manager->host_data.elastic_moduli;
  custom_vector<real>& cohesion = data_manager->host_data.cohesion_data;
  custom_vector<real>& mu = data_manager->host_data.mu;
  custom_vector<real>& alpha = data_manager->host_data.alpha;
  //custom_vector<real>& cr = data_manager->host_data.cr;

  ChSharedPtr<ChMaterialSurfaceDEM>& mat = ((ChBodyDEM*)body)->GetMaterialSurfaceDEM();

  elastic_moduli[index] = R2(mat->GetYoungModulus(), mat->GetPoissonRatio());
  mu[index] = mat->GetSfriction();
  cohesion[index] = mat->GetCohesion();
  //cr[index] = mat->GetRestitution();

  switch (normal_force_model) {
  case ChContactDEM::HuntCrossley:
    alpha[index] = mat->GetDissipationFactor();
    break;
  }
}


void ChSystemParallelDEM::ChangeCollisionSystem(COLLISIONSYSTEMTYPE type)
{
  ChSystemParallel::ChangeCollisionSystem(type);

  if (ChCollisionSystemParallel* coll_sys = dynamic_cast<ChCollisionSystemParallel*>(collision_system))
    coll_sys->SetCollisionEnvelope(0);
}

void ChSystemParallelDEM::PrintStepStats()
{
  double timer_solver_setup = data_manager->system_timer.GetTime("ChLcpSolverParallel_Setup");
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
  std::cout << "    stabilization          " << timer_solver_stab << std::endl;
  std::cout << std::endl;
}
