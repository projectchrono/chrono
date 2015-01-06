#include "chrono_parallel/physics/ChSystemParallel.h"
#include <omp.h>

using namespace chrono;
using namespace chrono::collision;

ChSystemParallel::ChSystemParallel(unsigned int max_objects)
      : ChSystem(1000, 10000, false) {

   data_manager = new ChParallelDataManager();

   contact_container = new ChContactContainerParallel();
   collision_system = new ChCollisionSystemParallel();
   ((ChCollisionSystemParallel *) collision_system)->data_container = data_manager;
   ((ChContactContainerParallel*) contact_container)->data_container = data_manager;

   counter = 0;
   timer_accumulator.resize(10, 0);
   cd_accumulator.resize(10, 0);
   frame_threads = 0;
   frame_bins = 0;
   old_timer = 0;
   old_timer_cd = 0;
   timer_collision = 0;
   detect_optimal_threads = false;
   detect_optimal_bins = false;
   current_threads = 2;

   data_manager->system_timer.AddTimer("step");
   data_manager->system_timer.AddTimer("update");
   data_manager->system_timer.AddTimer("collision");
   data_manager->system_timer.AddTimer("collision_broad");
   data_manager->system_timer.AddTimer("collision_narrow");
   data_manager->system_timer.AddTimer("lcp");

   data_manager->system_timer.AddTimer("ChLcpSolverParallel_Solve");
   data_manager->system_timer.AddTimer("ChLcpSolverParallel_Setup");
   data_manager->system_timer.AddTimer("ChLcpSolverParallel_Stab");
   data_manager->system_timer.AddTimer("ChLcpSolverParallel_Jacobians");
   data_manager->system_timer.AddTimer("ChLcpSolverParallel_RHS");

   data_manager->system_timer.AddTimer("ChConstraintBilateral_shurA_compute");
   data_manager->system_timer.AddTimer("ChConstraintBilateral_shurA_reduce");
   data_manager->system_timer.AddTimer("ChConstraintBilateral_shurB_compute");

   data_manager->system_timer.AddTimer("ChSolverParallel_shurA");
   data_manager->system_timer.AddTimer("ChSolverParallel_shurB");

}

ChSystemParallel::~ChSystemParallel() {
   delete data_manager;
}

int ChSystemParallel::Integrate_Y() {
   data_manager->system_timer.Reset();
   data_manager->system_timer.start("step");
   //=============================================================================================
   data_manager->system_timer.start("update");
   Setup();
   Update();
   data_manager->system_timer.stop("update");
   //=============================================================================================
   data_manager->system_timer.start("collision");
   collision_system->Run();
   collision_system->ReportContacts(this->contact_container);
   data_manager->system_timer.stop("collision");
   //=============================================================================================
   data_manager->system_timer.start("lcp");
   ((ChLcpSolverParallel *) (LCP_solver_speed))->RunTimeStep(GetStep());
   data_manager->system_timer.stop("lcp");
   //=============================================================================================
   data_manager->system_timer.start("update");

   uint cntr = 0;
   std::vector<ChLcpConstraint *> &mconstraints = (*this->LCP_descriptor).GetConstraintsList();
   for (uint ic = 0; ic < mconstraints.size(); ic++) {
      if (mconstraints[ic]->IsActive() == false) {
         continue;
      }
      ChLcpConstraintTwoBodies *mbilateral = (ChLcpConstraintTwoBodies *) (mconstraints[ic]);
      mconstraints[ic]->Set_l_i(data_manager->host_data.gamma_bilateral[cntr]);
      cntr++;
   }
   // updates the reactions of the constraint
   LCPresult_Li_into_reactions(1.0 / this->GetStep());     // R = l/dt  , approximately

#pragma omp parallel for
   for (int i = 0; i < bodylist.size(); i++) {
      if (data_manager->host_data.active_data[i] == true) {
         real3 vel = data_manager->host_data.vel_data[i];
         real3 omg = data_manager->host_data.omg_data[i];
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
         bodylist[i]->ClampSpeed();     // Apply limits (if in speed clamping mode) to speeds.
         bodylist[i]->ComputeGyro();     // Set the gyroscopic momentum.
         bodylist[i]->UpdateForces(ChTime);
         bodylist[i]->UpdateMarkers(ChTime);
      }
   }

////#pragma omp parallel for
   for (int i = 0; i < data_manager->num_shafts; i++) {
     if (!data_manager->host_data.shaft_active[i])
       continue;

     shaftlist[i]->Variables().Get_qb().SetElementN(0, data_manager->host_data.shaft_omg[i]);
     shaftlist[i]->VariablesQbIncrementPosition(GetStep());
     shaftlist[i]->VariablesQbSetSpeed(GetStep());
     shaftlist[i]->Update(ChTime);
   }

   data_manager->system_timer.stop("update");

   //=============================================================================================
   ChTime += GetStep();
   data_manager->system_timer.stop("step");

   if (ChCollisionSystemParallel* coll_sys = dynamic_cast<ChCollisionSystemParallel*>(collision_system)) {
      timer_collision_broad = data_manager->system_timer.GetTime("collision_broad");
      timer_collision_narrow = data_manager->system_timer.GetTime("collision_narrow");
   } else {
      timer_collision_broad = 0;
      timer_collision_narrow = 0;
   }

   timer_update = data_manager->system_timer.GetTime("update");
   timer_collision = data_manager->system_timer.GetTime("collision");
   timer_lcp = data_manager->system_timer.GetTime("lcp");
   timer_step = data_manager->system_timer.GetTime("step");

   if (data_manager->settings.perform_thread_tuning) {
      RecomputeThreads();
   }
   if (data_manager->settings.perform_bin_tuning) {
      RecomputeBins();
   }

   return 1;
}

void ChSystemParallel::AddBody(ChSharedPtr<ChBody> newbody) {

   newbody->AddRef();
   newbody->SetSystem(this);
   newbody->SetId(counter);
   bodylist.push_back(newbody.get_ptr());

   if (newbody->GetCollide()) {
      newbody->AddCollisionModelsToSystem();
   }

   ChLcpVariablesBodyOwnMass& mbodyvar = newbody->VariablesBody();
   real inv_mass = (1.0) / (mbodyvar.GetBodyMass());
   //newbody->GetRot().Normalize();
   ChMatrix33<>& inertia = mbodyvar.GetBodyInvInertia();

   data_manager->host_data.vel_data.push_back(
   R3(mbodyvar.Get_qb().GetElementN(0), mbodyvar.Get_qb().GetElementN(1), mbodyvar.Get_qb().GetElementN(2)));
   data_manager->host_data.omg_data.push_back(
   R3(mbodyvar.Get_qb().GetElementN(3), mbodyvar.Get_qb().GetElementN(4), mbodyvar.Get_qb().GetElementN(5)));
   data_manager->host_data.pos_data.push_back(
   R3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
   data_manager->host_data.rot_data.push_back(
   R4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
   data_manager->host_data.inr_data.push_back(
   R3(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2)));
   data_manager->host_data.frc_data.push_back(
   R3(mbodyvar.Get_fb().ElementN(0), mbodyvar.Get_fb().ElementN(1), mbodyvar.Get_fb().ElementN(2)));     //forces
   data_manager->host_data.trq_data.push_back(
   R3(mbodyvar.Get_fb().ElementN(3), mbodyvar.Get_fb().ElementN(4), mbodyvar.Get_fb().ElementN(5)));     //torques
   data_manager->host_data.active_data.push_back(newbody->IsActive());
   data_manager->host_data.collide_data.push_back(newbody->GetCollide());
   data_manager->host_data.mass_data.push_back(inv_mass);

   data_manager->host_data.lim_data.push_back(
   R3(newbody->GetLimitSpeed(), .05 / GetStep(), .05 / GetStep()));

   // Let derived classes load specific material surface data
   LoadMaterialSurfaceData(newbody);

   counter++;
   data_manager->num_bodies = counter;
}

void ChSystemParallel::AddOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> newitem) {
   //assert(std::find<std::vector<ChPhysicsItem*>::iterator>(otherphysicslist.begin(), otherphysicslist.end(), newitem.get_ptr()) == otherphysicslist.end());
   //assert(newitem->GetSystem()==0); // should remove from other system before adding here

   newitem->AddRef();
   newitem->SetSystem(this);
   otherphysicslist.push_back((newitem).get_ptr());

   // add to collision system too
   if (newitem->GetCollide()) {
      newitem->AddCollisionModelsToSystem();
   }

   //// Ideally, the function AddShaft() would be an override of a ChSystem
   //// virtual function and the vector shaftlist would be maintained by the base
   //// class ChSystem.  For now, users must use AddOtherPhysicsItem in order to
   //// properly account for the variables of a shaft elelement in ChSystem::Setup().
   //// Note that this also means we duplicate pointers in two lists: shaftlist
   //// and otherphysicslist...
   if (ChSharedPtr<ChShaft> shaft = newitem.DynamicCastTo<ChShaft>()) {
     AddShaft(shaft);
   }
}

//// Currently, this function is private to prevent the user from directly calling
//// it and instead force them to use AddOtherPhysicsItem().  See comment above.
//// Eventually, this should be an override of a virtual function declared by ChSystem.
void ChSystemParallel::AddShaft(ChSharedPtr<ChShaft> shaft)
{
  shaft->AddRef();
  shaft->SetId(data_manager->num_shafts);
  shaft->SetSystem(this);
  shaftlist.push_back(shaft.get_ptr());

  data_manager->host_data.shaft_rot.push_back(0);
  data_manager->host_data.shaft_omg.push_back(0);
  data_manager->host_data.shaft_trq.push_back(0);
  data_manager->host_data.shaft_inr.push_back(1);
  data_manager->host_data.shaft_active.push_back(true);

  data_manager->num_shafts++;
}

void ChSystemParallel::Update() {
#pragma omp parallel for
   for (int i = 0; i < bodylist.size(); i++) {
     bodylist[i]->VariablesFbReset();
   }
////#pragma omp parallel for
   for (int i = 0; i < data_manager->num_shafts; i++) {
     shaftlist[i]->VariablesFbReset();
   }
   this->LCP_descriptor->BeginInsertion();
   UpdateBilaterals();
   UpdateBodies();
   UpdateShafts();
   LCP_descriptor->EndInsertion();
}

void ChSystemParallel::UpdateShafts()
{
  real* shaft_rot = data_manager->host_data.shaft_rot.data();
  real* shaft_omg = data_manager->host_data.shaft_omg.data();
  real* shaft_trq = data_manager->host_data.shaft_trq.data();
  real* shaft_inr = data_manager->host_data.shaft_inr.data();
  bool* shaft_active = data_manager->host_data.shaft_active.data();

////#pragma omp parallel for
  for (int i = 0; i < data_manager->num_shafts; i++) {
    shaftlist[i]->Update(ChTime);
    shaftlist[i]->VariablesFbLoadForces(GetStep());
    shaftlist[i]->VariablesQbLoadSpeed();

    shaft_rot[i] = shaftlist[i]->GetPos();
    shaft_inr[i] = shaftlist[i]->Variables().GetInvMass().GetElementN(0);
    shaft_omg[i] = shaftlist[i]->Variables().Get_qb().GetElementN(0);
    shaft_trq[i] = shaftlist[i]->Variables().Get_fb().GetElementN(0);
    shaft_active[i] = shaftlist[i]->IsActive();
  }
}

void ChSystemParallel::UpdateBilaterals() {
   for (it = linklist.begin(); it != linklist.end(); it++) {
      (*it)->Update(ChTime);
      (*it)->ConstraintsBiReset();
      (*it)->ConstraintsBiLoad_C(1.0 / GetStep(),
                                 data_manager->settings.solver.bilateral_clamp_speed,
                                 data_manager->settings.solver.clamp_bilaterals);
      (*it)->ConstraintsBiLoad_Ct(1);
      (*it)->ConstraintsFbLoadForces(GetStep());
      (*it)->ConstraintsLoadJacobians();
      (*it)->ConstraintsLiLoadSuggestedSpeedSolution();
      (*it)->InjectConstraints(*this->LCP_descriptor);
   }
   unsigned int num_bilaterals = 0;
   std::vector<ChLcpConstraint *> &mconstraints = (*this->LCP_descriptor).GetConstraintsList();
   std::vector<int> mapping;

   for (uint ic = 0; ic < mconstraints.size(); ic++) {
      if (mconstraints[ic]->IsActive() == true) {
         num_bilaterals++;
         mapping.push_back(ic);
      }
   }
   data_manager->num_bilaterals = num_bilaterals;

   data_manager->host_data.JXYZA_bilateral.resize(num_bilaterals);
   data_manager->host_data.JXYZB_bilateral.resize(num_bilaterals);
   data_manager->host_data.JUVWA_bilateral.resize(num_bilaterals);
   data_manager->host_data.JUVWB_bilateral.resize(num_bilaterals);
   data_manager->host_data.residual_bilateral.resize(num_bilaterals);
   data_manager->host_data.correction_bilateral.resize(num_bilaterals);
   data_manager->host_data.bids_bilateral.resize(num_bilaterals);
   data_manager->host_data.gamma_bilateral.resize(num_bilaterals);
#pragma omp parallel for
   for (int i = 0; i < num_bilaterals; i++) {
      int cntr = mapping[i];
      ChLcpConstraintTwoBodies *mbilateral = (ChLcpConstraintTwoBodies *) (mconstraints[cntr]);
      int idA = ((ChBody *) ((ChLcpVariablesBody *) (mbilateral->GetVariables_a()))->GetUserData())->GetId();
      int idB = ((ChBody *) ((ChLcpVariablesBody *) (mbilateral->GetVariables_b()))->GetUserData())->GetId();
      // Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
      mconstraints[cntr]->Update_auxiliary();     //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
      real3 A, B, C, D;
      A = R3(mbilateral->Get_Cq_a()->GetElementN(0), mbilateral->Get_Cq_a()->GetElementN(1), mbilateral->Get_Cq_a()->GetElementN(2));     //J1x
      B = R3(mbilateral->Get_Cq_b()->GetElementN(0), mbilateral->Get_Cq_b()->GetElementN(1), mbilateral->Get_Cq_b()->GetElementN(2));     //J2x
      C = R3(mbilateral->Get_Cq_a()->GetElementN(3), mbilateral->Get_Cq_a()->GetElementN(4), mbilateral->Get_Cq_a()->GetElementN(5));     //J1w
      D = R3(mbilateral->Get_Cq_b()->GetElementN(3), mbilateral->Get_Cq_b()->GetElementN(4), mbilateral->Get_Cq_b()->GetElementN(5));     //J2w

      data_manager->host_data.JXYZA_bilateral[i] = A;
      data_manager->host_data.JXYZB_bilateral[i] = B;
      data_manager->host_data.JUVWA_bilateral[i] = C;
      data_manager->host_data.JUVWB_bilateral[i] = D;
      data_manager->host_data.residual_bilateral[i] = mbilateral->Get_b_i();     // b_i is residual b
      data_manager->host_data.correction_bilateral[i] = 1.0 / mbilateral->Get_g_i();     // eta = 1/g
      data_manager->host_data.bids_bilateral[i] = I2(idA, idB);
      //data_manager->host_data.gamma_bilateral[i] = -mbilateral->Get_l_i();
   }
}

void ChSystemParallel::RecomputeThreads() {
   timer_accumulator.insert(timer_accumulator.begin(), timer_step);
   timer_accumulator.pop_back();

   double sum_of_elems = std::accumulate(timer_accumulator.begin(), timer_accumulator.end(), 0.0);

   if (frame_threads == 50 && detect_optimal_threads == false) {
      frame_threads = 0;
      if (current_threads + 2 < data_manager->settings.max_threads) {
         detect_optimal_threads = true;
         old_timer = sum_of_elems / 10.0;
         current_threads += 2;
         omp_set_num_threads(current_threads);
#if PRINT_LEVEL==1
         cout << "current threads increased to " << current_threads << endl;
#endif
      } else {
         current_threads = data_manager->settings.max_threads;
         omp_set_num_threads(data_manager->settings.max_threads);
#if PRINT_LEVEL==1
         cout << "current threads increased to " << current_threads << endl;
#endif
      }
   } else if (frame_threads == 10 && detect_optimal_threads) {
      double current_timer = sum_of_elems / 10.0;
      detect_optimal_threads = false;
      frame_threads = 0;
      if (old_timer < current_timer) {
         current_threads -= 2;
         omp_set_num_threads(current_threads);
#if PRINT_LEVEL==1
         cout << "current threads reduced back to " << current_threads << endl;
#endif
      }
   }

   if (current_threads < data_manager->settings.min_threads) {
      current_threads = data_manager->settings.min_threads;
      omp_set_num_threads(data_manager->settings.min_threads);
   }
   frame_threads++;
}

void ChSystemParallel::PerturbBins(bool increase,
                                   int number) {

   ChCollisionSystemParallel* coll_sys = (ChCollisionSystemParallel *) collision_system;

   int3 grid_size = coll_sys->broadphase->getBinsPerAxis();
#if PRINT_LEVEL==1
   cout << "initial: " << grid_size.x << " " << grid_size.y << " " << grid_size.z << endl;
#endif

   if (increase) {
      grid_size.x = grid_size.x + number;
      grid_size.y = grid_size.y + number;
      grid_size.z = grid_size.z + number;
   } else {
      grid_size.x = grid_size.x - number;
      grid_size.y = grid_size.y - number;
      grid_size.z = grid_size.z - number;

      if (grid_size.x < 2)
         grid_size.x = 2;
      if (grid_size.y < 2)
         grid_size.y = 2;
      if (grid_size.z < 2)
         grid_size.z = 2;
   }

#if PRINT_LEVEL==1
   cout << "final: " << grid_size.x << " " << grid_size.y << " " << grid_size.z << endl;
#endif
   coll_sys->broadphase->setBinsPerAxis(grid_size);

   frame_bins++;
}

void ChSystemParallel::RecomputeBins() {
   // Do nothing if the current collision system does not support this feature
   if (!dynamic_cast<ChCollisionSystemParallel*>(collision_system))
      return;
   //Insert the current collision time into a list
   cd_accumulator.insert(cd_accumulator.begin(), timer_collision);
   //remove the last one from the list
   cd_accumulator.pop_back();
   //find the sum of all elements
   double sum_of_elems_cd = std::accumulate(cd_accumulator.begin(), cd_accumulator.end(), 0.0);
   //and then get the average time taken;
   double average_time = sum_of_elems_cd / 10.0;

   //if 0 increase and then measure
   //if 1 decrease and then measure
   if (frame_bins == data_manager->settings.bin_tuning_frequency && detect_optimal_bins == 0) {
      frame_bins = 0;
      detect_optimal_bins = 1;
      old_timer_cd = average_time;

      PerturbBins(true, data_manager->settings.bin_perturb_size);
#if PRINT_LEVEL==1
      cout << "current bins increased" << endl;
#endif
   } else if (frame_bins == 10 && detect_optimal_bins == 1) {
      double current_timer = average_time;
      if (old_timer_cd < current_timer) {
         PerturbBins(false, data_manager->settings.bin_perturb_size);
#if PRINT_LEVEL==1
         cout << "current bins reduced back" << endl;
#endif
      }
      detect_optimal_bins = 2;
      frame_bins = 0;
   }
   if (frame_bins == data_manager->settings.bin_tuning_frequency && detect_optimal_bins == 2) {
      frame_bins = 0;
      detect_optimal_bins = 3;
      old_timer_cd = average_time;
      PerturbBins(false, data_manager->settings.bin_perturb_size);
#if PRINT_LEVEL==1
      cout << "current bins decreased" << endl;
#endif
   } else if (frame_bins == 10 && detect_optimal_bins == 3) {
      double current_timer = average_time;
      if (old_timer_cd < current_timer) {
         PerturbBins(true, data_manager->settings.bin_perturb_size);
#if PRINT_LEVEL==1
         cout << "current bins increased back" << endl;
#endif
      }
      detect_optimal_bins = 0;
      frame_bins = 0;
   }

}

void ChSystemParallel::ChangeCollisionSystem(ChCollisionSystem *newcollsystem) {
   assert(this->GetNbodies() == 0);
   assert(newcollsystem);

   if (this->collision_system) {
      delete (this->collision_system);
   }
   this->collision_system = newcollsystem;

   if (ChCollisionSystemParallel* coll_sys = dynamic_cast<ChCollisionSystemParallel*>(collision_system)) {
      coll_sys->data_container = data_manager;
   } else if (ChCollisionSystemBulletParallel* coll_sys = dynamic_cast<ChCollisionSystemBulletParallel*>(collision_system)) {
      coll_sys->data_container = data_manager;
   }

}
