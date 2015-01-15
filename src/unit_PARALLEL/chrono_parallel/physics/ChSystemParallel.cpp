#include "physics/ChShaftsCouple.h"
#include "physics/ChShaftsGearbox.h"
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include <omp.h>

using namespace chrono;
using namespace chrono::collision;

ChSystemParallel::ChSystemParallel(unsigned int max_objects)
      : ChSystem(1000, 10000, false)
{
   data_manager = new ChParallelDataManager();

   LCP_descriptor = new ChLcpSystemDescriptorParallel(data_manager);
   contact_container = new ChContactContainerParallel(data_manager);
   collision_system = new ChCollisionSystemParallel();
   ((ChCollisionSystemParallel *) collision_system)->data_container = data_manager;

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

}

ChSystemParallel::~ChSystemParallel() {
   delete data_manager;
}

int ChSystemParallel::Integrate_Y() {
  //Get the pointer for the system descriptor and store it into the data manager
   data_manager->lcp_system_descriptor = this->LCP_descriptor;

   data_manager->system_timer.Reset();
   data_manager->system_timer.start("step");

   data_manager->system_timer.start("update");
   Setup();
   Update();
   data_manager->system_timer.stop("update");

   data_manager->system_timer.start("collision");
   collision_system->Run();
   collision_system->ReportContacts(this->contact_container);
   data_manager->system_timer.stop("collision");

   data_manager->system_timer.start("lcp");
   ((ChLcpSolverParallel *) (LCP_solver_speed))->RunTimeStep(GetStep());
   data_manager->system_timer.stop("lcp");

   data_manager->system_timer.start("update");

   std::vector<ChLcpConstraint*>& mconstraints = LCP_descriptor->GetConstraintsList();
   //Iterate over the active bilateral constraints
   for (int index = 0; index < data_manager->num_bilaterals; index++) {
       int cntr = data_manager->host_data.bilateral_mapping[index];
       mconstraints[cntr]->Set_l_i(data_manager->host_data.gamma_bilateral[index]);
   }
   // updates the reactions of the constraint
   LCPresult_Li_into_reactions(1.0 / this->GetStep());     // R = l/dt  , approximately

   // Scatter the states to the Chrono objects (bodies and shafts) and update
   // all physics items at the end of the step.
   DynamicVector<real>& velocities = data_manager->host_data.v;
   custom_vector<real3>& pos_pointer = data_manager->host_data.pos_data;
   custom_vector<real4>& rot_pointer = data_manager->host_data.rot_data;

#pragma omp parallel for
   for (int i = 0; i < bodylist.size(); i++) {
      if (data_manager->host_data.active_data[i] == true) {
         bodylist[i]->Variables().Get_qb().SetElement(0, 0, velocities[i * 6 + 0]);
         bodylist[i]->Variables().Get_qb().SetElement(1, 0, velocities[i * 6 + 1]);
         bodylist[i]->Variables().Get_qb().SetElement(2, 0, velocities[i * 6 + 2]);
         bodylist[i]->Variables().Get_qb().SetElement(3, 0, velocities[i * 6 + 3]);
         bodylist[i]->Variables().Get_qb().SetElement(4, 0, velocities[i * 6 + 4]);
         bodylist[i]->Variables().Get_qb().SetElement(5, 0, velocities[i * 6 + 5]);

         bodylist[i]->VariablesQbIncrementPosition(this->GetStep());
         bodylist[i]->VariablesQbSetSpeed(this->GetStep());
         bodylist[i]->UpdateTime(ChTime);
         //TrySleeping();			     // See if the body can fall asleep; if so, put it to sleeping
         bodylist[i]->ClampSpeed();      // Apply limits (if in speed clamping mode) to speeds.
         bodylist[i]->ComputeGyro();     // Set the gyroscopic momentum.
         bodylist[i]->UpdateForces(ChTime);
         bodylist[i]->UpdateMarkers(ChTime);

         //update the position and rotation vectors
         pos_pointer[i] = (R3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
         rot_pointer[i] = (R4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
      }
   }

////#pragma omp parallel for
   for (int i = 0; i < data_manager->num_shafts; i++) {
     if (!data_manager->host_data.shaft_active[i])
       continue;

     shaftlist[i]->Variables().Get_qb().SetElementN(0, velocities[data_manager->num_bodies * 6 + i]);
     shaftlist[i]->VariablesQbIncrementPosition(GetStep());
     shaftlist[i]->VariablesQbSetSpeed(GetStep());
     shaftlist[i]->Update(ChTime);
   }

   for (int i = 0; i < otherphysicslist.size(); i++) {
     otherphysicslist[i]->Update(ChTime);
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

//
// Add the specified body to the system.
// A unique identifier is assigned to each body for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the
// body.
//
void ChSystemParallel::AddBody(ChSharedPtr<ChBody> newbody)
{

   newbody->AddRef();
   newbody->SetSystem(this);
   // This is only need because bilaterals need to know what bodies to
   // refer to. Not used by contacts
   newbody->SetId(data_manager->num_bodies);

   bodylist.push_back(newbody.get_ptr());
   data_manager->num_bodies++;

   if (newbody->GetCollide()) {
      newbody->AddCollisionModelsToSystem();
   }

   // Reserve space for this body in the system-wide vectors. Note that the
   // actual data is set in UpdateBodies().
   data_manager->host_data.pos_data.push_back(R3());
   data_manager->host_data.rot_data.push_back(R4());
   data_manager->host_data.inv_mass_data.push_back(0);
   data_manager->host_data.inr_data.push_back(M33());
   data_manager->host_data.active_data.push_back(true);
   data_manager->host_data.collide_data.push_back(true);

   // Let derived classes reserve space for specific material surface data
   AddMaterialSurfaceData(newbody);
}

//
// Add physics items, other than bodies or links, to the system.
// We keep track separately of ChShaft elements which are maintained in their
// own list (shaftlist).  All other items are stored in otherphysicslist.
//
// Note that no test is performed to check if the item was already added.
//
// Ideally, the function AddShaft() would be an override of a ChSystem
// virtual function and the vector shaftlist would be maintained by the base
// class ChSystem.  For now, users must use AddOtherPhysicsItem in order to
// properly account for the variables of a shaft elelement in ChSystem::Setup().
//
void ChSystemParallel::AddOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> newitem)
{
   if (ChSharedPtr<ChShaft> shaft = newitem.DynamicCastTo<ChShaft>()) {
     AddShaft(shaft);
   } else {
     newitem->AddRef();
     newitem->SetSystem(this);
     otherphysicslist.push_back((newitem).get_ptr());

     if (newitem->GetCollide()) {
       newitem->AddCollisionModelsToSystem();
     }
   }
}

//
// Add the specified shaft to the system.
// A unique identifier is assigned to each shaft for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the
// shaft.
//
// Currently, this function is private to prevent the user from directly calling
// it and instead force them to use AddOtherPhysicsItem().  See comment above.
// Eventually, this should be an override of a virtual function declared by ChSystem.
//
void ChSystemParallel::AddShaft(ChSharedPtr<ChShaft> shaft)
{
  shaft->AddRef();
  shaft->SetId(data_manager->num_shafts);
  shaft->SetSystem(this);

  shaftlist.push_back(shaft.get_ptr());
  data_manager->num_shafts++;

  // Reserve space for this shaft in the system-wide vectors. Not that the
  // actual data is set in UpdateShafts().
  data_manager->host_data.shaft_rot.push_back(0);
  data_manager->host_data.shaft_inr.push_back(0);
  data_manager->host_data.shaft_active.push_back(true);
}

//
// Reset forces for all lcp variables
//
void ChSystemParallel::ClearForceVariables()
{
  ////#pragma omp parallel for
  for (int i = 0; i < data_manager->num_bodies; i++) {
    bodylist[i]->VariablesFbReset();
  }

  ////#pragma omp parallel for
  for (int i = 0; i < data_manager->num_shafts; i++) {
    shaftlist[i]->VariablesFbReset();
  }
}

//
// Update all items in the system. The following order of operations is important:
// 1. Clear the force vectors by calling VariablesFbReset for all objects
// 2. Compute link constraint forces
// 3. Update other physics items (other than shafts)
// 4. Update bodies (these introduce state variables)
// 5. Update shafts (these introduce state variables)
// 6. Process bilateral constraints
//
void ChSystemParallel::Update()
{
  // Clear the forces for all lcp variables
  ClearForceVariables();

  // Allocate space for the velocities and forces for all objects
  data_manager->host_data.v.resize(data_manager->num_bodies * 6 + data_manager->num_shafts * 1);
  data_manager->host_data.hf.resize(data_manager->num_bodies * 6 + data_manager->num_shafts * 1);

  // Clear system-wide vectors for bilateral constraints
  data_manager->host_data.bilateral_mapping.clear();
  data_manager->host_data.bilateral_type.clear();

  this->LCP_descriptor->BeginInsertion();
  UpdateLinks();
  UpdateOtherPhysics();
  UpdateBodies();
  UpdateShafts();
  LCP_descriptor->EndInsertion();

  UpdateBilaterals();
}

//
// Update all bodies in the system and populate system-wide state and force
// vectors.
//
void ChSystemParallel::UpdateBodies()
{
  custom_vector<real3>& position = data_manager->host_data.pos_data;
  custom_vector<real4>& rotation = data_manager->host_data.rot_data;
  custom_vector<real>& inv_mass = data_manager->host_data.inv_mass_data;
  custom_vector<M33>& inv_inertia = data_manager->host_data.inr_data;
  custom_vector<bool>& active = data_manager->host_data.active_data;
  custom_vector<bool>& collide = data_manager->host_data.collide_data;

#pragma omp parallel for
  for (int i = 0; i < bodylist.size(); i++) {
    bodylist[i]->UpdateTime(ChTime);
    //bodylist[i]->TrySleeping();
    bodylist[i]->ClampSpeed();
    bodylist[i]->ComputeGyro();
    bodylist[i]->UpdateForces(ChTime);
    bodylist[i]->VariablesFbLoadForces(GetStep());
    bodylist[i]->VariablesQbLoadSpeed();
    bodylist[i]->UpdateMarkers(ChTime);

    ChMatrix<>&     body_qb = bodylist[i]->Variables().Get_qb();
    ChMatrix<>&     body_fb = bodylist[i]->Variables().Get_fb();
    ChVector<>&     body_pos = bodylist[i]->GetPos();
    ChQuaternion<>& body_rot = bodylist[i]->GetRot();
    ChMatrix33<>&   body_inr = bodylist[i]->VariablesBody().GetBodyInvInertia();

    data_manager->host_data.v[i * 6 + 0] = body_qb.GetElementN(0);
    data_manager->host_data.v[i * 6 + 1] = body_qb.GetElementN(1);
    data_manager->host_data.v[i * 6 + 2] = body_qb.GetElementN(2);
    data_manager->host_data.v[i * 6 + 3] = body_qb.GetElementN(3);
    data_manager->host_data.v[i * 6 + 4] = body_qb.GetElementN(4);
    data_manager->host_data.v[i * 6 + 5] = body_qb.GetElementN(5);

    data_manager->host_data.hf[i * 6 + 0] = body_fb.ElementN(0);
    data_manager->host_data.hf[i * 6 + 1] = body_fb.ElementN(1);
    data_manager->host_data.hf[i * 6 + 2] = body_fb.ElementN(2);
    data_manager->host_data.hf[i * 6 + 3] = body_fb.ElementN(3);
    data_manager->host_data.hf[i * 6 + 4] = body_fb.ElementN(4);
    data_manager->host_data.hf[i * 6 + 5] = body_fb.ElementN(5);

    position[i] = R3(body_pos.x, body_pos.y, body_pos.z);
    rotation[i] = R4(body_rot.e0, body_rot.e1, body_rot.e2, body_rot.e3);

    inv_mass[i] = 1.0f / bodylist[i]->VariablesBody().GetBodyMass();
    inv_inertia[i] = M33(R3(body_inr.GetElement(0, 0), body_inr.GetElement(1, 0), body_inr.GetElement(2, 0)),
                         R3(body_inr.GetElement(0, 1), body_inr.GetElement(1, 1), body_inr.GetElement(2, 1)),
                         R3(body_inr.GetElement(0, 2), body_inr.GetElement(1, 2), body_inr.GetElement(2, 2)));

    active[i] = bodylist[i]->IsActive();
    collide[i] = bodylist[i]->GetCollide();

    // Let derived classes set the specific material surface data.
    UpdateMaterialSurfaceData(i, bodylist[i]);

    bodylist[i]->GetCollisionModel()->SyncPosition();
  }
}

//
// Update all shaft elements in the system and populate system-wide state and
// force vectors.
//
void ChSystemParallel::UpdateShafts()
{
  real* shaft_rot = data_manager->host_data.shaft_rot.data();
  real* shaft_inr = data_manager->host_data.shaft_inr.data();
  bool* shaft_active = data_manager->host_data.shaft_active.data();

////#pragma omp parallel for
  for (int i = 0; i < data_manager->num_shafts; i++) {
    shaftlist[i]->Update(ChTime);
    shaftlist[i]->VariablesFbLoadForces(GetStep());
    shaftlist[i]->VariablesQbLoadSpeed();

    shaft_rot[i] = shaftlist[i]->GetPos();
    shaft_inr[i] = shaftlist[i]->Variables().GetInvInertia();
    shaft_active[i] = shaftlist[i]->IsActive();

    data_manager->host_data.v[data_manager->num_bodies * 6 + i] = shaftlist[i]->Variables().Get_qb().GetElementN(0);
    data_manager->host_data.hf[data_manager->num_bodies * 6 + i] = shaftlist[i]->Variables().Get_fb().GetElementN(0);
  }
}

//
// Update all links in the system and set the type of the associated constraints
// to BODY_BODY.
//
void ChSystemParallel::UpdateLinks()
{
  double oostep = 1 / GetStep();
  real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
  bool clamp = data_manager->settings.solver.clamp_bilaterals;

  for (int i = 0; i < linklist.size(); i++) {
    linklist[i]->Update(ChTime);
    linklist[i]->ConstraintsBiReset();
    linklist[i]->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
    linklist[i]->ConstraintsBiLoad_Ct(1);
    linklist[i]->ConstraintsFbLoadForces(GetStep());
    linklist[i]->ConstraintsLoadJacobians();

    linklist[i]->InjectConstraints(*LCP_descriptor);

    for (int j = 0; j < linklist[i]->GetDOC_c(); j++)
      data_manager->host_data.bilateral_type.push_back(BODY_BODY);
  }
}

//
// This utility function returns the type of constraints associated with the
// specified physics item. Return UNKNOWN if the item has no associated
// bilateral constraints or if it is unsupported.
//
BILATERALTYPE GetBilateralType(ChPhysicsItem* item)
{
  if (item->GetDOC_c() == 0)
    return UNKNOWN;

  if (dynamic_cast<ChShaftsCouple*>(item))
    return SHAFT_SHAFT;

  if (dynamic_cast<ChShaftsPlanetary*>(item) ||
      dynamic_cast<ChShaftsGearbox*>(item)   ||
      dynamic_cast<ChShaftsGearboxAngled*>(item))
    return SHAFT_SHAFT_SHAFT;

  if (dynamic_cast<ChShaftsBody*>(item))
    return SHAFT_BODY;

  // Debug check - do we ignore any constraints?
  assert(item->GetDOC_c() == 0);

  return UNKNOWN;
}

//
// Update other physics items in the system and set the type of the associated
// constraints.
// Notes:
// - ChShaft elements have already been excluded (as these are treated separately)
// - allow all items to include body forces (required e.g. ChShaftsTorqueBase)
// - no support for any items that introduce additional state variables
// - only include constraints from items of supported type (see GetBilateralType above)
//
void ChSystemParallel::UpdateOtherPhysics()
{
  double oostep = 1 / GetStep();
  real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
  bool clamp = data_manager->settings.solver.clamp_bilaterals;

  for (int i = 0; i < otherphysicslist.size(); i++) {
    otherphysicslist[i]->Update(ChTime);
    otherphysicslist[i]->ConstraintsBiReset();
    otherphysicslist[i]->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
    otherphysicslist[i]->ConstraintsBiLoad_Ct(1);
    otherphysicslist[i]->ConstraintsFbLoadForces(GetStep());
    otherphysicslist[i]->ConstraintsLoadJacobians();
    otherphysicslist[i]->VariablesFbLoadForces(GetStep());
    otherphysicslist[i]->VariablesQbLoadSpeed();

    BILATERALTYPE type = GetBilateralType(otherphysicslist[i]);

    if (type == UNKNOWN)
      continue;

    otherphysicslist[i]->InjectConstraints(*LCP_descriptor);

    for (int j = 0; j < otherphysicslist[i]->GetDOC_c(); j++)
      data_manager->host_data.bilateral_type.push_back(type);
  }
}

//
// Collect indexes of all active bilateral constraints and calculate number of
// non-zero entries in the constraint Jacobian.
//
void ChSystemParallel::UpdateBilaterals()
{
  data_manager->nnz_bilaterals = 0;
  std::vector<ChLcpConstraint *> &mconstraints = LCP_descriptor->GetConstraintsList();

  for (uint ic = 0; ic < mconstraints.size(); ic++) {
    if (mconstraints[ic]->IsActive()) {
      data_manager->host_data.bilateral_mapping.push_back(ic);
      switch (data_manager->host_data.bilateral_type[ic]) {
      case BODY_BODY:
        data_manager->nnz_bilaterals += 12;
        break;
      case SHAFT_SHAFT:
        data_manager->nnz_bilaterals += 2;
        break;
      case SHAFT_SHAFT_SHAFT:
        data_manager->nnz_bilaterals += 3;
        break;
      case SHAFT_BODY:
        data_manager->nnz_bilaterals += 7;
        break;
      }
    }
  }

  // Set the number of currently active bilateral constraints.
  data_manager->num_bilaterals = data_manager->host_data.bilateral_mapping.size();
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
