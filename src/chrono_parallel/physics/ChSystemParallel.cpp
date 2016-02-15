#include "physics/ChShaftsCouple.h"
#include "physics/ChShaftsGearbox.h"
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include <numeric>

using namespace chrono;
using namespace chrono::collision;
#ifdef LOGGINGENABLED
INITIALIZE_EASYLOGGINGPP
#endif
ChSystemParallel::ChSystemParallel(unsigned int max_objects) : ChSystem(1000, 10000, false) {
  data_manager = new ChParallelDataManager();

  LCP_descriptor = new ChLcpSystemDescriptorParallel(data_manager);
  contact_container = std::make_shared<ChContactContainerParallel>(data_manager);
  collision_system = new ChCollisionSystemParallel(data_manager);

  collision_system_type = COLLSYS_PARALLEL;

  counter = 0;
  timer_accumulator.resize(10, 0);
  cd_accumulator.resize(10, 0);
  frame_threads = 0;
  frame_bins = 0;
  old_timer = 0;
  old_timer_cd = 0;
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
#ifdef LOGGINGENABLED
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToFile, "false");
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%datetime{%h:%m:%s:%g} %msg");
#endif
}

ChSystemParallel::~ChSystemParallel() {
  delete data_manager;
}

int ChSystemParallel::Integrate_Y() {
  LOG(INFO) << "ChSystemParallel::Integrate_Y()";
  // Get the pointer for the system descriptor and store it into the data manager
  data_manager->lcp_system_descriptor = this->LCP_descriptor;
  data_manager->body_list = &this->bodylist;
  data_manager->link_list = &this->linklist;
  data_manager->other_physics_list = &this->otherphysicslist;

  data_manager->system_timer.Reset();
  data_manager->system_timer.start("step");

  Setup();

  data_manager->system_timer.start("update");
  Update();
  data_manager->system_timer.stop("update");

  data_manager->system_timer.start("collision");
  collision_system->Run();
  collision_system->ReportContacts(this->contact_container.get());
  data_manager->system_timer.stop("collision");

  data_manager->system_timer.start("lcp");
  ((ChLcpSolverParallel*)(LCP_solver_speed))->RunTimeStep();
  data_manager->system_timer.stop("lcp");

  data_manager->system_timer.start("update");

  // Iterate over the active bilateral constraints and store their Lagrange
  // multiplier.
  std::vector<ChLcpConstraint*>& mconstraints = LCP_descriptor->GetConstraintsList();
  for (int index = 0; index < data_manager->num_bilaterals; index++) {
    int cntr = data_manager->host_data.bilateral_mapping[index];
    mconstraints[cntr]->Set_l_i(data_manager->host_data.gamma[data_manager->num_unilaterals + index]);
  }

  // Update the constraint reactions.
  LCPresult_Li_into_reactions(1.0 / this->GetStep());  // R = l/dt  , approximately

  // Scatter the states to the Chrono objects (bodies and shafts) and update
  // all physics items at the end of the step.
  DynamicVector<real>& velocities = data_manager->host_data.v;
  custom_vector<real3>& pos_pointer = data_manager->host_data.pos_rigid;
  custom_vector<real4>& rot_pointer = data_manager->host_data.rot_rigid;

#pragma omp parallel for
  for (int i = 0; i < bodylist.size(); i++) {
    if (data_manager->host_data.active_rigid[i] == true) {
      bodylist[i]->Variables().Get_qb().SetElement(0, 0, velocities[i * 6 + 0]);
      bodylist[i]->Variables().Get_qb().SetElement(1, 0, velocities[i * 6 + 1]);
      bodylist[i]->Variables().Get_qb().SetElement(2, 0, velocities[i * 6 + 2]);
      bodylist[i]->Variables().Get_qb().SetElement(3, 0, velocities[i * 6 + 3]);
      bodylist[i]->Variables().Get_qb().SetElement(4, 0, velocities[i * 6 + 4]);
      bodylist[i]->Variables().Get_qb().SetElement(5, 0, velocities[i * 6 + 5]);

      bodylist[i]->VariablesQbIncrementPosition(this->GetStep());
      bodylist[i]->VariablesQbSetSpeed(this->GetStep());

      bodylist[i]->Update(ChTime);

      // update the position and rotation vectors
      pos_pointer[i] = (R3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
      rot_pointer[i] =
          (R4(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2, bodylist[i]->GetRot().e3));
    }
  }

  ////#pragma omp parallel for
  for (int i = 0; i < data_manager->num_shafts; i++) {
    if (!data_manager->host_data.shaft_active[i])
      continue;

    shaftlist[i]->Variables().Get_qb().SetElementN(0, velocities[data_manager->num_rigid_bodies * 6 + i]);
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
  if (data_manager->settings.perform_thread_tuning) {
    RecomputeThreads();
  }

  return 1;
}

//
// Add the specified body to the system.
// A unique identifier is assigned to each body for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the
// body.
//
void ChSystemParallel::AddBody(std::shared_ptr<ChBody> newbody) {
  // This is only need because bilaterals need to know what bodies to
  // refer to. Not used by contacts
  newbody->SetId(data_manager->num_rigid_bodies);

  bodylist.push_back(newbody);
  data_manager->num_rigid_bodies++;

  // Set the system for the body.  Note that this will also add the body's
  // collision shapes to the collision system if not already done.
  newbody->SetSystem(this);

  // Reserve space for this body in the system-wide vectors. Note that the
  // actual data is set in UpdateBodies().
  data_manager->host_data.pos_rigid.push_back(R3());
  data_manager->host_data.rot_rigid.push_back(R4());
  data_manager->host_data.active_rigid.push_back(true);
  data_manager->host_data.collide_rigid.push_back(true);

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
void ChSystemParallel::AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> newitem) {
    if (auto shaft = std::dynamic_pointer_cast<ChShaft>(newitem)) {
        AddShaft(shaft);
    } else {
        newitem->SetSystem(this);
        otherphysicslist.push_back(newitem);

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
void ChSystemParallel::AddShaft(std::shared_ptr<ChShaft> shaft) {
  shaft->SetId(data_manager->num_shafts);
  shaft->SetSystem(this);

  shaftlist.push_back(shaft.get());
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
void ChSystemParallel::ClearForceVariables() {
#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
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
void ChSystemParallel::Update() {
  LOG(INFO) << "ChSystemParallel::Update()";
  // Clear the forces for all lcp variables
  ClearForceVariables();

  // Allocate space for the velocities and forces for all objects
  data_manager->host_data.v.resize(data_manager->num_dof);
  data_manager->host_data.hf.resize(data_manager->num_dof);

  // Clear system-wide vectors for bilateral constraints
  data_manager->host_data.bilateral_mapping.clear();
  data_manager->host_data.bilateral_type.clear();

  this->LCP_descriptor->BeginInsertion();
  UpdateLinks();
  UpdateOtherPhysics();
  UpdateRigidBodies();
  UpdateShafts();
  UpdateFluidBodies();
  LCP_descriptor->EndInsertion();

  UpdateBilaterals();
}

//
// Update all bodies in the system and populate system-wide state and force
// vectors. Note that visualization assets are not updated.
//
void ChSystemParallel::UpdateRigidBodies() {
  LOG(INFO) << "ChSystemParallel::UpdateBodies()";
  custom_vector<real3>& position = data_manager->host_data.pos_rigid;
  custom_vector<real4>& rotation = data_manager->host_data.rot_rigid;
  custom_vector<bool>& active = data_manager->host_data.active_rigid;
  custom_vector<bool>& collide = data_manager->host_data.collide_rigid;

#pragma omp parallel for
  for (int i = 0; i < bodylist.size(); i++) {
    bodylist[i]->Update(ChTime, false);
    bodylist[i]->VariablesFbLoadForces(GetStep());
    bodylist[i]->VariablesQbLoadSpeed();

    ChMatrix<>& body_qb = bodylist[i]->Variables().Get_qb();
    ChMatrix<>& body_fb = bodylist[i]->Variables().Get_fb();
    ChVector<>& body_pos = bodylist[i]->GetPos();
    ChQuaternion<>& body_rot = bodylist[i]->GetRot();

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

    active[i] = bodylist[i]->IsActive();
    collide[i] = bodylist[i]->GetCollide();

    // Let derived classes set the specific material surface data.
    UpdateMaterialSurfaceData(i, bodylist[i].get());

    bodylist[i]->GetCollisionModel()->SyncPosition();
  }
}

//
// Update all shaft elements in the system and populate system-wide state and
// force vectors. Note that visualization assets are not updated.
//
void ChSystemParallel::UpdateShafts() {
  real* shaft_rot = data_manager->host_data.shaft_rot.data();
  real* shaft_inr = data_manager->host_data.shaft_inr.data();
  bool* shaft_active = data_manager->host_data.shaft_active.data();

  ////#pragma omp parallel for
  for (int i = 0; i < data_manager->num_shafts; i++) {
    shaftlist[i]->Update(ChTime, false);
    shaftlist[i]->VariablesFbLoadForces(GetStep());
    shaftlist[i]->VariablesQbLoadSpeed();

    shaft_rot[i] = shaftlist[i]->GetPos();
    shaft_inr[i] = shaftlist[i]->Variables().GetInvInertia();
    shaft_active[i] = shaftlist[i]->IsActive();

    data_manager->host_data.v[data_manager->num_rigid_bodies * 6 + i] =
        shaftlist[i]->Variables().Get_qb().GetElementN(0);
    data_manager->host_data.hf[data_manager->num_rigid_bodies * 6 + i] =
        shaftlist[i]->Variables().Get_fb().GetElementN(0);
  }
}

//
// Update all fluid nodes
// currently a stub
void ChSystemParallel::UpdateFluidBodies() {
}

//
// Update all links in the system and set the type of the associated constraints
// to BODY_BODY. Note that visualization assets are not updated.
//
void ChSystemParallel::UpdateLinks() {
  double oostep = 1 / GetStep();
  real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
  bool clamp = data_manager->settings.solver.clamp_bilaterals;

  for (int i = 0; i < linklist.size(); i++) {
    linklist[i]->Update(ChTime, false);
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
BILATERALTYPE GetBilateralType(ChPhysicsItem* item) {
  if (item->GetDOC_c() == 0)
    return UNKNOWN;

  if (dynamic_cast<ChShaftsCouple*>(item))
    return SHAFT_SHAFT;

  if (dynamic_cast<ChShaftsPlanetary*>(item))
    return SHAFT_SHAFT_SHAFT;

  if (dynamic_cast<ChShaftsGearbox*>(item) || dynamic_cast<ChShaftsGearboxAngled*>(item))
    return SHAFT_SHAFT_BODY;

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
// - visualization assets are not updated
//
void ChSystemParallel::UpdateOtherPhysics() {
  double oostep = 1 / GetStep();
  real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
  bool clamp = data_manager->settings.solver.clamp_bilaterals;

  for (int i = 0; i < otherphysicslist.size(); i++) {
    otherphysicslist[i]->Update(ChTime, false);
    otherphysicslist[i]->ConstraintsBiReset();
    otherphysicslist[i]->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
    otherphysicslist[i]->ConstraintsBiLoad_Ct(1);
    otherphysicslist[i]->ConstraintsFbLoadForces(GetStep());
    otherphysicslist[i]->ConstraintsLoadJacobians();
    otherphysicslist[i]->VariablesFbLoadForces(GetStep());
    otherphysicslist[i]->VariablesQbLoadSpeed();

    BILATERALTYPE type = GetBilateralType(otherphysicslist[i].get());

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
void ChSystemParallel::UpdateBilaterals() {
  data_manager->nnz_bilaterals = 0;
  std::vector<ChLcpConstraint*>& mconstraints = LCP_descriptor->GetConstraintsList();

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
        case SHAFT_SHAFT_BODY:
          data_manager->nnz_bilaterals += 8;
          break;
      }
    }
  }

  // Set the number of currently active bilateral constraints.
  data_manager->num_bilaterals = data_manager->host_data.bilateral_mapping.size();
}

//
// Prepare simulation of the next step.  This function is called after
// the system update and before collision detection. A derived class can
// override this function, but it should invoke this default implementation.
//
void ChSystemParallel::Setup() {
  LOG(INFO) << "ChSystemParallel::Setup()";
  // Cache the integration step size and calculate the tolerance at impulse level.
  data_manager->settings.step_size = step;
  data_manager->settings.solver.tol_speed = step * data_manager->settings.solver.tolerance;

  // Calculate the total number of degrees of freedom (6 per rigid body and 1
  // for each shaft element).
  data_manager->num_dof =
      data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_fluid_bodies * 3;

  // Set variables that are stored in the ChSystem class
  nbodies = data_manager->num_rigid_bodies;
  nlinks = 0;
  nphysicsitems = 0;
  ncoords = 0;
  ndoc = 0;
  nsysvars = 0;
  ncoords_w = 0;
  ndoc_w = 0;
  nsysvars_w = 0;
  ndof = data_manager->num_dof;
  ndoc_w_C = 0;
  ndoc_w_D = 0;
  ncontacts =
      data_manager->num_rigid_contacts + data_manager->num_rigid_fluid_contacts + data_manager->num_fluid_contacts;
  nbodies_sleep = 0;
  nbodies_fixed = 0;
}

void ChSystemParallel::RecomputeThreads() {
#ifdef CHRONO_OMP_FOUND
  timer_accumulator.insert(timer_accumulator.begin(), data_manager->system_timer.GetTime("step"));
  timer_accumulator.pop_back();

  double sum_of_elems = std::accumulate(timer_accumulator.begin(), timer_accumulator.end(), 0.0);

  if (frame_threads == 50 && detect_optimal_threads == false) {
    frame_threads = 0;
    if (current_threads + 2 < data_manager->settings.max_threads) {
      detect_optimal_threads = true;
      old_timer = sum_of_elems / 10.0;
      current_threads += 2;
      omp_set_num_threads(current_threads);
#if PRINT_LEVEL == 1
      cout << "current threads increased to " << current_threads << endl;
#endif
    } else {
      current_threads = data_manager->settings.max_threads;
      omp_set_num_threads(data_manager->settings.max_threads);
#if PRINT_LEVEL == 1
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
#if PRINT_LEVEL == 1
      cout << "current threads reduced back to " << current_threads << endl;
#endif
    }
  }

  if (current_threads < data_manager->settings.min_threads) {
    current_threads = data_manager->settings.min_threads;
    omp_set_num_threads(data_manager->settings.min_threads);
  }
  frame_threads++;
#endif
}

void ChSystemParallel::ChangeCollisionSystem(COLLISIONSYSTEMTYPE type) {
  assert(GetNbodies() == 0);

  delete collision_system;

  collision_system_type = type;

  switch (type) {
    case COLLSYS_PARALLEL:
      collision_system = new ChCollisionSystemParallel(data_manager);
      break;
    case COLLSYS_BULLET_PARALLEL:
      collision_system = new ChCollisionSystemBulletParallel(data_manager);
      break;
  }
}

void ChSystemParallel::SetLoggingLevel(LOGGINGLEVEL level, bool state) {
#ifdef LOGGINGENABLED

  std::string value = state ? "true" : "false";

  switch (level) {
    case LOG_NONE:
      el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");
      break;
    case LOG_INFO:
      el::Loggers::reconfigureAllLoggers(el::Level::Info, el::ConfigurationType::ToStandardOutput, value);
      break;
    case LOG_TRACE:
      el::Loggers::reconfigureAllLoggers(el::Level::Trace, el::ConfigurationType::ToStandardOutput, value);
      break;
    case LOG_WARNING:
      el::Loggers::reconfigureAllLoggers(el::Level::Warning, el::ConfigurationType::ToStandardOutput, value);
      break;
    case LOG_ERROR:
      el::Loggers::reconfigureAllLoggers(el::Level::Error, el::ConfigurationType::ToStandardOutput, value);
      break;
  }
#endif
}

//
// Calculate the (linearized) bilateral constraint violations and store them in
// the provided vector. Return the maximum constraint violation.
//
double ChSystemParallel::CalculateConstraintViolation(std::vector<double>& cvec) {
  std::vector<ChLcpConstraint*>& mconstraints = LCP_descriptor->GetConstraintsList();
  cvec.resize(data_manager->num_bilaterals);
  double max_c = 0;

  for (int index = 0; index < data_manager->num_bilaterals; index++) {
    int cntr = data_manager->host_data.bilateral_mapping[index];
    cvec[index] = mconstraints[cntr]->Compute_c_i();
    double abs_c = std::abs(cvec[index]);
    if (abs_c > max_c)
      max_c = abs_c;
  }

  return max_c;
}
