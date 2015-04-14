// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: The definition of a parallel ChSystem, pretty much everything is
// done manually instead of using the functions used in ChSystem. This is to
// handle the different data structures present in the parallel implementation
// =============================================================================

#ifndef CH_SYSTEMPARALLEL_H
#define CH_SYSTEMPARALLEL_H

#include <stdlib.h>
#include <float.h>
#include <memory.h>
#include <algorithm>

#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChContactDEM.h"
#include "physics/ChGlobal.h"
#include "physics/ChContactContainer.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChCCollisionSystemParallel.h"
#include "chrono_parallel/collision/ChCCollisionSystemBulletParallel.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChCNarrowphaseR.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/physics/ChNodeFluid.h"
namespace chrono {

class CH_PARALLEL_API ChSystemParallel : public ChSystem {
  CH_RTTI(ChSystemParallel, ChSystem);

 public:
  ChSystemParallel(unsigned int max_objects);
  ~ChSystemParallel();

  virtual int Integrate_Y();
  virtual void AddBody(ChSharedPtr<ChBody> newbody);
  virtual void AddOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> newitem);

  void ClearForceVariables();
  void Update();
  void UpdateBilaterals();
  void UpdateLinks();
  void UpdateOtherPhysics();
  void UpdateRigidBodies();
  void UpdateShafts();
  void UpdateFluidBodies();
  void RecomputeThreads();

  virtual void AddMaterialSurfaceData(ChSharedPtr<ChBody> newbody) = 0;
  virtual void UpdateMaterialSurfaceData(int index, ChBody* body) = 0;
  virtual void Setup();
  virtual void ChangeCollisionSystem(COLLISIONSYSTEMTYPE type);

  virtual void PrintStepStats() { data_manager->system_timer.PrintReport(); }

  int GetNumBodies() { return data_manager->num_rigid_bodies + data_manager->num_fluid_bodies; }

  int GetNumShafts() { return data_manager->num_shafts; }

  int GetNcontacts() {
    return data_manager->num_rigid_contacts + data_manager->num_rigid_fluid_contacts + data_manager->num_fluid_contacts;
  }

  int GetNumBilaterals() { return data_manager->num_bilaterals; }

  double GetTimerCollision() { return timer_collision; }

  virtual real3 GetBodyContactForce(uint body_id) const = 0;
  virtual real3 GetBodyContactTorque(uint body_id) const = 0;

  settings_container* GetSettings() { return &(data_manager->settings); }

  // based on the passed logging level and the state of that level, enable or
  // disable logging level
  void SetLoggingLevel(LOGGINGLEVEL level, bool state = true);

  /// Calculate the (linearized) bilateral constraint violations.
  /// Return the maximum constraint violation.
  double CalculateConstraintViolation(std::vector<double>& cvec);

  ChParallelDataManager* data_manager;

 protected:
  double timer_collision, old_timer, old_timer_cd;
  bool detect_optimal_threads;

  int current_threads;
  int detect_optimal_bins;
  std::vector<double> timer_accumulator, cd_accumulator;
  uint frame_threads, frame_bins, counter;
  std::vector<ChLink*>::iterator it;

 private:
  void AddShaft(ChSharedPtr<ChShaft> shaft);

  std::vector<ChShaft*> shaftlist;
  ChSharedPtr<ChNodeFluid> fluid_container;
};

class CH_PARALLEL_API ChSystemParallelDVI : public ChSystemParallel {
  CH_RTTI(ChSystemParallelDVI, ChSystemParallel);

 public:
  ChSystemParallelDVI(unsigned int max_objects = 1000);

  void ChangeSolverType(SOLVERTYPE type) { ((ChLcpSolverParallelDVI*)(LCP_solver_speed))->ChangeSolverType(type); }

  virtual ChBody::ContactMethod GetContactMethod() const { return ChBody::DVI; }
  virtual void AddMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
  virtual void UpdateMaterialSurfaceData(int index, ChBody* body);

  void CalculateContactForces();

  virtual real3 GetBodyContactForce(uint body_id) const;
  virtual real3 GetBodyContactTorque(uint body_id) const;

  virtual void AssembleSystem();
  virtual void SolveSystem();
};

class CH_PARALLEL_API ChSystemParallelDEM : public ChSystemParallel {
  CH_RTTI(ChSystemParallelDEM, ChSystemParallel);

 public:
  ChSystemParallelDEM(unsigned int max_objects = 1000);

  virtual ChBody::ContactMethod GetContactMethod() const { return ChBody::DEM; }
  virtual void AddMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
  virtual void UpdateMaterialSurfaceData(int index, ChBody* body);

  virtual void Setup();
  virtual void ChangeCollisionSystem(COLLISIONSYSTEMTYPE type);

  virtual real3 GetBodyContactForce(uint body_id) const;
  virtual real3 GetBodyContactTorque(uint body_id) const;

  virtual void PrintStepStats();

  double GetTimerProcessContact() const {
    return data_manager->system_timer.GetTime("ChLcpSolverParallelDEM_ProcessContact");
  }
};

}  // end namespace chrono

#endif
