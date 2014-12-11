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
#include "physics/ChBodyDEM.h"
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

namespace chrono {

class CH_PARALLEL_API ChSystemParallel : public ChSystem {
CH_RTTI(ChSystemParallel, ChSystem)
   ;

 public:
   ChSystemParallel(unsigned int max_objects);
   ~ChSystemParallel();

   virtual int Integrate_Y();
   virtual void AddBody(ChSharedPtr<ChBody> newbody);
   virtual void AddOtherPhysicsItem (ChSharedPtr<ChPhysicsItem> newitem);
   virtual void Update();
   virtual void UpdateBilaterals();
   virtual void UpdateBodies() = 0;
   void RecomputeThreads();
   void RecomputeBins();
   void PerturbBins(bool increase,
                    int number = 2);

   virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody) = 0;
   virtual void ChangeCollisionSystem(collision::ChCollisionSystem *newcollsystem);

   virtual void PrintStepStats() {
      data_manager->system_timer.PrintReport();
   }

   int GetNumBodies() {
      return data_manager->num_bodies;
   }

   int GetNcontacts() {
      return data_manager->num_contacts;
   }

   int GetNumBilaterals() {
      return data_manager->num_bilaterals;
   }

   double GetTimerCollision() {
      return timer_collision;
   }
   settings_container * GetSettings() {
      return &(data_manager->settings);
   }
   ChParallelDataManager *data_manager;

 protected:

   double timer_collision, old_timer, old_timer_cd;
   bool detect_optimal_threads;

   int current_threads;
   int detect_optimal_bins;
   std::vector<double> timer_accumulator, cd_accumulator;
   uint frame_threads, frame_bins, counter;
   std::vector<ChLink *>::iterator it;
};

class CH_PARALLEL_API ChSystemParallelDVI : public ChSystemParallel {
CH_RTTI(ChSystemParallelDVI, ChSystemParallel)
   ;

 public:
   ChSystemParallelDVI(unsigned int max_objects = 1000);
   void ChangeSolverType(SOLVERTYPE type) {
      ((ChLcpSolverParallelDVI *) (LCP_solver_speed))->ChangeSolverType(type);
   }
   virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
   virtual void UpdateBodies();
   virtual void AssembleSystem();
   virtual void SolveSystem();
};

class CH_PARALLEL_API ChSystemParallelDEM : public ChSystemParallel {
CH_RTTI(ChSystemParallelDEM, ChSystemParallel)
   ;

 public:
   ChSystemParallelDEM(unsigned int max_objects = 1000,
                       ChContactDEM::NormalForceModel normal_model = ChContactDEM::HuntCrossley,
                       ChContactDEM::TangentialForceModel tangential_model = ChContactDEM::SimpleCoulombSliding);

   virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
   virtual void UpdateBodies();
   virtual void ChangeCollisionSystem(collision::ChCollisionSystem *newcollsystem);

   virtual void PrintStepStats();

   double GetTimerProcessContact() const {
      return data_manager->system_timer.GetTime("ChLcpSolverParallelDEM_ProcessContact");
   }

   ChContactDEM::NormalForceModel normal_force_model;
   ChContactDEM::TangentialForceModel tangential_force_model;
};

}  // end namespace chrono

#endif

