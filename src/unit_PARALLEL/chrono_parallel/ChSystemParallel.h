#ifndef CH_SYSTEMPARALLEL_H
#define CH_SYSTEMPARALLEL_H
//////////////////////////////////////////////////
//
//   ChSystemParallel.h
//
//   GPU Simulation System
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
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
#include "chrono_parallel/ChLcpSolverParallel.h"
#include "chrono_parallel/ChLcpSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChCCollisionSystemParallel.h"
#include "chrono_parallel/collision/ChCCollisionSystemBulletParallel.h"
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
   virtual void RemoveBody(ChSharedPtr<ChBody> mbody);
   void RemoveBody(int i);
   void Update();
   void UpdateBilaterals();
   void RecomputeThreads();
   void RecomputeBins();
   void PerturbBins(bool increase,
                    int number = 2);

   virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody) = 0;
   virtual void UpdateBodies() = 0;
   virtual void AssembleSystem(ChLcpSystemDescriptor* sys_descriptor) = 0;
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

   void SetAABB(real3 aabbmin,
                real3 aabbmax) {
      aabb_min = aabbmin;
      aabb_max = aabbmax;
      use_aabb_active = true;
   }

   bool GetAABB(real3 &aabbmin,
                real3 &aabbmax) {
      aabbmin = aabb_min;
      aabbmax = aabb_max;

      return use_aabb_active;
   }

   void DoThreadTuning(bool m) {
      perform_thread_tuning = m;
   }

   void DoBinTuning(bool m) {
      perform_bin_tuning = m;
   }

   void SetMinThreads(int m) {
      min_threads = m;
   }

   double GetTimerCollision() {
      return timer_collision;
   }

   ChParallelDataManager *data_manager;

 private:

   double timer_collision, old_timer, old_timer_cd;

   bool use_aabb_active;
   bool detect_optimal_threads, perform_thread_tuning, perform_bin_tuning;
   real3 aabb_min, aabb_max;

   int max_threads, current_threads, min_threads;
   int detect_optimal_bins;
   vector<double> timer_accumulator, cd_accumulator;
   uint frame_threads, frame_bins, counter;
   std::list<ChLink *>::iterator it;
};

class CH_PARALLEL_API ChSystemParallelDVI : public ChSystemParallel {
CH_RTTI(ChSystemParallelDVI, ChSystemParallel)
   ;

 public:
   ChSystemParallelDVI(unsigned int max_objects = 1000);

   virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
   virtual void UpdateBodies();
   virtual void AssembleSystem(ChLcpSystemDescriptor* sys_descriptor);
   void Assemble_load(bool load_jacobians,
         bool load_Mv,
         double F_factor,
         double K_factor,
         double R_factor,
         double M_factor,
         double Ct_factor,
         double C_factor,
         double recovery_clamp,
         bool do_clamp,
         ChContactContainerBase* mcontactcontainer
   );
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
   virtual void AssembleSystem(ChLcpSystemDescriptor* sys_descriptor) {
   }
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

