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

#include "core/ChTimer.h"
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

  virtual int Integrate_Y();

  virtual void AddBody(ChSharedPtr<ChBody> newbody);
  virtual void RemoveBody(ChSharedPtr<ChBody> mbody);
  void RemoveBody(int i);
  void Update();
  void UpdateBilaterals();
  void RecomputeThreads();
  void RecomputeBins();
  void PerturbBins(bool increase, int number = 2);

  virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody) = 0;
  virtual void UpdateBodies() = 0;
  virtual void ChangeCollisionSystem(ChCollisionSystem *newcollsystem);

  int GetNcontacts() {
    return gpu_data_manager->num_contacts;
  }

  void SetAABB(real3 aabbmin, real3 aabbmax) {
    aabb_min = aabbmin;
    aabb_max = aabbmax;
    use_aabb_active = true;
  }

  bool GetAABB(real3 &aabbmin, real3 &aabbmax) {
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

  ChParallelDataManager *gpu_data_manager;

 private:

  unsigned int counter;
  double timer_collision;
  std::list<ChLink *>::iterator it;

  bool use_aabb_active;
  real3 aabb_min, aabb_max;

  int max_threads, current_threads, min_threads;
  vector<double> timer_accumulator, cd_accumulator;
  double old_timer, old_timer_cd;
  bool detect_optimal_threads, perform_thread_tuning, perform_bin_tuning;
  int detect_optimal_bins;
  uint frame_threads;
  uint frame_bins;
};

class CH_PARALLEL_API ChSystemParallelDVI : public ChSystemParallel {
CH_RTTI(ChSystemParallelDVI, ChSystemParallel)
  ;

 public:
  ChSystemParallelDVI(unsigned int max_objects = 1000);

  virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
  virtual void UpdateBodies();

};

class CH_PARALLEL_API ChSystemParallelDEM : public ChSystemParallel {
  CH_RTTI(ChSystemParallelDEM, ChSystemParallel);

public:
  ChSystemParallelDEM(unsigned int max_objects = 1000,
                      ChContactDEM::NormalForceModel normal_model = ChContactDEM::HuntCrossley,
                      ChContactDEM::TangentialForceModel tangential_model = ChContactDEM::SimpleCoulombSliding);

  virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
  virtual void UpdateBodies();

  virtual void ChangeCollisionSystem(ChCollisionSystem *newcollsystem);

  double GetTimerProcessContact() const {return gpu_data_manager->system_timer.GetTime("ChLcpSolverParallelDEM_ProcessContact");}

  ChContactDEM::NormalForceModel normal_force_model;
  ChContactDEM::TangentialForceModel tangential_force_model;
};

}  // end namespace chrono

#endif

