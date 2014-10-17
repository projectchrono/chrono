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
// Authors: Radu Serban
// =============================================================================
//
// Miscelaneous common declarations and functions for the Chrono Parallel Utils
// library.
//
// =============================================================================

#ifndef CH_UTILS_COMMON_H
#define CH_UTILS_COMMON_H

#include <random>

#include "physics/ChSystem.h"
#include "physics/ChSystemDEM.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/collision/ChCCollisionSystemParallel.h"
#include "collision/ChCCollisionSystemBullet.h"

namespace chrono {
namespace utils {


enum SystemType {
  SEQUENTIAL_DVI,
  SEQUENTIAL_DEM,
  PARALLEL_DVI,
  PARALLEL_DEM
};

enum CollisionType {
  BULLET_CD,
  PARALLEL_CD
};

// -----------------------------------------------------------------------------
// Construct a single random engine (on first use)
//
// Note that this object is never destructed (but this is OK)
// -----------------------------------------------------------------------------
inline
std::default_random_engine& rengine()
{
  static std::default_random_engine* re = new std::default_random_engine;
  return *re;
}

// -----------------------------------------------------------------------------
// sampleTruncatedDist
//
// Utility function for generating samples from a truncated normal distribution.
// -----------------------------------------------------------------------------
template <typename T>
inline
T sampleTruncatedDist(std::normal_distribution<T>& distribution,
                      T                            minVal,
                      T                            maxVal)
{
  T val;

  do {
    val = distribution(rengine());
  } while (val < minVal || val > maxVal);

  return val;
}

// -----------------------------------------------------------------------------
// GetSystemType()
//
// This utility function infers the type of the specified ChSystem.
// -----------------------------------------------------------------------------
inline
SystemType GetSystemType(ChSystem* system)
{
  if (dynamic_cast<ChSystemParallelDVI*>(system))
    return PARALLEL_DVI;

  if (dynamic_cast<ChSystemParallelDEM*>(system))
    return PARALLEL_DEM;
  
  if (dynamic_cast<ChSystemDEM*>(system))
    return SEQUENTIAL_DEM;

  return SEQUENTIAL_DVI;
}

// -----------------------------------------------------------------------------
// GetCollisionType()
//
// This utility function infers the type of the specified ChCollisionSystem.
// -----------------------------------------------------------------------------
inline
CollisionType GetCollisionType(ChSystem* system)
{
  if (dynamic_cast<collision::ChCollisionSystemParallel*>(system->GetCollisionSystem()))
    return PARALLEL_CD;

  if(dynamic_cast<collision::ChCollisionSystemBulletParallel*>(system->GetCollisionSystem()))
    return BULLET_CD;

  if(dynamic_cast<collision::ChCollisionSystemBullet*>(system->GetCollisionSystem()))
     return BULLET_CD;

  return BULLET_CD;
}

} // end namespace utils
} // end namespace chrono


#endif
