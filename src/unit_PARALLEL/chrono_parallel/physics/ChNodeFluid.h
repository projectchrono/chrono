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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This file defines a fluid node, it is based on a 3DOF NodeXYZ
// element. This class is similar to ChMatterSPH but is meant to be a bit more
// general.
// =============================================================================

#ifndef CHNODEFLUID_H
#define CHNODEFLUID_H

#include <math.h>

#include "physics/ChNodeXYZ.h"
#include "chrono_parallel/collision/ChCCollisionModelSphere.h"
#include "lcp/ChLcpVariablesNode.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChSystem;

/// Class for a single node in the SPH cluster
/// (it does not define mass, inertia and shape becuase those
/// data are shared between them)

class CH_PARALLEL_API ChNodeFluid : public ChPhysicsItem {
 public:
  ChNodeFluid(real r);
  ~ChNodeFluid();

  ChNodeFluid(const ChNodeFluid& other);             // Copy constructor
  ChNodeFluid& operator=(const ChNodeFluid& other);  // Assignment operator

  //
  // FUNCTIONS
  //

  // Set collision radius (for colliding with bodies, boundaries, etc.)
  real GetCollisionRadius() { return coll_rad; }
  void SetCollisionRadius(real mr);
  bool GetCollide() { return true; }
  real GetDensity() { return density; }

  void SetDensity(real d) { density = d; }

  // Position of the node - in absolute csys.
  ChVector<> GetPos() { return pos; }
  // Position of the node - in absolute csys.
  void SetPos(const ChVector<>& mpos) { pos = mpos; }

  void AddCollisionModelsToSystem();

  // Velocity of the node - in absolute csys.
  ChVector<> GetPos_dt() { return pos_dt; }
  // Velocity of the node - in absolute csys.
  void SetPos_dt(const ChVector<>& mposdt) { pos_dt = mposdt; }

  // Acceleration of the node - in absolute csys.
  ChVector<> GetPos_dtdt() { return pos_dtdt; }
  // Acceleration of the node - in absolute csys.
  void SetPos_dtdt(const ChVector<>& mposdtdt) { pos_dtdt = mposdtdt; }
  /// Set the body identifier - HM
  void SetId(int identifier) { body_id = identifier; }
  /// Set the body identifier - HM
  unsigned int GetId() { return body_id; }

  //
  // DATA
  //

  // real mass;
  real density;   // density of the node
  real coll_rad;  // collision radius (for collision model)
  real pressure;  // pressure at node
  ChVector<> pos;
  ChVector<> pos_dt;
  ChVector<> pos_dtdt;
  unsigned int body_id;
};
}
#endif
