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

#pragma once

#include <math.h>

#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/solver/ChVariablesNode.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/ChDataManager.h"
namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

class CH_PARALLEL_API ChNodeFluid : public ChPhysicsItem {
 public:
  ChNodeFluid(real r);
  ~ChNodeFluid();

  ChNodeFluid(const ChNodeFluid& other);             // Copy constructor
  ChNodeFluid& operator=(const ChNodeFluid& other);  // Assignment operator

  /// "Virtual" copy constructor (covariant return type).
  virtual ChNodeFluid* Clone() const override { return new ChNodeFluid(*this); }

  //
  // FUNCTIONS
  //

  bool GetCollide() { return true; }
  real GetDensity() { return data_manager->host_data.den_fluid[body_id]; }

  // Position of the node - in absolute csys.
  ChVector<> GetPos() {
    real3 pos = data_manager->host_data.pos_fluid[body_id];
    return ChVector<>(pos.x, pos.y, pos.z);
  }
  // Position of the node - in absolute csys.
  void SetPos(const ChVector<>& mpos) { data_manager->host_data.pos_fluid[body_id] = R3(mpos.x, mpos.y, mpos.z); }

  void AddCollisionModelsToSystem() {}

  // Velocity of the node - in absolute csys.
  ChVector<> GetPos_dt() {
    real3 vel = data_manager->host_data.vel_fluid[body_id];
    return ChVector<>(vel.x, vel.y, vel.z);
  }
  // Velocity of the node - in absolute csys.
  void SetPos_dt(const ChVector<>& mposdt) {
    data_manager->host_data.vel_fluid[body_id] = R3(mposdt.x, mposdt.y, mposdt.z);
  }

  // Acceleration of the node - in absolute csys.
  ChVector<> GetPos_dtdt() { return ChVector<>(0); }
  // Acceleration of the node - in absolute csys.
  void SetPos_dtdt(const ChVector<>& mposdtdt) {}
  /// Set the body identifier
  void SetId(int identifier) { body_id = identifier; }
  /// Set the body identifier
  unsigned int GetId() { return body_id; }

  //
  // DATA

  unsigned int body_id;
  ChParallelDataManager* data_manager;
};
}
