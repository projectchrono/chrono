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
#include "collision/ChCCollisionModel.h"
#include "lcp/ChLcpVariablesNode.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChSystem;

/// Class for a single node in the SPH cluster
/// (it does not define mass, inertia and shape becuase those
/// data are shared between them)

class CH_PARALLEL_API ChNodeFluid : public ChNodeXYZ {
 public:
   ChNodeFluid();
   ~ChNodeFluid();

   ChNodeFluid(const ChNodeFluid& other);  // Copy constructor
   ChNodeFluid& operator=(const ChNodeFluid& other);  //Assignment operator

   //
   // FUNCTIONS
   //

   // Get the kernel radius (max. radius while checking surrounding particles)
   real GetKernelRadius() {
      return h_rad;
   }
   void SetKernelRadius(real mr);

   // Set collision radius (for colliding with bodies, boundaries, etc.)
   real GetCollisionRadius() {
      return coll_rad;
   }
   void SetCollisionRadius(real mr);

   // Set the mass of the node
   void SetMass(double mmass) {
      this->variables.SetNodeMass(mmass);
   }
   // Get the mass of the node
   double GetMass() const {
      return variables.GetNodeMass();
   }

   // Access the 'LCP variables' of the node
   ChLcpVariables& Variables() {
      return variables;
   }

   //
   // DATA
   //

   ChLcpVariablesNode variables;

   collision::ChCollisionModel* collision_model;

   ChVector<> UserForce;

   real volume;  //volume of the node
   real density;  //density of the node
   real h_rad;   //kernel radius of the node
   real coll_rad;   //collision radius (for collision model)
   real pressure;   //pressure at node
};

}
#endif
