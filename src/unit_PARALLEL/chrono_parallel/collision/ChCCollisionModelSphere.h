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
// Description: Specialized collision model for a  single sphere
// =============================================================================

#ifndef CHCCOLLISIONMODELSPHERE_H
#define CHCCOLLISIONMODELSPHERE_H

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCCollisionModelParallel.h"

#include "chrono_parallel/ChParallelDefines.h"

namespace chrono {
// forward references
class ChBody;

namespace collision {
///  A wrapper to uses GPU collision detection

class CH_PARALLEL_API ChCollisionModelSphere : public ChCollisionModelParallel {
 public:

   ChCollisionModelSphere(real rad = 1.0);
   virtual ~ChCollisionModelSphere();

   /// Add a sphere shape to this model, for collision purposes
   virtual bool AddSphere(real radius,
                          const ChVector<> &pos = ChVector<>());
   void SetSphereRadius(real sphere_radius);
 private:
   real radius;
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
#endif

