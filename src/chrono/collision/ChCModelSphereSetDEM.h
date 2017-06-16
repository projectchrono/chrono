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

#ifndef CHC_MODELSPHERESETDEM_H
#define CHC_MODELSPHERESETDEM_H

#include "collision/ChCModelSphereSet.h"

namespace chrono {

// forward references
class ChBodyDEM;

namespace collision {

/// Class for the collision model to be used in ChIndexedParticles shapes,
/// that are collections of point-like nodes (each with 3 DOFs)
/// using features of the Bullet library.

class ChApi ChModelSphereSetDEM : public ChModelSphereSet {
  public:
    ChModelSphereSetDEM();
    virtual ~ChModelSphereSetDEM();

    /// Gets the pointer to the client owner rigid body.
    ChBodyDEM* GetBody() const { return mbody; };
    /// Sets the pointer to the client owner rigid body
    void SetBody(ChBodyDEM* mbo) { mbody = mbo; };

    // Overrides and implementations of base members:

    /// Sets the position and orientation of the collision
    /// model as the current position of the corresponding ChBody
    virtual void SyncPosition();

    /// Gets the pointer to the client owner ChPhysicsItem.
    virtual ChPhysicsItem* GetPhysicsItem() { return (ChPhysicsItem*)GetBody(); };

  private:
    ChBodyDEM* mbody;
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
