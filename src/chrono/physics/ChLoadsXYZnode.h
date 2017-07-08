// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLOADSXYZNODE_H
#define CHLOADSXYZNODE_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChNodeXYZ.h"

namespace chrono {

// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to objects of ChNodeXYZ (and 
// inherited classes) type.
// These are 'simplified' tools, that save you from inheriting your custom 
// loads. 
// Or just look at these classes and learn how to implement some special type of load.


/// FORCE AT XYZ NODE
/// Loader for a constant force applied at a XYZ node.
/// Note that an equivalent shortcut is to set directly the node's own force member data
/// as mynode->SetForce(), but this other approach is more object-oriented (es. you can 
/// apply multiple forces at a single node, etc)
class ChLoaderXYZnode : public ChLoaderUVWatomic {
  private:
    ChVector<> force;

  public:
    // Useful: a constructor that also sets ChLoadable
    ChLoaderXYZnode(std::shared_ptr<ChLoadableUVW> mloadable) : ChLoaderUVWatomic(mloadable, 0, 0, 0) {
        this->force = VNULL;
    };

    // Compute F=F(u,v,w)
    // Implement it from base class.
    virtual void ComputeF(const double U,        ///< parametric coordinate -not used
                          const double V,        ///< parametric coordinate -not used
                          const double W,        ///< parametric coordinate -not used
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
        F.PasteVector(this->force, 0, 0);  // load, force part
    }

    /// Set force (ex. in [N] units), assumed to be constant in space and time,
    /// assumed applied at the node.
    void SetForce(const ChVector<>& mf) { this->force = mf; }
    ChVector<> GetForce() const { return this->force; }
};

/// Force at XYZ node (ready to use load)
class ChLoadXYZnode : public ChLoad<ChLoaderXYZnode> {
  public:
    ChLoadXYZnode(std::shared_ptr<ChNodeXYZ> mloadable) : ChLoad<ChLoaderXYZnode>(mloadable) {}
};

}  // end namespace chrono

#endif
