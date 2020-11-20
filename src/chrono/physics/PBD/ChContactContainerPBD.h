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
// Authors: Simone Benatti
// =============================================================================
//
// This inherits from a NSC ContactContainer but it makes the lists accessible
//
// =============================================================================

#ifndef CH_CONTACTCONTAINER_PBD_H
#define CH_CONTACTCONTAINER_PBD_H

#include <list>

#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChContactNSC.h"
#include "chrono/physics/ChContactNSCrolling.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many non-smooth contacts.
/// Implemented using linked lists of ChContactNSC objects (that is, contacts between two ChContactable objects, with 3
/// reactions). It might also contain ChContactNSCrolling objects (extended versions of ChContactNSC, with 6 reactions,
/// that account also for rolling and spinning resistance), but also for '6dof vs 6dof' contactables.
class ChApi ChContactContainerPBD : public ChContactContainerNSC {

  public:
    ChContactContainerPBD();
    ChContactContainerPBD(const ChContactContainerPBD& other);
    virtual ~ChContactContainerPBD();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerPBD* Clone() const override { return new ChContactContainerPBD(*this); }

    /// Report the number of added contacts.
    
	virtual int GetNcontacts() const override {
        return n_added_3_3 + n_added_6_3 + n_added_6_6 + n_added_333_3 + n_added_333_6 + n_added_333_333 +
               n_added_666_3 + n_added_666_6 + n_added_666_333 + n_added_666_666 + n_added_6_6_rolling;
    }

};

CH_CLASS_VERSION(ChContactContainerPBD, 0)

}  // end namespace chrono

#endif
