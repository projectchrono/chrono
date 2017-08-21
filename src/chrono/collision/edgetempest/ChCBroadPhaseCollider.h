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

#ifndef CHC_BROADPHASECOLLIDER_H
#define CHC_BROADPHASECOLLIDER_H

#if (_MSC_VER >= 1200)
#pragma warning(4 : 4786)
#endif

#include <list>

#include "chrono/collision/edgetempest/ChCMates.h"

namespace chrono {
namespace collision {

///
/// Base class for generic broad-phase collision engine.
/// Most methods are 'pure virtual': they need to be implemented
/// by child classes.
///

template <class model_type>
class ChBroadPhaseCollider {
  public:
    typedef typename std::list<ChMates<model_type>*> ChMatesPtrList;

  public:
    ChBroadPhaseCollider(){};
    virtual ~ChBroadPhaseCollider(){};

    /// Clears all data instanced by this algorithm.
    virtual void Clear(void) = 0;

    /// Adds a collision model to the broad-phase
    /// engine (custom data may be allocated).
    virtual void Add(model_type* model) = 0;

    /// Removes a collision model from the broad-phase
    /// engine (custom data may be deallocated).
    virtual void Remove(model_type* model) = 0;

    /// Run the broad-phase collision algorithm, which finds the
    /// overlapping pairs of bounding boxes
    virtual void Run() = 0;

    /// After the broad phase algorithm has run, you can use
    /// this method to access the list with the reported pairs of
    /// overlapping models.
    virtual ChMatesPtrList& GetReportedMates() = 0;
};

}  // end namespace collision
}  // end namespace chrono

#endif
