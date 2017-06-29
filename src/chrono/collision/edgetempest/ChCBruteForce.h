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

#ifndef CHC_BRUTEFORCE_H
#define CHC_BRUTEFORCE_H

#include "chrono/geometry/ChGeometry.h"
#include "chrono/collision/edgetempest/ChCBroadPhaseCollider.h"

namespace chrono {
namespace collision {

///
/// The 'brute force' broad-phase exploits a bad
/// combinatorial complexity. Therefore it is here just
/// for benchmark and profiling.
///

template <class model_type>
class ChBruteForce : public ChBroadPhaseCollider<model_type> {
  protected:
    ChMatesPtrList m_reported;        ///< A list containing all possible overlaps
    std::list<model_type*> m_models;  ///< A list of inserted models

  public:
    ChBruteForce(){};
    virtual ~ChBruteForce() { Clear(); };

    /// Clears all data instanced by this algorithm.
    virtual void Clear(void) {
        m_models.clear();

        while (!m_reported.empty()) {
            std::list<ChMates<model_type>*>::iterator mates = m_reported.begin();
            delete *mates;
            m_reported.erase(mates);
        }
        m_reported.clear();  // unuseful?
    }

    /// Adds a collision model to the broad-phase
    /// engine (custom data may be allocated).
    virtual void Add(model_type* model) {
        std::list<model_type*>::iterator pastmodel = m_models.begin();
        while (pastmodel != m_models.end()) {
            ChMates<model_type>* newmates = new ChMates<model_type>(*pastmodel, model);
            m_reported.push_back(newmates);
            pastmodel++;
        }

        m_models.push_back(model);
    }

    /// Removes a single collision model from the broad-phase
    /// engine (custom data may be deallocated).
    //***TO CHECK***
    virtual void Remove(model_type* model) {
        m_models.remove(model);

        ChMatesPtrList::iterator imate = m_reported.begin();
        while (imate != m_reported.end()) {
            if (((*imate)->GetModelA() == model) || ((*imate)->GetModelB() == model)) {
                ChMatesPtrList::iterator idelmate = imate;
                imate++;
                delete (*idelmate);
                m_reported.erase(idelmate);
            } else
                imate++;
        }
    }

    /// Run the 'brute force' algorithm, which finds the
    /// overlapping pairs of bounding boxes
    virtual void Run(){// nothing to do because overlaps are already instanced at 'Add() time..
    };

    /// After the broad phase algorithm has run, you can use
    /// this method to access the list with the reported pairs of
    /// overlapping models.
    virtual ChMatesPtrList& GetReportedMates() { return m_reported; };
};

}  // end namespace collision
}  // end namespace chrono

#endif
