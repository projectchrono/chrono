//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONSTRAINTNODEFRICTIONT_H
#define CHCONSTRAINTNODEFRICTIONT_H

#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChVariablesBody.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

/// Class used to represent friction constraint
/// between a 3DOF node and a 6DOF body.

class ChApi ChConstraintNodeFrictionT : public ChConstraintTwoGeneric {
    CH_RTTI(ChConstraintNodeFrictionT, ChConstraintTwoGeneric)

    //
    // DATA
    //

  protected:
  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChConstraintNodeFrictionT() { mode = CONSTRAINT_FRIC; }

    /// Construct and immediately set references to variables,
    /// also setting the  and the normal constraint
    /// other tangential constraint (the latter is mandatory only
    /// for the second of the two tangential constraints)
    ChConstraintNodeFrictionT(ChVariablesBody* mvariables_a, ChVariablesNode* mvariables_b)
        : ChConstraintTwoGeneric(mvariables_a, mvariables_b) {
        mode = CONSTRAINT_FRIC;
    }

    /// Copy constructor
    ChConstraintNodeFrictionT(const ChConstraintNodeFrictionT& other) : ChConstraintTwoGeneric(other) {}

    virtual ~ChConstraintNodeFrictionT() {}

    virtual ChConstraint* new_Duplicate() { return new ChConstraintNodeFrictionT(*this); };

    /// Assignment operator: copy from other object
    ChConstraintNodeFrictionT& operator=(const ChConstraintNodeFrictionT& other) {
        if (&other == this)
            return *this;

        // copy parent class data
        ChConstraintTwoGeneric::operator=(other);

        return *this;
    }

    //
    // FUNCTIONS
    //

    /// Tells that this constraint is not linear, that is: it cannot
    /// be solved with a plain simplex solver.
    virtual bool IsLinear() const { return false; }

    /// The constraint is satisfied?
    virtual double Violation(double mc_i);

    //
    // STREAMING
    //

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

}  // end namespace chrono

#endif
