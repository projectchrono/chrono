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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

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

  public:
    /// Default constructor
    ChConstraintNodeFrictionT();

    /// Construct and immediately set references to variables,
    /// also setting the  and the normal constraint
    /// other tangential constraint (the latter is mandatory only
    /// for the second of the two tangential constraints)
    ChConstraintNodeFrictionT(ChVariablesBody* mvariables_a, ChVariablesNode* mvariables_b);

    /// Copy constructor
    ChConstraintNodeFrictionT(const ChConstraintNodeFrictionT& other) : ChConstraintTwoGeneric(other) {}

    virtual ~ChConstraintNodeFrictionT() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintNodeFrictionT* Clone() const override { return new ChConstraintNodeFrictionT(*this); }

    /// Assignment operator: copy from other object
    ChConstraintNodeFrictionT& operator=(const ChConstraintNodeFrictionT& other);

    /// Indicate whether or not this constraint is linear.
    virtual bool IsLinear() const override { return false; }

    /// The constraint is satisfied?
    virtual double Violation(double mc_i) override;

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

}  // end namespace chrono

#endif
