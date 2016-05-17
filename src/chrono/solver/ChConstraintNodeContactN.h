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

#ifndef CHCONSTRAINTNODECONTACTN_H
#define CHCONSTRAINTNODECONTACTN_H

#include "chrono/solver/ChConstraintNodeFrictionT.h"

namespace chrono {

/// It is used to represent the normal reaction between a 3DOF node
/// and a 6 DOF body, only when also two ChConstraintNodeFrictionT
/// objects are used to represent friction.

class ChApi ChConstraintNodeContactN : public ChConstraintTwoGeneric {
    CH_RTTI(ChConstraintNodeContactN, ChConstraintTwoGeneric)

  protected:
    float friction;                           ///< the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
    ChConstraintNodeFrictionT* constraint_U;  ///< the pointer to U tangential component
    ChConstraintNodeFrictionT* constraint_V;  ///< the pointer to V tangential component

  public:
    /// Default constructor
    ChConstraintNodeContactN();

    /// Construct and immediately set references to variables,
    /// also setting the U and V tangential friction constraints
    ChConstraintNodeContactN(ChVariablesBody* mvariables_a,
                             ChVariablesNode* mvariables_b,
                             ChConstraintNodeFrictionT* aU = 0,
                             ChConstraintNodeFrictionT* aV = 0);

    /// Copy constructor
    ChConstraintNodeContactN(const ChConstraintNodeContactN& other);

    virtual ~ChConstraintNodeContactN() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintNodeContactN* Clone() const override { return new ChConstraintNodeContactN(*this); }

    /// Assignment operator: copy from other object
    ChConstraintNodeContactN& operator=(const ChConstraintNodeContactN& other);

    /// Get the friction coefficient
    float GetFrictionCoefficient() { return friction; }
    /// Set the friction coefficient
    void SetFrictionCoefficient(float mcoeff) { friction = mcoeff; }

    /// Get pointer to U tangential component
    ChConstraintNodeFrictionT* GetTangentialConstraintU() { return constraint_U; }
    /// Get pointer to V tangential component
    ChConstraintNodeFrictionT* GetTangentialConstraintV() { return constraint_V; }

    /// Set pointer to U tangential component
    void SetTangentialConstraintU(ChConstraintNodeFrictionT* mconstr) { constraint_U = mconstr; }
    /// Set pointer to V tangential component
    void SetTangentialConstraintV(ChConstraintNodeFrictionT* mconstr) { constraint_V = mconstr; }

    /// For iterative solvers: project the value of a possible
    /// 'l_i' value of constraint reaction onto admissible set.
    /// This projection will also modify the l_i values of the two
    /// tangential friction constraints (projection onto the friction cone,
    /// as by Anitescu-Tasora theory).
    virtual void Project() override;

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

}  // end namespace chrono

#endif
