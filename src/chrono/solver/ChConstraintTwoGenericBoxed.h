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

#ifndef CHCONSTRAINTTWOGENERICBOXED_H
#define CHCONSTRAINTTWOGENERICBOXED_H

#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

/// This class is inherited from the base ChConstraintTwoGeneric,
/// which can be used for most pairwise constraints, and adds the
/// feature that the multiplier must be
///        l_min < l < l_max
/// that is, the multiplier is 'boxed'.
/// Note that, if l_min = 0 and l_max = infinite, this can work
/// also as an unilateral constraint..
///  Before starting the solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChConstraintTwoGenericBoxed : public ChConstraintTwoGeneric {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChConstraintTwoGenericBoxed)

  protected:
    double l_min;
    double l_max;

  public:
    /// Default constructor
    ChConstraintTwoGenericBoxed() : l_min(-1), l_max(1) {}

    /// Construct and immediately set references to variables
    ChConstraintTwoGenericBoxed(ChVariables* mvariables_a, ChVariables* mvariables_b)
        : ChConstraintTwoGeneric(mvariables_a, mvariables_b), l_min(-1), l_max(1) {}

    /// Copy constructor
    ChConstraintTwoGenericBoxed(const ChConstraintTwoGenericBoxed& other);

    virtual ~ChConstraintTwoGenericBoxed() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoGenericBoxed* Clone() const override { return new ChConstraintTwoGenericBoxed(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoGenericBoxed& operator=(const ChConstraintTwoGenericBoxed& other);

    /// Set lower/upper limit for the multiplier.
    void SetBoxedMinMax(double mmin, double mmax);

    /// Get the lower limit for the multiplier
    double GetBoxedMin() { return l_min; }
    /// Get the upper limit for the multiplier
    double GetBoxedMax() { return l_max; }

    /// For iterative solvers: project the value of a possible
    /// 'l_i' value of constraint reaction onto admissible orthant/set.
    /// This 'boxed implementation overrides the default do-nothing case.
    virtual void Project() override;

    /// Given the residual of the constraint computed as the
    /// linear map  mc_i =  [Cq]*q + b_i + cfm*l_i , returns the
    /// violation of the constraint, considering inequalities, etc.
    virtual double Violation(double mc_i) override;

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream) override;

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream) override;
};

}  // end namespace chrono

#endif
