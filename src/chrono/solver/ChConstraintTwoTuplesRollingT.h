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

#ifndef CHCONSTRAINTTWOTUPLESROLLINGT_H
#define CHCONSTRAINTTWOTUPLESROLLINGT_H

#include "chrono/solver/ChConstraintTwoTuples.h"

namespace chrono {

/// This is enough to use dynamic_casting<> to detect all template types
/// from ChConstraintTwoTuplesRollingT
class ChApi ChConstraintTwoTuplesRollingTall {};

/// Base class for friction constraints between two objects,
/// each represented by a tuple of ChVariables objects.
/// This constraint cannot be used alone. It must be used together with
/// a ChConstraintTwoTuplesContactN

template <class Ta, class Tb>
class ChApi ChConstraintTwoTuplesRollingT : public ChConstraintTwoTuples<Ta, Tb>,
                                            public ChConstraintTwoTuplesRollingTall {
  public:
    /// Default constructor
    ChConstraintTwoTuplesRollingT() { this->mode = CONSTRAINT_FRIC; }

    /// Copy constructor
    ChConstraintTwoTuplesRollingT(const ChConstraintTwoTuplesRollingT& other) : ChConstraintTwoTuples<Ta, Tb>(other) {}

    virtual ~ChConstraintTwoTuplesRollingT() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuplesRollingT* Clone() const override { return new ChConstraintTwoTuplesRollingT(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoTuplesRollingT& operator=(const ChConstraintTwoTuplesRollingT& other) {
        if (&other == this)
            return *this;

        // copy parent class data
        ChConstraintTwoTuples<Ta, Tb>::operator=(other);

        return *this;
    }

    /// Tells that this constraint is not linear, that is: it cannot
    /// be solved with a plain simplex solver.
    virtual bool IsLinear() const { return false; }

    /// The constraint is satisfied?
    virtual double Violation(double mc_i) { return 0.0; }
};

}  // end namespace chrono

#endif
