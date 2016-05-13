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

#ifndef CHCONSTRAINTTWOTUPLESFRICTIONT_H
#define CHCONSTRAINTTWOTUPLESFRICTIONT_H

#include "chrono/solver/ChConstraintTwoTuples.h"

namespace chrono {

/// This is enough to use dynamic_casting<> to detect all template types
/// from ChConstraintTwoTuplesFrictionT
class ChApi ChConstraintTwoTuplesFrictionTall {};

/// Base class for friction constraints between two objects,
/// each represented by a tuple of ChVariables objects.
/// This constraint cannot be used alone. It must be used together with
/// a ChConstraintTwoTuplesContactN

template <class Ta, class Tb>
class ChApi ChConstraintTwoTuplesFrictionT : public ChConstraintTwoTuples<Ta, Tb>,
                                             public ChConstraintTwoTuplesFrictionTall {
  public:
    /// Default constructor
    ChConstraintTwoTuplesFrictionT() { this->mode = CONSTRAINT_FRIC; }

    /// Copy constructor
    ChConstraintTwoTuplesFrictionT(const ChConstraintTwoTuplesFrictionT& other)
        : ChConstraintTwoTuples<Ta, Tb>(other) {}

    virtual ~ChConstraintTwoTuplesFrictionT() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuplesFrictionT* Clone() const override { return new ChConstraintTwoTuplesFrictionT(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoTuplesFrictionT& operator=(const ChConstraintTwoTuplesFrictionT& other) {
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
    virtual double Violation(double mc_i) { return 0; }
};

}  // end namespace chrono

#endif
