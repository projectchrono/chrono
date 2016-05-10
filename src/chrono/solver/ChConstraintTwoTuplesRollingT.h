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

#ifndef CHLCPCONSTRAINTTWOTUPLESROLLINGT_H
#define CHLCPCONSTRAINTTWOTUPLESROLLINGT_H


#include "chrono/solver/ChConstraintTwoTuples.h"

namespace chrono {

/// This is enough to use dynamic_casting<> to detect all template types
/// from ChLcpConstraintTwoTuplesRollingT
class ChApi ChLcpConstraintTwoTuplesRollingTall {
};

/// Base class for friction constraints between two objects,
/// each represented by a tuple of ChVariables objects.
/// This constraint cannot be used alone. It must be used together with 
/// a ChLcpConstraintTwoTuplesContactN

template <class Ta, class Tb >
class ChApi ChLcpConstraintTwoTuplesRollingT : 
                public ChLcpConstraintTwoTuples< Ta, Tb >, 
                public ChLcpConstraintTwoTuplesRollingTall {

    //
    // DATA
    //

  protected:
  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChLcpConstraintTwoTuplesRollingT() { this->mode = CONSTRAINT_FRIC; };


    /// Copy constructor
    ChLcpConstraintTwoTuplesRollingT(const ChLcpConstraintTwoTuplesRollingT& other) : ChLcpConstraintTwoTuples< Ta, Tb >(other) {}

    virtual ~ChLcpConstraintTwoTuplesRollingT(){};

    virtual ChLcpConstraint* new_Duplicate() { return new ChLcpConstraintTwoTuplesRollingT(*this); };

    /// Assignment operator: copy from other object
    ChLcpConstraintTwoTuplesRollingT& operator=(const ChLcpConstraintTwoTuplesRollingT& other) {
        if (&other == this)
            return *this;

        // copy parent class data
        ChLcpConstraintTwoTuples< Ta, Tb >::operator=(other);

        return *this;
    }

    //
    // FUNCTIONS
    //

    /// Tells that this constraint is not linear, that is: it cannot
    /// be solved with a plain simplex solver.
    virtual bool IsLinear() const { return false; }

    /// The constraint is satisfied?
    virtual double Violation(double mc_i) { return 0.0;}

};

}  // end namespace chrono

#endif  
