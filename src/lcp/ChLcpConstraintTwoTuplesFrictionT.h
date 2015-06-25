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

#ifndef CHLCPCONSTRAINTTWOTUPLESFRICTIONT_H
#define CHLCPCONSTRAINTTWOTUPLESFRICTIONT_H


#include "ChLcpConstraintTwoTuples.h"

namespace chrono {

/// Base class for friction constraints between two objects,
/// each represented by a tuple of ChVariables objects.
/// This constraint cannot be used alone. It must be used together with 
/// a ChLcpConstraintTwoTuplesContactN

template <class Ta, class Tb >
class ChApi ChLcpConstraintTwoTuplesFrictionT : public ChLcpConstraintTwoTuples< Ta, Tb >{

    //
    // DATA
    //

  protected:
  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChLcpConstraintTwoTuplesFrictionT() { mode = CONSTRAINT_FRIC; };


    /// Copy constructor
    ChLcpConstraintTwoTuplesFrictionT(const ChLcpConstraintTwoTuplesFrictionT& other) : ChLcpConstraintTwoTuples< Ta, Tb >(other) {}

    virtual ~ChLcpConstraintTwoTuplesFrictionT(){};

    virtual ChLcpConstraint* new_Duplicate() { return new ChLcpConstraintTwoTuplesFrictionT(*this); };

    /// Assignment operator: copy from other object
    ChLcpConstraintTwoTuplesFrictionT& operator=(const ChLcpConstraintTwoTuplesFrictionT& other) {
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

}  // END_OF_NAMESPACE____

#endif  // END of ChLcpConstraintTwoFrictionT.h
