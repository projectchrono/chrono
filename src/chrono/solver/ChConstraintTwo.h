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

#ifndef CHCONSTRAINTTWO_H
#define CHCONSTRAINTTWO_H

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// This class is inherited by the base ChConstraint(),
/// which does almost nothing. So here this class implements
/// the functionality for a constrint between a COUPLE of TWO
/// objects of type ChVariables(), and defines two constraint
/// matrices, whose column number automatically matches the number
/// of elements in variables vectors.
///  Before starting the solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChConstraintTwo : public ChConstraint {
    CH_RTTI(ChConstraintTwo, ChConstraint)

    //
    // DATA
    //

  protected:
    /// The first  constrained object
    ChLcpVariables* variables_a;
    /// The second constrained object
    ChLcpVariables* variables_b;

  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChConstraintTwo() { variables_a = variables_b = NULL; }

    /// Copy constructor
    ChConstraintTwo(const ChConstraintTwo& other) : ChConstraint(other) {
        variables_a = other.variables_a;
        variables_b = other.variables_b;
    }

    virtual ~ChConstraintTwo(){};

    /// Assignment operator: copy from other object
    ChConstraintTwo& operator=(const ChConstraintTwo& other);

    //
    // FUNCTIONS
    //

    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_a() = 0;
    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_b() = 0;

    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_a() = 0;
    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_b() = 0;

    /// Access the first variable object
    ChLcpVariables* GetVariables_a() { return variables_a; }
    /// Access the second variable object
    ChLcpVariables* GetVariables_b() { return variables_b; }

    /// Set references to the constrained objects, each of ChVariables type,
    /// automatically creating/resizing jacobians if needed.
    virtual void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b) = 0;

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
