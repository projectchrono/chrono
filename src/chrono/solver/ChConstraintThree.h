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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONSTRAINTTHREE_H
#define CHCONSTRAINTTHREE_H

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// This class is inherited by the base ChConstraint(),
/// which does almost nothing. So here this class implements
/// the functionality for a constraint between a THREE
/// objects of type ChVariables(), and defines three constraint
/// matrices, whose column number automatically matches the number
/// of elements in variables vectors.
///  Before starting the solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChConstraintThree : public ChConstraint {
  protected:
    ChVariables* variables_a;  ///< The first  constrained object
    ChVariables* variables_b;  ///< The second constrained object
    ChVariables* variables_c;  ///< The third constrained object

  public:
    /// Default constructor
    ChConstraintThree() : variables_a(nullptr), variables_b(nullptr), variables_c(nullptr) {}

    /// Copy constructor
    ChConstraintThree(const ChConstraintThree& other);

    virtual ~ChConstraintThree() {}

    /// Assignment operator: copy from other object
    ChConstraintThree& operator=(const ChConstraintThree& other);

    /// Access jacobian vector.
    virtual ChRowVectorRef Get_Cq_a() = 0;
    /// Access jacobian vector.
    virtual ChRowVectorRef Get_Cq_b() = 0;
    /// Access jacobian vector.
    virtual ChRowVectorRef Get_Cq_c() = 0;

    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_a() = 0;
    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_b() = 0;
    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_c() = 0;

    /// Access the first variable object.
    ChVariables* GetVariables_a() { return variables_a; }
    /// Access the second variable object.
    ChVariables* GetVariables_b() { return variables_b; }
    /// Access the second variable object.
    ChVariables* GetVariables_c() { return variables_c; }

    /// Set references to the constrained objects, each of ChVariables type,
    /// automatically creating/resizing jacobians if needed.
    virtual void SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b, ChVariables* mvariables_c) = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
