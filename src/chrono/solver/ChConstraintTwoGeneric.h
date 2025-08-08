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

#ifndef CHCONSTRAINTTWOGENERIC_H
#define CHCONSTRAINTTWOGENERIC_H

#include "chrono/solver/ChConstraintTwo.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// This class implements the functionality for a constraint between a COUPLE of TWO objects of type ChVariables(), with
/// generic number of scalar variables each (ex.ChVariablesGeneric() or ChVariablesBody() ) and defines two constraint
/// matrices, whose column number automatically matches the number of elements in variables vectors.
/// Before starting the solver one must provide the proper values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChConstraintTwoGeneric : public ChConstraintTwo {
  protected:
    ChRowVectorDynamic<double> Cq_a;  ///< The [Cq_a] jacobian of the constraint
    ChRowVectorDynamic<double> Cq_b;  ///< The [Cq_b] jacobian of the constraint

    // Auxiliary data: will be used by iterative constraint solvers:

    ChVectorDynamic<double> Eq_a;  ///< The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
    ChVectorDynamic<double> Eq_b;  ///< The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'

  public:
    /// Default constructor
    ChConstraintTwoGeneric() {}

    /// Construct and immediately set references to variables
    ChConstraintTwoGeneric(ChVariables* mvariables_a, ChVariables* mvariables_b);

    /// Copy constructor
    ChConstraintTwoGeneric(const ChConstraintTwoGeneric& other);

    virtual ~ChConstraintTwoGeneric() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoGeneric* Clone() const override { return new ChConstraintTwoGeneric(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoGeneric& operator=(const ChConstraintTwoGeneric& other);

    /// Access Jacobian vector.
    virtual ChRowVectorRef Get_Cq_a() override { return Cq_a; }

    /// Access Jacobian vector.
    virtual ChRowVectorRef Get_Cq_b() override { return Cq_b; }

    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_a() override { return Eq_a; }

    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_b() override { return Eq_b; }

    /// Set references to the constrained ChVariables objects, automatically creating/resizing Jacobians as needed.
    virtual void SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b) override;

    /// This function updates the following auxiliary data:
    ///  - the Eq_a and Eq_b matrices
    ///  - the g_i product
    /// This is often called by solvers at the beginning of the solution process.
    virtual void UpdateAuxiliary() override;

    /// Compute the product between the Jacobian of this constraint, [Cq_i], and the vector of variables.
    /// In other words, perform the operation:
    /// <pre>
    ///   CV = [Cq_i] * v
    /// </pre>
    virtual double ComputeJacobianTimesState() override;

    /// Increment the vector of variables with the quantity [invM]*[Cq_i]'*deltal.
    /// In other words, perform the operation:
    /// <pre>
    ///    v += [invM] * [Cq_i]' * deltal
    /// or else
    ///    v+=[Eq_i] * deltal
    /// </pre>
    virtual void IncrementState(double deltal) override;

    /// Add the product of the corresponding block in the system matrix by 'vect' and add to result.
    /// Note: 'vect' is assumed to be of proper size; the procedure uses the ChVariable offsets to index in 'vect'.
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const override;

    /// Add the product of the corresponding transposed block in the system matrix by 'l' and add to result.
    /// Note: 'result' is assumed to be of proper size; the procedure uses the ChVariable offsets to index in 'result'.
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const override;

    /// Write the constraint Jacobian into the specified global matrix at the offsets of the associated variables.
    /// The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian in the
    /// provided matrix.
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const override;

    /// Write the transposed constraint Jacobian into the specified global matrix at the offsets of the associated
    /// variables. The (start_row, start_col) pair specifies the top-left corner of the system-level transposed
    /// constraint Jacobian in the provided matrix.
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

}  // end namespace chrono

#endif
