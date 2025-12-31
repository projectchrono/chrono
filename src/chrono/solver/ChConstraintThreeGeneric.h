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

#ifndef CHCONSTRAINTTHREEGENERIC_H
#define CHCONSTRAINTTHREEGENERIC_H

#include "chrono/solver/ChConstraintThree.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Constraint between three objects of type ChVariables(), with generic number of scalar variables each.
/// The constraint variables can be of type ChVariablesGeneric() or ChVariablesBody().
///  Defines three Jacobian matrices, whose column number automatically matches the number of elements in variables
///  vectors.
/// Before starting the solver one must provide the proper values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and Jacobians.
class ChApi ChConstraintThreeGeneric : public ChConstraintThree {
  public:
    ChConstraintThreeGeneric();
    ChConstraintThreeGeneric(ChVariables* mvariables_a, ChVariables* mvariables_b, ChVariables* mvariables_c);
    ChConstraintThreeGeneric(const ChConstraintThreeGeneric& other);
    virtual ~ChConstraintThreeGeneric() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintThreeGeneric* Clone() const override { return new ChConstraintThreeGeneric(*this); }

    /// Assignment operator: copy from other object
    ChConstraintThreeGeneric& operator=(const ChConstraintThreeGeneric& other);

    /// Access Jacobian vector.
    virtual ChRowVectorRef Get_Cq_a() override { return Cq_a; }

    /// Access Jacobian vector.
    virtual ChRowVectorRef Get_Cq_b() override { return Cq_b; }

    /// Access Jacobian vector.
    virtual ChRowVectorRef Get_Cq_c() override { return Cq_c; }

    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_a() override { return Eq_a; }
    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_b() override { return Eq_b; }
    /// Access auxiliary vector (ex: used by iterative solvers).
    virtual ChVectorRef Get_Eq_c() override { return Eq_c; }

    /// Set references to the constrained ChVariables objects,automatically creating/resizing Jacobians as needed.
    virtual void SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b, ChVariables* mvariables_c) override;

    /// This function updates the following auxiliary data:
    ///  - the Eq_a and Eq_b matrices
    ///  - the g_i product
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
    virtual void PasteJacobianInto(ChSparseMatrix& mat,
                                   unsigned int start_row,
                                   unsigned int start_col) const override;

    /// Write the transposed Jacobian into the specified global matrix at the offsets of the associated variables.
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    ChRowVectorDynamic<double> Cq_a;  ///< [Cq_a] Jacobian of the constraint
    ChRowVectorDynamic<double> Cq_b;  ///< [Cq_b] Jacobian of the constraint
    ChRowVectorDynamic<double> Cq_c;  ///< [Cq_c] Jacobian of the constraint

    // Auxiliary data: will be used by iterative constraint solvers:

    ChVectorDynamic<double> Eq_a;  ///< [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
    ChVectorDynamic<double> Eq_b;  ///< [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'
    ChVectorDynamic<double> Eq_c;  ///< [Eq_a] product [Eq_c]=[invM_b]*[Cq_c]'
};

}  // end namespace chrono

#endif
