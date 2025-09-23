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

#ifndef CHCONSTRAINTNGENERIC_H
#define CHCONSTRAINTNGENERIC_H

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Constraint between N objects of type ChVariables().
/// Defines three constraint matrices, whose column number automatically matches the number of elements in variables
/// vectors. Before starting the solver one must provide the proper values in constraints (and update them if
/// necessary), i.e. must set at least the c_i and b_i values, and Jacobians.
class ChApi ChConstraintNgeneric : public ChConstraint {
  public:
    ChConstraintNgeneric() {}
    ChConstraintNgeneric(const ChConstraintNgeneric& other);
    virtual ~ChConstraintNgeneric() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintNgeneric* Clone() const override { return new ChConstraintNgeneric(*this); }

    /// Assignment operator: copy from other object.
    ChConstraintNgeneric& operator=(const ChConstraintNgeneric& other);

    /// Access the Nth Jacobian vector.
    ChRowVectorRef Get_Cq_N(size_t n) { return Cq[n]; }

    /// Access the Nth auxiliary vector (ex: used by iterative solvers).
    ChVectorRef Get_Eq_N(size_t n) { return Eq[n]; }

    /// Access the Nth variable object.
    ChVariables* GetVariables_N(size_t n) { return variables[n]; }

    /// Set references to the constrained ChVariables objects,automatically creating/resizing Jacobians as needed.
    void SetVariables(std::vector<ChVariables*> mvars);

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

    /// Write the transposed constraint Jacobian into the specified global matrix at the offsets of the associated
    /// variables. The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian
    /// in the provided matrix.
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    std::vector<ChVariables*> variables;

    std::vector<ChRowVectorDynamic<double>> Cq;  ///< [Cq] Jacobian slices
    std::vector<ChVectorDynamic<double>> Eq;     ///< [Eq] product [Eq]=[invM]*[Cq]'
};

}  // end namespace chrono

#endif
