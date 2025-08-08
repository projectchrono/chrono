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

#ifndef CH_CONSTRAINT_TWO_TUPLES_H
#define CH_CONSTRAINT_TWO_TUPLES_H

#include "chrono/solver/ChConstraintTuple.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Constraint combining two tuples, each referencing 1, 2, or 3 variable objects.
/// This allows representing a constraint between two objects with different number of variable sets.
/// Typically used for implementing unilateral contact constraints, this could represent for example a contact between
/// an edge with 2 "xyz" variables and a triangle with 3 "xyz" variables; in this case, the contact constraint Jacobian
/// is split in 2 + 3 chunks.
class ChApi ChConstraintTwoTuples : public ChConstraint {
  public:
    ChConstraintTwoTuples();
    ChConstraintTwoTuples(const ChConstraintTwoTuples& other);
    virtual ~ChConstraintTwoTuples();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuples* Clone() const override { return new ChConstraintTwoTuples(*this); }

    /// Assignment operator: copy from other object.
    ChConstraintTwoTuples& operator=(const ChConstraintTwoTuples& other);

    /// Access tuple a.
    ChConstraintTuple* TupleA() { return tuple_a; }

    /// Access tuple b.
    ChConstraintTuple* TupleB() { return tuple_b; }

    /// Set the two tuples.
    void SetTuples(ChConstraintTuple* tupleA, ChConstraintTuple* tupleB);

    /// Set the two tuples from two contactable objects.
    void SetTuplesFromContactables(ChContactable* objA, ChContactable* objB);

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
    /// variables. The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian
    /// in the provided matrix.
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const override;

  protected:
    // Note: the two constraint tuples are managed by this container
    ChConstraintTuple* tuple_a;
    ChConstraintTuple* tuple_b;
};

}  // end namespace chrono

#endif
