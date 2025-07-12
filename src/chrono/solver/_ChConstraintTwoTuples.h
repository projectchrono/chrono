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

#ifndef CHCONSTRAINTTWOTUPLES_H
#define CHCONSTRAINTTWOTUPLES_H

#include "chrono/solver/ChConstraintTuple.h"

namespace chrono {

/// This constraint is built on top of two ChConstraintTuple objects, each with a tuple of
/// 1 or 2 or 3 differently-sized jacobian chunks. For instance, this might
/// happen because you want a constraint between an edge (i.e. two xyz variables, each
/// per end nodes) and a triangle face (i.e. three xyz variables, each per corner), so
/// the jacobian row matrix is split in 2 + 3 chunks, here as two tuples.
/// Templates Ta and Tb are of ChVariableTupleCarrier_Nvars classes

template <class Ta, class Tb>
class ChConstraintTwoTuples : public ChConstraint {
    typedef typename Ta::type_constraint_tuple type_constraint_tuple_a;
    typedef typename Tb::type_constraint_tuple type_constraint_tuple_b;

  protected:
    type_constraint_tuple_a tuple_a;
    type_constraint_tuple_b tuple_b;

  public:
    ChConstraintTwoTuples() {}
    ChConstraintTwoTuples(const ChConstraintTwoTuples& other) : tuple_a(other.tuple_a), tuple_b(other.tuple_b) {}
    virtual ~ChConstraintTwoTuples() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuples* Clone() const override { return new ChConstraintTwoTuples(*this); }

    /// Assignment operator: copy from other object.
    ChConstraintTwoTuples& operator=(const ChConstraintTwoTuples& other) {
        tuple_a = other.tuple_a;
        tuple_b = other.tuple_b;
        return *this;
    }

    /// Access tuple a.
    type_constraint_tuple_a& Get_tuple_a() { return tuple_a; }

    /// Access tuple b.
    type_constraint_tuple_b& Get_tuple_b() { return tuple_b; }

    virtual void Update_auxiliary() override {
        g_i = 0;
        tuple_a.Update_auxiliary(g_i);
        tuple_b.Update_auxiliary(g_i);
        //  adds the constraint force mixing term (usually zero):
        if (cfm_i != 0)
            g_i += cfm_i;
    }

    /// Compute the product between the Jacobian of this constraint, [Cq_i], and the vector of variables.
    /// In other words, perform the operation:
    /// <pre>
    ///   CV = [Cq_i] * v
    /// </pre>
    virtual double ComputeJacobianTimesState() override {
        double ret = 0;
        ret += tuple_a.ComputeJacobianTimesState();
        ret += tuple_b.ComputeJacobianTimesState();
        return ret;
    }

    /// Increment the vector of variables with the quantity [invM]*[Cq_i]'*deltal.
    /// In other words, perform the operation:
    /// <pre>
    ///    v += [invM] * [Cq_i]' * deltal
    /// or else
    ///    v+=[Eq_i] * deltal
    /// </pre>
    virtual void IncrementState(double deltal) override {
        tuple_a.IncrementState(deltal);
        tuple_b.IncrementState(deltal);
    }

    /// Add the product of the corresponding block in the system matrix by 'vect' and add to result.
    /// Note: 'vect' is assumed to be of proper size; the procedure uses the ChVariable offsets to index in 'vect'.
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const override {
        tuple_a.AddJacobianTimesVectorInto(result, vect);
        tuple_b.AddJacobianTimesVectorInto(result, vect);
    }

    /// Add the product of the corresponding transposed block in the system matrix by 'l' and add to result.
    /// Note: 'result' is assumed to be of proper size; the procedure uses the ChVariable offsets to index in 'result'.
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const override {
        tuple_a.AddJacobianTransposedTimesScalarInto(result, l);
        tuple_b.AddJacobianTransposedTimesScalarInto(result, l);
    }

    /// Write the constraint Jacobian into the specified global matrix at the offsets of the associated variables.
    /// The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian in the
    /// provided matrix.
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const override {
        tuple_a.PasteJacobianInto(mat, start_row, start_col);
        tuple_b.PasteJacobianInto(mat, start_row, start_col);
    }

    /// Write the transposed constraint Jacobian into the specified global matrix at the offsets of the associated
    /// variables. The (start_row, start_col) pair specifies the top-left corner of the system-level constraint Jacobian
    /// in the provided matrix.
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const override {
        tuple_a.PasteJacobianTransposedInto(mat, start_row, start_col);
        tuple_b.PasteJacobianTransposedInto(mat, start_row, start_col);
    }
};

}  // end namespace chrono

#endif
