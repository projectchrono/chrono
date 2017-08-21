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
    /// Default constructor
    ChConstraintTwoTuples() {}

    /// Copy constructor
    ChConstraintTwoTuples(const ChConstraintTwoTuples& other) : tuple_a(other.tuple_a), tuple_b(other.tuple_b) {}

    virtual ~ChConstraintTwoTuples() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuples* Clone() const override { return new ChConstraintTwoTuples(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoTuples& operator=(const ChConstraintTwoTuples& other) {
        tuple_a = other.tuple_a;
        tuple_b = other.tuple_b;
        return *this;
    }

    /// Access tuple a
    type_constraint_tuple_a& Get_tuple_a() { return tuple_a; }
    /// Access tuple b
    type_constraint_tuple_b& Get_tuple_b() { return tuple_b; }

    virtual void Update_auxiliary() override {
        g_i = 0;
        tuple_a.Update_auxiliary(g_i);
        tuple_b.Update_auxiliary(g_i);
        //  adds the constraint force mixing term (usually zero):
        if (cfm_i)
            g_i += cfm_i;
    }

    /// This function must computes the product between
    /// the row-jacobian of this constraint '[Cq_i]' and the
    /// vector of variables, 'v'. that is    CV=[Cq_i]*v
    /// This is used for some iterative solvers.
    virtual double Compute_Cq_q() override {
        double ret = 0;
        ret += tuple_a.Compute_Cq_q();
        ret += tuple_b.Compute_Cq_q();
        return ret;
    }

    ///  This function must increment the vector of variables
    /// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
    ///  v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
    ///  This is used for some iterative solvers.
    virtual void Increment_q(const double deltal) override {
        tuple_a.Increment_q(deltal);
        tuple_b.Increment_q(deltal);
    }

    /// Computes the product of the corresponding block in the
    /// system matrix by 'vect', and add to 'result'.
    /// NOTE: the 'vect' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const override {
        tuple_a.MultiplyAndAdd(result, vect);
        tuple_b.MultiplyAndAdd(result, vect);
    }

    /// Computes the product of the corresponding transposed blocks in the
    /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'.
    /// NOTE: the 'result' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyTandAdd(ChMatrix<double>& result, double l) override {
        tuple_a.MultiplyTandAdd(result, l);
        tuple_b.MultiplyTandAdd(result, l);
    }

    /// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
    /// where both portions of the jacobian are shifted in order to match the
    /// offset of the corresponding ChVariable.The same is done
    /// on the 'insrow' column, so that the sparse matrix is kept symmetric.
    virtual void Build_Cq(ChSparseMatrix& storage, int insrow) override {
        tuple_a.Build_Cq(storage, insrow);
        tuple_b.Build_Cq(storage, insrow);
    }
    virtual void Build_CqT(ChSparseMatrix& storage, int inscol) override {
        tuple_a.Build_CqT(storage, inscol);
        tuple_b.Build_CqT(storage, inscol);
    }
};

}  // end namespace chrono

#endif
