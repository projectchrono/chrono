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

#ifndef CHLCPCONSTRAINTTWOTUPLES_H
#define CHLCPCONSTRAINTTWOTUPLES_H



#include "ChLcpConstraintTuple.h"

namespace chrono {


/// This constraint is built on top of two ChConstraintTuple objects, each with a tuple of
/// 1 or 2 or 3 differently-sized jacobian chunks. For instance, this might
/// happen because you want a constraint between an edge (i.e. two xyz variables, each
/// per end nodes) and a triangle face (i.e. three xyz variables, each per corner), so
/// the jacobian row matrix is split in 2 + 3 chunks, here as two tuples.
/// Templates Ta and Tb are of ChLcpVariableTupleCarrier_Nvars classes

template <class Ta, class Tb>
class ChApi ChLcpConstraintTwoTuples : public ChLcpConstraint {

    //CH_RTTI(ChLcpConstraintTwoTuples, ChLcpConstraint)

    typedef typename Ta::type_constraint_tuple type_constraint_tuple_a;
    typedef typename Tb::type_constraint_tuple type_constraint_tuple_b;

    //
    // DATA
    //

  protected:
    type_constraint_tuple_a tuple_a;
    type_constraint_tuple_b tuple_b;

  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChLcpConstraintTwoTuples(){};

    /// Copy constructor
    ChLcpConstraintTwoTuples(const ChLcpConstraintTwoTuples& other) : 
        tuple_a(other.tuple_a),
        tuple_b(other.tuple_b) {}

    virtual ~ChLcpConstraintTwoTuples(){};

    virtual ChLcpConstraint* new_Duplicate() { return new ChLcpConstraintTwoTuples(*this); }

    /// Assignment operator: copy from other object
    ChLcpConstraintTwoTuples& operator=(const ChLcpConstraintTwoTuples& other) {
        tuple_a = other.tuple_a;
        tuple_b = other.tuple_b;
    }

    //
    // FUNCTIONS
    //

    /// Access tuple a
    type_constraint_tuple_a& Get_tuple_a() { return tuple_a; }
    /// Access tuple b
    type_constraint_tuple_b& Get_tuple_b() { return tuple_b; }

    virtual void Update_auxiliary() {
        g_i = 0;
        tuple_a.Update_auxiliary(g_i);
        tuple_b.Update_auxiliary(g_i);
        //  adds the constraint force mixing term (usually zero):
        if (cfm_i)
            g_i += cfm_i;
    }

    ///  This function must computes the product between
    /// the row-jacobian of this constraint '[Cq_i]' and the
    /// vector of variables, 'v'. that is    CV=[Cq_i]*v
    ///  This is used for some iterative LCP solvers.
    virtual double Compute_Cq_q() {
        double ret = 0;
        ret += tuple_a.Compute_Cq_q();
        ret += tuple_b.Compute_Cq_q();
        return ret;
    }

    ///  This function must increment the vector of variables
    /// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
    ///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
    ///  This is used for some iterative LCP solvers.

    virtual void Increment_q(const double deltal) {
        tuple_a.Increment_q(deltal);
        tuple_b.Increment_q(deltal);
    };

    /// Computes the product of the corresponding block in the
    /// system matrix by 'vect', and add to 'result'.
    /// NOTE: the 'vect' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
        tuple_a.MultiplyAndAdd(result, vect);
        tuple_b.MultiplyAndAdd(result, vect);
    };

    /// Computes the product of the corresponding transposed blocks in the
    /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'.
    /// NOTE: the 'result' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyTandAdd(ChMatrix<double>& result, double l) {
        tuple_a.MultiplyTandAdd(result, l);
        tuple_b.MultiplyTandAdd(result, l);
    };

    /// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
    /// where both portions of the jacobian are shifted in order to match the
    /// offset of the corresponding ChLcpVariable.The same is done
    /// on the 'insrow' column, so that the sparse matrix is kept symmetric.
    /// This is used only by the ChLcpSimplex solver (iterative solvers
    /// don't need to know jacobians explicitly)
    virtual void Build_Cq(ChSparseMatrix& storage, int insrow) {
        tuple_a.Build_Cq(storage, insrow);
        tuple_b.Build_Cq(storage, insrow);
    }
    virtual void Build_CqT(ChSparseMatrix& storage, int inscol) {
        tuple_a.Build_CqT(storage, inscol);
        tuple_b.Build_CqT(storage, inscol);
    }
};



}  // END_OF_NAMESPACE____

#endif  
