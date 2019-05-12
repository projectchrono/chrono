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

#ifndef CHCONSTRAINTTWOBODIES_H
#define CHCONSTRAINTTWOBODIES_H

#include "chrono/solver/ChConstraintTwo.h"
#include "chrono/solver/ChVariablesBody.h"

namespace chrono {

/// This class inherits from the base ChConstraintTwo(),
/// that implements the functionality for a constraint between
/// a couple of two objects of type ChVariablesBody().

class ChApi ChConstraintTwoBodies : public ChConstraintTwo {
  protected:
    ChMatrixNM<double, 1, 6> Cq_a;  ///< The [Cq_a] jacobian of the constraint
    ChMatrixNM<double, 1, 6> Cq_b;  ///< The [Cq_b] jacobian of the constraint

    // Auxiliary data: will be used by iterative constraint solvers:

    ChMatrixNM<double, 6, 1> Eq_a;  ///< The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
    ChMatrixNM<double, 6, 1> Eq_b;  ///< The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'

  public:
    /// Default constructor
    ChConstraintTwoBodies();

    /// Construct and immediately set references to variables
    ChConstraintTwoBodies(ChVariablesBody* mvariables_a, ChVariablesBody* mvariables_b);

    /// Copy constructor
    ChConstraintTwoBodies(const ChConstraintTwoBodies& other);

    virtual ~ChConstraintTwoBodies() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoBodies* Clone() const override { return new ChConstraintTwoBodies(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoBodies& operator=(const ChConstraintTwoBodies& other);

    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_a() override { return &Cq_a; }
    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_b() override { return &Cq_b; }

    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_a() override { return &Eq_a; }
    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_b() override { return &Eq_b; }

    /// Set references to the constrained objects, each of ChVariablesBody type,
    /// automatically creating/resizing jacobians if needed.
    /// If variables aren't from ChVariablesBody class, an assert failure happens.
    void SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b) override;

    /// This function updates the following auxiliary data:
    ///  - the Eq_a and Eq_b matrices
    ///  - the g_i product
    /// This is often called by solvers at the beginning
    /// of the solution process.
    /// Most often, inherited classes won't need to override this.
    virtual void Update_auxiliary() override;

    ///  This function must computes the product between
    /// the row-jacobian of this constraint '[Cq_i]' and the
    /// vector of variables, 'v'. that is    CV=[Cq_i]*v
    ///  This is used for some iterative solvers.
    virtual double Compute_Cq_q() override;

    ///  This function must increment the vector of variables
    /// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
    ///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
    ///  This is used for some iterative solvers.
    virtual void Increment_q(const double deltal) override;

    /// Computes the product of the corresponding block in the
    /// system matrix by 'vect', and add to 'result'.
    /// NOTE: the 'vect' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const override;

    /// Computes the product of the corresponding transposed blocks in the
    /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'.
    /// NOTE: the 'result' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyTandAdd(ChMatrix<double>& result, double l) override;

    /// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
    /// where both portions of the jacobian are shifted in order to match the
    /// offset of the corresponding ChVariable.The same is done
    /// on the 'insrow' column, so that the sparse matrix is kept symmetric.
    virtual void Build_Cq(ChSparseMatrix& storage, int insrow) override;
    virtual void Build_CqT(ChSparseMatrix& storage, int inscol) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
