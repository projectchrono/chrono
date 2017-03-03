// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONSTRAINTTHREEBBSHAFT_H
#define CHCONSTRAINTTHREEBBSHAFT_H

#include "chrono/solver/ChConstraintThree.h"
#include "chrono/solver/ChVariablesBody.h"

namespace chrono {

/// A class for representing a constraint between
/// two bodies (2x6dof in space) and a 1D dof (a shaft)

class ChApi ChConstraintThreeBBShaft : public ChConstraintThree {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChConstraintThreeBBShaft)

  protected:
    ChMatrixNM<double, 1, 6> Cq_a;  ///< The [Cq_a] jacobian of the constraint
    ChMatrixNM<double, 1, 6> Cq_b;  ///< The [Cq_b] jacobian of the constraint
    ChMatrixNM<double, 1, 1> Cq_c;  ///< The [Cq_c] jacobian of the constraint

    // Auxiliary data: will be used by iterative constraint solvers:

    ChMatrixNM<double, 6, 1> Eq_a;  ///< The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
    ChMatrixNM<double, 6, 1> Eq_b;  ///< The [Eq_b] product [Eq_b]=[invM_b]*[Cq_b]'
    ChMatrixNM<double, 1, 1> Eq_c;  ///< The [Eq_c] product [Eq_c]=[invM_c]*[Cq_c]'

  public:
    /// Default constructor
    ChConstraintThreeBBShaft() {}

    /// Construct and immediately set references to variables
    ChConstraintThreeBBShaft(ChVariablesBody* mvariables_a, ChVariablesBody* mvariables_b, ChVariables* mvariables_c);

    /// Copy constructor
    ChConstraintThreeBBShaft(const ChConstraintThreeBBShaft& other);

    virtual ~ChConstraintThreeBBShaft() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintThreeBBShaft* Clone() const override { return new ChConstraintThreeBBShaft(*this); }

    /// Assignment operator: copy from other object
    ChConstraintThreeBBShaft& operator=(const ChConstraintThreeBBShaft& other);

    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_a() override { return &Cq_a; }
    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_b() override { return &Cq_b; }
    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_c() override { return &Cq_c; }

    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_a() override { return &Eq_a; }
    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_b() override { return &Eq_b; }
    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_c() override { return &Eq_c; }

    /// Set references to the constrained objects,
    /// If first two variables aren't from ChVariablesBody class, an assert failure happens.
    void SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b, ChVariables* mvariables_c) override;

    /// This function updates the following auxiliary data:
    ///  - the Eq_a and Eq_b and Eq_c matrices
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

    /// Puts the jacobian parts into the 'insrow' row of a sparse matrix,
    /// where both portions of the jacobian are shifted in order to match the
    /// offset of the corresponding ChVariable.
    virtual void Build_Cq(ChSparseMatrix& storage, int insrow) override;
    virtual void Build_CqT(ChSparseMatrix& storage, int inscol) override;

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream) override;

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream) override;
};

}  // end namespace chrono

#endif
