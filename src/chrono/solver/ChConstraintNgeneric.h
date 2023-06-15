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

/// This class implements the functionality for a constraint between N objects of type ChVariables(), and defines three
/// constraint matrices, whose column number automatically matches the number of elements in variables vectors.
/// Before starting the solver one must provide the proper values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChConstraintNgeneric : public ChConstraint {
  protected:
    std::vector<ChVariables*> variables;

    std::vector<ChRowVectorDynamic<double>> Cq;  ///< The [Cq] jacobian slices
    std::vector<ChVectorDynamic<double>> Eq;     ///< The [Eq] product [Eq]=[invM]*[Cq]'

  public:
    /// Default constructor
    ChConstraintNgeneric() {}

    /// Copy constructor
    ChConstraintNgeneric(const ChConstraintNgeneric& other);

    virtual ~ChConstraintNgeneric() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintNgeneric* Clone() const override { return new ChConstraintNgeneric(*this); }

    /// Assignment operator: copy from other object
    ChConstraintNgeneric& operator=(const ChConstraintNgeneric& other);

    /// Access the Nth jacobian vector.
    ChRowVectorRef Get_Cq_N(size_t n) { return Cq[n]; }

    /// Access the Nth auxiliary vector (ex: used by iterative solvers).
    ChVectorRef Get_Eq_N(size_t n) { return Eq[n]; }

    /// Access the Nth variable object
    ChVariables* GetVariables_N(size_t n) { return variables[n]; }

    /// Set references to the constrained objects, each of ChVariables type,
    /// automatically creating/resizing jacobians if needed.
    void SetVariables(std::vector<ChVariables*> mvars);

    /// This function updates the following auxiliary data:
    ///  - the Eq  matrices
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
    virtual void MultiplyAndAdd(double& result, const ChVectorDynamic<double>& vect) const override;

    /// Computes the product of the corresponding transposed blocks in the
    /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'.
    /// NOTE: the 'result' vector must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect;
    virtual void MultiplyTandAdd(ChVectorDynamic<double>& result, double l) override;

    /// Puts the three jacobian parts into the 'insrow' row of a sparse matrix,
    /// where both portions of the jacobian are shifted in order to match the
    /// offset of the corresponding ChVariable.
    virtual void Build_Cq(ChSparseMatrix& storage, int insrow) override;
    virtual void Build_CqT(ChSparseMatrix& storage, int inscol) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
