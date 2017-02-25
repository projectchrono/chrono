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

#ifndef CHCONSTRAINTTHREEGENERIC_H
#define CHCONSTRAINTTHREEGENERIC_H

#include "chrono/solver/ChConstraintThree.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// This class is inherited by the base ChConstraintThree(),
/// which is a base for triple constraints. So here this class implements
/// the functionality for a constraint between a TRHEE
/// objects of type ChVariables(), with generic number of scalar
/// variables each (ex.ChVariablesGeneric() or ChVariablesBody() )
/// and defines three jacobian matrices, whose column number automatically
/// matches the number of elements in variables vectors.
///  Before starting the solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChConstraintThreeGeneric : public ChConstraintThree {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChConstraintThreeGeneric)

  protected:
    ChMatrixDynamic<double>* Cq_a;  ///< The [Cq_a] jacobian of the constraint
    ChMatrixDynamic<double>* Cq_b;  ///< The [Cq_b] jacobian of the constraint
    ChMatrixDynamic<double>* Cq_c;  ///< The [Cq_c] jacobian of the constraint

    // Auxiliary data: will be used by iterative constraint solvers:

    ChMatrixDynamic<double>* Eq_a;  ///< The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
    ChMatrixDynamic<double>* Eq_b;  ///< The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'
    ChMatrixDynamic<double>* Eq_c;  ///< The [Eq_a] product [Eq_c]=[invM_b]*[Cq_c]'

  public:
    /// Default constructor
    ChConstraintThreeGeneric() : Cq_a(NULL), Cq_b(NULL), Cq_c(NULL), Eq_a(NULL), Eq_b(NULL), Eq_c(NULL) {}

    /// Construct and immediately set references to variables
    ChConstraintThreeGeneric(ChVariables* mvariables_a, ChVariables* mvariables_b, ChVariables* mvariables_c);

    /// Copy constructor
    ChConstraintThreeGeneric(const ChConstraintThreeGeneric& other);

    virtual ~ChConstraintThreeGeneric();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintThreeGeneric* Clone() const override { return new ChConstraintThreeGeneric(*this); }

    /// Assignment operator: copy from other object
    ChConstraintThreeGeneric& operator=(const ChConstraintThreeGeneric& other);

    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_a() override { return Cq_a; }
    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_b() override { return Cq_b; }
    /// Access jacobian matrix
    virtual ChMatrix<double>* Get_Cq_c() override { return Cq_c; }

    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_a() override { return Eq_a; }
    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_b() override { return Eq_b; }
    /// Access auxiliary matrix (ex: used by iterative solvers)
    virtual ChMatrix<double>* Get_Eq_c() override { return Eq_c; }

    /// Set references to the constrained objects, each of ChVariables type,
    /// automatically creating/resizing jacobians if needed.
    virtual void SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b, ChVariables* mvariables_c) override;

    /// This function updates the following auxiliary data:
    ///  - the Eq_a and Eq_b and Eq_c  matrices
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

    /// Puts the three jacobian parts into the 'insrow' row of a sparse matrix,
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
