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

#ifndef CH_CONSTRAINT_TUPLE_H
#define CH_CONSTRAINT_TUPLE_H

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Container representing "half" of a constraint between objects with different numbers of variable sets.
/// Two such tuples, aggregated in a ChConstraintTwoTuples, form a full constraint.
/// This object manages the Jacobian chunks for one object which uses 1, 2, or 3 variable objects.
/// Derived classes specialize this for objects represented by 1, 2, or 3 variable objects of different sizes.
class ChConstraintTuple {
  public:
    virtual ~ChConstraintTuple() {}

  protected:
    ChConstraintTuple() {}

    virtual void UpdateAuxiliary(double& g_i) = 0;
    virtual double ComputeJacobianTimesState() = 0;
    virtual void IncrementState(double deltal) = 0;
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) = 0;
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) = 0;
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) = 0;
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) = 0;

    friend class ChConstraintTwoTuples;
};

// -----------------------------------------------------------------------------

/// Constraint tuple for objects with 1 variable set.
template <int N>
class ChConstraintTuple_1vars : public ChConstraintTuple {
  public:
    ChConstraintTuple_1vars(ChVariables* variables1) {
        assert(variables1->GetDOF() == N);
        variables_1 = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual void UpdateAuxiliary(double& g_i) override {
        if (variables_1->IsActive()) {
            variables_1->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
            g_i += Cq_1 * Eq_1;
        }
    }
    virtual double ComputeJacobianTimesState() override {
        if (variables_1->IsActive())
            return Cq_1 * variables_1->State();
        return 0;
    }
    virtual void IncrementState(double deltal) override {
        if (variables_1->IsActive())
            variables_1->State() += Eq_1 * deltal;
    }
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) override {
        if (variables_1->IsActive())
            result += Cq_1 * vect.segment(variables_1->GetOffset(), N);
    }
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) override {
        if (variables_1->IsActive())
            result.segment(variables_1->GetOffset(), N) += Cq_1.transpose() * l;
    }
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) override {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1, start_row, variables_1->GetOffset() + start_col);
    }
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) override {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1.transpose(), variables_1->GetOffset() + start_row, start_col);
    }

  private:
    ChVariables* variables_1;

    ChRowVectorN<double, N> Cq_1;
    ChVectorN<double, N> Eq_1;
};

// -----------------------------------------------------------------------------

/// Constraint tuple for objects with 2 variable sets.
template <int N1, int N2>
class ChConstraintTuple_2vars : public ChConstraintTuple {
  public:
    ChConstraintTuple_2vars(ChVariables* variables1, ChVariables* variables2) {
        assert(variables1->GetDOF() == N1);
        assert(variables2->GetDOF() == N2);
        variables_1 = variables1;
        variables_2 = variables2;

        Cq_1.setZero();
        Cq_2.setZero();
        Eq_1.setZero();
        Eq_2.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }

    virtual void UpdateAuxiliary(double& g_i) override {
        if (variables_1->IsActive()) {
            variables_1->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
            g_i += Cq_1 * Eq_1;
        }
        if (variables_2->IsActive()) {
            variables_2->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
            g_i += Cq_2 * Eq_2;
        }
    }
    virtual double ComputeJacobianTimesState() override {
        double result = 0;
        if (variables_1->IsActive())
            result += Cq_1 * variables_1->State();
        if (variables_2->IsActive())
            result += Cq_2 * variables_2->State();
        return result;
    }
    virtual void IncrementState(double deltal) override {
        if (variables_1->IsActive())
            variables_1->State() += Eq_1 * deltal;
        if (variables_2->IsActive())
            variables_2->State() += Eq_2 * deltal;
    }
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) override {
        if (variables_1->IsActive())
            result += Cq_1 * vect.segment(variables_1->GetOffset(), N1);
        if (variables_2->IsActive())
            result += Cq_2 * vect.segment(variables_2->GetOffset(), N2);
    }
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) override {
        if (variables_1->IsActive())
            result.segment(variables_1->GetOffset(), N1) += Cq_1.transpose() * l;
        if (variables_2->IsActive())
            result.segment(variables_2->GetOffset(), N2) += Cq_2.transpose() * l;
    }
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) override {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1, start_row, variables_1->GetOffset() + start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2, start_row, variables_2->GetOffset() + start_col);
    }
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) override {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1.transpose(), variables_1->GetOffset() + start_row, start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2.transpose(), variables_2->GetOffset() + start_row, start_col);
    }

  private:
    ChVariables* variables_1;
    ChVariables* variables_2;

    ChRowVectorN<double, N1> Cq_1;
    ChRowVectorN<double, N2> Cq_2;
    ChVectorN<double, N1> Eq_1;
    ChVectorN<double, N2> Eq_2;
};

// -----------------------------------------------------------------------------

/// Constraint tuple for objects with 3 variable sets.
template <int N1, int N2, int N3>
class ChConstraintTuple_3vars : public ChConstraintTuple {
  public:
    ChConstraintTuple_3vars(ChVariables* variables1, ChVariables* variables2, ChVariables* variables3) {
        assert(variables1->GetDOF() == N1);
        assert(variables2->GetDOF() == N2);
        assert(variables3->GetDOF() == N3);
        variables_1 = variables1;
        variables_2 = variables2;
        variables_3 = variables3;

        Cq_1.setZero();
        Cq_2.setZero();
        Cq_3.setZero();
        Eq_1.setZero();
        Eq_2.setZero();
        Eq_3.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChRowVectorRef Cq3() { return Cq_3; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }
    ChVectorRef Eq3() { return Eq_3; }

    virtual void UpdateAuxiliary(double& g_i) override {
        if (variables_1->IsActive()) {
            variables_1->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
            g_i += Cq_1 * Eq_1;
        }
        if (variables_2->IsActive()) {
            variables_2->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
            g_i += Cq_2 * Eq_2;
        }
        if (variables_3->IsActive()) {
            variables_3->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
            g_i += Cq_3 * Eq_3;
        }
    }
    virtual double ComputeJacobianTimesState() override {
        double result = 0;
        if (variables_1->IsActive())
            result += Cq_1 * variables_1->State();
        if (variables_2->IsActive())
            result += Cq_2 * variables_2->State();
        if (variables_3->IsActive())
            result += Cq_3 * variables_3->State();
        return result;
    }
    virtual void IncrementState(double deltal) override {
        if (variables_1->IsActive())
            variables_1->State() += Eq_1 * deltal;
        if (variables_2->IsActive())
            variables_2->State() += Eq_2 * deltal;
        if (variables_3->IsActive())
            variables_3->State() += Eq_3 * deltal;
    }
    virtual void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) override {
        if (variables_1->IsActive())
            result += Cq_1 * vect.segment(variables_1->GetOffset(), N1);
        if (variables_2->IsActive())
            result += Cq_2 * vect.segment(variables_2->GetOffset(), N2);
        if (variables_3->IsActive())
            result += Cq_3 * vect.segment(variables_2->GetOffset(), N3);
    }
    virtual void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) override {
        if (variables_1->IsActive())
            result.segment(variables_1->GetOffset(), N1) += Cq_1.transpose() * l;
        if (variables_2->IsActive())
            result.segment(variables_2->GetOffset(), N2) += Cq_2.transpose() * l;
        if (variables_3->IsActive())
            result.segment(variables_3->GetOffset(), N3) += Cq_3.transpose() * l;
    }
    virtual void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) override {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1, start_row, variables_1->GetOffset() + start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2, start_row, variables_2->GetOffset() + start_col);
        if (variables_3->IsActive())
            PasteMatrix(mat, Cq_3, start_row, variables_3->GetOffset() + start_col);
    }
    virtual void PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) override {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1.transpose(), variables_1->GetOffset() + start_row, start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2.transpose(), variables_2->GetOffset() + start_row, start_col);
        if (variables_3->IsActive())
            PasteMatrix(mat, Cq_3.transpose(), variables_3->GetOffset() + start_row, start_col);
    }

  private:
    ChVariables* variables_1;
    ChVariables* variables_2;
    ChVariables* variables_3;

    ChRowVectorN<double, N1> Cq_1;
    ChRowVectorN<double, N2> Cq_2;
    ChRowVectorN<double, N3> Cq_3;
    ChVectorN<double, N1> Eq_1;
    ChVectorN<double, N2> Eq_2;
    ChVectorN<double, N3> Eq_3;
};

}  // end namespace chrono

#endif
