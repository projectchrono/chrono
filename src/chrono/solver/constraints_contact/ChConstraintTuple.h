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

class ChContactable;

/// Container representing "half" of a constraint between objects with different numbers of variable sets.
/// Two such tuples, aggregated in a ChConstraintTwoTuples, form a full constraint.
/// This object manages the Jacobian chunk for one object which uses 1, 2, or 3 variable objects.
/// Derived classes specialize this for objects represented by 1, 2, or 3 variable objects of different sizes.
class ChConstraintTuple {
  public:
    virtual ~ChConstraintTuple() {}

    virtual ChRowVectorRef Cq(int i) = 0;
    virtual ChVectorRef Eq(int i) = 0;

    virtual void CalculateEq() = 0;

  protected:
    /// Construct the tuple, setting variables and Jacobian blocks to null.
    /// Derived classes must set 1, 2, or 3 variable objects and the Jacobian blocks of appropriate size.
    ChConstraintTuple() {
        for (int i = 0; i < 3; i++) {
            variables[i] = nullptr;
        }
    }

    void Update_auxiliary(double& g_i) {
        CalculateEq();
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                g_i += Cq(i) * Eq(i);
            }
        }
    }

    double ComputeJacobianTimesState() {
        double result = 0;
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                result += Cq(i) * variables[i]->State();
            }
        }
        return result;
    }

    void IncrementState(double deltal) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                variables[i]->State() += Eq(i) * deltal;
            }
        }
    }

    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                result += Cq(i) * vect.segment(variables[i]->GetOffset(), variables[i]->GetDOF());
            }
        }
    }

    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                result.segment(variables[i]->GetOffset(), variables[i]->GetDOF()) += Cq(i).transpose() * l;
            }
        }
    }

    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                PasteMatrix(mat, Cq(i), start_row, variables[i]->GetOffset() + start_col);
            }
        }
    }

    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                PasteMatrix(mat, Cq(i).transpose(), variables[i]->GetOffset() + start_row, start_col);
            }
        }
    }

    int num_variables;          ///< number of referenced variables
    ChVariables* variables[3];  ///< referenced variable objects

    friend class ChConstraintTwoTuples;
};

// -----------------------------------------------------------------------------
// Constraint tuples for collision objects with 1 variable object

/// Constraint tuple representing a collision object with 1 variable of size 1.
class ChConstraintTuple_1 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 1> Cq_1;
    ChVectorN<double, 1> Eq_1;

  public:
    ChConstraintTuple_1(ChVariables* variables1) {
        assert(variables1->GetDOF() == 1);
        num_variables = 1;
        variables[0] = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual ChRowVectorRef Cq(int i) override {
        assert(i == 0);
        return Cq_1;
    }

    virtual ChVectorRef Eq(int i) override {
        assert(i == 0);
        return Eq_1;
    }

    virtual void CalculateEq() override { variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose()); }
};

/// Constraint tuple representing a collision object with 1 variable of size 2.
class ChConstraintTuple_2 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 2> Cq_1;
    ChVectorN<double, 2> Eq_1;

  public:
    ChConstraintTuple_2(ChVariables* variables1) {
        assert(variables1->GetDOF() == 2);
        num_variables = 1;
        variables[0] = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual ChRowVectorRef Cq(int i) override {
        assert(i == 0);
        return Cq_1;
    }

    virtual ChVectorRef Eq(int i) override {
        assert(i == 0);
        return Eq_1;
    }

    virtual void CalculateEq() override { variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose()); }
};

/// Constraint tuple representing a collision object with 1 variable of size 3.
class ChConstraintTuple_3 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChVectorN<double, 3> Eq_1;

  public:
    ChConstraintTuple_3(ChVariables* variables1) {
        assert(variables1->GetDOF() == 3);
        num_variables = 1;
        variables[0] = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual ChRowVectorRef Cq(int i) override {
        assert(i == 0);
        return Cq_1;
    }

    virtual ChVectorRef Eq(int i) override {
        assert(i == 0);
        return Eq_1;
    }

    virtual void CalculateEq() override { variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose()); }
};

/// Constraint tuple representing a collision object with 1 variable of size 4.
class ChConstraintTuple_4 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 4> Cq_1;
    ChVectorN<double, 4> Eq_1;

  public:
    ChConstraintTuple_4(ChVariables* variables1) {
        assert(variables1->GetDOF() == 4);
        num_variables = 1;
        variables[0] = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual ChRowVectorRef Cq(int i) override {
        assert(i == 0);
        return Cq_1;
    }

    virtual ChVectorRef Eq(int i) override {
        assert(i == 0);
        return Eq_1;
    }

    virtual void CalculateEq() override { variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose()); }
};

/// Constraint tuple representing a collision object with 1 variable of size 5.
class ChConstraintTuple_5 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 5> Cq_1;
    ChVectorN<double, 5> Eq_1;

  public:
    ChConstraintTuple_5(ChVariables* variables1) {
        assert(variables1->GetDOF() == 5);
        num_variables = 1;
        variables[0] = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual ChRowVectorRef Cq(int i) override {
        assert(i == 0);
        return Cq_1;
    }

    virtual ChVectorRef Eq(int i) override {
        assert(i == 0);
        return Eq_1;
    }

    virtual void CalculateEq() override { variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose()); }
};

/// Constraint tuple representing a collision object with 1 variable of size 6.
class ChConstraintTuple_6 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChVectorN<double, 6> Eq_1;

  public:
    ChConstraintTuple_6(ChVariables* variables1) {
        assert(variables1->GetDOF() == 6);
        num_variables = 1;
        variables[0] = variables1;

        Cq_1.setZero();
        Eq_1.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }

    virtual ChRowVectorRef Cq(int i) override {
        assert(i == 0);
        return Cq_1;
    }

    virtual ChVectorRef Eq(int i) override {
        assert(i == 0);
        return Eq_1;
    }

    virtual void CalculateEq() override { variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose()); }
};

// -----------------------------------------------------------------------------
// Constraint tuples for collision objects with 2 variable objects

/// Constraint tuple representing a collision object with 2 variable, each of size 3.
class ChConstraintTuple_33 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChRowVectorN<double, 3> Cq_2;
    ChVectorN<double, 3> Eq_1;
    ChVectorN<double, 3> Eq_2;

  public:
    ChConstraintTuple_33(ChVariables* variables1, ChVariables* variables2) {
        assert(variables1->GetDOF() == 3);
        assert(variables2->GetDOF() == 3);
        num_variables = 2;
        variables[0] = variables1;
        variables[1] = variables2;

        Cq_1.setZero();
        Cq_2.setZero();
        Eq_1.setZero();
        Eq_2.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }

    virtual ChRowVectorRef Cq(int i) override {
        switch (i) {
            case 0: {
                return Cq_1;
            }
            case 1: {
                return Cq_2;
            }
            default:
                assert(false);
        }
    }

    virtual ChVectorRef Eq(int i) override {
        switch (i) {
            case 0: {
                return Eq_1;
            }
            case 1: {
                return Eq_2;
            }
            default:
                assert(false);
        }
    }

    virtual void CalculateEq() override {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
    }
};

/// Constraint tuple representing a collision object with 2 variable, each of size 6.
class ChConstraintTuple_66 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChRowVectorN<double, 6> Cq_2;
    ChVectorN<double, 6> Eq_1;
    ChVectorN<double, 6> Eq_2;

  public:
    ChConstraintTuple_66(ChVariables* variables1, ChVariables* variables2) {
        assert(variables1->GetDOF() == 6);
        assert(variables2->GetDOF() == 6);
        num_variables = 2;
        variables[0] = variables1;
        variables[1] = variables2;

        Cq_1.setZero();
        Cq_2.setZero();
        Eq_1.setZero();
        Eq_2.setZero();
    }

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }

    virtual ChRowVectorRef Cq(int i) override {
        switch (i) {
            case 0: {
                return Cq_1;
            }
            case 1: {
                return Cq_2;
            }
            default:
                assert(false);
        }
    }

    virtual ChVectorRef Eq(int i) override {
        switch (i) {
            case 0: {
                return Eq_1;
            }
            case 1: {
                return Eq_2;
            }
            default:
                assert(false);
        }
    }

    virtual void CalculateEq() override {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
    }
};

// -----------------------------------------------------------------------------
// Constraint tuples for collision objects with 2 variable objects

/// Constraint tuple representing a collision object with 3 variable, each of size 3.
class ChConstraintTuple_333 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChRowVectorN<double, 3> Cq_2;
    ChRowVectorN<double, 3> Cq_3;
    ChVectorN<double, 3> Eq_1;
    ChVectorN<double, 3> Eq_2;
    ChVectorN<double, 3> Eq_3;

  public:
    ChConstraintTuple_333(ChVariables* variables1, ChVariables* variables2, ChVariables* variables3) {
        assert(variables1->GetDOF() == 3);
        assert(variables2->GetDOF() == 3);
        assert(variables3->GetDOF() == 3);
        num_variables = 3;
        variables[0] = variables1;
        variables[1] = variables2;
        variables[2] = variables3;

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

    virtual ChRowVectorRef Cq(int i) override {
        switch (i) {
            case 0:
                return Cq_1;
            case 1:
                return Cq_2;
            case 2:
                return Cq_3;
        }
    }

    virtual ChVectorRef Eq(int i) override {
        switch (i) {
            case 0:
                return Eq_1;
            case 1:
                return Eq_2;
            case 2:
                return Eq_3;
        }
    }

    virtual void CalculateEq() override {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        variables[2]->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
    }
};

/// Constraint tuple representing a collision object with 3 variable, each of size 6.
class ChConstraintTuple_666 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChRowVectorN<double, 6> Cq_2;
    ChRowVectorN<double, 6> Cq_3;
    ChVectorN<double, 6> Eq_1;
    ChVectorN<double, 6> Eq_2;
    ChVectorN<double, 6> Eq_3;

  public:
    ChConstraintTuple_666(ChVariables* variables1, ChVariables* variables2, ChVariables* variables3) {
        assert(variables1->GetDOF() == 6);
        assert(variables2->GetDOF() == 6);
        assert(variables3->GetDOF() == 6);
        num_variables = 3;
        variables[0] = variables1;
        variables[1] = variables2;
        variables[2] = variables3;

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


    virtual ChRowVectorRef Cq(int i) override {
        switch (i) {
            case 0:
                return Cq_1;
            case 1:
                return Cq_2;
            case 2:
                return Cq_3;
        }
    }

    virtual ChVectorRef Eq(int i) override {
        switch (i) {
            case 0:
                return Eq_1;
            case 1:
                return Eq_2;
            case 2:
                return Eq_3;
        }
    }

    virtual void CalculateEq() override {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        variables[2]->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
    }
};

}  // end namespace chrono

#endif
