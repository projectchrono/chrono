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
/// This object manages the Jacobian chunk for one object which uses 1, 2, or 3 variable objects.
/// Derived classes specialize this for objects represented by 1, 2, or 3 variable objects of different sizes.
class ChApi ChConstraintTuple {
  public:
    virtual ~ChConstraintTuple() {}

    virtual ChRowVectorRef Cq(int i) = 0;
    virtual ChVectorRef Eq(int i) = 0;

    virtual void CalculateEq() = 0;

  protected:
    /// Construct the tuple, setting variables and Jacobian blocks to null.
    /// Derived classes must set 1, 2, or 3 variable objects and the Jacobian blocks of appropriate size.
    ChConstraintTuple();

    void Update_auxiliary(double& g_i);
    double ComputeJacobianTimesState();
    void IncrementState(double deltal);
    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect);
    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l);
    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col);
    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col);

    int num_variables;          ///< number of referenced variables
    ChVariables* variables[3];  ///< referenced variable objects

    friend class ChConstraintTwoTuples;
};

// -----------------------------------------------------------------------------
// Constraint tuples for objects with 1 variable object

/// Constraint tuple representing an object with 1 variable of size 1.
class ChConstraintTuple_1 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 1> Cq_1;
    ChVectorN<double, 1> Eq_1;

  public:
    ChConstraintTuple_1(ChVariables* variables1);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 1 variable of size 2.
class ChConstraintTuple_2 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 2> Cq_1;
    ChVectorN<double, 2> Eq_1;

  public:
    ChConstraintTuple_2(ChVariables* variables1);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 1 variable of size 3.
class ChConstraintTuple_3 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChVectorN<double, 3> Eq_1;

  public:
    ChConstraintTuple_3(ChVariables* variables1);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 1 variable of size 4.
class ChConstraintTuple_4 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 4> Cq_1;
    ChVectorN<double, 4> Eq_1;

  public:
    ChConstraintTuple_4(ChVariables* variables1);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 1 variable of size 5.
class ChConstraintTuple_5 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 5> Cq_1;
    ChVectorN<double, 5> Eq_1;

  public:
    ChConstraintTuple_5(ChVariables* variables1);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 1 variable of size 6.
class ChConstraintTuple_6 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChVectorN<double, 6> Eq_1;

  public:
    ChConstraintTuple_6(ChVariables* variables1);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChVectorRef Eq1() { return Eq_1; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

// -----------------------------------------------------------------------------
// Constraint tuples for objects with 2 variable objects

/// Constraint tuple representing an object with 2 variables, each of size 3.
class ChConstraintTuple_33 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChRowVectorN<double, 3> Cq_2;
    ChVectorN<double, 3> Eq_1;
    ChVectorN<double, 3> Eq_2;

  public:
    ChConstraintTuple_33(ChVariables* variables1, ChVariables* variables2);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 2 variables, each of size 6.
class ChConstraintTuple_66 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChRowVectorN<double, 6> Cq_2;
    ChVectorN<double, 6> Eq_1;
    ChVectorN<double, 6> Eq_2;

  public:
    ChConstraintTuple_66(ChVariables* variables1, ChVariables* variables2);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

// -----------------------------------------------------------------------------
// Constraint tuples for objects with 3 variable objects

/// Constraint tuple representing an object with 3 variables, each of size 3.
class ChConstraintTuple_333 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChRowVectorN<double, 3> Cq_2;
    ChRowVectorN<double, 3> Cq_3;
    ChVectorN<double, 3> Eq_1;
    ChVectorN<double, 3> Eq_2;
    ChVectorN<double, 3> Eq_3;

  public:
    ChConstraintTuple_333(ChVariables* variables1, ChVariables* variables2, ChVariables* variables3);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChRowVectorRef Cq3() { return Cq_3; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }
    ChVectorRef Eq3() { return Eq_3; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

/// Constraint tuple representing an object with 3 variables, each of size 6.
class ChConstraintTuple_666 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChRowVectorN<double, 6> Cq_2;
    ChRowVectorN<double, 6> Cq_3;
    ChVectorN<double, 6> Eq_1;
    ChVectorN<double, 6> Eq_2;
    ChVectorN<double, 6> Eq_3;

  public:
    ChConstraintTuple_666(ChVariables* variables1, ChVariables* variables2, ChVariables* variables3);

    ChRowVectorRef Cq1() { return Cq_1; }
    ChRowVectorRef Cq2() { return Cq_2; }
    ChRowVectorRef Cq3() { return Cq_3; }
    ChVectorRef Eq1() { return Eq_1; }
    ChVectorRef Eq2() { return Eq_2; }
    ChVectorRef Eq3() { return Eq_3; }
    virtual ChRowVectorRef Cq(int i) override;
    virtual ChVectorRef Eq(int i) override;
    virtual void CalculateEq() override;
};

}  // end namespace chrono

#endif
