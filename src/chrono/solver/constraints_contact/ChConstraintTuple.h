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

#include "chrono/physics/ChContactable.h"
#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Base class for a container representing "half" of a contact constraint.
/// This object manages the Jacobian chunk for one collision object which uses 1, 2, or 3 variable objects.
/// Derived classes specialize this for objects represented by 1, 2, or 3 variable objects of different sizes.
class ChConstraintTuple {
  public:
    virtual ~ChConstraintTuple() {}

  protected:
    /// Construct the tuple, setting variables and Jacobian blocks to null.
    /// Derived classes must set 1, 2, or 3 variable objects and the Jacobian blocks of appropriate size. 
    ChConstraintTuple() {
        for (int i = 0; i < 3; i++) {
            variables[i] = nullptr;
            Cq[i] = nullptr;
            Eq[i] = nullptr;
        }
    }

    /// Set the variable objects associated with the given contactable.
    /// A contactable will only set the number of variable objects it uses (the rest are set to nullptr).
    virtual void SetVariables(ChContactable* obj) {
        variables[0] = obj->GetVariables(0);
        variables[1] = obj->GetVariables(1);
        variables[2] = obj->GetVariables(2);
    }

    void Update_auxiliary(double& g_i) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                variables[i]->ComputeMassInverseTimesVector(*Eq[i], (*Cq[i]).transpose());
                g_i += *Cq[i] * *Eq[i];
            }
        }
    }

    double ComputeJacobianTimesState() {
        double result = 0;
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                result += *Cq[i] * variables[i]->State();
            }
        }
        return result;
    }

    void IncrementState(double deltal) {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                variables[i]->State() += *Eq[i] * deltal;
            }
        }
    }

    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                result += *Cq[i] * vect.segment(variables[i]->GetOffset(), variables[i]->GetDOF());
            }
        }
    }

    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                result.segment(variables[i]->GetOffset(), variables[i]->GetDOF()) += (*Cq[i]).transpose() * l;
            }
        }
    }

    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                PasteMatrix(mat, *Cq[i], start_row, variables[i]->GetOffset() + start_col);
            }
        }
    }

    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        for (int i = 0; i < num_variables; i++) {
            if (variables[i]->IsActive()) {
                PasteMatrix(mat, (*Cq[i]).transpose(), variables[i]->GetOffset() + start_row, start_col);
            }
        }
    }

    int num_variables;          ///< number of referenced variables
    ChVariables* variables[3];  ///< referenced variable objects
    ChRowVectorRef* Cq[3];      ///< [Cq] Jacobian chunks of the constraint
    ChVectorRef* Eq[3];         ///< [Eq] products [Eq]=[invM]*[Cq]'

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
    ChConstraintTuple_1() {
        Cq_1.setZero();

        num_variables = 1;
        ChRowVectorRef Cq1 = Cq_1;
        ChVectorRef Eq1 = Eq_1;
        Cq[0] = &Cq1;
        Eq[0] = &Eq1;
    }
};

/// Constraint tuple representing a collision object with 1 variable of size 2.
class ChConstraintTuple_2 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 2> Cq_1;
    ChVectorN<double, 2> Eq_1;

  public:
    ChConstraintTuple_2() {
        Cq_1.setZero();

        num_variables = 1;
        ChRowVectorRef Cq1 = Cq_1;
        ChVectorRef Eq1 = Eq_1;
        Cq[0] = &Cq1;
        Eq[0] = &Eq1;
    }
};

/// Constraint tuple representing a collision object with 1 variable of size 3.
class ChConstraintTuple_3 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 3> Cq_1;
    ChVectorN<double, 3> Eq_1;

  public:
    ChConstraintTuple_3() {
        Cq_1.setZero();

        num_variables = 1;
        ChRowVectorRef Cq1 = Cq_1;
        ChVectorRef Eq1 = Eq_1;
        Cq[0] = &Cq1;
        Eq[0] = &Eq1;
    }
};

/// Constraint tuple representing a collision object with 1 variable of size 4.
class ChConstraintTuple_4 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 4> Cq_1;
    ChVectorN<double, 4> Eq_1;

  public:
    ChConstraintTuple_4() {
        Cq_1.setZero();

        num_variables = 1;
        ChRowVectorRef Cq1 = Cq_1;
        ChVectorRef Eq1 = Eq_1;
        Cq[0] = &Cq1;
        Eq[0] = &Eq1;
    }
};

/// Constraint tuple representing a collision object with 1 variable of size 5.
class ChConstraintTuple_5 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 5> Cq_1;
    ChVectorN<double, 5> Eq_1;

  public:
    ChConstraintTuple_5() {
        Cq_1.setZero();

        num_variables = 1;
        ChRowVectorRef Cq1 = Cq_1;
        ChVectorRef Eq1 = Eq_1;
        Cq[0] = &Cq1;
        Eq[0] = &Eq1;
    }
};

/// Constraint tuple representing a collision object with 1 variable of size 6.
class ChConstraintTuple_6 : public ChConstraintTuple {
  private:
    ChRowVectorN<double, 6> Cq_1;
    ChVectorN<double, 6> Eq_1;

  public:
    ChConstraintTuple_6() {
        Cq_1.setZero();

        num_variables = 1;
        ChRowVectorRef Cq1 = Cq_1;
        ChVectorRef Eq1 = Eq_1;
        Cq[0] = &Cq1;
        Eq[0] = &Eq1;
    }
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
    ChConstraintTuple_33() {
        Cq_1.setZero();
        Cq_2.setZero();

        num_variables = 2;
        ChRowVectorRef Cq1 = Cq_1;
        ChRowVectorRef Cq2 = Cq_2;
        ChVectorRef Eq1 = Eq_1;
        ChVectorRef Eq2 = Eq_2;
        Cq[0] = &Cq1;
        Cq[1] = &Cq2;
        Eq[0] = &Eq1;
        Eq[1] = &Eq2;
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
    ChConstraintTuple_66() {
        Cq_1.setZero();
        Cq_2.setZero();

        num_variables = 2;
        ChRowVectorRef Cq1 = Cq_1;
        ChRowVectorRef Cq2 = Cq_2;
        ChVectorRef Eq1 = Eq_1;
        ChVectorRef Eq2 = Eq_2;
        Cq[0] = &Cq1;
        Cq[1] = &Cq2;
        Eq[0] = &Eq1;
        Eq[1] = &Eq2;
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
    ChConstraintTuple_333() {
        Cq_1.setZero();
        Cq_2.setZero();
        Cq_3.setZero();

        num_variables = 3;
        ChRowVectorRef Cq1 = Cq_1;
        ChRowVectorRef Cq2 = Cq_2;
        ChRowVectorRef Cq3 = Cq_3;
        ChVectorRef Eq1 = Eq_1;
        ChVectorRef Eq2 = Eq_2;
        ChVectorRef Eq3 = Eq_3;

        Cq[0] = &Cq1;
        Cq[1] = &Cq2;
        Cq[2] = &Cq3;
        Eq[0] = &Eq1;
        Eq[1] = &Eq2;
        Eq[2] = &Eq3;
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
    ChConstraintTuple_666() {
        Cq_1.setZero();
        Cq_2.setZero();
        Cq_3.setZero();

        num_variables = 3;
        ChRowVectorRef Cq1 = Cq_1;
        ChRowVectorRef Cq2 = Cq_2;
        ChRowVectorRef Cq3 = Cq_3;
        ChVectorRef Eq1 = Eq_1;
        ChVectorRef Eq2 = Eq_2;
        ChVectorRef Eq3 = Eq_3;

        Cq[0] = &Cq1;
        Cq[1] = &Cq2;
        Cq[2] = &Cq3;
        Eq[0] = &Eq1;
        Eq[1] = &Eq2;
        Eq[2] = &Eq3;
    }
};

}  // end namespace chrono

#endif
