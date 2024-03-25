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

#ifndef CHCONSTRAINTTUPLE_H
#define CHCONSTRAINTTUPLE_H

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// This is a container for 'half' of a constraint, and contains a tuple of
/// 1 or 2 or 3 differently-sized jacobian chunks. For instance, this might
/// happen because you want a constraint between an edge (i.e. two xyz variables, each
/// per end nodes) and a triangle face (i.e. three xyz variables, each per corner), so
/// the jacobian row matrix is split in 2 + 3 chunks, here as two tuples.
/// The complete constraint, ChConstraintTwoTuples, will use two of these classes.
/// Template T is a class of ChVariableTupleCarrier_Nvars type

template <class T>
class ChConstraintTuple_1vars {
  protected:
    ChVariables* variables;              ///< The constrained object
    ChRowVectorN<double, T::nvars1> Cq;  ///< The [Cq] jacobian of the constraint
    ChVectorN<double, T::nvars1> Eq;     ///< The [Eq] product [Eq]=[invM]*[Cq]'

  public:
    /// Default constructor
    ChConstraintTuple_1vars() : variables(nullptr) {
        Cq.setZero();
        Eq.setZero();
    }

    /// Copy constructor
    ChConstraintTuple_1vars(const ChConstraintTuple_1vars& other) {
        variables = other.variables;
        Cq = other.Cq;
        Eq = other.Eq;
    }

    ~ChConstraintTuple_1vars() {}

    /// Assignment operator: copy from other object
    ChConstraintTuple_1vars& operator=(const ChConstraintTuple_1vars& other) {
        variables = other.variables;
        Cq = other.Cq;
        Eq = other.Eq;
        return *this;
    }

    ChRowVectorRef Get_Cq() { return Cq; }

    ChVectorRef Get_Eq() { return Eq; }

    ChVariables* GetVariables() { return variables; }

    void SetVariables(T& m_tuple_carrier) {
        if (!m_tuple_carrier.GetVariables1()) {
            throw std::runtime_error("ERROR: SetVariables() getting null pointer.");
        }
        variables = m_tuple_carrier.GetVariables1();
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq]=[invM]*[Cq]' and [Eq]
        if (variables->IsActive()) {
            variables->ComputeMassInverseTimesVector(Eq, Cq.transpose());
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        if (variables->IsActive()) {
            g_i += Cq * Eq;
        }
    }

    double ComputeJacobianTimesState() {
        double ret = 0;

        if (variables->IsActive()) {
            ret += Cq * variables->State();
        }

        return ret;
    }

    void IncrementState(double deltal) {
        if (variables->IsActive()) {
            variables->State() += Eq * deltal;
        }
    }

    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
        if (variables->IsActive()) {
            result += Cq * vect.segment(variables->GetOffset(), T::nvars1);
        }
    }

    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
        if (variables->IsActive()) {
            result.segment(variables->GetOffset(), T::nvars1) += Cq.transpose() * l;
        }
    }

    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables->IsActive())
            PasteMatrix(mat, Cq, start_row, variables->GetOffset() + start_col);
    }

    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables->IsActive())
            PasteMatrix(mat, Cq.transpose(), variables->GetOffset() + start_row, start_col);
    }
};

/// Case of tuple with reference to 2 ChVariable objects:

template <class T>
class ChConstraintTuple_2vars {
  protected:
    ChVariables* variables_1;
    ChVariables* variables_2;

    /// The [Cq] jacobian of the constraint, split in horizontal chunks
    ChRowVectorN<double, T::nvars1> Cq_1;
    ChRowVectorN<double, T::nvars2> Cq_2;

    /// The [Eq] product [Eq]=[invM]*[Cq]'
    ChVectorN<double, T::nvars1> Eq_1;
    ChVectorN<double, T::nvars2> Eq_2;

  public:
    /// Default constructor
    ChConstraintTuple_2vars() : variables_1(nullptr), variables_2(nullptr) {
        Cq_1.setZero();
        Cq_2.setZero();
    }

    /// Copy constructor
    ChConstraintTuple_2vars(const ChConstraintTuple_2vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
    }

    ~ChConstraintTuple_2vars() {}

    /// Assignment operator: copy from other object
    ChConstraintTuple_2vars& operator=(const ChConstraintTuple_2vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
        return *this;
    }

    ChRowVectorRef Get_Cq_1() { return Cq_1; }
    ChRowVectorRef Get_Cq_2() { return Cq_2; }

    ChVectorRef Get_Eq_1() { return Eq_1; }
    ChVectorRef Get_Eq_2() { return Eq_2; }

    ChVariables* GetVariables_1() { return variables_1; }
    ChVariables* GetVariables_2() { return variables_2; }

    void SetVariables(T& m_tuple_carrier) {
        if (!m_tuple_carrier.GetVariables1() || !m_tuple_carrier.GetVariables2()) {
            throw std::runtime_error("ERROR: SetVariables() getting null pointer.");
        }
        variables_1 = m_tuple_carrier.GetVariables1();
        variables_2 = m_tuple_carrier.GetVariables2();
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
        if (variables_1->IsActive()) {
            variables_1->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        }
        if (variables_2->IsActive()) {
            variables_2->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        if (variables_1->IsActive()) {
            g_i += Cq_1 * Eq_1;
        }
        if (variables_2->IsActive()) {
            g_i += Cq_2 * Eq_2;
        }
    }

    double ComputeJacobianTimesState() {
        double ret = 0;

        if (variables_1->IsActive()) {
            ret += Cq_1 * variables_1->State();
        }

        if (variables_2->IsActive()) {
            ret += Cq_2 * variables_2->State();
        }

        return ret;
    }

    void IncrementState(double deltal) {
        if (variables_1->IsActive()) {
            variables_1->State() += Eq_1 * deltal;
        }

        if (variables_2->IsActive()) {
            variables_2->State() += Eq_2 * deltal;
        }
    }

    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
        if (variables_1->IsActive()) {
            result += Cq_1 * vect.segment(variables_1->GetOffset(), T::nvars1);
        }

        if (variables_2->IsActive()) {
            result += Cq_2 * vect.segment(variables_2->GetOffset(), T::nvars2);
        }
    }

    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
        if (variables_1->IsActive()) {
            result.segment(variables_1->GetOffset(), T::nvars1) += Cq_1.transpose() * l;
        }

        if (variables_2->IsActive()) {
            result.segment(variables_2->GetOffset(), T::nvars2) += Cq_2.transpose() * l;
        }
    }

    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1, start_row, variables_1->GetOffset() + start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2, start_row, variables_2->GetOffset() + start_col);
    }

    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1.transpose(), variables_1->GetOffset() + start_row, start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2.transpose(), variables_2->GetOffset() + start_row, start_col);
    }
};

/// Case of tuple with reference to 3 ChVariable objects:

template <class T>
class ChConstraintTuple_3vars {
  protected:
    ChVariables* variables_1;
    ChVariables* variables_2;
    ChVariables* variables_3;

    /// The [Cq] jacobian of the constraint, split in horizontal chunks
    ChRowVectorN<double, T::nvars1> Cq_1;
    ChRowVectorN<double, T::nvars2> Cq_2;
    ChRowVectorN<double, T::nvars3> Cq_3;

    /// The [Eq] product [Eq]=[invM]*[Cq]'
    ChVectorN<double, T::nvars1> Eq_1;
    ChVectorN<double, T::nvars2> Eq_2;
    ChVectorN<double, T::nvars3> Eq_3;

  public:
    /// Default constructor
    ChConstraintTuple_3vars() : variables_1(nullptr), variables_2(nullptr), variables_3(nullptr) {
        Cq_1.setZero();
        Cq_2.setZero();
        Cq_3.setZero();
    }

    /// Copy constructor
    ChConstraintTuple_3vars(const ChConstraintTuple_3vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        variables_3 = other.variables_3;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Cq_3 = other.Cq_3;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
        Eq_3 = other.Eq_3;
    }

    ~ChConstraintTuple_3vars() {}

    /// Assignment operator: copy from other object
    ChConstraintTuple_3vars& operator=(const ChConstraintTuple_3vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        variables_3 = other.variables_3;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Cq_3 = other.Cq_3;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
        Eq_3 = other.Eq_3;
        return *this;
    }

    ChRowVectorRef Get_Cq_1() { return Cq_1; }
    ChRowVectorRef Get_Cq_2() { return Cq_2; }
    ChRowVectorRef Get_Cq_3() { return Cq_3; }

    ChVectorRef Get_Eq_1() { return Eq_1; }
    ChVectorRef Get_Eq_2() { return Eq_2; }
    ChVectorRef Get_Eq_3() { return Eq_3; }

    ChVariables* GetVariables_1() { return variables_1; }
    ChVariables* GetVariables_2() { return variables_2; }
    ChVariables* GetVariables_3() { return variables_3; }

    void SetVariables(T& m_tuple_carrier) {
        if (!m_tuple_carrier.GetVariables1() || !m_tuple_carrier.GetVariables2() || !m_tuple_carrier.GetVariables3()) {
            throw std::runtime_error("ERROR: SetVariables() getting null pointer.");
        }
        variables_1 = m_tuple_carrier.GetVariables1();
        variables_2 = m_tuple_carrier.GetVariables2();
        variables_3 = m_tuple_carrier.GetVariables3();
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
        if (variables_1->IsActive()) {
            variables_1->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        }
        if (variables_2->IsActive()) {
            variables_2->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        }
        if (variables_3->IsActive()) {
            variables_3->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        if (variables_1->IsActive()) {
            g_i += Cq_1 * Eq_1;
        }
        if (variables_2->IsActive()) {
            g_i += Cq_2 * Eq_2;
        }
        if (variables_3->IsActive()) {
            g_i += Cq_3 * Eq_3;
        }
    }

    double ComputeJacobianTimesState() {
        double ret = 0;

        if (variables_1->IsActive()) {
            ret += Cq_1 * variables_1->State();
        }

        if (variables_2->IsActive()) {
            ret += Cq_2 * variables_2->State();
        }

        if (variables_3->IsActive()) {
            ret += Cq_3 * variables_3->State();
        }

        return ret;
    }

    void IncrementState(double deltal) {
        if (variables_1->IsActive()) {
            variables_1->State() += Eq_1 * deltal;
        }

        if (variables_2->IsActive()) {
            variables_2->State() += Eq_2 * deltal;
        }

        if (variables_3->IsActive()) {
            variables_3->State() += Eq_3 * deltal;
        }
    }

    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
        if (variables_1->IsActive()) {
            result += Cq_1 * vect.segment(variables_1->GetOffset(), T::nvars1);
        }

        if (variables_2->IsActive()) {
            result += Cq_2 * vect.segment(variables_2->GetOffset(), T::nvars2);
        }

        if (variables_3->IsActive()) {
            result += Cq_3 * vect.segment(variables_3->GetOffset(), T::nvars3);
        }
    }

    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
        if (variables_1->IsActive()) {
            result.segment(variables_1->GetOffset(), T::nvars1) += Cq_1.transpose() * l;
        }

        if (variables_2->IsActive()) {
            result.segment(variables_2->GetOffset(), T::nvars2) += Cq_2.transpose() * l;
        }

        if (variables_3->IsActive()) {
            result.segment(variables_3->GetOffset(), T::nvars3) += Cq_3.transpose() * l;
        }
    }

    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1, start_row, variables_1->GetOffset() + start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2, start_row, variables_2->GetOffset() + start_col);
        if (variables_3->IsActive())
            PasteMatrix(mat, Cq_3, start_row, variables_3->GetOffset() + start_col);
    }

    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, int start_col) const {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1.transpose(), variables_1->GetOffset() + start_row, start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2.transpose(), variables_2->GetOffset() + start_row, start_col);
        if (variables_3->IsActive())
            PasteMatrix(mat, Cq_3.transpose(), variables_3->GetOffset() + start_row, start_col);
    }
};

/// Case of tuple with reference to 4 ChVariable objects:

template <class T>
class ChConstraintTuple_4vars {
  protected:
    ChVariables* variables_1;
    ChVariables* variables_2;
    ChVariables* variables_3;
    ChVariables* variables_4;

    /// The [Cq] jacobian of the constraint, split in horizontal chunks
    ChRowVectorN<double, T::nvars1> Cq_1;
    ChRowVectorN<double, T::nvars2> Cq_2;
    ChRowVectorN<double, T::nvars3> Cq_3;
    ChRowVectorN<double, T::nvars4> Cq_4;

    /// The [Eq] product [Eq]=[invM]*[Cq]'
    ChVectorN<double, T::nvars1> Eq_1;
    ChVectorN<double, T::nvars2> Eq_2;
    ChVectorN<double, T::nvars3> Eq_3;
    ChVectorN<double, T::nvars4> Eq_4;

  public:
    /// Default constructor
    ChConstraintTuple_4vars() : variables_1(nullptr), variables_2(nullptr), variables_3(nullptr), variables_4(nullptr) {
        Cq_1.setZero();
        Cq_2.setZero();
        Cq_3.setZero();
        Cq_4.setZero();
    }

    /// Copy constructor
    ChConstraintTuple_4vars(const ChConstraintTuple_4vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        variables_3 = other.variables_3;
        variables_4 = other.variables_4;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Cq_3 = other.Cq_3;
        Cq_4 = other.Cq_4;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
        Eq_3 = other.Eq_3;
        Eq_4 = other.Eq_4;
    }

    ~ChConstraintTuple_4vars() {}

    /// Assignment operator: copy from other object
    ChConstraintTuple_4vars& operator=(const ChConstraintTuple_4vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        variables_3 = other.variables_3;
        variables_4 = other.variables_4;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Cq_3 = other.Cq_3;
        Cq_4 = other.Cq_4;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
        Eq_3 = other.Eq_3;
        Eq_4 = other.Eq_4;
        return *this;
    }

    ChRowVectorRef Get_Cq_1() { return Cq_1; }
    ChRowVectorRef Get_Cq_2() { return Cq_2; }
    ChRowVectorRef Get_Cq_3() { return Cq_3; }
    ChRowVectorRef Get_Cq_4() { return Cq_4; }

    ChVectorRef Get_Eq_1() { return Eq_1; }
    ChVectorRef Get_Eq_2() { return Eq_2; }
    ChVectorRef Get_Eq_3() { return Eq_3; }
    ChVectorRef Get_Eq_4() { return Eq_4; }

    ChVariables* GetVariables_1() { return variables_1; }
    ChVariables* GetVariables_2() { return variables_2; }
    ChVariables* GetVariables_3() { return variables_3; }
    ChVariables* GetVariables_4() { return variables_4; }

    void SetVariables(T& m_tuple_carrier) {
        if (!m_tuple_carrier.GetVariables1() || !m_tuple_carrier.GetVariables2() || !m_tuple_carrier.GetVariables3() ||
            !m_tuple_carrier.GetVariables4()) {
            throw std::runtime_error("ERROR: SetVariables() getting null pointer.");
        }
        variables_1 = m_tuple_carrier.GetVariables1();
        variables_2 = m_tuple_carrier.GetVariables2();
        variables_3 = m_tuple_carrier.GetVariables3();
        variables_4 = m_tuple_carrier.GetVariables4();
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
        if (variables_1->IsActive()) {
            variables_1->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        }
        if (variables_2->IsActive()) {
            variables_2->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        }
        if (variables_3->IsActive()) {
            variables_3->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
        }
        if (variables_4->IsActive()) {
            variables_4->ComputeMassInverseTimesVector(Eq_4, Cq_4.transpose());
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        if (variables_1->IsActive()) {
            g_i += Cq_1 * Eq_1;
        }
        if (variables_2->IsActive()) {
            g_i += Cq_2 * Eq_2;
        }
        if (variables_3->IsActive()) {
            g_i += Cq_3 * Eq_3;
        }
        if (variables_4->IsActive()) {
            g_i += Cq_4 * Eq_4;
        }
    }

    double ComputeJacobianTimesState() {
        double ret = 0;

        if (variables_1->IsActive()) {
            ret += Cq_1 * variables_1->State();
        }

        if (variables_2->IsActive()) {
            ret += Cq_2 * variables_2->State();
        }

        if (variables_3->IsActive()) {
            ret += Cq_3 * variables_3->State();
        }

        if (variables_4->IsActive()) {
            ret += Cq_4 * variables_4->State();
        }

        return ret;
    }

    void IncrementState(double deltal) {
        if (variables_1->IsActive()) {
            variables_1->State() += Eq_1 * deltal;
        }

        if (variables_2->IsActive()) {
            variables_2->State() += Eq_2 * deltal;
        }

        if (variables_3->IsActive()) {
            variables_3->State() += Eq_3 * deltal;
        }

        if (variables_4->IsActive()) {
            variables_4->State() += Eq_4 * deltal;
        }
    }

    void AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
        if (variables_1->IsActive()) {
            result += Cq_1 * vect.segment(variables_1->GetOffset(), T::nvars1);
        }

        if (variables_2->IsActive()) {
            result += Cq_2 * vect.segment(variables_2->GetOffset(), T::nvars2);
        }

        if (variables_3->IsActive()) {
            result += Cq_3 * vect.segment(variables_3->GetOffset(), T::nvars3);
        }

        if (variables_4->IsActive()) {
            result += Cq_4 * vect.segment(variables_4->GetOffset(), T::nvars4);
        }
    }

    void AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
        if (variables_1->IsActive()) {
            result.segment(variables_1->GetOffset(), T::nvars1) += Cq_1.transpose() * l;
        }

        if (variables_2->IsActive()) {
            result.segment(variables_2->GetOffset(), T::nvars2) += Cq_2.transpose() * l;
        }

        if (variables_3->IsActive()) {
            result.segment(variables_3->GetOffset(), T::nvars3) += Cq_3.transpose() * l;
        }

        if (variables_4->IsActive()) {
            result.segment(variables_4->GetOffset(), T::nvars4) += Cq_4.transpose() * l;
        }
    }

    void PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1, start_row, variables_1->GetOffset() + start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2, start_row, variables_2->GetOffset() + start_col);
        if (variables_3->IsActive())
            PasteMatrix(mat, Cq_3, start_row, variables_3->GetOffset() + start_col);
        if (variables_4->IsActive())
            PasteMatrix(mat, Cq_4, start_row, variables_4->GetOffset() + start_col);
    }

    void PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
        if (variables_1->IsActive())
            PasteMatrix(mat, Cq_1.transpose(), variables_1->GetOffset() + start_row, start_col);
        if (variables_2->IsActive())
            PasteMatrix(mat, Cq_2.transpose(), variables_2->GetOffset() + start_row, start_col);
        if (variables_3->IsActive())
            PasteMatrix(mat, Cq_3.transpose(), variables_3->GetOffset() + start_row, start_col);
        if (variables_4->IsActive())
            PasteMatrix(mat, Cq_4.transpose(), variables_4->GetOffset() + start_row, start_col);
    }
};

/// This is a set of 'helper' classes that make easier to manage the templated
/// structure of the tuple constraints.

template <int N1>
class ChVariableTupleCarrier_1vars {
  public:
    typedef ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<N1>> type_constraint_tuple;
    static const int nvars1 = N1;
    virtual ~ChVariableTupleCarrier_1vars() {}
    virtual ChVariables* GetVariables1() = 0;
};

template <int N1, int N2>
class ChVariableTupleCarrier_2vars {
  public:
    typedef ChConstraintTuple_2vars<ChVariableTupleCarrier_2vars<N1, N2>> type_constraint_tuple;
    static int const nvars1 = N1;
    static int const nvars2 = N2;
    virtual ~ChVariableTupleCarrier_2vars() {}
    virtual ChVariables* GetVariables1() = 0;
    virtual ChVariables* GetVariables2() = 0;
};

template <int N1, int N2, int N3>
class ChVariableTupleCarrier_3vars {
  public:
    typedef ChConstraintTuple_3vars<ChVariableTupleCarrier_3vars<N1, N2, N3>> type_constraint_tuple;
    static int const nvars1 = N1;
    static int const nvars2 = N2;
    static int const nvars3 = N3;
    virtual ~ChVariableTupleCarrier_3vars() {}
    virtual ChVariables* GetVariables1() = 0;
    virtual ChVariables* GetVariables2() = 0;
    virtual ChVariables* GetVariables3() = 0;
};

template <int N1, int N2, int N3, int N4>
class ChVariableTupleCarrier_4vars {
  public:
    typedef ChConstraintTuple_4vars<ChVariableTupleCarrier_4vars<N1, N2, N3, N4>> type_constraint_tuple;
    static int const nvars1 = N1;
    static int const nvars2 = N2;
    static int const nvars3 = N3;
    static int const nvars4 = N4;
    virtual ~ChVariableTupleCarrier_4vars() {}
    virtual ChVariables* GetVariables1() = 0;
    virtual ChVariables* GetVariables2() = 0;
    virtual ChVariables* GetVariables3() = 0;
    virtual ChVariables* GetVariables4() = 0;
};

}  // end namespace chrono

#endif
