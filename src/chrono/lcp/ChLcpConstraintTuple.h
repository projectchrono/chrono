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

#ifndef CHLCPCONSTRAINTTUPLE_H
#define CHLCPCONSTRAINTTUPLE_H



#include "ChLcpConstraint.h"
#include "ChLcpVariables.h"
#include "core/ChMatrixNM.h"

namespace chrono {



/// This is a container for 'half' of a constraint, and contains a tuple of
/// 1 or 2 or 3 differently-sized jacobian chunks. For instance, this might
/// happen because you want a constraint between an edge (i.e. two xyz variables, each
/// per end nodes) and a triangle face (i.e. three xyz variables, each per corner), so
/// the jacobian row matrix is split in 2 + 3 chunks, here as two tuples.
/// The complete constraint, ChLcpConstraintTwoTuples, will use two of these classes.
/// Template T is a class of ChLcpVariableTupleCarrier_Nvars type

template <class T>
class ChLcpConstraintTuple_1vars {

  protected:
    /// The first  constrained object
    ChLcpVariables* variables;

    /// The [Cq] jacobian of the constraint 
    ChMatrixNM<double, 1, T::nvars1> Cq;
    /// The [Eq] product [Eq]=[invM]*[Cq]'
    ChMatrixNM<double, T::nvars1, 1> Eq;

  public:

    /// Default constructor
    ChLcpConstraintTuple_1vars() : variables(0) {}

    /// Copy constructor
    ChLcpConstraintTuple_1vars(const ChLcpConstraintTuple_1vars& other) {
        variables = other.variables;
        Cq = other.Cq;
        Eq = other.Eq;
    }

    ~ChLcpConstraintTuple_1vars() {}

    /// Assignment operator: copy from other object
    ChLcpConstraintTuple_1vars& operator=(const ChLcpConstraintTuple_1vars& other) {
        variables = other.variables;
        Cq = other.Cq;
        Eq = other.Eq;
    }

    ChMatrix<double>* Get_Cq() { return &Cq; }

    ChMatrix<double>* Get_Eq() { return &Eq; }

    ChLcpVariables* GetVariables() { return variables; }

    void SetVariables(T& m_tuple_carrier) {

        if (!m_tuple_carrier.GetVariables1()) {
            throw ChException("ERROR. SetVariables() getting null pointer. \n");
        }
        variables = m_tuple_carrier.GetVariables1();
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq]=[invM]*[Cq]' and [Eq]
        if (variables->IsActive()) {
            ChMatrixNM<double, T::nvars1, 1> mtemp1;
            mtemp1.CopyFromMatrixT(Cq);
            variables->Compute_invMb_v(Eq, mtemp1);
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        ChMatrixNM<double, 1, 1> res;
        if (variables->IsActive()) {
            res.MatrMultiply(Cq, Eq);
            g_i += res(0, 0);
        }
    }

    double Compute_Cq_q() {
        double ret = 0;

        if (variables->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                ret += Cq.ElementN(i) * variables->Get_qb().ElementN(i);

        return ret;
    }

    void Increment_q(const double deltal) {
        if (variables->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                variables->Get_qb()(i) += Eq.ElementN(i) * deltal;
    }

    void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
        int off = variables->GetOffset();

        if (variables->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                result += vect(off + i) * Cq.ElementN(i);
    }


    void MultiplyTandAdd(ChMatrix<double>& result, double l) {
        int off = variables->GetOffset();

        if (variables->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                result(off + i) += Cq.ElementN(i) * l;
    }

    void Build_Cq(ChSparseMatrixBase& storage, int insrow) {
        if (variables->IsActive())
            storage.PasteMatrix(&Cq, insrow, variables->GetOffset());
    }

    void Build_CqT(ChSparseMatrixBase& storage, int inscol) {
        if (variables->IsActive())
            storage.PasteTranspMatrix(&Cq, variables->GetOffset(), inscol);
    }

};


///// Case of tuple with reference to 2 ChLcpVariable objects:


template <class T>
class ChLcpConstraintTuple_2vars { 

  protected:
    ChLcpVariables* variables_1;
    ChLcpVariables* variables_2;

    /// The [Cq] jacobian of the constraint, split in horizontal chunks
    ChMatrixNM<double, 1, T::nvars1> Cq_1;
    ChMatrixNM<double, 1, T::nvars2> Cq_2;

    /// The [Eq] product [Eq]=[invM]*[Cq]' , split in horizontal chunks
    ChMatrixNM<double, T::nvars1, 1> Eq_1;
    ChMatrixNM<double, T::nvars2, 1> Eq_2;

  public:

    /// Default constructor
    ChLcpConstraintTuple_2vars() : 
        variables_1(0), 
        variables_2(0)
        {}

    /// Copy constructor
    ChLcpConstraintTuple_2vars(const ChLcpConstraintTuple_2vars& other)  {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
    }

    ~ChLcpConstraintTuple_2vars() {}

    /// Assignment operator: copy from other object
    ChLcpConstraintTuple_2vars& operator=(const ChLcpConstraintTuple_2vars& other) {
        variables_1 = other.variables_1;
        variables_2 = other.variables_2;
        Cq_1 = other.Cq_1;
        Cq_2 = other.Cq_2;
        Eq_1 = other.Eq_1;
        Eq_2 = other.Eq_2;
    }

    ChMatrix<double>* Get_Cq_1() { return &Cq_1; }
    ChMatrix<double>* Get_Cq_2() { return &Cq_2; }

    ChMatrix<double>* Get_Eq_1() { return &Eq_1; }
    ChMatrix<double>* Get_Eq_2() { return &Eq_2; }

    ChLcpVariables* GetVariables_1() { return variables_1; }
    ChLcpVariables* GetVariables_2() { return variables_2; }

    void SetVariables(T& m_tuple_carrier) {
        if (!m_tuple_carrier.GetVariables1() || !m_tuple_carrier.GetVariables2()) {
            throw ChException("ERROR. SetVariables() getting null pointer. \n");
        }
        variables_1 = m_tuple_carrier.GetVariables1();
        variables_2 = m_tuple_carrier.GetVariables2();
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
        if (variables_1->IsActive()) {
            ChMatrixNM<double, T::nvars1, 1> mtemp1;
            mtemp1.CopyFromMatrixT(Cq_1);
            variables_1->Compute_invMb_v(Eq_1, mtemp1);
        }
        if (variables_2->IsActive()) {
            ChMatrixNM<double, T::nvars2, 1> mtemp1;
            mtemp1.CopyFromMatrixT(Cq_2);
            variables_2->Compute_invMb_v(Eq_2, mtemp1);
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        ChMatrixNM<double, 1, 1> res;
        if (variables_1->IsActive()) {
            res.MatrMultiply(Cq_1, Eq_1);
            g_i += res(0, 0);
        }
        if (variables_2->IsActive()) {
            res.MatrMultiply(Cq_2, Eq_2);
            g_i += res(0, 0);
        }
    }

    double Compute_Cq_q() {
        double ret = 0;

        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                ret += Cq_1.ElementN(i) * variables_1->Get_qb().ElementN(i);

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                ret += Cq_2.ElementN(i) * variables_2->Get_qb().ElementN(i);

        return ret;
    }

    void Increment_q(const double deltal) {
        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                variables_1->Get_qb()(i) += Eq_1.ElementN(i) * deltal;

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                variables_2->Get_qb()(i) += Eq_2.ElementN(i) * deltal;
    }

    void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
        int off_1 = variables_1->GetOffset();

        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                result += vect(off_1 + i) * Cq_1.ElementN(i);

        int off_2 = variables_2->GetOffset();

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                result += vect(off_2 + i) * Cq_2.ElementN(i);
    }

    void MultiplyTandAdd(ChMatrix<double>& result, double l) {
        int off_1 = variables_1->GetOffset();

        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                result(off_1 + i) += Cq_1.ElementN(i) * l;

        int off_2 = variables_2->GetOffset();

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                result(off_2 + i) += Cq_2.ElementN(i) * l;
    }

    void Build_Cq(ChSparseMatrixBase& storage, int insrow) {
        if (variables_1->IsActive())
            storage.PasteMatrix(&Cq_1, insrow, variables_1->GetOffset());
        if (variables_2->IsActive())
            storage.PasteMatrix(&Cq_2, insrow, variables_2->GetOffset());
    }

    void Build_CqT(ChSparseMatrixBase& storage, int inscol) {
        if (variables_1->IsActive())
            storage.PasteTranspMatrix(&Cq_1, variables_1->GetOffset(), inscol);
        if (variables_2->IsActive())
            storage.PasteTranspMatrix(&Cq_2, variables_2->GetOffset(), inscol);
    }

};


///// Case of tuple with reference to 3 ChLcpVariable objects:




template <class T>
class ChLcpConstraintTuple_3vars { 

  protected:
    ChLcpVariables* variables_1;
    ChLcpVariables* variables_2;
    ChLcpVariables* variables_3;

    /// The [Cq] jacobian of the constraint, split in horizontal chunks
    ChMatrixNM<double, 1, T::nvars1> Cq_1;
    ChMatrixNM<double, 1, T::nvars2> Cq_2;
    ChMatrixNM<double, 1, T::nvars3> Cq_3;

    /// The [Eq] product [Eq]=[invM]*[Cq]' , split in horizontal chunks
    ChMatrixNM<double, T::nvars1, 1> Eq_1;
    ChMatrixNM<double, T::nvars2, 1> Eq_2;
    ChMatrixNM<double, T::nvars3, 1> Eq_3;

  public:

    /// Default constructor
    ChLcpConstraintTuple_3vars() : 
        variables_1(0), 
        variables_2(0),
        variables_3(0)
        {}

    /// Copy constructor
    ChLcpConstraintTuple_3vars(const ChLcpConstraintTuple_3vars& other) {
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

    ~ChLcpConstraintTuple_3vars() {}

    /// Assignment operator: copy from other object
    ChLcpConstraintTuple_3vars& operator=(const ChLcpConstraintTuple_3vars& other) {
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

    ChMatrix<double>* Get_Cq_1() { return &Cq_1; }
    ChMatrix<double>* Get_Cq_2() { return &Cq_2; }
    ChMatrix<double>* Get_Cq_3() { return &Cq_3; }

    ChMatrix<double>* Get_Eq_1() { return &Eq_1; }
    ChMatrix<double>* Get_Eq_2() { return &Eq_2; }
    ChMatrix<double>* Get_Eq_3() { return &Eq_3; }

    ChLcpVariables* GetVariables_1() { return variables_1; }
    ChLcpVariables* GetVariables_2() { return variables_2; }
    ChLcpVariables* GetVariables_3() { return variables_3; }

    void SetVariables(T& m_tuple_carrier) {
        if (!m_tuple_carrier.GetVariables1() || !m_tuple_carrier.GetVariables2() || !m_tuple_carrier.GetVariables3()) {
            throw ChException("ERROR. SetVariables() getting null pointer. \n");
        }
        variables_1 = m_tuple_carrier.GetVariables1() ;
        variables_2 = m_tuple_carrier.GetVariables2() ;
        variables_3 = m_tuple_carrier.GetVariables3() ;
    }

    void Update_auxiliary(double& g_i) {
        // 1- Assuming jacobians are already computed, now compute
        //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
        if (variables_1->IsActive()) {
            ChMatrixNM<double, T::nvars1, 1> mtemp1;
            mtemp1.CopyFromMatrixT(Cq_1);
            variables_1->Compute_invMb_v(Eq_1, mtemp1);
        }
        if (variables_2->IsActive()) {
            ChMatrixNM<double, T::nvars2, 1> mtemp1;
            mtemp1.CopyFromMatrixT(Cq_2);
            variables_2->Compute_invMb_v(Eq_2, mtemp1);
        }
        if (variables_3->IsActive()) {
            ChMatrixNM<double, T::nvars3, 1> mtemp1;
            mtemp1.CopyFromMatrixT(Cq_3);
            variables_3->Compute_invMb_v(Eq_3, mtemp1);
        }

        // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
        ChMatrixNM<double, 1, 1> res;
        if (variables_1->IsActive()) {
            res.MatrMultiply(Cq_1, Eq_1);
            g_i += res(0, 0);
        }
        if (variables_2->IsActive()) {
            res.MatrMultiply(Cq_2, Eq_2);
            g_i += res(0, 0);
        }
        if (variables_3->IsActive()) {
            res.MatrMultiply(Cq_3, Eq_3);
            g_i += res(0, 0);
        }
    }

    double Compute_Cq_q() {
        double ret = 0;

        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                ret += Cq_1.ElementN(i) * variables_1->Get_qb().ElementN(i);

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                ret += Cq_2.ElementN(i) * variables_2->Get_qb().ElementN(i);

        if (variables_3->IsActive())
            for (int i = 0; i < T::nvars3; i++)
                ret += Cq_3.ElementN(i) * variables_3->Get_qb().ElementN(i);

        return ret;
    }

    void Increment_q(const double deltal) {
        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                variables_1->Get_qb()(i) += Eq_1.ElementN(i) * deltal;

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                variables_2->Get_qb()(i) += Eq_2.ElementN(i) * deltal;

        if (variables_3->IsActive())
            for (int i = 0; i < T::nvars3; i++)
                variables_3->Get_qb()(i) += Eq_3.ElementN(i) * deltal;

    }

    void MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
        int off_1 = variables_1->GetOffset();

        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                result += vect(off_1 + i) * Cq_1.ElementN(i);

        int off_2 = variables_2->GetOffset();

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                result += vect(off_2 + i) * Cq_2.ElementN(i);

        int off_3 = variables_3->GetOffset();

        if (variables_3->IsActive())
            for (int i = 0; i < T::nvars3; i++)
                result += vect(off_3 + i) * Cq_3.ElementN(i);
    }

    void MultiplyTandAdd(ChMatrix<double>& result, double l) {
        int off_1 = variables_1->GetOffset();

        if (variables_1->IsActive())
            for (int i = 0; i < T::nvars1; i++)
                result(off_1 + i) += Cq_1.ElementN(i) * l;

        int off_2 = variables_2->GetOffset();

        if (variables_2->IsActive())
            for (int i = 0; i < T::nvars2; i++)
                result(off_2 + i) += Cq_2.ElementN(i) * l;

        int off_3 = variables_3->GetOffset();

        if (variables_3->IsActive())
            for (int i = 0; i < T::nvars3; i++)
                result(off_3 + i) += Cq_3.ElementN(i) * l;
    }

    void Build_Cq(ChSparseMatrixBase& storage, int insrow) {
        if (variables_1->IsActive())
            storage.PasteMatrix(&Cq_1, insrow, variables_1->GetOffset());
        if (variables_2->IsActive())
            storage.PasteMatrix(&Cq_2, insrow, variables_2->GetOffset());
        if (variables_3->IsActive())
            storage.PasteMatrix(&Cq_3, insrow, variables_3->GetOffset());
    }

    void Build_CqT(ChSparseMatrixBase& storage, int inscol) {
        if (variables_1->IsActive())
            storage.PasteTranspMatrix(&Cq_1, variables_1->GetOffset(), inscol);
        if (variables_2->IsActive())
            storage.PasteTranspMatrix(&Cq_2, variables_2->GetOffset(), inscol);
        if (variables_3->IsActive())
            storage.PasteTranspMatrix(&Cq_3, variables_3->GetOffset(), inscol);
    }

};




/// This is a set of 'helper' classes that make easier to manage the templated 
/// structure of the tuple constraints.

template <int N1>
class ChLcpVariableTupleCarrier_1vars {
public:
    typedef ChLcpConstraintTuple_1vars< ChLcpVariableTupleCarrier_1vars< N1 > > type_constraint_tuple;
    static const int nvars1 = N1;
    virtual ChLcpVariables* GetVariables1() = 0;
};

template <int N1, int N2>
class ChLcpVariableTupleCarrier_2vars {
public:
    typedef ChLcpConstraintTuple_3vars< ChLcpVariableTupleCarrier_2vars< N1, N2> > type_constraint_tuple;
    static int  const nvars1 = N1;
    static int  const nvars2 = N2;
    virtual ChLcpVariables* GetVariables1() = 0;
    virtual ChLcpVariables* GetVariables2() = 0;
};

template <int N1, int N2, int N3>
class ChLcpVariableTupleCarrier_3vars {
public:
    typedef ChLcpConstraintTuple_3vars< ChLcpVariableTupleCarrier_3vars< N1, N2, N3 > > type_constraint_tuple;
    static int  const nvars1 = N1;
    static int  const nvars2 = N2;
    static int  const nvars3 = N3;
    virtual ChLcpVariables* GetVariables1() = 0;
    virtual ChLcpVariables* GetVariables2() = 0;
    virtual ChLcpVariables* GetVariables3() = 0;
};



}  // END_OF_NAMESPACE____

#endif  
