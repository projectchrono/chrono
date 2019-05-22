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

#include "chrono/solver/ChConstraintThreeGeneric.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintThreeGeneric)

ChConstraintThreeGeneric::ChConstraintThreeGeneric()
    : Cq_a(nullptr), Cq_b(nullptr), Cq_c(nullptr), Eq_a(nullptr), Eq_b(nullptr), Eq_c(nullptr) {}

ChConstraintThreeGeneric::ChConstraintThreeGeneric(ChVariables* mvariables_a,
                                                   ChVariables* mvariables_b,
                                                   ChVariables* mvariables_c)
    : Cq_a(nullptr), Cq_b(nullptr), Cq_c(nullptr), Eq_a(nullptr), Eq_b(nullptr), Eq_c(nullptr) {
    SetVariables(mvariables_a, mvariables_b, mvariables_c);
}

ChConstraintThreeGeneric::ChConstraintThreeGeneric(const ChConstraintThreeGeneric& other) : ChConstraintThree(other) {
    Cq_a = Cq_b = Cq_c = Eq_a = Eq_b = Eq_c = nullptr;
    if (other.Cq_a)
        Cq_a = new ChMatrixDynamic<double>(*other.Cq_a);
    if (other.Cq_b)
        Cq_b = new ChMatrixDynamic<double>(*other.Cq_b);
    if (other.Cq_c)
        Cq_c = new ChMatrixDynamic<double>(*other.Cq_c);
    if (other.Eq_a)
        Eq_a = new ChMatrixDynamic<double>(*other.Eq_a);
    if (other.Eq_b)
        Eq_b = new ChMatrixDynamic<double>(*other.Eq_b);
    if (other.Eq_c)
        Eq_c = new ChMatrixDynamic<double>(*other.Eq_c);
}

ChConstraintThreeGeneric::~ChConstraintThreeGeneric() {
    delete Cq_a;
    delete Cq_b;
    delete Cq_c;
    delete Eq_a;
    delete Eq_b;
    delete Eq_c;
}

ChConstraintThreeGeneric& ChConstraintThreeGeneric::operator=(const ChConstraintThreeGeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintThree::operator=(other);

    if (other.Cq_a) {
        if (!Cq_a)
            Cq_a = new ChMatrixDynamic<double>;
        Cq_a->CopyFromMatrix(*other.Cq_a);
    } else {
        delete Cq_a;
        Cq_a = nullptr;
    }

    if (other.Cq_b) {
        if (!Cq_b)
            Cq_b = new ChMatrixDynamic<double>;
        Cq_b->CopyFromMatrix(*other.Cq_b);
    } else {
        delete Cq_b;
        Cq_b = nullptr;
    }

    if (other.Cq_c) {
        if (!Cq_c)
            Cq_c = new ChMatrixDynamic<double>;
        Cq_c->CopyFromMatrix(*other.Cq_c);
    } else {
        delete Cq_c;
        Cq_c = nullptr;
    }

    if (other.Eq_a) {
        if (!Eq_a)
            Eq_a = new ChMatrixDynamic<double>;
        Eq_a->CopyFromMatrix(*other.Eq_a);
    } else {
        delete Eq_a;
        Eq_a = nullptr;
    }

    if (other.Eq_b) {
        if (!Eq_b)
            Eq_b = new ChMatrixDynamic<double>;
        Eq_b->CopyFromMatrix(*other.Eq_b);
    } else {
        delete Eq_b;
        Eq_b = nullptr;
    }

    if (other.Eq_c) {
        if (!Eq_c)
            Eq_c = new ChMatrixDynamic<double>;
        Eq_c->CopyFromMatrix(*other.Eq_c);
    } else {
        delete Eq_c;
        Eq_c = nullptr;
    }

    return *this;
}

void ChConstraintThreeGeneric::SetVariables(ChVariables* mvariables_a,
                                            ChVariables* mvariables_b,
                                            ChVariables* mvariables_c) {
    if (!mvariables_a || !mvariables_b || !mvariables_c) {
        SetValid(false);
        return;
    }

    SetValid(true);
    variables_a = mvariables_a;
    variables_b = mvariables_b;
    variables_c = mvariables_c;

    if (variables_a->Get_ndof()) {
        if (!Cq_a)
            Cq_a = new ChMatrixDynamic<double>(1, variables_a->Get_ndof());
        else
            Cq_a->Resize(1, variables_a->Get_ndof());

        if (!Eq_a)
            Eq_a = new ChMatrixDynamic<double>(variables_a->Get_ndof(), 1);
        else
            Eq_a->Resize(variables_a->Get_ndof(), 1);

        Cq_a->Reset();
    } else {
        delete Cq_a;
        delete Eq_a;
        Cq_a = nullptr;
        Eq_a = nullptr;
    }

    if (variables_b->Get_ndof()) {
        if (!Cq_b)
            Cq_b = new ChMatrixDynamic<double>(1, variables_b->Get_ndof());
        else
            Cq_b->Resize(1, variables_b->Get_ndof());

        if (!Eq_b)
            Eq_b = new ChMatrixDynamic<double>(variables_b->Get_ndof(), 1);
        else
            Eq_b->Resize(variables_b->Get_ndof(), 1);

        Cq_b->Reset();
    } else {
        delete Cq_b;
        delete Eq_b;
        Cq_b = nullptr;
        Eq_b = nullptr;
    }

    if (variables_c->Get_ndof()) {
        if (!Cq_c)
            Cq_c = new ChMatrixDynamic<double>(1, variables_c->Get_ndof());
        else
            Cq_c->Resize(1, variables_c->Get_ndof());

        if (!Eq_c)
            Eq_c = new ChMatrixDynamic<double>(variables_c->Get_ndof(), 1);
        else
            Eq_c->Resize(variables_c->Get_ndof(), 1);

        Cq_c->Reset();
    } else {
        delete Cq_c;
        delete Eq_c;
        Cq_c = nullptr;
        Eq_c = nullptr;
    }
}

void ChConstraintThreeGeneric::Update_auxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
    if (variables_a->IsActive())
        if (variables_a->Get_ndof()) {
            ChMatrixDynamic<double> mtemp1(variables_a->Get_ndof(), 1);
            mtemp1.CopyFromMatrixT(*Cq_a);
            variables_a->Compute_invMb_v(*Eq_a, mtemp1);
        }
    if (variables_b->IsActive())
        if (variables_b->Get_ndof()) {
            ChMatrixDynamic<double> mtemp1(variables_b->Get_ndof(), 1);
            mtemp1.CopyFromMatrixT(*Cq_b);
            variables_b->Compute_invMb_v(*Eq_b, mtemp1);
        }
    if (variables_c->IsActive())
        if (variables_c->Get_ndof()) {
            ChMatrixDynamic<double> mtemp1(variables_c->Get_ndof(), 1);
            mtemp1.CopyFromMatrixT(*Cq_c);
            variables_c->Compute_invMb_v(*Eq_c, mtemp1);
        }

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    ChMatrixDynamic<double> res(1, 1);
    g_i = 0;
    if (variables_a->IsActive())
        if (variables_a->Get_ndof()) {
            res.MatrMultiply(*Cq_a, *Eq_a);
            g_i = res(0, 0);
        }
    if (variables_b->IsActive())
        if (variables_b->Get_ndof()) {
            res.MatrMultiply(*Cq_b, *Eq_b);
            g_i += res(0, 0);
        }
    if (variables_c->IsActive())
        if (variables_c->Get_ndof()) {
            res.MatrMultiply(*Cq_c, *Eq_c);
            g_i += res(0, 0);
        }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintThreeGeneric::Compute_Cq_q() {
    double ret = 0;

    if (variables_a->IsActive())
        for (int i = 0; i < Cq_a->GetColumns(); i++)
            ret += Cq_a->ElementN(i) * variables_a->Get_qb().ElementN(i);

    if (variables_b->IsActive())
        for (int i = 0; i < Cq_b->GetColumns(); i++)
            ret += Cq_b->ElementN(i) * variables_b->Get_qb().ElementN(i);

    if (variables_c->IsActive())
        for (int i = 0; i < Cq_c->GetColumns(); i++)
            ret += Cq_c->ElementN(i) * variables_c->Get_qb().ElementN(i);

    return ret;
}

void ChConstraintThreeGeneric::Increment_q(const double deltal) {
    if (variables_a->IsActive())
        for (int i = 0; i < Eq_a->GetRows(); i++)
            variables_a->Get_qb()(i) += Eq_a->ElementN(i) * deltal;

    if (variables_b->IsActive())
        for (int i = 0; i < Eq_b->GetRows(); i++)
            variables_b->Get_qb()(i) += Eq_b->ElementN(i) * deltal;

    if (variables_c->IsActive())
        for (int i = 0; i < Eq_c->GetRows(); i++)
            variables_c->Get_qb()(i) += Eq_c->ElementN(i) * deltal;
}

void ChConstraintThreeGeneric::MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
    if (variables_a->IsActive())
        for (int i = 0; i < Cq_a->GetRows(); i++)
            result += vect(variables_a->GetOffset() + i) * Cq_a->ElementN(i);

    if (variables_b->IsActive())
        for (int i = 0; i < Cq_b->GetRows(); i++)
            result += vect(variables_b->GetOffset() + i) * Cq_b->ElementN(i);

    if (variables_c->IsActive())
        for (int i = 0; i < Cq_c->GetRows(); i++)
            result += vect(variables_c->GetOffset() + i) * Cq_c->ElementN(i);
}

void ChConstraintThreeGeneric::MultiplyTandAdd(ChMatrix<double>& result, double l) {
    if (variables_a->IsActive())
        for (int i = 0; i < Cq_a->GetRows(); i++)
            result(variables_a->GetOffset() + i) += Cq_a->ElementN(i) * l;

    if (variables_b->IsActive())
        for (int i = 0; i < Cq_b->GetRows(); i++)
            result(variables_b->GetOffset() + i) += Cq_b->ElementN(i) * l;

    if (variables_c->IsActive())
        for (int i = 0; i < Cq_c->GetRows(); i++)
            result(variables_c->GetOffset() + i) += Cq_c->ElementN(i) * l;
}

void ChConstraintThreeGeneric::Build_Cq(ChSparseMatrix& storage, int insrow) {
    if (variables_a->IsActive())
        storage.PasteMatrix(*Cq_a, insrow, variables_a->GetOffset());
    if (variables_b->IsActive())
        storage.PasteMatrix(*Cq_b, insrow, variables_b->GetOffset());
    if (variables_c->IsActive())
        storage.PasteMatrix(*Cq_c, insrow, variables_c->GetOffset());
}

void ChConstraintThreeGeneric::Build_CqT(ChSparseMatrix& storage, int inscol) {
    if (variables_a->IsActive())
        storage.PasteTranspMatrix(*Cq_a, variables_a->GetOffset(), inscol);
    if (variables_b->IsActive())
        storage.PasteTranspMatrix(*Cq_b, variables_b->GetOffset(), inscol);
    if (variables_c->IsActive())
        storage.PasteTranspMatrix(*Cq_c, variables_c->GetOffset(), inscol);
}

void ChConstraintThreeGeneric::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintThreeGeneric>();

    // serialize the parent class data too
    ChConstraintThree::ArchiveOUT(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

void ChConstraintThreeGeneric::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChConstraintThreeGeneric>();

    // deserialize the parent class data too
    ChConstraintThree::ArchiveIN(marchive);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

}  // end namespace chrono
