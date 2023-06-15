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

#include "chrono/physics/ChLinkMask.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMask)

ChLinkMask::ChLinkMask() : nconstr(0) {}

ChLinkMask::ChLinkMask(int mnconstr) {
    nconstr = mnconstr;
    constraints.resize(nconstr);
    for (int i = 0; i < nconstr; i++) {
        constraints[i] = new ChConstraintTwoBodies;
    }
}

ChLinkMask::ChLinkMask(const ChLinkMask& other) {
    nconstr = other.nconstr;
    constraints.resize(other.nconstr);
    for (int i = 0; i < nconstr; i++) {
        constraints[i] = other.constraints[i]->Clone();
    }
}

ChLinkMask::~ChLinkMask() {
    for (int i = 0; i < nconstr; i++) {
        if (constraints[i]) {
            delete constraints[i];
        }
    }
}

ChLinkMask& ChLinkMask::operator=(const ChLinkMask& other) {
    if (this == &other)
        return *this;

    nconstr = other.nconstr;
    constraints.resize(nconstr);
    for (int i = 0; i < nconstr; i++) {
        constraints[i] = other.constraints[i]->Clone();
    }

    return *this;
}

void ChLinkMask::ResetNconstr(int newnconstr) {
    if (nconstr != newnconstr) {
        for (int i = 0; i < nconstr; i++)
            if (constraints[i]) {
                delete constraints[i];
                constraints[i] = nullptr;
            }

        nconstr = newnconstr;

        constraints.resize(nconstr);

        for (int i = 0; i < nconstr; i++) {
            constraints[i] = new ChConstraintTwoBodies;
        }
    }
}

void ChLinkMask::AddConstraint(ChConstraintTwoBodies* aconstr) {
    nconstr++;
    constraints.push_back(aconstr);
}

void ChLinkMask::SetTwoBodiesVariables(ChVariables* var1, ChVariables* var2) {
    for (int i = 0; i < nconstr; i++)
        constraints[i]->SetVariables(var1, var2);
}

bool ChLinkMask::IsEqual(ChLinkMask& mask2) {
    if (nconstr != mask2.nconstr)
        return false;
    for (int j = 0; j < nconstr; j++) {
        if (!(Constr_N(j) == mask2.Constr_N(j)))
            return false;
    }
    return true;
}

int ChLinkMask::GetMaskDoc() {
    int tot = 0;
    for (int j = 0; j < nconstr; j++) {
        if (Constr_N(j).IsActive())
            tot++;
    }
    return tot;
}

int ChLinkMask::GetMaskDoc_d() {
    int cnt = 0;
    for (int i = 0; i < nconstr; i++) {
        if (Constr_N(i).IsActive() && Constr_N(i).IsUnilateral())
            cnt++;
    }
    return cnt;
}

int ChLinkMask::GetMaskDoc_c() {
    return (GetMaskDoc() - GetMaskDoc_d());
}

ChConstraintTwoBodies* ChLinkMask::GetActiveConstrByNum(int mnum) {
    int cnt = 0;
    for (int i = 0; i < nconstr; i++) {
        if (Constr_N(i).IsActive()) {
            if (cnt == mnum)
                return &Constr_N(i);
            cnt++;
        }
    }
    return NULL;
}

int ChLinkMask::SetActiveRedundantByArray(int* mvector, int mcount) {
    int cnt;

    ChLinkMask newmask = *this;
    for (int elem = 0; elem < mcount; elem++) {
        cnt = 0;
        for (int i = 0; i < nconstr; i++) {
            if (constraints[i]->IsActive()) {
                if (cnt == mvector[elem])
                    newmask.constraints[i]->SetRedundant(true);
                cnt++;
            }
        }
    }

    // Replace the mask with updated one.
    for (int i = 0; i < nconstr; i++) {
        delete constraints[i];
        constraints[i] = newmask.constraints[i]->Clone();
    }

    return mcount;
}

// set lock =ON for constraints which were disabled because redundant
int ChLinkMask::RestoreRedundant() {
    int tot = 0;
    for (int j = 0; j < nconstr; j++) {
        if (Constr_N(j).IsRedundant()) {
            Constr_N(j).SetRedundant(false);
            tot++;
        }
    }
    return tot;
}

int ChLinkMask::SetAllDisabled(bool mdis) {
    int cnt = 0;

    for (int i = 0; i < nconstr; i++) {
        if (Constr_N(i).IsDisabled() != mdis) {
            Constr_N(i).SetDisabled(mdis);
            cnt++;
        }
    }

    return cnt;
}

int ChLinkMask::SetAllBroken(bool mdis) {
    int cnt = 0;

    for (int i = 0; i < nconstr; i++) {
        if (Constr_N(i).IsBroken() != mdis) {
            Constr_N(i).SetBroken(mdis);
            cnt++;
        }
    }

    return cnt;
}

void ChLinkMask::ArchiveOut(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChLinkMask>();

    marchive << CHNVP(constraints);
}

void ChLinkMask::ArchiveIn(ChArchiveIn& marchive) {
    /*int version =*/ marchive.VersionRead<ChLinkMask>();

    marchive >> CHNVP(constraints);

}

// -----------------------------------------------------------------------------
// Lock formulation LF link mask:
// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMaskLF)

ChLinkMaskLF::ChLinkMaskLF() {
    ResetNconstr(7);  // the LF formulation uses 7 constraint flags
}

ChLinkMaskLF& ChLinkMaskLF::operator=(const ChLinkMaskLF& other) {
    ChLinkMask::operator=(other);
    return *this;
}

void ChLinkMaskLF::SetLockMask(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3) {
    if (x)
        Constr_X().SetMode(CONSTRAINT_LOCK);
    else
        Constr_X().SetMode(CONSTRAINT_FREE);

    if (y)
        Constr_Y().SetMode(CONSTRAINT_LOCK);
    else
        Constr_Y().SetMode(CONSTRAINT_FREE);

    if (z)
        Constr_Z().SetMode(CONSTRAINT_LOCK);
    else
        Constr_Z().SetMode(CONSTRAINT_FREE);

    if (e0)
        Constr_E0().SetMode(CONSTRAINT_LOCK);
    else
        Constr_E0().SetMode(CONSTRAINT_FREE);

    if (e1)
        Constr_E1().SetMode(CONSTRAINT_LOCK);
    else
        Constr_E1().SetMode(CONSTRAINT_FREE);

    if (e2)
        Constr_E2().SetMode(CONSTRAINT_LOCK);
    else
        Constr_E2().SetMode(CONSTRAINT_FREE);

    if (e3)
        Constr_E3().SetMode(CONSTRAINT_LOCK);
    else
        Constr_E3().SetMode(CONSTRAINT_FREE);
}

void ChLinkMaskLF::ArchiveOut(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChLinkMaskLF>();
    ChLinkMask::ArchiveOut(marchive);
}

void ChLinkMaskLF::ArchiveIn(ChArchiveIn& marchive) {
    /*int version =*/ marchive.VersionRead<ChLinkMaskLF>();
    ChLinkMask::ArchiveIn(marchive);
}


}  // end namespace chrono
