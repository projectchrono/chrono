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

ChLinkMask::ChLinkMask(unsigned int mnconstr) {
    nconstr = mnconstr;
    constraints.resize(nconstr);
    for (unsigned int i = 0; i < nconstr; i++) {
        constraints[i] = new ChConstraintTwoBodies;
    }
}

ChLinkMask::ChLinkMask(const ChLinkMask& other) {
    nconstr = other.nconstr;
    constraints.resize(other.nconstr);
    for (unsigned int i = 0; i < nconstr; i++) {
        constraints[i] = other.constraints[i]->Clone();
    }
}

ChLinkMask::~ChLinkMask() {
    for (unsigned int i = 0; i < nconstr; i++) {
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
    for (unsigned int i = 0; i < nconstr; i++) {
        constraints[i] = other.constraints[i]->Clone();
    }

    return *this;
}

void ChLinkMask::SetNumConstraints(unsigned int newnconstr) {
    if (nconstr != newnconstr) {
        for (unsigned int i = 0; i < nconstr; i++)
            if (constraints[i]) {
                delete constraints[i];
                constraints[i] = nullptr;
            }

        nconstr = newnconstr;

        constraints.resize(nconstr);

        for (unsigned int i = 0; i < nconstr; i++) {
            constraints[i] = new ChConstraintTwoBodies;
        }
    }
}

void ChLinkMask::AddConstraint(ChConstraintTwoBodies* aconstr) {
    nconstr++;
    constraints.push_back(aconstr);
}

void ChLinkMask::SetTwoBodiesVariables(ChVariables* var1, ChVariables* var2) {
    for (unsigned int i = 0; i < nconstr; i++)
        constraints[i]->SetVariables(var1, var2);
}

ChConstraintTwoBodies& ChLinkMask::GetConstraint(unsigned int i) {
    assert((i >= 0) && (i < nconstr));
    return *constraints[i];
}

bool ChLinkMask::IsEqual(ChLinkMask& mask2) {
    if (nconstr != mask2.nconstr)
        return false;
    for (unsigned int j = 0; j < nconstr; j++) {
        if (!(GetConstraint(j) == mask2.GetConstraint(j)))
            return false;
    }
    return true;
}

unsigned int ChLinkMask::GetNumConstraintsActive() {
    unsigned int tot = 0;
    for (unsigned int j = 0; j < nconstr; j++) {
        if (GetConstraint(j).IsActive())
            tot++;
    }
    return tot;
}

unsigned int ChLinkMask::GetNumConstraintsUnilateralActive() {
    unsigned int cnt = 0;
    for (unsigned int i = 0; i < nconstr; i++) {
        if (GetConstraint(i).IsActive() && GetConstraint(i).IsUnilateral())
            cnt++;
    }
    return cnt;
}

unsigned int ChLinkMask::GetNumConstraintsBilateralActive() {
    return (GetNumConstraintsActive() - GetNumConstraintsUnilateralActive());
}

ChConstraintTwoBodies* ChLinkMask::GetActiveConstraint(unsigned int mnum) {
    unsigned int cnt = 0;
    for (unsigned int i = 0; i < nconstr; i++) {
        if (GetConstraint(i).IsActive()) {
            if (cnt == mnum)
                return &GetConstraint(i);
            cnt++;
        }
    }
    return NULL;
}

unsigned int ChLinkMask::SetAllDisabled(bool mdis) {
    unsigned int cnt = 0;

    for (unsigned int i = 0; i < nconstr; i++) {
        if (GetConstraint(i).IsDisabled() != mdis) {
            GetConstraint(i).SetDisabled(mdis);
            cnt++;
        }
    }

    return cnt;
}

unsigned int ChLinkMask::SetAllBroken(bool mdis) {
    unsigned int cnt = 0;

    for (unsigned int i = 0; i < nconstr; i++) {
        if (GetConstraint(i).IsBroken() != mdis) {
            GetConstraint(i).SetBroken(mdis);
            cnt++;
        }
    }

    return cnt;
}

void ChLinkMask::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChLinkMask>();

    archive_out << CHNVP(constraints);
}

void ChLinkMask::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChLinkMask>();

    archive_in >> CHNVP(constraints);
}

// -----------------------------------------------------------------------------
// Lock formulation LF link mask:
// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMaskLF)

ChLinkMaskLF::ChLinkMaskLF() {
    SetNumConstraints(7);  // the LF formulation uses 7 constraint flags
}

ChLinkMaskLF& ChLinkMaskLF::operator=(const ChLinkMaskLF& other) {
    ChLinkMask::operator=(other);
    return *this;
}

void ChLinkMaskLF::SetLockMask(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3) {
    if (x)
        Constr_X().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_X().SetMode(ChConstraint::Mode::FREE);

    if (y)
        Constr_Y().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_Y().SetMode(ChConstraint::Mode::FREE);

    if (z)
        Constr_Z().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_Z().SetMode(ChConstraint::Mode::FREE);

    if (e0)
        Constr_E0().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_E0().SetMode(ChConstraint::Mode::FREE);

    if (e1)
        Constr_E1().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_E1().SetMode(ChConstraint::Mode::FREE);

    if (e2)
        Constr_E2().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_E2().SetMode(ChConstraint::Mode::FREE);

    if (e3)
        Constr_E3().SetMode(ChConstraint::Mode::LOCK);
    else
        Constr_E3().SetMode(ChConstraint::Mode::FREE);
}

void ChLinkMaskLF::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChLinkMaskLF>();
    ChLinkMask::ArchiveOut(archive_out);
}

void ChLinkMaskLF::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChLinkMaskLF>();
    ChLinkMask::ArchiveIn(archive_in);
}

}  // end namespace chrono
