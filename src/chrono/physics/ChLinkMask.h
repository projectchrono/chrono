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

#ifndef CHLINKMASK_H
#define CHLINKMASK_H

#include <cmath>
#include <vector>

#include "chrono/core/ChMath.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Mask structure for N scalar constraint equations between two bodies.

class ChApi ChLinkMask {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkMask)

  protected:
    std::vector<ChConstraintTwoBodies*> constraints;  ///< array of pointers to 'n' scalar constraint states

  public:
    int nconstr;  ///< number of scalar eq.of constraint.

    /// Build a link mask with a single constraint of class ChConstraintTwoBodies().
    ChLinkMask();

    /// Build a link mask with a default array of mnconstr constraints
    /// of class ChConstraintTwoBodies().
    ChLinkMask(int mnconstr);

    /// Copy constructor
    ChLinkMask(const ChLinkMask& source);

    /// Destructor
    virtual ~ChLinkMask();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMask* Clone() const { return new ChLinkMask(*this); }

    /// Set references to variables of two connected bodies to all
    /// constraints at once, therefore also sets all the constraints as active.
    void SetTwoBodiesVariables(ChVariables* var1, ChVariables* var2);

    /// Obtain the reference to the i-th scalar constraint data
    /// in the collection link mask.
    ChConstraintTwoBodies& Constr_N(int i) {
        assert((i >= 0) && (i < nconstr));
        return *constraints[i];
    }

    /// Utility: to change the size of the mask, reallocating constraints
    /// (all of type ChConstraintTwo).
    /// No action if newnconstr == nconstr
    void ResetNconstr(int newnconstr);

    /// Add a ChConstraintTwoBodies to mask (NOTE: later, the constraint will
    /// be automatically deleted when the mask will be deleted)
    void AddConstraint(ChConstraintTwoBodies* aconstr);

    /// To compare two masks, return true if equal
    bool IsEqual(ChLinkMask& mask2);

    /// Tells if i-th equation is a unilateral constraint
    bool IsUnilateral(int i);

    // Get the number of removed degrees of freedom (n.of costraints)

    /// Count both bilaterals and unilaterals
    int GetMaskDoc();
    /// Count only bilaterals
    int GetMaskDoc_c();
    /// Count only unilaterals
    int GetMaskDoc_d();

    /// Get the i-th active scalar costraint (not active constr. won't be considered)
    ChConstraintTwoBodies* GetActiveConstrByNum(int mnum);

    /// Sets some active constraints as redundant.
    int SetActiveRedundantByArray(int* mvector, int mcount);

    /// Set lock =ON for costraints which were disabled because redundant
    int RestoreRedundant();

    /// If SetAllDisabled(true), all the constraints are temporarily turned
    /// off (inactive) at once, because marked as 'disabled'. Return n.of changed
    int SetAllDisabled(bool mdis);

    /// If SetAllBroken(true), all the constraints are temporarily turned
    /// off (broken) at once, because marked as 'broken'. Return n.of changed.
    int SetAllBroken(bool mdis);

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChLinkMask,0)


// -----------------------------------------------------------------------------

/// Specialized ChLinkMask class, for constraint equations of
/// the ChLinkLock link.

class ChApi ChLinkMaskLF : public ChLinkMask {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkMaskLF)

  public:
    /// Create a ChLinkMaskLF which has 7 scalar constraints of
    /// class ChConstraintTwoBodies(). This is useful in case it must
    /// be used for the ChLinkLock link.
    ChLinkMaskLF();
    ChLinkMaskLF(const ChLinkMaskLF& other) : ChLinkMask(other) {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMaskLF* Clone() const override { return new ChLinkMaskLF(*this); }

    /// Set all mask data at once
    void SetLockMask(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3);

    /// Obtain the reference to specific scalar constraint data
    /// in the collection of this link mask.
    ChConstraintTwoBodies& Constr_X() { return *constraints[0]; }
    ChConstraintTwoBodies& Constr_Y() { return *constraints[1]; }
    ChConstraintTwoBodies& Constr_Z() { return *constraints[2]; }
    ChConstraintTwoBodies& Constr_E0() { return *constraints[3]; }
    ChConstraintTwoBodies& Constr_E1() { return *constraints[4]; }
    ChConstraintTwoBodies& Constr_E2() { return *constraints[5]; }
    ChConstraintTwoBodies& Constr_E3() { return *constraints[6]; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkMaskLF,0)


}  // end namespace chrono

#endif
