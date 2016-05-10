//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKMASK_H
#define CHLINKMASK_H

#include <math.h>
#include <vector>

#include "chrono/core/ChMath.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

///
/// Mask structure for N scalar constraint equations between two bodies.
///

class ChApi ChLinkMask {
    CH_RTTI_ROOT(ChLinkMask);

    //
    // DATA
    //

  protected:
    // Array of pointers to 'n' scalar constraint states (own by this object)
    std::vector<ChConstraintTwoBodies*> constraints;

  public:
    int nconstr;  // Number of scalar eq.of constraint.
                  // Maybe different from effective nDOC because someone may be free/redundant/etc.

    //
    // CONSTRUCTORS
    //

    /// Build a link mask with a single constraint
    /// of class ChConstraintTwoBodies().
    ChLinkMask();

    /// Build a link mask with a default array of mnconstr constraints
    /// of class ChConstraintTwoBodies().
    ChLinkMask(int mnconstr);

    virtual ~ChLinkMask();

    ChLinkMask(ChLinkMask& source);
    virtual void Copy(ChLinkMask* source);
    virtual ChLinkMask* NewDuplicate();

    //
    // FUNCTIONS
    //

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

    /// To compare two masks, return TRUE if equal
    int IsEqual(ChLinkMask& mask2);

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

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

///
/// Specialized ChLinkMask class, for constraint equations of
/// the ChLinkLock link.
///

class ChApi ChLinkMaskLF : public ChLinkMask {
    CH_RTTI(ChLinkMaskLF, ChLinkMask);

  public:
    /// Create a ChLinkMaskLF which has 7 scalar constraints of
    /// class ChConstraintTwoBodies(). This is useful in case it must
    /// be used for the ChLinkLock link.
    ChLinkMaskLF();

    void Copy(ChLinkMaskLF* source);
    ChLinkMask* NewDuplicate();

    /// set all mask data at once
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

    //
    // STREAMING
    //

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

}  // end namespace chrono

#endif
