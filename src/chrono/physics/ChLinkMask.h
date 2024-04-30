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

#ifndef CHLINKMASK_H
#define CHLINKMASK_H

#include <cmath>
#include <vector>

#include "chrono/core/ChFrame.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Mask structure for N scalar constraint equations between two bodies.
class ChApi ChLinkMask {
  public:
    /// Build a link mask with no constraints.
    ChLinkMask();

    /// Build a link mask with a default array of mnconstr constraints of class ChConstraintTwoBodies().
    ChLinkMask(unsigned int mnconstr);

    /// Copy constructor
    ChLinkMask(const ChLinkMask& source);

    /// Destructor
    virtual ~ChLinkMask();

    /// Assignment operator.
    ChLinkMask& operator=(const ChLinkMask& other);

    /// Set references to variables of two connected bodies to all
    /// constraints at once, therefore also sets all the constraints as active.
    void SetTwoBodiesVariables(ChVariables* var1, ChVariables* var2);

    /// Obtain the reference to the i-th scalar constraint data in the collection link mask.
    ChConstraintTwoBodies& GetConstraint(unsigned int i);

    /// Get the i-th active scalar constraint (inactive constraints won't be considered)
    ChConstraintTwoBodies* GetActiveConstraint(unsigned int mnum);

    /// Add a ChConstraintTwoBodies to mask (NOTE: later, the constraint will
    /// be automatically deleted when the mask will be deleted)
    void AddConstraint(ChConstraintTwoBodies* aconstr);

    /// To compare two masks, return true if equal
    bool IsEqual(ChLinkMask& mask2);

    /// Get the number of constraints.
    unsigned int GetNumConstraints() { return nconstr; }

    /// Set a new number of constraints.
    /// If \a newnconstr differs from current size a reallocation will be triggered.
    void SetNumConstraints(unsigned int newnconstr);

    /// Get the number of active constraints.
    unsigned int GetNumConstraintsActive();

    /// Get the number of active bilateral constraints.
    unsigned int GetNumConstraintsBilateralActive();

    /// Get the number of active unilateral constraints.
    unsigned int GetNumConstraintsUnilateralActive();

    /// If SetAllDisabled(true), all the constraints are temporarily turned
    /// off (inactive) at once, because marked as 'disabled'. Return n.of changed
    unsigned int SetAllDisabled(bool mdis);

    /// If SetAllBroken(true), all the constraints are temporarily turned
    /// off (broken) at once, because marked as 'broken'. Return n.of changed.
    unsigned int SetAllBroken(bool mdis);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    std::vector<ChConstraintTwoBodies*> constraints;  ///< array of pointers to 'n' scalar constraints
    unsigned int nconstr;                             ///< number of scalar constraint equations.
};

CH_CLASS_VERSION(ChLinkMask, 0)

// -----------------------------------------------------------------------------

/// Specialized ChLinkMask class, for constraint equations of the ChLinkLock link.
class ChApi ChLinkMaskLF : public ChLinkMask {
  public:
    /// Create a ChLinkMaskLF which has 7 scalar constraints of class ChConstraintTwoBodies().
    /// This is useful in case it must be used for the ChLinkLock link.
    ChLinkMaskLF();
    ChLinkMaskLF(const ChLinkMaskLF& other) : ChLinkMask(other) {}

    /// Assignment operator.
    ChLinkMaskLF& operator=(const ChLinkMaskLF& other);

    /// Set all mask data at once.
    void SetLockMask(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3);

    /// Obtain the reference to specific scalar constraint data in the collection of this link mask.
    ChConstraintTwoBodies& Constr_X() { return *constraints[0]; }
    ChConstraintTwoBodies& Constr_Y() { return *constraints[1]; }
    ChConstraintTwoBodies& Constr_Z() { return *constraints[2]; }
    ChConstraintTwoBodies& Constr_E0() { return *constraints[3]; }
    ChConstraintTwoBodies& Constr_E1() { return *constraints[4]; }
    ChConstraintTwoBodies& Constr_E2() { return *constraints[5]; }
    ChConstraintTwoBodies& Constr_E3() { return *constraints[6]; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChLinkMaskLF, 0)

}  // end namespace chrono

#endif
