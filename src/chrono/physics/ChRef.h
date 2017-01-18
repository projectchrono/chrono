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

#ifndef CHREF_H
#define CHREF_H

#include <cmath>

#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

// CLASS FOR BASIC REFERENCE (a bit OBSOLETE?)
//
/// This is the base data for all types of references.
/// The base implemetation is basically _useless_ unless it has some
/// inherited implementation (see other classes below)

class ChApi ChRef {
  protected:
    bool valid;

  public:
    ChRef() { valid = false; };

    /// Tells if the reference is currently valid.
    /// Instead of implementing it, child classes may simply
    /// set valif=false (or true) depending on the result of their
    /// implementations of RestoreReference();
    virtual bool IsValid() { return valid; }

    /// This may be overloaded by child classes. Argument should be
    /// the 'database' where the reference restoring takes place.
    /// Should return false if referencing was not possible.
    /// Should set valid=true/false depending on referencing success.
    virtual bool RestoreReference() { return false; };
};

// CLASSES FOR REFERENCING CH-FUNCTIONS
//
/// Reference to a generic functions of class ChFunction.
/// This also may be the database for
/// referencing variables in ChFunction objects....

class ChApi ChRefFunction : public ChRef {
  protected:
    ChFunction* function;

  public:
    ChRefFunction() {
        function = NULL;
        valid = false;
    }
    ChRefFunction(ChFunction* mf) {
        function = mf;
        valid = true;
    }

    virtual bool RestoreReference(ChFunction* mrootf) {
        function = mrootf;
        valid = true;
        return true;
    };

    /// Returns the referenced function
    /// ==========>>>>>>
    virtual ChFunction* GetFunction() {
        if (valid)
            return function;
        else
            return NULL;
    };
};

/// Reference to a segment of a ChFunction (of type ChFunctSequence), identified
/// with a sequence of IDs for navigating the segments tree..

#define CHREF_TREE_IDS_MAXLENGTH 20

class ChApi ChRefFunctionSegment : public ChRefFunction {
  protected:
    // identifier of child segment
    char treeIDs[CHREF_TREE_IDS_MAXLENGTH];
    // optim.:storage place for last fetched function segment
    ChFunction* function_segment;

  public:
    ChRefFunctionSegment() {
        function = NULL;
        valid = false;
        strcpy(treeIDs, "");
    }
    ChRefFunctionSegment(ChFunction* mrootf, char* myIDs);

    virtual bool RestoreReference(ChFunction* mrootf);

    /// Returns the referenced segment function
    /// ==========>>>>>>
    virtual ChFunction* GetFunction() {
        if (valid)
            return function_segment;
        else
            return NULL;
    };

    /// Returns the root function (i.e. father sequence containing this segment)
    ChFunction* GetRootFunction() { return ChRefFunction::GetFunction(); }

    /// Change the IDs for ravigating the tree
    bool SetTreeIDs(char* myIDs);
};

/// A reference to an handle of a ChFunction, that is one of the points
/// which can be also dragged with the mouse

class ChApi ChRefFunctionHandle : public ChRefFunction {
  protected:
    // identifier of handle
    int handle_id;

  public:
    ChRefFunctionHandle() {
        function = NULL;
        valid = false;
        handle_id = 0;
    }
    ChRefFunctionHandle(ChFunction* mrootf, int m_hid);

    virtual bool RestoreReference(ChFunction* mrootf);

    /// Access the referenced handle
    /// ==========>>>>>>
    int AccessHandle(double& mx, double& my, bool set_mode);

    /// Change the handle ID
    void SetHandleId(int m_hid);
    int GetHandleId() { return handle_id; };
};

}  // end namespace chrono

#endif
