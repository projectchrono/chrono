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

///////////////////////////////////////////////////
//
//   ChRef.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>
#include "physics/ChRef.h"

namespace chrono {

////////////////////////////////////
//
// CLASS  ChRef
//
////////////////////////////////////

////////////////////////////////////
//
// CLASS  ChRefFunctionSegment
//
////////////////////////////////////

ChRefFunctionSegment::ChRefFunctionSegment(ChFunction* mrootf, char* myIDs) {
    this->function = mrootf;
    this->SetTreeIDs(myIDs);
    this->RestoreReference(this->function);
}

bool ChRefFunctionSegment::RestoreReference(ChFunction* mrootf) {
    // inherit parent behavoiur
    ChRefFunction::RestoreReference(mrootf);

    // default segment is root function, also for ID=""
    this->function_segment = GetRootFunction();
    if (!function_segment)
        return (valid = false);

    static char buffer[100];
    int mpos = 0;
    int mbupos = 0;
    int mid = 0;
    while (treeIDs[mpos] != 0) {
        mbupos = 0;
        while ((treeIDs[mpos] != *":") || (treeIDs[mpos] != 0)) {
            buffer[mbupos] = treeIDs[mpos];
            mpos++;
            mbupos++;
        }
        buffer[mbupos] = 0;
        int mIDi = atoi(buffer);

        if (this->function_segment->Get_Type() != FUNCT_SEQUENCE)
            return (valid = false);

        //***TODO*** use shared_ptr everywhere
        this->function_segment = (((ChFunction_Sequence*)function_segment)->GetNthFunction(mIDi)).get();

        mid++;
    }
    return (valid = true);
}

int ChRefFunctionSegment::SetTreeIDs(char* myIDs) {
    if (strlen(myIDs) < CHREF_TREE_IDS_MAXLENGTH) {
        strcpy(this->treeIDs, myIDs);
        this->RestoreReference(this->function);
        return TRUE;
    } else
        return FALSE;
}

////////////////////////////////////
//
// CLASS  ChRefFunctionHandle
//
////////////////////////////////////

ChRefFunctionHandle::ChRefFunctionHandle(ChFunction* mrootf, int m_hid) {
    this->function = mrootf;
    this->SetHandleId(m_hid);
    this->RestoreReference(this->function);
}

bool ChRefFunctionHandle::RestoreReference(ChFunction* mrootf) {
    // inherit parent behavoiur
    ChRefFunction::RestoreReference(mrootf);

    // implement
    // ***TO DO***???
    return (valid = true);
}

int ChRefFunctionHandle::AccessHandle(double& mx, double& my, bool set_mode) {
    if (valid) {
        if (set_mode) {
        } else {
        }
        return true;
    } else
        return false;
}

int ChRefFunctionHandle::SetHandleId(int m_hid) {
    handle_id = m_hid;
    this->RestoreReference(this->function);

    return TRUE;
}

}  // END_OF_NAMESPACE____

////// end
