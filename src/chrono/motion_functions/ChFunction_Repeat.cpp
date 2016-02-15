//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Repeat.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Repeat.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Repeat> a_registration_repeat;

void ChFunction_Repeat::Copy(ChFunction_Repeat* source) {
    window_start = source->window_start;
    window_length = source->window_length;
    // fa = source->fa;		//***? shallow copy (now sharing same object)...
    fa = std::shared_ptr<ChFunction>(source->fa->new_Duplicate());  //***? ..or deep copy? make optional with flag?
}

ChFunction* ChFunction_Repeat::new_Duplicate() {
    ChFunction_Repeat* m_func;
    m_func = new ChFunction_Repeat;
    m_func->Copy(this);
    return (m_func);
}

double ChFunction_Repeat::Get_y(double x) {
    return fa->Get_y(this->window_start + fmod(x, this->window_length));
}

void ChFunction_Repeat::Estimate_x_range(double& xmin, double& xmax) {
    fa->Estimate_x_range(xmin, xmax);
}

int ChFunction_Repeat::MakeOptVariableTree(ChList<chjs_propdata>* mtree) {
    int i = 0;

    // inherit parent behaviour
    ChFunction::MakeOptVariableTree(mtree);

    // expand tree for children..

    chjs_propdata* mdataA = new chjs_propdata;
    strcpy(mdataA->propname, "fa");
    strcpy(mdataA->label, mdataA->propname);
    mdataA->haschildren = TRUE;
    mtree->AddTail(mdataA);

    i += this->fa->MakeOptVariableTree(&mdataA->children);

    return i;
}



}  // END_OF_NAMESPACE____

// eof
